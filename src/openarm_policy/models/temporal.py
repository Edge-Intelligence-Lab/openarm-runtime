"""Optional causal action aggregation, owned by one policy episode, not a server.

NumPy is loaded only on add; SciPy only for the opt-in absolute rotation chart.
This changes predictions, not action semantics or execution safety limits.
"""
import math
from numbers import Integral, Real


class TemporalActionEnsembler:
    """Blend H x D chunks at their actual integer control-clock origins.

    Older overlapping predictions get conventional ACT weights exp(k * age).
    Zero rows are valid holds. Only received, unexpired chunks are retained (at
    most H chunks). Returned arrays and retained predictions never alias callers.
    Use linear rotation averaging for deltas; rotvec_pi for absolute rotvec/pi.
    """
    def __init__(self, coefficient=.01, rotation_average="linear"):
        if (isinstance(coefficient, bool) or not isinstance(coefficient, Real)
                or not 0 <= coefficient <= 1 or not math.isfinite(coefficient)):
            raise ValueError("coefficient must be finite and in [0, 1]")
        if rotation_average not in ("linear", "rotvec_pi"):
            raise ValueError("rotation_average must be linear or rotvec_pi")
        self.coefficient = float(coefficient)
        self.rotation_average = rotation_average
        self.reset()

    @property
    def metadata(self):
        return dict(coefficient=self.coefficient, weight="exp(coefficient * age_steps)",
                    order="older_predictions_preferred", rotation_average=self.rotation_average,
                    alignment="integer_control_clock", valid_rows="all_including_zero")

    def reset(self):
        self._chunks = []
        self._shape = None
        self._last_step = None

    def add(self, chunk, step):
        import numpy as np

        if (isinstance(step, bool) or not isinstance(step, Integral) or step < 0
                or (self._last_step is not None and step <= self._last_step)):
            raise ValueError("step must be a nonnegative strictly increasing integer")
        step = int(step)
        # Support tensors without importing Torch (including device tensors).
        if hasattr(chunk, "detach"):
            chunk = chunk.detach().cpu().numpy()
        try:
            source = np.asarray(chunk)
            if source.dtype.kind not in "fiu":
                raise ValueError("Predictions must be real numeric values")
            with np.errstate(over="ignore", invalid="ignore"):
                actions = np.array(source, dtype=np.float32, copy=True)
        except (TypeError, OverflowError) as exc:
            raise ValueError("Expected finite H x D predictions") from exc
        if (actions.ndim != 2 or min(actions.shape) < 1
                or not np.isfinite(actions).all()):
            raise ValueError("Expected finite nonempty H x D predictions")
        if self._shape is not None and actions.shape != self._shape:
            raise ValueError("Prediction horizon and width must remain fixed until reset")
        if self.rotation_average == "rotvec_pi":
            if actions.shape[1] < 6:
                raise ValueError("rotvec_pi requires rotation columns 3:6")
            from scipy.spatial.transform import Rotation

        horizon = len(actions)
        chunks = [(origin, old) for origin, old in self._chunks if origin + horizon > step]
        chunks.append((step, actions))
        blended = np.empty_like(actions)
        for offset in range(horizon):
            timestamp = step + offset
            contributors = [(origin, old[timestamp-origin]) for origin, old in chunks
                            if timestamp < origin + horizon]
            ages = np.asarray([step-origin for origin, _ in contributors], dtype=np.float64)
            logs = self.coefficient * ages
            weights = np.exp(logs - logs.max())
            weights /= weights.sum()
            rows = np.asarray([row for _, row in contributors], dtype=np.float64)
            blended[offset] = weights @ rows
            if self.rotation_average == "rotvec_pi":
                blended[offset, 3:6] = Rotation.from_rotvec(rows[:, 3:6] * np.pi).mean(
                    weights=weights).as_rotvec() / np.pi
        self._chunks, self._shape, self._last_step = chunks, actions.shape, step
        return blended


class TemporalEnsembleActions:
    """Optional component-spec decoder wrapper; all inner safety checks stay active."""
    def __init__(self, action, coefficient=.01, rotation_average="linear"):
        self.action = action
        self.ensemble = TemporalActionEnsembler(coefficient, rotation_average)
        self.inner = None
        self.period = None
        self.epoch = None
        self._last_capture = None

    @property
    def metadata(self):
        metadata = getattr(self.inner, "metadata", {})
        if not isinstance(metadata, dict):
            raise ValueError("Invalid inner action decoder metadata")
        settings = dict(self.ensemble.metadata, input_normalization=(
            "clip[-1,1] before absolute-goal aggregation" if self.ensemble.rotation_average == "rotvec_pi"
            else "raw finite delta predictions"))
        return dict(metadata, temporal_ensemble=settings)

    def reset(self, context):
        from .policy import component

        if self.inner is not None:
            raise RuntimeError("Close temporal decoder before resetting it")
        hz = context.control_hz
        if (isinstance(hz, bool) or not isinstance(hz, Real)
                or not math.isfinite(hz) or hz <= 0):
            raise ValueError("control_hz must be finite and positive")
        self.period = 1 / hz
        if not math.isfinite(self.period) or self.period <= 0:
            raise ValueError("Invalid control period")
        self.epoch = self._last_capture = None
        self.ensemble.reset()
        self.inner = component(self.action)
        # ModelPolicy owns reset-error cleanup and calls this wrapper's close.
        self.inner.reset(context)

    def decode(self, output, observation):
        if self.inner is None or self.period is None:
            raise RuntimeError("Temporal decoder must be reset before decode")
        captured = observation.captured_at
        if (isinstance(captured, bool) or not isinstance(captured, Real)
                or not math.isfinite(captured)
                or (self._last_capture is not None and captured <= self._last_capture)):
            raise ValueError("Observation capture times must be finite and strictly increasing")
        epoch = captured if self.epoch is None else self.epoch
        elapsed_steps = (captured - epoch) / self.period
        if not math.isfinite(elapsed_steps):
            raise ValueError("Invalid observation control-clock time")
        if self.ensemble.rotation_average == "rotvec_pi":
            import numpy as np
            source = output.detach().cpu().numpy() if hasattr(output, "detach") else np.asarray(output)
            if source.dtype.kind not in "fiu" or not np.isfinite(source).all():
                raise ValueError("Expected finite normalized absolute-goal predictions")
            # Interpret only valid normalized goals as rotations; clipping after
            # a principal-angle mean would give different semantics for >1 rows.
            output = np.clip(source, -1, 1)
        blended = self.ensemble.add(output, int(round(elapsed_steps)))
        self.epoch, self._last_capture = epoch, captured
        return self.inner.decode(blended, observation)

    def close(self):
        inner, self.inner = self.inner, None
        try:
            if inner is not None and hasattr(inner, "close"):
                inner.close()
        finally:
            self.ensemble.reset()
            self.period = self.epoch = self._last_capture = None
