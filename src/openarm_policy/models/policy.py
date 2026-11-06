"""Three independent plugin seams: predictor, observation encoder, action decoder.

Factories receive only JSON kwargs. Components and optional libraries are created
in reset, inside the existing spawned inference worker, never on a CAN thread.
"""
import importlib

from ..contracts import ActionChunk, finite


def component(spec):
    if not isinstance(spec, dict) or set(spec) - {"factory", "kwargs"}:
        raise ValueError("Component needs factory and optional kwargs")
    module, name = spec["factory"].split(":", 1)
    return getattr(importlib.import_module(module), name)(**spec.get("kwargs", {}))


class ModelPolicy:
    """Predictor.reset/predict/close, encoder.reset/encode, decoder.reset/decode.

    Encoder owns sensor selection and preprocessing. Predictor owns checkpoint
    format/normalization. Decoder owns action semantics, not motor control.
    Future ONNX, remote or other policy-family backends use the same three seams.
    """
    def __init__(self, model, observation, action, period_s=.05, lead_s=0.):
        if not finite(period_s) or period_s <= 0 or not finite(lead_s) or lead_s < 0:
            raise ValueError("Invalid model action timing")
        self.specs = (model, observation, action)
        self.period_s, self.lead_s = period_s, lead_s
        self.components = []

    def reset(self, context):
        if self.components:
            raise RuntimeError("Construct a fresh ModelPolicy per episode")
        if abs(self.period_s * context.control_hz - 1) > 1e-8:
            raise ValueError("Model period must match control_hz")
        self.context = context
        try:
            for spec in self.specs:
                obj = component(spec)
                self.components.append(obj)
                obj.reset(context)
            model, encoder, decoder = self.components
            model_meta = getattr(model, 'metadata', {})
            if not isinstance(model_meta, dict):
                raise ValueError('Invalid model metadata')
            model_meta = model_meta.get('model', model_meta)
            if not isinstance(model_meta, dict):
                raise ValueError('Invalid wrapped model metadata')
            width = model_meta.get('state_width')
            encoded_width = getattr(encoder, 'metadata', {}).get('state_width')
            if encoded_width == 10 and width is None:
                raise ValueError('Execution-aware encoder requires a checkpoint publishing its state width')
            if width is not None and encoded_width is not None and width != encoded_width:
                raise ValueError('Checkpoint and observation encoder state widths differ')
            decoder_meta = getattr(decoder, 'metadata', {})
            if not isinstance(decoder_meta, dict):
                raise ValueError('Invalid action decoder metadata')
            key = 'action_contract_sha256'
            if key in model_meta or key in decoder_meta:
                for meta in (model_meta, decoder_meta):
                    digest = meta.get(key)
                    if (not isinstance(digest, str) or len(digest) != 64
                            or any(c not in '0123456789abcdef' for c in digest)):
                        raise ValueError('Both model and decoder must publish valid action_contract_sha256')
                if model_meta[key] != decoder_meta[key]:
                    raise ValueError('Checkpoint and action decoder contract SHA256 differ')
        except BaseException:
            self.close()
            raise

    def infer(self, observation):
        model, encoder, decoder = self.components
        output = model.predict(encoder.encode(observation))
        steps = tuple(tuple(float(v) for v in row) for row in decoder.decode(output, observation))
        for row in steps:
            self.context.action.validate(row)
        return ActionChunk(observation.sequence, observation.captured_at + self.lead_s,
                           self.period_s, steps, self.context.action.names,
                           kind=self.context.action.kind,
                           diagnostics={"model": getattr(model, "metadata", {}),
                                        "observation_adapter": getattr(encoder, "metadata", {}),
                                        "action_adapter": getattr(decoder, "metadata", {}),
                                        "raw_actions": output.tolist() if hasattr(output, "tolist") else output})

    def close(self):
        errors = []
        for obj in reversed(self.components):
            try:
                if hasattr(obj, "close"):
                    obj.close()
            except BaseException as exc:
                errors.append(exc)
        self.components = []
        if errors:
            raise BaseExceptionGroup("Model component cleanup failed", errors)
