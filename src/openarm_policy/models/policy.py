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
    def __init__(self, model, observation, action, period_s=.05, lead_s=0., diagnostic_inputs=False):
        if not finite(period_s) or period_s <= 0 or not finite(lead_s) or lead_s < 0:
            raise ValueError("Invalid model action timing")
        self.specs = (model, observation, action)
        self.period_s, self.lead_s = period_s, lead_s
        self.components = []
        if type(diagnostic_inputs) is not bool:
            raise ValueError('diagnostic_inputs must be boolean')
        self.diagnostic_inputs = diagnostic_inputs

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
            encoder_meta = getattr(encoder, 'metadata', {})
            state_key = 'state_contract'
            if state_key in model_meta or state_key in encoder_meta:
                for meta in (model_meta, encoder_meta):
                    if not isinstance(meta.get(state_key), dict) or not meta[state_key]:
                        raise ValueError('Both model and encoder must publish a state contract')
                if model_meta[state_key] != encoder_meta[state_key]:
                    raise ValueError('Checkpoint and observation encoder state contracts differ')
            image_key = 'image_contract'
            if image_key in model_meta or image_key in encoder_meta:
                for meta in (model_meta, encoder_meta):
                    if not isinstance(meta.get(image_key), dict) or not meta[image_key]:
                        raise ValueError('Both model and encoder must publish an image contract')
                if model_meta[image_key] != encoder_meta[image_key]:
                    raise ValueError('Checkpoint and observation encoder image contracts differ')
            decoder_meta = getattr(decoder, 'metadata', {})
            if not isinstance(decoder_meta, dict):
                raise ValueError('Invalid action decoder metadata')
            key = 'action_contract'
            if key in model_meta or key in decoder_meta:
                for meta in (model_meta, decoder_meta):
                    if not isinstance(meta.get(key), dict) or not meta[key]:
                        raise ValueError('Both model and decoder must publish an action contract')
                if model_meta[key] != decoder_meta[key]:
                    raise ValueError('Checkpoint and action decoder contracts differ')
        except BaseException:
            self.close()
            raise

    def infer(self, observation):
        model, encoder, decoder = self.components
        inputs = encoder.encode(observation)
        output = model.predict(inputs)
        steps = tuple(tuple(float(v) for v in row) for row in decoder.decode(output, observation))
        for row in steps:
            self.context.action.validate(row)
        return ActionChunk(observation.sequence, observation.captured_at + self.lead_s,
                           self.period_s, steps, self.context.action.names,
                           kind=self.context.action.kind,
                           diagnostics={"model": getattr(model, "metadata", {}),
                                        "observation_adapter": getattr(encoder, "metadata", {}),
                                         "action_adapter": getattr(decoder, "metadata", {}),
                                         **({'encoded_inputs': inputs,
                                             'input_captured_at': observation.captured_at,
                                             'input_sources': {key: {'sequence': samples[-1].sequence,
                                                 'received_at': samples[-1].received_at}
                                                 for key, samples in observation.streams.items() if samples}}
                                            if self.diagnostic_inputs else {}),
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
