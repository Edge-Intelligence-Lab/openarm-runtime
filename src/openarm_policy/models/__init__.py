"""Lazy model plugins; importing pure numerical helpers needs no execution stack."""

__all__ = ["ModelPolicy"]


def __getattr__(name):
    if name != 'ModelPolicy':
        raise AttributeError('module %r has no attribute %r' % (__name__,name))
    from .policy import ModelPolicy
    globals()[name] = ModelPolicy
    return ModelPolicy


def __dir__():
    return sorted(set(globals()) | set(__all__))
