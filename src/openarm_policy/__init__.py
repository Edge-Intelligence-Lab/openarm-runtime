"""Passive, lazy public exports; numerical plugins need not load robot contracts."""

__all__ = ["ActionChunk", "ActionSpec", "Context", "Observation", "Policy", "SensorSample", "StreamSpec",
            "ObservationBuffer", "TactileObservation", "VideoObservation", "HTTPVideoObservation", "ClassicRobot", "SimulatedRobot",
           "PolicySession", "SessionConfig", "ChunkPolicyAdapter", "ImportedPolicy", "EpisodeRecorder",
             "replay_observations", "FixedJointPreparation"]

_MODULES = {
    'contracts': ('ActionChunk','ActionSpec','Context','Observation','Policy','SensorSample','StreamSpec'),
    'observations': ('ObservationBuffer','TactileObservation','VideoObservation','HTTPVideoObservation'),
    'robots': ('ClassicRobot','SimulatedRobot'),
    'session': ('PolicySession','SessionConfig'),
    'adapters': ('ChunkPolicyAdapter','ImportedPolicy'),
    'recording': ('EpisodeRecorder','replay_observations'),
    'preparation': ('FixedJointPreparation',),
}


def __getattr__(name):
    from importlib import import_module
    for module,names in _MODULES.items():
        if name in names:
            value = getattr(import_module('.'+module,__name__),name)
            globals()[name] = value
            return value
    raise AttributeError('module %r has no attribute %r' % (__name__,name))


def __dir__():
    return sorted(set(globals()) | set(__all__))
