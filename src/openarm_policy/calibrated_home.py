"""Model-free adapter for the existing synchronized follower motor-zero return."""
from types import SimpleNamespace

from openarm_runtime.homing import HomeCoordinator
from .contracts import POSITION, finite
from .preparation import PreparationStep


class CalibratedZeroReturn:
    """Reuse the established home controller, not another pose/IK configuration.

    Calibration lives in the motor encoders. Only J1..J7 return to zero; each
    gripper holds its entry position. This joint route is not collision planning.
    """
    def __init__(self):
        self.spec = None
        self.started = False
        self.commands = {}
        self.sides = ('left', 'right')
        self.home = HomeCoordinator(self.sides)
        self.metadata = dict(kind='calibrated_bimanual_zero_v1', learned=False,
            destination='existing_calibrated_motor_zero', gripper='hold_entry',
            sides=list(self.sides), peak_speed_deg_s=self.home.speed_deg_s,
            tolerance_deg=self.home.tolerance_deg, settle_s=self.home.settle_s,
            settle_timeout_s=self.home.settle_timeout_s,
            collision_scope='joint_return_not_obstacle_planning')

    def bind(self, robot, context):
        expected = tuple(f'{side}.joint{i}' if i < 8 else f'{side}.gripper_motor'
                         for side in self.sides for i in range(1, 9))
        self.spec = robot.spec
        if self.spec.kind != POSITION or self.spec.names != expected:
            raise ValueError('Calibrated zero return requires both arms in left/right joint order')
        if not finite(context.control_hz) or not 20 <= context.control_hz <= 200:
            raise ValueError('Calibrated zero return requires 20..200Hz control')
        # Validate the calibrated arm target before any hardware startup.
        for grip in (0, 1):
            self.spec.validate(tuple((self.spec.lower if grip == 0 else self.spec.upper)[i]
                                     if i % 8 == 7 else 0. for i in range(16)))

    def measured(self, observation):
        states = {}
        for side in self.sides:
            history = observation.streams.get('robot/'+side, ())
            if not history or not history[-1].valid or history[-1].error:
                raise RuntimeError('Both arms require valid return feedback')
            sample = history[-1]
            if (not finite(sample.received_at) or
                    not 0 <= observation.captured_at-sample.received_at <= .15):
                raise RuntimeError('Calibrated return feedback stale')
            if len(sample.data) != 8 or any(len(row) != 3 or not all(finite(v) for v in row)
                                            for row in sample.data):
                raise ValueError('Return requires eight finite joint feedback rows per arm')
            states[side] = {i: SimpleNamespace(position=float(row[0]), velocity=float(row[1]))
                            for i, row in enumerate(sample.data, 1)}
        return states

    def values(self):
        return tuple(self.commands[side][mid] for side in self.sides for mid in range(1, 9))

    def begin(self, observation):
        if self.spec is None or self.started:
            raise RuntimeError('Bind once and use a fresh calibrated return')
        states = self.measured(observation)
        measured = tuple(states[side][mid].position for side in self.sides for mid in range(1, 9))
        # Bound references, not feedback. At a one-sided zero stop, quantized
        # feedback can be just outside the command interval. Never write/re-zero
        # encoders or replace measurements to manufacture convergence.
        initial = tuple(max(lo, min(hi, q)) for q, lo, hi in
                        zip(measured, self.spec.lower, self.spec.upper))
        if any(abs(a-b) > .15 for a, b in zip(initial, measured)):
            raise RuntimeError('Initial return reference exceeds tracking boundary')
        self.spec.validate(initial)
        self.home.request()
        for index, side in enumerate(self.sides):
            self.commands[side] = dict(enumerate(initial[index*8:index*8+8], 1))
            self.home.step(side, self.commands[side], states[side], observation.captured_at)
        self.started = True
        return dict(metadata=self.metadata, initial_joints_rad=list(measured),
                    initial_command_rad=list(initial), planned_motion_s=self.home.duration,
                    skipped_to_ready=False)

    def step(self, observation):
        if not self.started:
            raise RuntimeError('Return has not started')
        states = self.measured(observation)
        for side in self.sides:
            for mid in range(1, 9):
                if abs(self.commands[side][mid]-states[side][mid].position) > (.20 if mid == 8 else .15):
                    raise RuntimeError('Calibrated return tracking error exceeded')
        for side in self.sides:
            self.commands[side] = self.home.step(side, self.commands[side], states[side], observation.captured_at)
        values = self.values()
        self.spec.validate(values)
        complete = self.home.settled(observation.captured_at)
        if self.home.phase == 'holding' and not complete:
            raise RuntimeError('Both arms must remain settled at calibrated zero')
        return PreparationStep(values, 'calibrated_bimanual_zero', self.home.progress, complete,
                               dict(phase=self.home.phase, sides=list(self.sides)))
