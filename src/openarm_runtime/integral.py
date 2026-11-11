"""Optional slow-error torque integral; no hardware or optional dependencies."""
import math


class TrackingIntegral:
    def __init__(self, gain_per_s=.5, torque_fraction=.25, slew_fraction_per_s=.1):
        for name, value, upper in (("gain_per_s", gain_per_s, 5.),
                                   ("torque_fraction", torque_fraction, .8),
                                   ("slew_fraction_per_s", slew_fraction_per_s, 1.)):
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not 0 < value <= upper:
                raise ValueError('Invalid integral ' + name)
        self.gain, self.fraction, self.slew = gain_per_s, torque_fraction, slew_fraction_per_s
        self.reset()

    def reset(self):
        self.values = dict.fromkeys(range(1, 9), 0.)
        self.previous = None

    def update(self, states, targets, kp, kd, ranges, feedforward, now):
        dt = 0. if self.previous is None else now-self.previous
        if not math.isfinite(dt) or not 0 <= dt <= .25:
            self.reset()
            raise RuntimeError('Integral control deadline missed')
        self.previous = now
        result = dict(feedforward)
        for mid in range(1, 9):
            state = states[mid]
            error = targets[mid]-state.position
            # Never integrate gripper contact, fast travel, or large tracking errors.
            if mid == 8 or abs(error) > .15 or abs(state.velocity) > .1:
                self.values[mid] = 0.
                continue
            limit = ranges[mid].torque
            base = kp[mid-1]*error-kd[mid-1]*state.velocity+feedforward[mid]
            if not all(math.isfinite(v) for v in (error, base, limit)) or limit <= 0:
                self.reset()
                raise RuntimeError('Invalid integral feedback or torque range')
            cap = self.fraction*limit
            step = max(-self.slew*limit*dt, min(self.slew*limit*dt,
                       self.gain*kp[mid-1]*error*dt))
            # Project the integrator itself, not just its output: no hidden windup.
            lower = max(-cap, -.89*limit-base)
            upper = min(cap, .89*limit-base)
            if lower > upper:
                self.values[mid] = 0.  # Existing combined-effort guard handles this.
            else:
                self.values[mid] = max(lower, min(upper, self.values[mid]+step))
            result[mid] += self.values[mid]
        return result
