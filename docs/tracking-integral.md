# Policy tracking integral

Opt-in `integral` in the external policy JSON enables a torque integral alongside
the unchanged follower PD and gravity feedforward. Missing/null means disabled;
it does not affect manual control or calibration sessions.

```json
{"integral": {"gain_per_s": 0.5, "torque_fraction": 0.25, "slew_fraction_per_s": 0.1}}
```

For J1–J7, torque increment is `gain_per_s * Kp * position_error * dt`.
Each joint's integral cap is 25% of its freshly read firmware torque range;
its slew bound is 10% of that range per second. These are not total joint torque
caps. Existing combined PD/feedforward rejection at 90% remains unchanged;
the integral additionally projects its state into 89% combined-effort headroom.
Firmware encoding range is not a measured continuous-duty or contact-force rating.

Integration is disabled and cleared for gripper J8, speed above 0.1 rad/s,
or position error above 0.15 rad. It uses measured elapsed time, rejects intervals
over 0.25 s, projects the stored integral to prevent windup, and clears on exchange
failure and disable. Feedback validation, watchdogs, gains and joint limits remain.
Lifecycle records include configuration and decimated integral torque samples.

Offline tests do not establish physical stability or sufficient tracking accuracy.
The operator must start the physical trial; no automatic test motion is performed.
