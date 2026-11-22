# PICO real-arm teleoperation with dora

This example records the single-entry change validated on the lab's PICO
WebXR and Classic CAN deployment. The patch applies to the lab's dora
deployment checkout. Its base includes the Classic bridge, state merge,
and measured-state, clutch-aware IK nodes. Apply it from that checkout
with `git apply --check /path/to/single-entry.patch`, followed by
`git apply /path/to/single-entry.patch`. The stock Enactic checkout
does not contain the Classic bridge.

The dedicated dataflow uses the PICO WebXR page, one head camera, the
existing dora Cartesian IK, and the Classic CAN bridge. The right arm is
eligible to start only after the operator enters WebXR and a valid,
fresh IK target arrives in that session. Leaving WebXR stops the arm.
The existing target watchdog also stops the arm when updates cease.
Restoring traffic alone does not start an arm again.

The bridge uses OpenArm Runtime's joint limits and gravity feedforward.
Set `OPENARM_CLASSIC_PROFILE` to the separately managed machine profile and
`OPENARM_HEAD_CAMERA` to the camera device. Use the deployment's existing
HTTPS configuration and Runtime gravity profile and dependency. Machine
profiles, URDF calibration overrides, credentials and logs stay external.

The patch passed dora configuration validation and offline checks for
fresh, stale and invalid targets, operator session end and left-arm
inhibit. Packaging the base deployment as a reproducible integration and
verifying the full Start/exit cycle on hardware are open review items.
