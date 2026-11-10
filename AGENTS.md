# OpenArm runtime maintenance

- This repository is an installable Python 3.12 package for OpenArm hardware. Keep
  hardware transport, lifecycle/control, leader mapping and optional GUI separate.
- Core imports and unit tests must not require Viser, serial hardware or the native
  CAN SDK. Load optional dependencies only in the operation that needs them.
- Follower hardware uses Linux SocketCAN Classic 1 Mbps, official CAN SDK1.4.1.
  Keep validated gains, limits, firmware range decoding and disable cleanup intact.
- Leader reads never write servo registers. Per the operator's explicit 2026-10-31
  decision, voltage-only alarms on fresh valid feedback are nonblocking for both
  virtual display and physical following. Other/combined fault bits, corrupt or
  stale feedback and enabled leaders still reject physical following.
- Do not run real motor movement for packaging or refactoring verification. Use
  offline tests, installed-wheel tests and explicit user-operated real-machine trials.
- Do not commit machine profiles, calibration, credentials, logs, models, SDK builds
  or caches. Configure device paths externally. Preserve third-party license notices.
- Sole calibration exception: the operator explicitly approved publishing the
  sanitized, installation-specific head-camera baseline under `configs/cameras/`.
  Keep its versions, hashes, frames and limitations; never include raw evidence,
  personal paths/device topology, or auto-activate it on another installation.
- Maintain one CLI, concise Viser interfaces, and public APIs documented in README.
- Initial extraction keeps the legacy CAN lock directory to interoperate with the
  original bimanual_tactile installation; do not change one side independently.
