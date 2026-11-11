# Contributing and releasing

Keep the package small: transport, control lifecycle, mapping and optional UI
have separate responsibilities. Add features when a real consumer needs them.
New optional features should not add heavy dependencies to core imports.

## Development

Use Python3.12 and uv. `uv sync --locked` installs an editable core package;
`uv run --locked python -m unittest discover -s tests` runs offline regressions.
For Viser development, add `--extra viewer --extra leader` to `uv sync`.
For the unified NiceGUI dashboard and aiohttp gateway, include `--extra platform`
as well. Keep the extra selections required by the modules you develop.
For RGB/depth capture and the camera web interface, use `--extra video`; run
`openarm-runtime video --simulate` for offline browser development.
For tactile capture, use `--extra tactile`; add `--extra viewer` for visualization.
`openarm-runtime tactile --simulate` uses software data without opening serial ports.
Native follower setup is documented in [workstation setup](docs/workstation.md).

For offline camera calibration, add `--extra calibration` (and `--extra platform`
for the web page). Run `python -m unittest discover -s tests -p 'test_camera*.py'`
and `python -m unittest discover -s tests -p 'test_platform_camera*.py'`.
These use known synthetic projections, indexed board images and borrowed fake
capture snapshots, never physical cameras or CAN. For local installed-wheel
checks, verify ChArUco capability rather than accepting a fully skipped
calibration suite. Keep real camera identities and board measurements
in external profiles/evidence, not fixtures claiming hardware validation.

When developing the runtime together with another application, install this
checkout into that application's environment in editable mode. The checkout can
live anywhere; no sibling-directory layout is required. See the README's
[installation modes](README.md#using-the-package-vs-developing-it) for pip and
uv commands, when to reinstall, and how to return to a pinned release.

Platform framework tests run with
`python -m unittest discover -s tests -p 'test_platform*.py'` after installing
`.[platform]`. They use fake services and local test upstreams, never hardware.
Core-only tests skip framework integration checks while still checking passive
imports, configuration and subprocess ownership. Tests are run manually as needed;
there is no automatic push, PR or release test suite. Browser acceptance
should exercise persistent views, real Viser WebSocket events, both nested
tactile viewers and camera images through one port using software fixtures.

Tests must not open physical hardware. Test behavior at transport boundaries,
including exception cleanup, rather than copying implementation details.
Record actual hardware checks separately in [validation](docs/validation.md).
Keep device configuration, calibration, logs and cached assets outside Git.

Head and wrist camera parameters have a single source on Hugging Face. `configs/cameras/`
keeps only a repository/commit/path reference, without parameter copies, file
hashes or per-file sizes. Raw evidence and machine/device configuration stay
external. See `configs/cameras/README.md` for usage and validity limits.

## API changes

Keep examples and [the API guide](docs/api.md) aligned with public exports.
Use patch versions for compatible fixes and minor versions for breaking0.x API
changes. Document migrations in the changelog. Pin releases in consuming projects;
an editable install is for joint development, not a reproducible deployment.

## Release checklist

1. Update the version in `pyproject.toml`, `src/openarm_runtime/__init__.py` and
   version assertions in `tests/test_public_api.py`; update the changelog and
   README install tag. Run `uv lock` if package metadata/dependencies changed.
2. Run only the local checks relevant to the change, as needed. Local tests remain
   available; a full regression suite or cross-platform CI is not a release gate.
3. Commit and push. Tag the release commit `vX.Y.Z` and push that tag. The release
   workflow validates the tag/version, builds the distributions without running
   tests, then publishes a GitHub release with the changelog, wheel, source
   distribution from that same commit. Version identity is the Git tag/commit;
   no separate file-hash manifest is published.
4. Update downstream pins explicitly after acceptance. A release does not require
   restarting a user's live hardware services.

Releases are on GitHub. The follower extra currently contains a pinned direct Git
dependency, so a future PyPI publication needs a separate packaging review.
