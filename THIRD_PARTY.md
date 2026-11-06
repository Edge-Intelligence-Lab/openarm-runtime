# Third-party dependencies and reference material

- OpenArm CAN1.4.1: `https://github.com/enactic/openarm_can`, pinned revision
  `bfd4e52194aefe38389fa6e670be3738136facb9`. Used as a dependency, not vendored.
- Feetech Servo SDK1.0.0: `https://pypi.org/project/feetech-servo-sdk/`;
  PySerial3.5: `https://pypi.org/project/pyserial/`. Installed with the leader extra.
- Paxini SDK0.1.0 (MIT): `https://github.com/Jingyi-Z/paxini-sdk`, pinned revision
  `79c82145f88e158dd7706736e8c07f14f11cbd7a`. Community SDK installed with the
  tactile extra; SDK code and sensor coordinates are not vendored. PySerial3.5
  is also installed with this extra.
- Video extra: opencv-python-headless4.13.0.92 (Apache2.0 upstream), NumPy2.5.4
  (BSD3-Clause), Flask3.1.3 (BSD3-Clause), installed as dependencies, not vendored.
- Astra Pro depth uses caller-provided Orbbec OpenNI2 headers, library and drivers.
  Reference distribution: https://github.com/orbbec/ros2_astra_camera . No vendor
  binary, SDK header, license grant or firmware is bundled. This package includes
  only its own C++ adapter source; users must obtain an SDK under its own terms.
- Platform extra: NiceGUI3.18.0 (MIT) and aiohttp3.14.5 (Apache-2.0 AND MIT),
  installed as dependencies. NiceGUI supplies its frontend assets and uses
  FastAPI (MIT), Starlette and Uvicorn (BSD3-Clause); their sources are not vendored.
- Viser1.1.1, yourdfpy and pycollada are optional visualization dependencies.
- Official OpenArm description assets are downloaded into a caller-supplied cache
  and hash-verified; the asset preparation workflow retains upstream LICENSE.txt.
- Follower gains and leader direction/J6–J7 mapping were checked against the user's
  LeRobot/OpenArm reference integration. That imported reference tree is not copied
  into this repository. Protocol facts and retained validation are documented in
  docs/validation.md.

Upstream packages/assets retain their own licenses; the MIT license in this
repository covers this project's own source, not those separate dependencies.

## TacPilot ACT adapter

`src/openarm_policy/models/act.py` adapts the strict loader, normalization and
fixed-relative quaternion chart from TacPilot's `policies/act_runtime.py` and
`policies/act_observation.py`, repository
`https://github.com/Edge-Intelligence-Lab/tactile_robotics_manipulation`.
LeRobot, Torch, torchvision and safetensors remain optional external dependencies;
no model weights or their licenses are bundled.

The upright preparation example also adapts the waypoint recipe from TacPilot's
`experiments/openarm_upright/preparation.py`, under the same MIT notice. The
generic setup sequencer and hardware lifecycle remain runtime-owned code.

The adapted TacPilot portions retain this notice:

MIT License

Copyright (c) 2026 TacPilot contributors

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
