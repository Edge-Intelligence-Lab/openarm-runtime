# 头部相机基准配置

这是当前 OpenArm V2 安装上，经操作者确认的 **Astra Pro RGB 头部相机基准**。
它包含实测内参、畸变、相机到机器人基座的外参，以及数据来源和适用条件。
它不是所有 Astra Pro 的通用出厂参数，也不是程序启动时自动应用的全局默认值。

入口是 [`current-head.json`](current-head.json)，其中的相对路径和 SHA-256 指向
不可就地覆盖的版本文件 [`head-astra-pro/20261110-v1.json`](head-astra-pro/20261110-v1.json)。
后续修订应增加版本文件、保留旧版，并在重新核对后更新入口及其哈希；不要只改矩阵。

## 怎么使用

- `camera.matrix` 是原生 **1280×720** 图像的 OpenCV 内参 K。
  `camera.distortion` 的顺序是 `[k1, k2, p1, p2, k3]`。
- `extrinsics.base_from_camera` 将相机光学坐标转换到机器人 URDF 基座坐标：
  `p_base = base_from_camera @ p_camera`，使用齐次列向量，平移单位为米。
  相机轴是 **右、下、前**。不要直接把这个矩阵当作相机的 view matrix；
  基座点转到相机坐标时，应取它的逆。
- 基座名 `world` 是已固定 URDF 的根坐标，不是测量过的房间/桌面世界坐标。
  仿真场景若有自己的世界偏移，应另外组合该变换。
- MuJoCo 相机轴是 **右、上、后**，因此旋转使用
  `R_base_mujoco = R_base_camera @ diag(1, -1, -1)`。
  保留 K 中不相等的焦距和非居中的主点，不要只用一个近似视场角替代。
- 仿真是理想针孔投影。与实拍对照时，使用同一个 K 和已保存的畸变参数去畸变；
  不做额外图像配准，也不手调相机以掩盖模型误差。
- `robot.kinematics` 记录标定使用的 URDF 哈希、关节映射和工具坐标。
  这些 locked 关节仅是当时 FK 的建模约定，不是当前右臂姿态。
  `calibration_fixture.tool_from_board` 仅描述当时的装板，不是永久 TCP。

使用前，调用者必须明确选用此安装实例并核对外部设备身份；不能仅因型号相同就套用。
仓库不包含 USB 拓扑、设备端口、个人主机路径或认证信息。本次发布不修改实物运行配置，
不自动启用跟随、相机流或电机；也不依赖尚未发布的仿真重放实现。
文件随 Git 和源码分发包保存，消费程序应显式选择配置路径，而非假设安装 wheel 会自动启用它。

## 这版已经检查了什么

- 原图、观测和机器人 FK 身份已核对，离线重算与保存结果一致。
- **12 张拟合 + 4 张独立留出检查**；留出 RMS 为 **3.38 mm / 0.78°**。
  这是同组一致性残差，不是独立测量的绝对定位精度。
- 已按 16 个实测关节姿态复现仿真相机，并查看 4 个留出姿态的实拍/仿真对应关系；
  没有手调相机。渲染器的轴向、非居中主点和索引板纹理也已离线核对。
- 内参边缘覆盖不足、静止时间区间不等于精确曝光同步等限制保留在版本文件中。

**相机移动、安装松动、对焦/镜头改变，或图像裁剪/缩放后，都要重新核对适用性。**
原始图像及连续关节反馈仍在外部证据存储，版本文件记录其原始文件与规范 JSON 的哈希，
两类哈希不可混用；精简后的本文件有自己的哈希，由 `current-head.json` 固定。

这是操作者明确要求发布的、去除个人环境信息的相机基准例外。
其他机器配置、原始标定证据、日志和模型仍遵循仓库的外部存储规则，不应一起提交。

## 已绑定的 Hugging Face 仿真资产

[`huggingface-head.lock.json`](huggingface-head.lock.json) 固定私有模型仓库
`tacpilot/tacpilot` 的完整提交和 `v0.3.1` 资产清单哈希；不要用不断变化的 `main`
代替锁中的 revision。仿真组件是 `head-camera`，其入口为
`assets/simulation/cameras/head-astra-pro/20261110-v1/scene.xml`，相机名为
**`head_calibrated`**。HF 内的 `calibration.json` 与这里的版本文件逐字节相同。

有 TacPilot 源码环境时，可复用其现有 SSH/LFS 资产下载器，只下载相机和模型依赖：

```bash
python -m tacpilot.artifacts fetch \
  --lock /path/to/openarm-runtime/configs/cameras/huggingface-head.lock.json \
  --directory /path/to/new/external/assets --component head-camera
```

需要已有 HF SSH 授权和 Git LFS，不要求额外 API Token。不下载策略权重或示教数据。
下载目录不能已存在，模型/场景保持原目录结构。

```python
import mujoco

model = mujoco.MjModel.from_xml_path(
    asset_root + '/assets/simulation/cameras/head-astra-pro/20261110-v1/scene.xml')
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)
with mujoco.Renderer(model, height=720, width=1280) as renderer:
    renderer.update_scene(data, camera='head_calibrated')
    rgb = renderer.render()
```

这只加载并渲染仿真，不连接实物。入口引用原有机器人/夹爪场景，只增加固定相机和
离屏分辨率；模型原始场景、历史训练/评测相机与模型权重不变。新工作明确选择这个
相机入口；旧策略仍保留原有图像契约，不能因为标定已固定就宣称旧策略可直接换视角。
场景的基座偏移另见绑定文件，不要将场景世界坐标误作物理桌面测量。
