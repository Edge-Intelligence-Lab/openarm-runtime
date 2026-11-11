# 相机参数引用

统一入口是 [`current.json`](current.json)。头部与腕部共用 HF 仓库和完整提交号，
分别在 `cameras.head.path` 和 `cameras.wrist.path` 中记录路径；runtime 不保存参数副本、
文件哈希或逐文件大小。后续在 HF 修订参数，核对后更新这里的提交号。

## 怎么使用

按 `repository`、`repo_type`、`revision` 和所选相机的 `path` 从 HF 获取文件，
保存在仓库外，再把文件路径显式传给消费程序。入口本身不是参数文件；runtime
不会自动下载、加载或启用硬件，单元测试也不访问网络。引用随 Git 和源码分发包保存。

- 头部：`assets/simulation/cameras/head/calibration.json`，当前 OpenArm V2 安装上
  经操作者确认的 Astra Pro RGB 基准，不是通用出厂参数。
- 腕部：`assets/simulation/cameras/wrist/calibration.json`，包含一组内参、畸变和
  左右腕位置，不是两台相机各自的完整实测标定，也不包含完整旋转外参。
  不能把位置字段当作完整的 `tool_from_camera` 变换。

## 头部坐标约定

以下约定与验证结果仅描述头部基准，不适用于腕部文件。

- `camera.matrix` 是原生 1280×720 图像的 OpenCV 内参 K；畸变顺序为
  `[k1, k2, p1, p2, k3]`。
- `extrinsics.base_from_camera` 使用齐次列向量，将相机光学坐标转换到 URDF 基座：
  `p_base = base_from_camera @ p_camera`，平移单位为米。相机轴为右、下、前。
  基座到相机转换需取逆，不能直接当作 view matrix。
- 基座 `world` 是 URDF 根坐标，不是测量过的房间或桌面坐标；仿真世界偏移需另行组合。
- MuJoCo 相机轴为右、上、后，使用
  `R_base_mujoco = R_base_camera @ diag(1, -1, -1)`。保留不相等的焦距和非居中主点。
- 仿真采用理想针孔投影；实拍对照用同一个 K 和保存的畸变参数去畸变，不额外配准或
  手调相机掩盖模型误差。
- `robot.kinematics` 保存关节映射和工具坐标；locked 关节是标定时的 FK 建模约定，
  不是当前右臂姿态。`calibration_fixture.tool_from_board` 描述当时装板，不是永久 TCP。

## 这版已经检查了什么

- 原图、观测和 FK 身份已核对，离线重算与保存结果一致。
- 12 张拟合、4 张独立留出检查；留出 RMS 为 3.38 mm / 0.78°，是同组一致性残差，
  不是独立测量的绝对定位精度。
- 已按 16 个实测关节姿态复现仿真相机，并查看 4 个留出姿态的实拍/仿真对照；
  没有手调相机。渲染轴向、非居中主点和索引板纹理已离线核对。
- 内参边缘覆盖不足、静止区间不等于精确曝光同步等限制保留在 HF 文件中。

使用前必须明确选用安装实例并核对外部设备身份，不能仅凭相机型号套用。
相机移动、安装松动、对焦/镜头改变或图像裁剪/缩放后，都要重新核对适用性。
不宣称独立实物精度认证，也不自动启用跟随、相机流或电机。
原始图像、连续关节反馈、机器配置、日志和模型仍保存在外部，不应一起提交。
