# 在统一网页中部署模型

使用现有 Platform 的 **模型与执行** 页面（`/#policy`），不是另开控制网页。
日常流程是 **选模型 → 加载 → 启动**。加载自动检查身份和已配置的样例输入输出，
不需要单独点测试。打开平台或选择模型不会加载权重，更不会启用机械臂。
推理环境与控制/相机环境可以分开；详细说明放在控件与状态的悬停提示中。

## 日常使用

1. 正常退出旧控制入口，避免两套网页或相机进程争用同一设备。按工作站
   配置从仓库运行 `python3 run_platform.py --profile /ABS/platform.json`。
2. 打开网页，进入 **模型与执行**。从目录中选模型，核对名称、版本、
   身份和任务说明，再点 **加载**。这只启动平台拥有的本地推理进程。
3. 等待 **已就绪**：加载自动使用注册样例检查有限输出、动作块形状和推理时延。
   不读取设备、不下发动作；失败或取消不会放行执行。没有配置样例时仅为
   **已加载**，不能执行，不会虚构输入或用读取设备来补齐。
   **这不是双路实景输入检查、硬件标定或任务成功率验证。**
4. 查看执行配置提示、机械臂准备位和视频。若配置没有准备流程，先在遥操
   工作区手动就位，不要期望模型从任意姿态开始。工作站硬件、限位、
   TCP/坐标系、夹爪映射、相机归属、反馈新鲜度和退出保护仍须核对。
5. 点 **启动**直接启动既有 policy CLI。完成准备后持续执行，没有总运行时长限制，
   由操作员手动停止；不要无人值守。页面不提供时长或结束后自动归位选项。
   没有声明勾选或二次确认。已明确声明的名义几何配置可运行，
   但仍保留可见的“名义几何”提示，详情可悬停查看；未知几何状态仍拒绝启动。
6. **停止** 请求原控制会话正常退出、验证失能与清理。等待结束后才可
   切换模型、卸载或交回遥操。页面连接断开会请求停止由该页面启动的会话。
   停止和退出会走原有失能清理，不是持续保持姿态；必须确保机械臂有支撑。
   这不是取代实体急停的保证；网络和控制线程故障仍靠既有本地看门狗处理。
   需要返回自然下垂时，等待退出清理完成，再手动点 **双臂下垂**。它复用两臂
   已标定的电机零位，不需加载模型或另填下垂角度；停止不会自动返程。
7. 空闲时可直接选另一个模型并加载，自动释放原推理进程；不用先点卸载。
   **卸载模型** 图标也可释放推理进程，不自动恢复其他控制服务。会话默认只写紧凑
   `summary.json`，保留身份、计数、时延和清理结果，不保存图像或动作块。
   程序正常退出不等于任务成功，成功判断默认仍为空。

按模型显示源预览、实际预测输入和实物Rerun回放，见
[physical-rerun.md](physical-rerun.md)。显式启用`rerun_recording`后会额外保存
`.rrd`，不再是仅摘要；旧的摘要回合不能补出图像。

同一时刻只有一个模型执行控制租约。加载及其样例校验不抢占遥操。显式启动可正常
交接平台管理的控制服务；外部控制服务不能被平台强停。清理不完整或异常
退出且缺少完整清理回执会阻止后续控制接管，不能只凭进程消失就声称电机已安全停止。
策略失败但完整回执确认清理正常时，显示异常停止，不冒充成功，也不因非零
退出码单独判为失能未确认。
纯推理进程可在超时后终止；拥有实机控制的进程不被强杀。

## 注册一个模型

在**工作站外部配置**中给 `platform.json` 增加：

```json
{"model_catalog": "/ABS/models/catalog.json"}
```

模型目录是可信的本机配置，不是从浏览器执行任意路径或命令的接口。
网页只传注册ID。端口不能与平台或其他服务重叠；被占用的模型端口不会
被当成自己的服务接管。修改目录后先卸载，再点 **刷新目录**。

```json
{
  "schema": "openarm.model-catalog.v1",
  "port": 18765,
  "models": [{
    "id": "upright-current-camera-v040",
    "label": "圆柱扶正 · 当前相机 · v0.4.0",
    "description": "选中45000步；既有仿真30/300，不是可靠自主实物能力。",
    "model_profile": "predictor.json",
    "identity": {
      "model_sha256": "71197475f4540873de0df7462c44855555fa4a1b3d1e93e541b5927d93f1e08e",
      "image_contract_sha256": "f9c60adf083f76a12463631b97a93649429a759aca4cae02b014a3ba665e53da"
    },
    "output_shape": [100, 7],
    "probe_inputs": "encoded-sample.npz",
    "inference_python": "/ABS/model-env/bin/python",
    "runtime_python": "/ABS/robot-env/bin/python",
    "inference_env": {"PYTHONPATH": "/ABS/installed-source:/ABS/isolated-extra"},
    "runtime_env": {"PYTHONPATH": "/ABS/installed-source"},
    "policy_profile": "policy.json",
    "robot_config": "/ABS/robot/classic.json",
    "robot": "classic",
    "sides": ["left"]
  }]
}
```

目录中的文件路径相对于目录文件解析；解释器路径保留虚拟环境符号链接。
旧目录中的 `max_duration_s` 仅约束显式指定时长的程序调用，不给网页手动停止模式
施加隐藏时限。网页使用 `policy --until-stopped`，不会回退到 CLI 的默认十秒诊断回合。
预测器 kwargs 中的路径由插件解释，建议使用绝对路径；模型子进程的工作
目录是目录文件所在目录。`policy.json` 中的相对路径遵循原 policy CLI。
示例中的`/ABS`需要替换，不能直接执行。没有`policy_profile`的条目可以
加载并自动校验，但启动按钮会报告未完成机器人执行接入。

`predictor.json` 沿用 [已有组件接口](model-adapters.md)：

```json
{
  "factory": "openarm_policy.models.act:TacPilotACT",
  "kwargs": {
    "checkpoint": "/ABS/checkpoint",
    "device": "cpu",
    "expected_model_sha256": "71197475f4540873de0df7462c44855555fa4a1b3d1e93e541b5927d93f1e08e"
  }
}
```

`encoded-sample.npz` 是送入 **predictor** 的命名数组，无 pickle。ACT 的键为
`state`、`image`、`wrist`；state形状`1×8`，图像`1×256×256×3`、RGB uint8。
它不是未预处理的摄像头图。文件及解压后内容均有大小限制，不保留预测块。

执行profile继续使用 `ModelPolicy` 的三层组件。其 model 必须是
`openarm_policy.models.http:NumpyHTTP`，`url` 精确指向目录的本地端口，
`expected_identity` 与目录的完整 identity 一致；observation/action 则使用
匹配模型的编码器和解码器。加载/测试后改动相关配置或样例会使前检失效。
已有图像、状态和动作合同哈希检查在 policy reset 中保留，不绕过执行器。

## 原生双路相机

当前相机ACT不能使用缩小过的JPEG预览。可借用同一个视频服务：

```json
{
  "video_http": {
    "url": "http://127.0.0.1:5003",
    "names": ["head", "left"],
    "native": true,
    "max_age_s": 0.5,
    "timeout_s": 0.25,
    "max_bytes": 8000000
  }
}
```

`native: true` 使用既有`/calibration/frame/<name>.png`接口。视频配置须为
每一路设置明确的`calibration_identity`，但这个字段本身不是标定验收。
源/相机身份、模式、PNG哈希、尺寸和保守帧龄逐次校验，代际变化要求重新
建立观察会话。数据不重存到磁盘，不另开摄像头句柄，也不声称双路曝光同步。
头部原生1280×720经过冻结标定去畸变，再按训练合同缩放；腕部仍保留名义
标定限制。旧JPEG模式保持原行为，但不能冒充当前模型的原生输入。

## 接入其他模型

### 只记录触觉，不作为模型输入

在安装机的 policy profile 中配置已有 `tactile` 设备映射，并显式设置
`"tactile_recording": true`。每个设备需有 `identity.serial` 和 `identity.mount`；
物理指垫尚未核对时应明确写 unverified，不能凭串口编号猜左右。
保留模型原有 observation encoder；无需给 ACT 增加触觉特征。

每次执行目录的 `episode/tactile.json` 指向同目录下的 `tactile/<id>/`：
`samples.jsonl` 按传感器原始采样率保存序号、主机接收时间、合力三轴与68测点三轴，
`manifest.json` 保存设备身份、原始 counts 单位和丢帧/连续性检查。
复用已有异步采集写入器，不复制相机数据，也不把高频触觉挤入 Rerun 图像队列。
summary 模式也会保留这份显式请求的旁路数据；未启用时保持原有最小记录策略。

启用后先检查触觉可读，再允许进入机器人启动；运行期间触觉不进入模型或控制数据流。
传感器中断或写入溢出会把触觉记录标为 incomplete，不补零，不声称数据完整，
也不因触觉丢失而新增一条运动控制规则。归位操作不打开触觉设备。
时间使用共同的 host monotonic 时钟，不声称硬件同步；合力与分布来自两次串口事务。
没有绝对力标定时 counts 不能当作 N，正常通信也不代表已验证受压响应。

### 模型接口

发布的 upright foundation 模型使用
`openarm_policy.models.foundation:FoundationObservation` 和 `FoundationActions`，
而不是历史模型的图像/动作配置。编码器显式读取模型图像契约与 HF 头部/腕部内参，
对两路原生图像去畸变并全幅 bilinear 缩放；状态仍为外部 state8。
解码器检查原始动作契约，只截断夹爪，位姿超界拒绝，保持 25/100 的动作块时序。
实机准备位由安装配置给出，不把仿真复用标成实测标定；控制器、限位和停止路径不变。

同一格式的新权重通常只需增加一个目录条目和对应profile，不改网页。
新的模型家族需实现`reset(None)`、`predict(dict)`、`close()`预测器，返回
有限`H×A`数组并发布稳定 metadata；编码器负责输入，解码器负责动作。
单步输出可表示为`1×A`。不同动作语义必须提供对应解码器，不能把绝对目标
当增量指令，或把关节动作当TCP目标。原始 PyTorch/ONNX/HF 权重不是天然
兼容的机器人策略，尚未提供的适配器不能声称“已支持”。

插件是管理员信任的代码，不是安全沙箱；预测器必须不访问相机或机器人，
控制仍只走既有执行层。推理所需依赖装在选定模型环境，不覆盖现场相机/
控制环境。模型下载、可信包安装和机器人几何配置是注册前的准备步骤，
网页不会替用户从任意URL下载和执行代码。

加载默认超时60秒、样例预测默认超时2秒，可在目录条目中用
`load_timeout_s`、`test_timeout_s`分别设置（正数，最高3600秒）。慢模型还需
在执行profile中显式匹配HTTP推理、会话及控制时序，不会为加载成功而自动
放宽实时控制的反馈或安全超时。

若平台只用于查看，`--attach-only`禁止模型加载、测试和执行。
工作站配置可为控制服务设置`"lazy": true`，让平台启动时只打开入口，
由操作员选用相应控制工作区；模型永远不会自动加载或执行。
