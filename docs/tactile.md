# Paxini IP-M2324-Core 触觉可视化

适用 PX6AX-GEN3 IP-M2324-Core（PXSR-STDIP03A，68 测点）及单路串口
USB 转接板。使用外部社区项目 [paxini-sdk](https://github.com/Jingyi-Z/paxini-sdk)，
不是厂家官方 SDK。本次适配基于 SDK commit
`79c82145f88e158dd7706736e8c07f14f11cbd7a`。
不支持 SPI 十路盒或高速板的协议。

## 环境

在 Python 3.12 虚拟环境内安装；SDK 保持在仓库外，不复制其代码或坐标：

```sh
python -m pip install -e '.[tactile,viewer]'
```

发布版可直接安装 `openarm-runtime[tactile,viewer]` 的 Git 标签；`tactile` extra
固定社区 SDK 的已验证提交。开发时使用 `uv sync --locked --extra tactile --extra viewer`。
核心导入不依赖 SDK、Viser 或实际串口。SDK 与坐标不随本项目分发。

## 启动

```sh
# 本次已验证的传感器通信地址是 2；使用实际设备的稳定路径
.venv/bin/python -m openarm_runtime tactile \
  --device /dev/serial/by-id/YOUR_USB_CONVERTER --device-id 2 --port 8891

# 独立模拟页（不会打开硬件）
.venv/bin/python -m openarm_runtime tactile --simulate --port 8892
```

`--device-id` 是传感器模块编号加 1，不是 Linux 串口编号；默认值 1 保持 SDK
惯例，但本次实机须显式指定 2。`--baudrate` 默认 921600，8N1、无流控。
Linux 串口使用独占锁，避免本集成的多个采集者冲突；其他不使用该锁的工具仍需
先退出。启动不清零、不改传感器参数。

服务只绑定 127.0.0.1；在客户端 SSH 转发后打开同端口：

```sh
ssh -N -L 8891:127.0.0.1:8891 -L 8892:127.0.0.1:8892 YOUR_WORKSTATION
```

打开 http://127.0.0.1:8891 看实机，http://127.0.0.1:8892 看模拟。

## 界面与数据

界面参考 [mjviser](https://github.com/mujocolab/mjviser) 的 Viser 工作区，
使用同类控制面板与三维场景，不依赖 MuJoCo 动力学仿真：

- **控制**：暂停显示、重连、立体/俯视/侧视、重置视角。
- **显示**：Fz 色标上限、XYZ 向量长度及显示开关、座体示意。
- **读数**：选中测点 XYZ、合力 XYZ、峰值 Fz、最近 300 个显示样本曲线。

68 点坐标来自 SDK，单位 mm，编号从 0 开始。座体只是方位示意，不是厂家 CAD。
模拟接触中心跟随实际测点坐标中心；页面明确标注 SIMULATION。

力保留原始 counts：SDK README 写 0.1 N/count，但串口方法文档要求厂家标定系数，
本集成未声明绝对牛顿值。合力与分布来自两次读取，非同步帧。
后台采集上限 60 Hz，画面约 30 Hz；暂停只冻结显示，状态仍更新。

读取失败会关闭串口，保留页面、显示错误并灰化测点。最后读数明确标记为失效，
不会回退为模拟。点击“重新连接”重新打开设备。超过 0.5 秒未更新标记过期。
Ctrl+C/SIGTERM 关闭采集和服务器。暂未提供清零、录像或自动重连。

## 实机排查与验证（2026-10-31）

最初超时的根因已确定为 **传感器设备地址不匹配**：SDK 默认 ID 1，实机响应 ID 2。

1. USB `1a86:55d3` 正常绑定 `cdc_acm`，独占打开成功，无其他进程占用。
2. 在 ID 1 下，合力与 68 点分布读取重复请求均收到 0 字节；11 种主机波特率
   也未返回数据，故不是坐标或 GUI 解析造成的超时。
3. 在默认 921600 下只读探测 ID 1–28，仅 ID 2 返回完整 18 字节合力响应。
4. 显式 ID 2 后，直接 SDK 连读 300 帧，全部 68 点、零异常；新增适配层
   连续运行 15 秒收到 896 帧，约 59.7 Hz、零异常，退出正常释放串口。

已验证静态读数的合力约 `(1, 0, 4)` counts、分布 Fz 范围 0–5 counts。
这些数据验证通信、帧长及解析；受压响应、绝对力精度和安装方向仍需实物对照验证。
可在实机页轻按、松开，核实局部热度和合力曲线随触碰变化。

本板读应答状态为 `0x01 / ERR_LEN`。社区 SDK 明确兼容此读帧固件行为，
本集成沿用 SDK 的状态及 LRC 校验，并额外严格匹配应答 ID、功能码、地址、
数据长度和完整帧长度。不能据此认为任意错误帧都可接受；其他状态仍报错。

如将来再超时，首先核对 `--device-id` 与模块配置，再查实际设备路径、排线、供电。
`ttyACM*` 不表示缺少驱动；[WCH 官方说明](https://github.com/WCHSoftGroup/ch343ser_linux)
确认标准 CDC-ACM 支持普通 UART。

离线回归：151 项测试，149 通过、2 跳过；干净环境安装 wheel 后，未安装 SDK/Viser
仍可导入触觉模块并查看 CLI 帮助。WebKit 实际浏览器验证通过：三维场景、三个
标签页、合力曲线、暂停/恢复和重置视角，未记录 JavaScript 错误。


## 双传感器使用

两个 USB 转接板各使用独立采集线程/服务和稳定的 `/dev/serial/by-id/` 路径，
避免重新插拔后 ttyACM 编号交换。分别启动同一条实机命令，指定各自的
`--device` 路径及不同的 `--port`（例如 8891、8894），两块本次设备均使用
`--device-id 2`。模拟页 8892 不代表硬件状态。

页面标题和面板显示设备身份，可并排打开两个窗口。默认 Fz 色标上限调整为
10 counts，便于观察轻触；较大压力时在“显示”中提高上限。
暂停时若设备断开，读数仍优先标记“已失效”。

2026-10-31 双设备只读并行检查：两路各连续读取 20 秒，分别获得
5945 / 5961 个完整 68 点样本，无读取异常，约 297 / 298 Hz。
此速率是直接轮询测试速率；可视化采集仍限制为 60 Hz、显示约 30 Hz。
通信通过不等于绝对力精度已标定，仍需结合按压响应判断感应状态。

随后一分钟双路采集各收到 3587 帧，约 59.7 Hz、零错误；用户按压时两路
分布 Fz 峰值分别达到 56、69 counts，松开后恢复原静态读数，基本感应响应通过。
本次离线回归 151 项，149 通过、2 跳过。

## 同页左右显示（单终端）

已配置的开发工作站运行（替换仓库路径）：

```sh
cd /path/to/openarm-runtime
python3 tactile.py
```

脚本自动使用项目 `.venv`，无需激活环境。左右设备及通信参数保存在仓库外的
`~/.config/openarm-runtime/tactile.json`；换 USB 口无需修改稳定的 by-id 路径。
本机系统命令是 `python3`，已激活虚拟环境时也可用 `python tactile.py`。
启动器复用下面的统一 CLI；Ctrl+C 同时关闭两路服务。

完整 CLI（需要手动指定设备时）：

```sh
.venv/bin/python -m openarm_runtime tactile \
  --device /dev/serial/by-id/YOUR_LEFT_CONVERTER \
  --right-device /dev/serial/by-id/YOUR_RIGHT_CONVERTER \
  --device-id 2 --port 8891
```

打开 http://127.0.0.1:8891，同页左右并排显示原有三维界面。USB 换口不会改变设备序列号。
界面沿用当前浏览器已确认的左右顺序，不再提供交换按钮。两路均属于左臂，右臂未安装触觉。两路独立暂停、重连和读数。
一个 Ctrl+C 退出整个服务。内部视图使用主端口 +10 / +11（8901、8902），
这三个端口必须空闲。远程浏览时需同时 SSH 转发这三个端口。


## 开发启动器配置

将本机配置保存在 `~/.config/openarm-runtime/tactile.json`，不要提交到 Git。
例如（将占位路径替换为本机两个不同的稳定串口路径）：

```json
{
  "left_device": "/dev/serial/by-id/YOUR_LEFT_CONVERTER",
  "right_device": "/dev/serial/by-id/YOUR_RIGHT_CONVERTER",
  "device_id": 2,
  "baudrate": 921600,
  "port": 8891
}
```

界面中的左侧、右侧均指左臂上的传感器位置；右臂未安装触觉。
Mac 浏览器通过 SSH 使用双路页面时，在 Mac 终端保持以下连接：

```sh
ssh -N -L 8891:127.0.0.1:8891 -L 8901:127.0.0.1:8901 -L 8902:127.0.0.1:8902 USER@WORKSTATION
```
## 四路网页与位置保存

`openarm-runtime tactile --profile /path/to/tactile.json --port 8891`
复用单传感器视图显示配置中的多路传感器。外部 JSON 使用
`{"tactile": {"pad_serial": {"device": "/dev/serial/by-id/...", "device_id": 2,
"baudrate": 921600, "identity": {"serial": "...", "mount": "physical_pad_assignment_unverified"}}}}`。

网页为每个序列号提供未配置、左手左爪/右爪、右手左爪/右爪选项。
左右爪统一以从该手的手腕朝指尖看为准，不随操作者站位改变。
配置键保持兼容：`left_pad_1`/`left_pad_2` 分别为左手左爪/右爪，
`right_pad_1`/`right_pad_2` 分别为右手左爪/右爪。
保存时校验位置不能重复，并原子写回这份机器配置，不依赖浏览器存储。
策略 profile 使用 `"tactile_profile": "/path/to/tactile.json"` 替代内联 `tactile`，
每次新执行读取最新映射；已保存的历史记录不会重标注。配合 `tactile_recording: true`
将位置元数据随原始数据一起保存，不改变模型输入。

网页预览独占传感器串口。试跑前点击“暂停预览并释放串口”，或停止触觉服务；
关闭浏览器标签本身不等于停止采集。网页只读传感器，不启用电机。
平台代理需将四个视图端口（主端口+10至+13）列在 `extra_ports`。
