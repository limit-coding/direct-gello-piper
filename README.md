# GELLO + Piper X 本机直连控制库

这是一套不经过服务器、Ray、网页、ZMQ 或网络隧道的本机 Python 控制库：

```text
GELLO ──USB 串口──┐
                  ├── Mac 上的单个 Python 进程 ──USB-CAN── Piper X
Piper USB-CAN ────┘
```

联动时采用“当前姿态锚定”：启动瞬间的 GELLO 姿态对应启动瞬间的 Piper
姿态。库不会自动回零，也不会突然移动到某个保存姿态。

## 先确认 USB-CAN 类型

GELLO 通常通过 FTDI/U2D2 暴露为 `/dev/cu.usbserial-*`，macOS 可以直接用。

Piper 本体使用 1 Mbps CAN。当前扩展坞上已识别到 bytewerk candleLight
USB-CAN，它使用 GS-USB 协议，不会生成 `/dev/cu.*`。本项目默认通过
`python-can` 的 `gs_usb` 后端直接访问它。

`gs-usb 0.3.1` 的默认启动流程在 macOS 存在 USB reset/重枚举竞态。本库只在
Darwin 上跳过这次多余 reset，并在关闭时显式释放 libusb 句柄；Linux 行为不变。

另一个可用方案是串口式 SLCAN，它会暴露为 `/dev/cu.*`。如果换用其他
USB-CAN，必须确认 `python-can` 在 macOS 上有对应后端；仅仅能在 USB 树中看到
设备，并不代表能收发 CAN。

当前已确认 GELLO FTDI 串口 `/dev/cu.usbserial-A50285BI` 通信正常，也确认了
candleLight 的 USB 枚举。仍应先运行下面的 `devices` 和只读 `doctor`；不要跳过
检查直接使能。

## 安装

在 Finder 中双击 `install_mac.command`，或在终端运行：

```bash
cd "/Users/limit/3D打印/direct_gello_piper"
./install_mac.command
```

安装脚本会创建独立 `.venv`，并安装工作区里已有的 GELLO、DynamixelSDK 和
Piper SDK 0.6.2，不依赖原服务器工程。

如果 macOS 首次阻止 `.command` 文件，可在终端先执行：

```bash
chmod +x install_mac.command run_teleop.command
```

## 配置与只读检查

安装会从 `config.example.yaml` 创建 `config.yaml`。两个设备都接好后：

```bash
.venv/bin/gello-piper devices
.venv/bin/gello-piper doctor --config config.yaml
```

`doctor` 不使能 Piper、不发送运动目标；连接 GELLO 时会明确写入“扭矩关闭”，
确保主臂保持卸力，不会移动任何电机。

如果其中一个设备被其他程序占用，可分别检查：

```bash
.venv/bin/gello-piper doctor --device gello --config config.yaml
.venv/bin/gello-piper doctor --device piper --config config.yaml
```

如果有多个串口，建议把设备路径或序列号写死：

```yaml
gello:
  port: /dev/cu.usbserial-A50285BI

piper:
  channel: 0
  interface: gs_usb
  bitrate: 1000000
```

`gs_usb` 的 `channel` 是从 0 开始的设备索引；只有一个适配器时可写 `auto`。

## 新 GELLO 首次校准

新主臂不要直接套用旧主臂的零偏和方向。先关闭网页 Web Serial 或其他
占用 GELLO 串口的程序，再运行：

```bash
.venv/bin/gello-piper calibrate-gello --config config.yaml
```

校准器会自动识别连续的七个物理 Dynamixel ID。当前新臂的映射为
`L1→P8, …, L7→P14`，即 `physical = logical + 7`。随后依次采集：

1. 六轴中立零点；
2. 每轴期望的 Piper 正方向，用于自动判定 `joint_signs`；
3. 每轴的安全负端和正端；
4. 夹爪完全打开和闭合位置。

全程会强制七个舅机卸力，不会下发位置。只有最后输入 `SAVE` 才会写入
`config.yaml`，并先生成 `config.yaml.before-gello-calibration` 备份。

如果 ID、方向和夹爪已经调好，只需要重新记录六轴可用行程，运行：

```bash
.venv/bin/gello-piper calibrate-ranges --config config.yaml
```

开始采样后把六个关节分别扫过希望使用的两个安全端点。保存时只更新六轴偏移和
范围，不会改 ID、方向或夹爪参数，并自动启用 `teleop.range_mapping`。该模式保持
启动瞬间不跳变，同时将 GELLO 两端分段缩放到 Piper 配置的两端限位。

SLCAN 配置示例：

```yaml
piper:
  channel: /dev/cu.usbmodem-PIPERCAN
  interface: slcan
  bitrate: 1000000
```

串口设备路径可能随接口变化，`serial_number` 比路径更稳定。不要让 GELLO 和
Piper 配置成同一个串口。

Linux SocketCAN 配置示例：

```yaml
piper:
  channel: can0
  interface: socketcan
  bitrate: 1000000
```

## 启动联动

最方便的方法是双击 `run_teleop.command`。终端方式：

```bash
.venv/bin/gello-piper teleop --config config.yaml --yes
```

程序先只读显示两边姿态，随后直接使能 Piper；不再要求输入 `ENABLE`。启动后
移动 GELLO 即可控制 Piper。按 `Ctrl-C` 会按 `teleop.stop_mode` 停止，默认
发送 Piper 电子急停。

电子急停后，机械臂通常需要按厂商流程恢复/重新使能。实体急停应始终放在手边；
软件急停不能替代实体急停。

## 双 GELLO 控制双 Piper

两组 GELLO 共用同一个 FTDI/Dynamixel 总线时，不能运行两个单路进程。本库的
双路模式由一个进程统一读取 ID 1..14，并将两块 GS-USB 按硬件序列号固定：

- GELLO 物理 ID 1..7 → Piper C；
- GELLO 物理 ID 8..14 → Piper B；
- 任一 GELLO 反馈、CAN 反馈或发送发生故障，两台 Piper 同时电子急停。

先执行只读检查：

```bash
.venv/bin/gello-piper dual-doctor --config dual_config.yaml
```

只有共享总线 ID 1..14 和两台 Piper 反馈全部正常时，才运行：

```bash
.venv/bin/gello-piper dual-teleop --config dual_config.yaml
```

也可以双击 `run_dual_teleop.command`，它会先运行只读检查，通过后才进入双路
联动。双路模式不要求输入 `ENABLE`；当前双路配置为 `stop_mode: hold`，
按 `Ctrl-C` 后会在关闭 CAN 前连续下发当前姿态，两台 Piper 保持使能，
不再因正常退出时的电子急停而失力下坠。异常断线或控制失败仍同时急停两台。

逐台排查时使用两个独立入口（不要同时运行，因为两组 GELLO 共用一个串口）：

```bash
# ID 8..14 → Piper B
./run_piper_b.command

# ID 1..7 → Piper C
./run_piper_c.command
```

独立入口同样先执行只读检查，然后直接使能选中的一台 Piper，不要求输入
`ENABLE`。另一台 Piper 不会连接、使能或收到运动指令。

如果某台 Piper 连接时仍处于拖动示教状态，程序会先恢复受控臂角色，再切换到
CAN/MOVE_J 并使能；此恢复阶段不发送位置目标。

如果需要手动清除控制器保留的示教/输入臂模式，无需断电，可运行：

```bash
./recover_pipers.command
# 然后重新运行
./run_dual_teleop.command
```

`recover_pipers.command` 只发送电子急停、退出示教和运动输出臂配置，不使能、
不发送关节/夹爪位置目标。

要逐步学习或排查 Piper C，可直接运行带注释的最小程序：

```bash
# 默认只读状态
.venv/bin/python examples/learn_piper_c.py

# 只发送运动输出臂配置，不使能、不移动
.venv/bin/python examples/learn_piper_c.py --prepare-output

# 模式正常后，让 J6 相对当前位置小幅移动 2°
.venv/bin/python examples/learn_piper_c.py --jog-j6-deg 2
```

两组 GELLO 共用的 14 舵机总线当前为 57600 baud，实测完整新样本约 14.6 Hz。
双路配置恢复使用原来的 12 Hz 控制节拍。时间调度过载不再导致程序
自动退出；若 GELLO 反馈超过 0.25 秒没有更新，两台 Piper 仍会同时
安全停止。

如果相对 1:1 映射容易在 J2/J3 单侧边界停住，分别记录两组 GELLO 的完整安全
行程：

```bash
# ID 8..14 → Piper B
.venv/bin/gello-piper dual-calibrate-ranges --arm B --config dual_config.yaml

# ID 1..7 → Piper C
.venv/bin/gello-piper dual-calibrate-ranges --arm C --config dual_config.yaml
```

也可以直接双击 `calibrate_piper_c.command`，按屏幕说明自行完成
`ID 1..7 → Piper C` 的六轴行程标定。该入口不会连接 Piper，并且只有最后输入
大写 `SAVE` 才会写入配置；保存时会自动创建原配置备份。

范围校准不连接 Piper，也不修改 ID、方向或夹爪参数。保存后会为对应机械臂启用
`range_mapping`：保持启动姿态不跳变，并把 GELLO 的安全端点映射到 Piper 配置的
完整合法关节范围。Piper 固件硬限位不会被扩大。

`dual_config.yaml` 当前绑定关系为：Piper B 使用 GS-USB 序列号
`0029004A5547571120343930`，Piper C 使用
`003D003D4148571320343133`。程序按完整序列号绑定，不依赖容易随 USB 插拔变化的
裸 index。

## Python API

### GELLO 联动 Piper

```python
import threading

from gello_piper_direct import DirectController, load_config

config = load_config("config.yaml")
stop = threading.Event()

with DirectController(config) as controller:
    # __enter__ 仅连接、读取，不使能。
    controller.arm()  # 明确使能，并以两边当前姿态建立锚点。
    try:
        controller.run(stop)
    finally:
        controller.stop()
```

也可以把 `controller.step()` 放进自己的实时循环；每次调用完成一次读取、映射、
限位、变化率限制、发送和反馈读取。

### 单独控制 Piper

```python
import numpy as np

from gello_piper_direct import PiperDevice, load_config

config = load_config("config.yaml")
piper = PiperDevice(config.piper)
piper.connect()
try:
    current = piper.read_state()
    piper.enable()
    # 先发送当前姿态，避免使能后的跳变。
    piper.command(current.joints_rad, current.gripper)

    target = current.joints_rad.copy()
    target[0] += np.deg2rad(1.0)  # 示例：J1 仅移动 1°
    piper.command(target, current.gripper)
finally:
    piper.emergency_stop()
    piper.close()
```

`PiperDevice.command()` 使用 6 个弧度关节值和一个 0..1 的夹爪开度值：0 是闭合，
1 是完全打开。

### 主动控制 GELLO

GELLO 默认完全被动、舵机卸力。只有显式调用 `set_torque(True)` 后才允许主动
移动，而且单次目标变化默认不得超过 0.15 rad：

```python
from gello_piper_direct import GelloDevice, load_config

config = load_config("config.yaml")
gello = GelloDevice(config.gello)
gello.connect()
try:
    current = gello.read_state()
    gello.set_torque(True)
    gello.move_joints(current.joints_rad, current.gripper)
finally:
    gello.close()  # 自动关闭 GELLO 扭矩并释放串口
```

主动移动 GELLO 时请使用小步插值；不要一次下发远目标。

## 内置安全约束

- 启动相对锚定，禁止自动回零；
- Piper 下发使用严格硬限位；J2/J3 的 ±5° 编码器余量只允许用于反馈读取；
- 每周期关节目标变化率限制，默认 0.025 rad；
- GELLO 单周期异常跳变检测；
- 控制循环看门狗和 Piper 反馈时间戳看门狗；
- 串口选择有歧义时拒绝猜测；
- Piper 使能前需要显式 API 调用，CLI 还需要 `ENABLE` 文本确认；
- 异常、`Ctrl-C` 和控制循环崩溃会进入配置的停止策略。

`stop_mode: emergency` 安全性更强但会进入急停状态；`stop_mode: hold` 只保持当前位置，
电机仍使能，适合已经有外部安全回路的调试环境。

## 测试

不接硬件也能运行单元测试：

```bash
.venv/bin/pytest
```

单元测试覆盖配置校验、USB 选择、相对映射、限速、关节限位、GELLO 跳变和控制
看门狗。真实 USB/CAN 通信必须在设备接好后用只读 `doctor` 验证。
