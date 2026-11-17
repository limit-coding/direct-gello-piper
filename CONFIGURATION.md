# GELLO → Piper 配置与运行索引

本页依据仓库现有配置与代码整理。标定值属于既有实机，不代表新硬件已完成标定。
硬件结构、BOM 与打印修正见 [gello-yam](https://github.com/limit-coding/gello-yam)。

## 配置文件

| 文件 | 用途 | 关键区别 |
|---|---|---|
| `config.example.yaml` | 单臂安装模板，首次安装复制为 `config.yaml` | 30 Hz，相对映射，正常退出电子急停 |
| `dual_config.yaml` | 共享 Dynamixel 总线的 B/C 双臂及逐臂入口 | 12 Hz，正常退出保持姿态 |

单臂模板与双臂配置中的零偏和夹爪行程不同；启动时以对应配置为准。
本次整理未重新测量硬件或改写标定数值。

## 双臂绑定与标定状态

| 项目 | Piper C | Piper B |
|---|---|---|
| GELLO 关节 J1…J6 对应物理 ID | `[1, 6, 4, 5, 3, 2]` | `[8, 9, 10, 11, 12, 13]` |
| GELLO 夹爪物理 ID | 7 | 14 |
| GELLO 总线 | 两臂共用一个 USB 串口 | 两臂共用一个 USB 串口 |
| 波特率 | 57600 | 57600 |
| GS-USB 选择 | 通过配置中的硬件序列号绑定 | 通过配置中的硬件序列号绑定 |
| Piper 夹爪最大开度 | 0.10 m | 0.08 m |
| 速度配置 | 20% | 20% |
| 行程映射 | `range_mapping: true`，已保存六轴范围与零偏 | `range_mapping: false`，六轴零偏为 0、范围为 null |
| 正常退出策略 | `hold`，电机继续使能 | `hold`，电机继续使能 |

B 的配置通过启动相对锚定运行；零偏为 0 不等于已经记录了绝对零位。
若要为 B 启用全行程映射，先运行行程标定并检查保存结果。
两组 GELLO 共享串口，不能同时运行两个独立进程。

## 运行入口

| 目的 | 命令或脚本 | 是否使能 Piper |
|---|---|---|
| 枚举设备 | `.venv/bin/gello-piper devices` | 否 |
| 双臂只读检查 | `.venv/bin/gello-piper dual-doctor --config dual_config.yaml` | 否 |
| 双臂遥操 | `./run_dual_teleop.command` | 检查通过后使能两台，无 ENABLE 确认 |
| 仅运行 B | `./run_piper_b.command` | 检查通过后使能 B，无 ENABLE 确认 |
| 仅运行 C | `./run_piper_c.command` | 检查通过后使能 C，无 ENABLE 确认 |
| 单臂模板遥操 | `.venv/bin/gello-piper teleop --config config.yaml` | 输入 ENABLE 后使能；加 `--yes` 跳过 |
| 标定 C 行程 | `./calibrate_piper_c.command` | 不连接 Piper，输入 SAVE 后保存 |

B 行程标定：

```bash
.venv/bin/gello-piper dual-calibrate-ranges --arm B --config dual_config.yaml
```

新主臂完整标定：

```bash
.venv/bin/gello-piper calibrate-gello --config config.yaml
```

## 安装依赖

Python 3.10+。`pyproject.toml` 中声明了普通 Python 依赖，另需以下硬件源码：

- GELLO 软件：[wuphilipp/gello_software](https://github.com/wuphilipp/gello_software)，提供 `gello.robots.dynamixel.DynamixelRobot`。
- DynamixelSDK：通常位于上述源码的 `third_party/DynamixelSDK/python`。
- 兼容本库调用的 Piper SDK 源码，提供 `C_PiperInterface_V2`、`CreateCanBus`、`ConnectPort(piper_init=False)` 等接口。

历史安装脚本使用工作区内的 Piper SDK；该适配源码没有随本仓库发布，
不能仅凭版本号认定官方同版本包支持这里的 GS-USB 初始化接口。
安装脚本保留原目录默认值，并支持用以下环境变量覆盖：

```bash
GELLO_SOFTWARE_DIR=/absolute/path/to/gello_software \
DYNAMIXEL_SDK_DIR=/absolute/path/to/DynamixelSDK/python \
PIPER_SDK_DIR=/absolute/path/to/compatible/piper_sdk \
./install_mac.command
```

依赖目录缺失时会在创建虚拟环境前报错。安装完成后先运行设备枚举与 doctor。
当前更新仅做离线验证，没有重新验证实机 USB/CAN 通信。
