# CAN 初始化自动恢复（2026-10-31）

已修改、构建并烧录当前板。旧代码在启动时只调用一次 `BoardCan_Init()`，失败后前台仍运行，但不会重新初始化 CAN；现场在 PCAN 就绪后复位 MCU 才恢复收发。新固件在停止状态下自动重试失败的初始化。

## 后续 4015 板部署与连续失败恢复验证

同日换入的 4015/11 极对板仍运行早期 32972 B 固件，CAN 初始化失败后不会重试。本次执行 `./build.ps1 -Configuration Release -Motor 4015 -Test`，将工程中已有的初始化恢复机制构建为 4015 固件并烧录该板，OpenOCD 校验通过。没有启动电机。

- 固件：`software/build/Release/gimbal_foc_4015.hex`，Flash 33832 B，上电保持停机。
- HEX SHA256：`5A023FBBAE69BFF181FFC1CB3E0D4E0C126702FB98A471A80536C40AE089668C`。
- C 回归全部通过，包含三组 CAN 栈和初始化重试用例；Python 111 项中 100 项通过、11 项因构建 Python 无 Tkinter 跳过。

实板故障注入先确认 STOP、EN 低、无排队命令，再复现 `init_failed()` 的外设/软件状态。临时改变 RAM 中的 `SystemCoreClock` 声明，使 `BoardCan_Init()` 的时钟校验连续拒绝两次重试；实际时钟、目标电流、PWM 输出和 Flash 均未因注入而改变。恢复正确声明后，由前台轮询自动初始化，无需再次复位。

| 阶段 | 初始化尝试/成功次数 | initialized | 模式/驱动 |
| --- | --- | --- | --- |
| 注入前 | 1/1 | true | STOP/关闭 |
| 注入后 100 ms | 1/1 | false | STOP/关闭 |
| 第一次重试失败 | 2/1 | false | STOP/关闭 |
| 第二次重试失败 | 3/1 | false | STOP/关闭 |
| 条件恢复后重试成功 | 4/2 | true | STOP/关闭 |

恢复后 `require_stop=true`；应用/板级故障、ADC pending 和 elapsed overrun 均为 0。测试恢复了原始 RAM 时钟声明，调试器已释放，MCU 继续运行。此次验证的是实际前台重试与恢复流程，不是模拟电源电压变化的模拟电路测试。

日志：`build/board-debug/can-init-4015-20261031-build.log`、`can-init-4015-20261031-flash.log`、`can-init-4015-20261031-recovery.log`。可重复执行的停机故障注入脚本为同目录 `can-init-4015-20261031-recovery.ps1`。

当时初始化恢复已验证，但外部 CAN 链路尚未恢复：烧录后 PCAN 保持 1 Mbps ACTIVE 监听 3 秒，接收仍为 0；同期板端 initialized=true、LEC=3（ACK error）、TEC=128。不能将初始化成功解释为总线已连通，也不能据此认定收发器损坏。记录为 `can-init-4015-20261031-post-flash-pcan.json` 和 `can-init-4015-20261031-post-flash-pins.log`。

### 后续 ACK 与上位机通信复测

用户要求继续检查后，再次连接 PCAN，3 秒收到状态/诊断/遥测共 901 帧；同期硬件 ESR=0、TEC=0、REC=0，板端发送成功计数增加。本轮没有修改 CAN 位时序、滤波器或再次烧录，不能从这些记录唯一确定先前物理链路失效的原因。

随后使用真实 `desktop.controller.Controller` 完成 STOP 执行 ACK、全部 9 项参数读取以及 20 秒连续监测，收到 6035 帧。会话持续有效，编码器角度有效，无 bus-off、error-passive、应用/板级故障和反馈过期，驱动始终关闭。发送仅包含 STOP 和参数查询，未写运行参数或启动电机。记录为 `can-ack-4015-20261031-desktop.json`、`can-ack-4015-20261031-soak.json`；原生 UI 在内的 111 项 Python 回归全部通过，日志为 `can-ack-4015-20261031-ui-tests.log`。

上位机默认接口改为 PCAN，仍保持启动未连接，避免打开模拟器后误认为正在监测真实板子。下文保留较早板子的验证记录，其固件 SHA 和磁铁状态属于当时现场。

### 另一块 4015 板：PCAN 设备重启后恢复接收

同日后续烧录的另一块 4015 板 UID 为 `066DFF49-57547566-87053851`，固件仍为上述 `5A023F...` 版本。刚烧录时 PA11 RX 为低、初始化失败；再次检查时 RX 已高且 initialized=true，说明停机重试已恢复初始化。随后出现板端 ACK 错误、PCAN 接收 0 帧。

用户确认总线上仅接该板。PCAN 打开期间，SWD 观察到板端 TXOK 完成计数增加、TEC/REC 回落为 0，但 Python 和原生 PCANBasic `Read` 均收不到帧（持续 ReceiveQueueEmpty）。原生参数检查为 receive_status=ON、message_filter=OPEN、listen_only=OFF；无 Python 过滤条件。显式重设接收开关、关闭/打开硬件过滤器及清队列后仍无帧。发送仅有 STOP，板端接收计数增加，支持发送通路尚可用。

PCANBasic 为 4.6.2.753、PCAN-USB 驱动 4.4.0.16619、适配器固件 8.4.0。调用 Windows `pnputil /restart-device` 精确重启当前 PCAN-USB 设备，命令返回 0 / Device restarted successfully；未重新烧录、未改变 CAN 位时序或接线。随后同一桌面 Controller 的 STOP 握手和全部 9 项参数读取立即通过。该差异把故障范围缩小到 PCAN/USB 接收路径状态，不能仅凭一次恢复认定某个驱动版本或器件是唯一原因。

恢复后连续监测 20 秒收到 **6040 帧**，状态/诊断数据年龄最大约 16 ms、角度页约 32 ms，编码器有效；无 bus-off、error-passive、应用/板级故障或反馈超期。驱动始终关闭，未清应用故障、写参数或发送运动目标。早先累计的 tx_loss 提示可能保留，不能据此把已恢复的总线判为当前仍失败。

证据前缀为 `build/board-debug/can-low-rx-4015-20261031-`：`baseline.log`、`active-pcan-swd.log`、`native-pcan.json`、`pcan-reset.json`、`adapter-reset-swd.log`、`desktop-after-restart.json`、`recovered-soak.json`。本次只重启所选 PCAN-USB 设备，没有重启 DAPLink 或整个 USB 集线器。

### 新换 4015 板：烧录通过，通信仍待硬件复查

16:18 烧录板 UID `0668FF37-33355234-57231452`，4015/11 极对 Release 写入并校验通过，固件 SHA256 为 `5A023FBBAE69BFF181FFC1CB3E0D4E0C126702FB98A471A80536C40AE089668C`。当时编码器、ADC 偏置正常，驱动关闭，CAN 初始化失败且 PA11 RX 为低。烧录记录为 `build/board-debug/4015-reflash-20261031-161829.json`。

用户随后确认 14 V 主电源开启。16:35 使用增强后的桌面 Controller 再次执行 PCAN 精确设备重启和最多三次 STOP 握手：设备重启成功，但接收为 0、未收到节点 1 的 0x381 执行 ACK。全程只发送 STOP，未提交运动或参数写入；工作线程正常关闭。结果见 `build/board-debug/can-robustness-20261031-hardware.json`。同时电脑未检测到 DAPLink，OpenOCD 报 `unable to find a matching CMSIS-DAP device`，故无法重新读取此时 RX 电平；不能把先前 RX 低直接当作当前测量。主电源开启也不能代替收发器芯片处实际 5 V、电源地及 CAN 接线的检查。

### 16:41–16:43 追测：同一块板恢复通信并通过一分钟监测

用户重新接回 DAPLink 后，SWD 确认仍为上述 `0668FF37` 板、11 极对、驱动关闭。CAN 已初始化且 RX 为高；初始接收错误计数 REC=255、error-passive=true。PCAN 打开期间板端 TXOK 持续增加，但电脑 15 秒接收为 0，板端 RX 计数也未随新 STOP 增加。PCAN GetStatus 在这段时间持续返回 0，说明“适配器未报告错误”不能代替节点反馈确认。

仅精确重启当前 PCAN-USB 设备，随后 STOP 握手和 9 项参数读取立即通过；没有烧录、修改 CAN 配置、接线或运行参数。恢复后的 SWD 记录显示板端接收计数增加，REC 下降并退出 error-passive。此对照把本次故障范围缩小到 PCAN/USB 通信路径状态；尚不能区分适配器、驱动或 USB 链路的具体原因，也不能据此排除所有物理信号问题。

随后真实桌面 Controller 停机监测 60 秒，收到 **18,044 帧**，状态/诊断最大数据年龄约 16 ms、角度页约 32 ms；无反馈超时、bus-off、error-passive、应用/板级故障或会话中断。编码器有效，机械角度约 76.43–76.60°。全程仅 STOP 和参数查询（0x00/0x10），未发运动或写参数，工作线程正常关闭。该结果验证停机通信，未验证运动干扰下的通信。

本轮证据为 `build/board-debug/can-stability-20261031-` 前缀的 `capture.json`、`active-swd.log`、`recover.json`、`after-recovery-swd.log` 和 `soak.json`。固件保持原版本；再次遇到适配器打开但 RX=0 时，应先停机并使用上位机“恢复 PCAN”，而不能将打开适配器直接判作连接成功。

## 行为与维护入口

- `software/Communication/Src/can_service.c` 负责外设初始化与协议生命周期；`main.c` 通过 `CanService_Init/Poll` 装配。
- 仅当 `BoardCanStatus.initialized=false` 时重试；间隔由 `software/Config/can_config.h` 的 `CAN_INIT_RETRY_PERIOD_MS=500` 决定，从上一次初始化尝试结束起计时。
- `App_CanReinitializeCan()` 要求 STOP 或 FAULT、驱动关闭、无待执行命令、无待处理 CAN 故障、无运动控制源；专用开环自动测试未执行完也会阻止重试。
- 原 BSP 初始化等待有超时，等待期间 ADC 中断保持开启。正在运行或已经排入运动请求时，不执行初始化重试。
- 成功恢复后重置协议会话，丢弃旧接收队列及待回复状态，必须重新发送 STOP 并收到执行成功 ACK。恢复通信不会清除应用故障、对齐、使能驱动或启动电机。
- 已初始化外设的 bus-off 继续采用原 ABOM 自动恢复、应用故障和 STOP 握手规则；此补丁不会周期性重置健康或 bus-off 的外设。

GCC 工程清单、测试源码清单及 Keil 两个目标均登记了 `can_service.c`。Keil XML 已解析验证：每个目标恰有一个有效源码路径；本轮未重新执行 ARMCC 编译。

## 离线验证

执行 `./build.ps1 -Configuration Release -Test`，退出码 0，GCC 的 `-Wall -Wextra -Werror` 检查通过。真实 CanService → CanProtocol → App → FOC 集成测试覆盖连续初始化失败、500 ms 节流、旧运动命令丢弃、重新 STOP 握手、待执行/正在执行本地运动、故障保留、计时回绕、健康/bus-off 外设不重置，以及专用镜像的待执行开环自动测试。三组 CAN 栈配置均通过。

完整 C 回归通过；Python 100 项中 94 项通过、6 项因当前构建 Python 无 Tkinter 跳过。桌面界面代码未修改。日志：`build/board-debug/can-init-retry-20261031-build.log`。

Release Flash 33732 B，应用 RAM 2928 B，另有独立 2048 B 栈。HEX SHA256：

```text
7AA47364A781926E9162E34CFD2D45BD0846333AA456C52DE258B99EAF5B0317
```

固件为 `software/build/Release/gimbal_foc.hex`，OpenOCD 已烧录并校验 33732 字节，随后复位运行。日志：`build/board-debug/can-init-retry-20261031-flash.log`。修改前源码和固件保留在 `build/board-debug/pre-can-init-retry-20261031/`。

## 实板停机恢复验证

PCAN_USBBUS1、1 Mbps、节点 1。先执行 STOP/sequence=71 并确认 ACK=OK、`require_stop=false`。随后通过 SWD 将 CAN 置入初始化状态、关闭 CAN 中断并将软件初始化标志设为 false，模拟需要重新初始化的状态；不改 Flash，不提交运动命令。

| 阶段 | 初始化尝试/成功次数 | 已初始化 | require_stop | 模式/驱动 |
| --- | --- | --- | --- | --- |
| 注入前，millis=75416 | 1 / 1 | true | false | STOP / 关闭 |
| 约 109 ms 后 | 1 / 1 | false | — | STOP / 关闭 |
| 再运行约 713 ms 后 | 2 / 2 | true | true | STOP / 关闭 |

重试在第二个观察窗口内完成，符合 500 ms 门限；这两个快照不构成精确延时测量。最终应用故障、板级故障、ADC pending/overrun 均为 0，PA4 EN 为低，`RECOVERY_TEST_FAILED=0`。此测试验证停止状态的软件恢复路径，没有模拟真实收发器掉电或测量上电波形。

约 14 秒接收窗口共收到 3893 帧：STOP ACK 1、状态 1298、诊断 1297、遥测 1297。恢复后反馈继续，诊断无 bus-off/error-passive、故障为 0，且要求新的 STOP。日志及原始帧：

- `build/board-debug/can-init-retry-20261031-fault.log`
- `build/board-debug/can-init-retry-20261031-capture.json`

随后直接使用 `desktop.controller.Controller` 经真实 PCAN 连接：新的 STOP 握手成功，9 项参数全部读回，断开前确认停机，工作线程正常关闭。TX 仅包含 STOP (`0x00`) 和参数查询 (`0x10`)，无参数写入或运动指令。参数实际值为 Kp=1、Ki=1000、Iq限幅=1 A、矢量限压=5 V、Iq斜坡=1 A/s、速度Kp≈0.03/Ki≈0.1、速度限幅=30 rad/s、母线配置=14 V。

桌面实测日志：`build/board-debug/can-init-retry-20261031-desktop.log`，完整会话为同名 `.json`。临时 OpenOCD 和 PCAN 检查进程已退出，MCU 继续运行，驱动关闭。

## 当前编码器状态

本次烧录及后续检查中 AS5600 状态为 `BOARD_ENCODER_NO_MAGNET`，`sensor_valid=false`；这与此前 I²C BUS_ERROR 不同。CAN 及停机参数查询不依赖有效转子角度，已验证正常；角度显示和闭环运动仍需要恢复磁铁检测。此轮保留此前的软件弱上拉和约 50 kHz 标称 I²C 配置，未进行电机试转。
