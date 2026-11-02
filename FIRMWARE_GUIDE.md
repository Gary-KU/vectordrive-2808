# 固件详细指南与历史调试记录

> 当前入口：[项目首页](README.md)、[软件下载](https://github.com/Gary-KU/vectordrive-2808/releases/latest)、[2026-11-02 实板报告](docs/PERFORMANCE_20261102.md)。本文保留历次调试背景，旧测试配置不代表当前发布配置。当前构建的默认 Iq 目标上限为 3.3 A、相电流保护为 2.0 A；这不是持续输出额定值。运行时以参数 3、LIMITS_READ 和发布包构建清单为准。Windows 二进制在 Releases 下载，本仓库不提交 dist 构建目录。

**2026-11-02 实板性能测试：** 2808 / 节点 7 在 60 秒 CAN 测试中完成 **592/592 条请求 ACK**，主机往返延迟中位 **2.76 ms**、P95 **3.65 ms**，快速反馈实测 **195.4 Hz**。±50 mA 短时电流平均反馈为 +46.7 / −46.8 mA，停止续发后约 **105.8 ms** 观察到驱动关闭。详细条件、±100 mA 跟随限制、累计 TX 丢弃标志、原始证据和复现步骤见 [实板性能测试报告](docs/PERFORMANCE_20261102.md)。本次未验证持续大电流、带载转矩或温升。

2808 与 4015 的 Iq 协议请求范围均为 ±3.3 A，当前发布配置默认目标上限为 3.3 A、独立相电流保护为 2.0 A。上位机以参数 3 和 LIMITS_READ 真实读回为准。目标请求范围不保证电流可以达到，更不代表连续输出能力。下文历史记录保留当时的 1 A / 1.5 A 等设置，用于追溯调试过程。

新版上位机已增加机械角度表盘、度数/弧度、rpm及角度/速度曲线，界面采用浅色少边框布局，并显示缺失的启动条件。[启动上位机](start_can_workbench.cmd)；[新版 Windows 包](https://github.com/Gary-KU/vectordrive-2808/releases/latest)。原固件CAN角度反馈已兼容，不需要为界面升级重新烧录。

调参和代码维护从 [调参指南](docs/TUNING_GUIDE.md) 开始。运行参数的默认值、范围、单位集中在 [config/parameters.json](config/parameters.json)；上位机支持方案保存/导入和停机批量写入，固件与 PC 使用同一份生成的参数定义。

新增电机编译选项：`-Motor 2808` 使用 **7 极对**，`-Motor 4015` 使用 **11 极对**，默认 2808。配置在 [motor_profile.h](software/Config/motor_profile.h)，构建产物带型号标记；操作见 [电机选择说明](docs/MOTOR_SELECTION.md)。普通 Release 若检测到匹配的已保存电角度校准，会在 ADC 偏置完成后直接恢复而不吸合；未保存时才在真正断电重上电后自动对齐一次。切换型号仍须重新烧录对应固件。

双击 [start_can_workbench.cmd](start_can_workbench.cmd) 打开中文 CAN 调试台：电流/速度/开环控制、PI 与限幅参数读写、实时曲线、CSV 和命令终端。默认选择 PCAN / PCAN_USBBUS1 / 1 Mbps / 节点 1，启动保持未连接；模拟器需手动选择。实机接线、驱动选择与新固件要求见 [上位机使用说明](docs/CAN_WORKBENCH.md)。

上位机提供 PCAN 自动重连、停机恢复入口、会话 TX/RX 计数与 ACK/参数读回超时分类。失联时先停止运动流，再恢复通信；恢复后保持停机，丢弃旧运动和写参数请求。详见 [连接与故障恢复](docs/CAN_WORKBENCH.md#连接与故障恢复)。

2026-11-01：新版支持 **CAN 节点 ID 1..63** 和 **电角度零点** 分区掉电保存。第一次应在空载、机械安全状态下完成对齐，再在参数页点“保存当前有效零点并重启”；以后上电只重采 ADC 偏置并恢复零点，不再吸合。普通 PI 参数仍只存 RAM。Flash `0x0800F000..0x0800F7FF` 保存校准，`0x0800F800..0x0800FFFF` 保存节点 ID，两者互不覆盖。

MyLegwheel 适配按用户硬件分为：**左右轮 ID 3／7 使用 2808／7 极对，四关节 ID 1／2／5／6 使用 4015／11 极对**，默认六路 A1 电流接口。4015 的 Kt 按用户提供的 0.23 Nm/A；2808 的 Kt 单独配置，不能套用关节值。机器人主控将 Nm 力矩换算为 Iq，使用 200 Hz 快速机械反馈和 100 Hz 电流目标；旧电压标定/反电动势补偿不会再用于电流轮。

已按本目录 `SCH_SimpleFOC103_2026-10-24.pdf` 新建完整的 STM32F103C8T6 工程。支持 **2808 / 7 极对** 与 **4015 / 11 极对**，当前母线配置为 **14 V**。支持先开环试转、dq 电流环，以及以电流环为内环的速度环。**CAN 以 Iq 电流目标为主要控制接口，默认 1 Mbps、节点 1**，协议见 [CAN 设计与接线/报文说明](docs/CAN_PROTOCOL.md)，上位机帧生成工具见 [examples/gimbal_can.py](examples/gimbal_can.py)。普通 Release 始终先保持 EN 低完成 ADC 偏置：有匹配的 Flash 校准时静默恢复；没有时才把吸合电压平滑升至 3 V、总计对齐 4 秒并回到 STOP。它不会自动进入电流环或速度环。

**DRV8313 修复后，U/W 电流反馈和正负 Iq 电流闭环已实板验证。** 当前采用 24 kHz PWM、2 kHz 同步采样/电流环、1 kHz 速度环；正式默认 Iq 上限、矢量限压及相保护以构建清单和板端读回为准。编码器方向实测为 `+1`，已固化为当前相线与 AS5600 安装的默认方向；真正断电上电会自动吸合对齐，完成后仍保持 STOP。速度外环已实现并有植物测试，尚未做实板 SPEED 整定。实测数据见 [电流环记录](docs/CURRENT_LOOP_BRINGUP.md)，构建参数见 [构建验证](software/BUILD_VERIFICATION.md)。CAN 连续电流主机示例见 [run_can_current.py](examples/run_can_current.py)。

## 打开与编译

这是采用 **CMSIS 直接寄存器访问**的工程，目录按 STM32 常见的 `Core / Drivers / MDK-ARM` 习惯整理；没有使用 HAL、CubeMX 生成流程或 RTOS，也没有 `.ioc` 文件。目录职责和依赖见 [工程结构说明](docs/PROJECT_STRUCTURE.md)。

Keil 5 打开 `software/MDK-ARM/GimbalFOC.uvprojx`，使用 ARM Compiler 5、MicroLIB，目标为 STM32F103C8，64 KiB Flash / 20 KiB RAM。程序单独预留 2 KiB 栈，无动态堆。SWD 默认 CMSIS-DAP、1 MHz；换用 ST-Link 时在调试器设置中选择对应探针。

GCC 构建，在本目录运行：

```powershell
.\build.ps1 -Configuration Release -Motor 2808 -Test
# 4015 / 11 极对版本
.\build.ps1 -Configuration Release -Motor 4015 -Test
# 调试版
.\build.ps1 -Configuration Debug
# 旧参数基线捕获版：0.1 A / 1 V，Kp=0.2、Ki=20
.\software\build.ps1 -Configuration CurrentLoopTest
# 短时试转捕获版：0.2 A / 2 V，Kp=1、Ki=1000
.\software\build.ps1 -Configuration CurrentLoopRunTest
# 本地短脉冲捕获版：1 A / 5 V，20 ms 命令期限
.\software\build.ps1 -Configuration CurrentPulseTest
# 若编译器不在 PATH，追加：-Toolchain '实际 GCC bin 目录'
```

Release / Debug 真正上电时优先恢复匹配的 Flash 电角度校准；未保存时才执行一次受保护的吸合对齐并回到 STOP。所有 CurrentLoop* 调试配置仍保持显式命令启动。

| 配置 | Iq 上限 / 矢量限压 / 相过流阈值 | Kp / Ki | Iq 斜坡 | 本地命令超时 | 捕获 |
| --- | --- | --- | --- | --- | --- |
| Release / Debug / Keil GimbalFOC | 1 A / 5 V / 1.5 A | 1 / 1000 | 1 A/s | 2 s | 无 |
| CurrentLoopTest（旧参数基线） | 0.1 A / 1 V / 0.5 A | 0.2 / 20 | 1 A/s | 2 s | 128 条，每 32 拍，16 ms/条 |
| CurrentLoopRunTest | 0.2 A / 2 V / 0.5 A | 1 / 1000 | 1 A/s | 1 s | 256 条，每 8 拍，4 ms/条 |
| CurrentPulseTest | 1 A / 5 V / 1.5 A | 1 / 1000 | 200 A/s | 20 ms | 128 条，逐拍，0.5 ms/条 |

`CurrentPulseTest` 用于本地短脉冲，1 A 目标约 5 ms 达到斜坡终值，20 ms 未续发则自主停机。CAN 期限独立为 100 ms，不能把上述本地期限用于 CAN。正式 1 A 上限表示可接受的目标范围；实际达到的电流及持续能力需结合实测，当前 1 A 验证仍在进行。

结果为 `software/build/Release/gimbal_foc.elf`、`.hex`、`.bin`、`.map`。HEX 可通过 SWD 烧录到芯片；BIN 的起始地址为 `0x08000000`。2026-10-24 已通过 DAPLink/SWD 下载并调试专用开环固件，用户确认转动顺滑、24 kHz 载波下啸叫明显减轻；当前板子已停机，记录见 [实板调试](docs/BOARD_DEBUG.md)。

根目录 `build.ps1` 是主入口；原来的 `.\software\build.ps1` 保持兼容，参数和产物位置一致。`software/project.psd1` 集中声明源码分组、包含目录以及 GCC/ARMCC 启动文件和链接脚本，GCC、Keil 同步脚本及主机测试共用这份布局。

在已登记模块的 `Src` 下新增 C 文件后，GCC 会自动收集；运行 `.\software\MDK-ARM\update_project.ps1` 将分组同步到 Keil。新增模块或包含目录时先更新 `software/project.psd1`。Keil 工程仍保留 GimbalFOC 和 OpenLoopTest 两个目标。

## 先做开环试转

**没有焊按键时，仍可通过串口或 CAN 控制。Release 真正上电会先自动吸合对齐，然后回到 STOP。** OpenLoopTest/OpenLoopSpin 是单独编译的自动开环试验目标；两种自动测试策略见 [无按键开环首跑](docs/OPEN_LOOP_FIRST_RUN.md)。下面为普通固件的手动控制方法。

1. 将电机 U/V/W 接到板上 U/V/W，电源设为 **14 V**，第一次使用限流电源、让转轴可以自由转动。未核实采样参数时，程序没有标定后的软件安培限流；开环测试从低电压开始。
2. 接 SWD：PA13/SWDIO、PA14/SWCLK、GND，探针电压参考为 3.3 V。串口使用 **3.3 V TTL、115200、8N1**：PA9/TX 接串口接收、PA10/RX 接串口发送，并共地。原理图没有专用串口插座，需按实际 PCB 找到这两根网络。
3. 启动后 EN 先保持低，采集 1024 组 ADC 数据进行零电流偏置校准，2 kHz 下约 512 ms。若 Flash 没有匹配校准，编码器有效时随后自动吸合约 4 秒并回到 STOP；若已有匹配校准，则不吸合并直接置 `zero_ok=1`。
4. 发送下面的命令，每行以回车或换行结尾：

```text
status
open 20 0.8
```

`20` 的单位是**电角 rad/s**，`0.8` 是 q 轴电压幅值 V。程序按斜坡升到目标，约 1 秒到达目标速度，约 0.8 秒到达目标电压。2808 的 7 极对对应约 `20 / 7 = 2.86 rad/s = 27.3 rpm`，4015 的 11 极对对应约 `20 / 11 = 1.82 rad/s = 17.4 rpm`；实际开环可能失步，速度不是闭环保证值。

**每条运动指令只保持 2 秒**，超时自动关闭 EN。需要持续运转时，每 0.5～1 秒重复发送同一条 `open` 指令；串口工具不要只发送一次后等待它一直转。`stop` 立即关闭 EN；PB12 按键短按也可以开始默认的 `open 20 0.8` 两秒测试，运行中再按停止。

上面 2 秒是**串口/按键**测试期限。CAN 运动指令采用 **100 ms** 期限，必须持续发送新序号，推荐 100 Hz。CAN 和本地控制互斥，任何一侧的 `stop` 都能停止；串口/按键停止 CAN 运动后，上位机必须重新发送 STOP 并等待执行 ACK，再发送目标。`estop` 会锁存急停故障，清故障并重新校准后才能闭环。

反向测试先 `stop`，再发送 `open -20 0.8`。若只抖动或无法启动，检查三相接线与供电，可逐步试 `open 10 1.0`。正式软件矢量电压上限为 5 V，首次开环仍从 0.8 V 开始；OPEN 不受 Iq 目标上限约束。4015 具体绕组电阻、负载和摩擦未知，0.8 V 是首试起点，不保证适用于所有 4015 绕组。

## 电流采样配置与电流环

电机、采样和 PI 参数集中在 `software/Config/motor_config.h`；板级 PWM/时钟常量在 `software/Config/board_config.h`，CAN 节点/波特率/报文期限在 `software/Config/can_config.h`。

| 配置项 | 当前值/用途 |
| --- | --- |
| `MOTOR_POLE_PAIRS` | 2808 为 7；4015 为 11，由 `-Motor` 选择 |
| `MOTOR_BUS_VOLTAGE_V` | 14 V；原理图没有母线分压，程序不能自动测电源电压 |
| `MOTOR_SHUNT_U_OHM`、`MOTOR_SHUNT_W_OHM` | 0.01 Ω，两路由用户确认为 10 mΩ |
| `MOTOR_SENSE_GAIN_U`、`MOTOR_SENSE_GAIN_W` | 50，用户确认 INA181A2IDBVT |
| `MOTOR_SENSE_POLARITY_U/W` | 默认 -1，必须根据实际电路验证，可分别为 +1 或 -1 |
| `MOTOR_CURRENT_SENSE_CONFIRMED` | 默认 1；本板两路映射和极性已实测，换板需重新核对 |
| `MOTOR_CURRENT_LIMIT_A` | Iq 目标限值 1 A；不是瞬态电流硬限 |
| `MOTOR_VOLTAGE_LIMIT_V` | dq 矢量幅值上限 5 V，d 轴优先分配 |
| `MOTOR_OVERCURRENT_TRIP_A` | 任一相，包括重构 V 相，超过 1.5 A 即停机 |
| `MOTOR_CURRENT_KP/KI` | 1 V/A、1000 V/(A·s)；±100 mA 已实测，1 A 继续验证 |
| `MOTOR_IQ_SLEW_A_PER_S` | 正式默认 1 A/s，从零到 1 A 约需 1 s |
| `MOTOR_SPEED_KP/KI` | 速度 PI 起始值 0.03、0.10，输出为 Iq 安培 |

电流计算：

```text
Iu = polarity_u * (ADC_U - offset_u) * VDDA / (4095 * Rshunt_u * gain_u)
Iw = polarity_w * (ADC_W - offset_w) * VDDA / (4095 * Rshunt_w * gain_w)
Iv = -Iu - Iw
```

软件定义相电流正方向为**逆变器流向电机绕组**。低侧采样电阻的电压方向往往与此相反，因此不能直接假定增益取正就得到正确相电流。U 与 W 两路的通道、斜率和符号需要分别核实。零电流偏置校准只能校正偏置，不能辨识电阻、增益和极性。建议先用已知的小电流检查电压变化和软件安培数，再开启反馈；只把 `CONFIRMED` 改为 1 而保持零参数会导致初始化失败并保持 EN 低。

本板 U/W 通道、相电流映射和极性已经验证。普通 Release 在完整上电后自动使用已验证的 `dir +1` 执行下述对齐；以下手动步骤用于自动过程被 STOP 取消、软件复位、故障恢复或改线后的重新验证。

1. 先做正向开环测试，观察 AS5600 机械角的变化方向。`encoder_ok=1` 表示本次 I²C 读数及磁铁状态有效；`status` 速度是有符号机械速度，单位 mrad/s。
2. 停机后设置编码器方向。如果正向开环时原始角度递增，使用 `dir 1`；如果递减，使用 `dir -1`。不能靠“电机能转”就假定方向正确。
3. 发送 `align`。程序将静态 d 轴电压按 1 V/s 平滑升至 3 V，总计对齐 4 秒，并检查最后 300 ms 的角度稳定性；成功后自动停机并输出 `zero_ok=1`。带负载时仅“保持静止”不能证明转子已经到达正确电角位置。
4. 从小 Iq 开始，例如：

```text
stop
dir 1
align
# 等待 Alignment complete 后再发送；不要把这些注释发到串口
iq 0.05
```

`iq 0.05` 是 **50 mA q 轴电流目标**，不是 0.05 V；`Id` 目标固定为 0。重复发送以刷新 2 秒超时，`iq -0.05` 可测试反向转矩。转矩模式没有速度调节，空载时可能继续加速。需要停机时用 `stop`；`iq 0` 仍是激活的零电流调节模式。

对齐成功后，零点先只在 RAM 中生效，不会自动擦写 Flash。确认空载对齐正确后，在 CAN 上位机参数页点“保存当前有效零点并重启”，或在终端执行 `zero save`。固件使用双页追加日志、CRC 和提交标志保存电机型号、极对数、方向与电角零点；重启后只有配置完全匹配才恢复。拆板、换电机、改相线或改变编码器安装时，点击“拆板/换电机：清除保存值”并二次确认，或执行 `zero clear`。

## 速度环

采样配置和方向/电零点都有效后，停机再切换：

```text
stop
speed 1.0
```

速度单位为**机械 rad/s**，`1 rad/s ≈ 9.55 rpm`。速度 PI 输出限幅后的 Iq 目标，内层仍为 Id/Iq 电流 PI；正式默认目标上限 30 rad/s，Iq 上限为 1 A。速度指令同样需要在 2 秒内刷新。你以电流控制为主时使用 `iq`，无需经过速度环。本轮未做实板速度环整定。

## 命令与状态

| 命令 | 功能 |
| --- | --- |
| `help` / `status` | 帮助/状态、ADC 原始值、偏置、dq 电流、故障码、中断耗时 |
| `open <电角rad/s> [Vq]` | 开环斜坡试转；省略 Vq 时使用 0.8 V |
| `iq <A>` | q 轴电流目标，直接转矩控制 |
| `speed <机械rad/s>` | 串级速度闭环 |
| `stop` | 立即撤销驱动使能，优先于未执行的运动指令 |
| `estop` | 撤销使能并锁存急停故障，电零点失效 |
| `dir 1` / `dir -1` | 停机时确认编码器方向，并废弃原电零点 |
| `align` | 对齐、标定电零点，完成后停机 |
| `cal` | 停机时重新采集零电流偏置，并废弃原电零点 |
| `bus 14` | 停机时修改母线电压换算值，范围 8～30 V；不改变电源实际输出 |
| `clear` | 清除应用故障并重做偏置校准；硬件层故障需要检查原因后复位 |

数字参数支持普通正负十进制和最多六位小数，不接受指数、NaN、Inf。更换运动模式前先 `stop`；在同一模式下可以直接更新目标。参数指令经队列在控制 ISR 内执行，`QUEUED` 表示排队，随后返回执行结果。`SENSE_UNKNOWN`、`DIRECTION_REQUIRED`、`ZERO_REQUIRED` 分别指出尚未满足的闭环条件；`CONTROL_OWNED` 表示另一个输入源正在控制，先停机再接管。串口丢字或行过长时丢弃输入，避免被损坏的目标数值生效。`status` 中 `owner=0/1/2` 分别表示无控制源、本地、CAN。

`status` 的 `sense_ok=0` 时，所有安培遥测都无效，串口会明确提示；此时只能查看 ADC 原始值/偏置。`angle_mrad`、`speed_mrad/s`、`id_mA`、`iq_mA`、`vd_mV` 等采用整数缩放输出，避免浮点 printf 和动态堆。

应用故障位可以相加：

| 位值 | 含义 |
| --- | --- |
| `0x00000001`～`0x00000020` | 核心配置/输入/编码器/过流/命令/数值错误，见 `foc.h` |
| `0x00010000` | BSP 硬件层故障；同时查看 `hw` 字段 |
| `0x00020000` | 零电流偏置超范围或采样噪声过大 |
| `0x00040000` | 运行时 ADC 接近上下电源轨 |
| `0x00080000` | 对齐/闭环期间编码器失效或数据超时 |
| `0x00100000` | 对齐末段转子仍在移动 |
| `0x00200000` | 控制回调超过可用中断时间 |
| `0x00400000` | 电机或电流采样配置非法 |
| `0x00800000` | CAN 运动期间收帧丢失或 bus-off，撤销控制并废弃零点 |
| `0x01000000` | 急停锁存 |

## 硬件和控制时序

| 信号 | STM32 引脚/外设 | 原理图对应 |
| --- | --- | --- |
| U/V/W PWM | PA0/PA1/PA2，TIM2 CH1/2/3 | DRV8313 IN1/IN2/IN3 |
| 驱动使能 | PA4，高有效 | 三路 ENx 共接 DRV_EN |
| U 相电流 | PB0，ADC1_IN8 | ADC_U / ISENSE_1 |
| W 相电流 | PB1，ADC2_IN9 | ADC_W / ISENSE_3 |
| 编码器 | PB6 SCL / PB7 SDA | AS5600，地址 0x36 |
| 按键 | PB12，内部上拉、低有效 | KEY，注意与参考程序 PA3 不同 |
| LED | PC13，高有效 | LED1 经电阻接地 |
| 串口 | PA9 TX / PA10 RX | U1TX/U1RX，115200 8N1 |
| CAN | PA11 RX / PA12 TX | TJA1050；默认 1 Mbps，节点 1，标准 8 字节报文 |
| SWD | PA13/PA14 | SWDIO/SWCLK |

HSE 8 MHz 倍频至 72 MHz；ADC 时钟为 12 MHz。TIM2 使用 **24 kHz 中心对齐 PWM，ARR=1500**。内部 CH4 为 PWM2，在上数 CNT=1425 时产生 OC4REF 上升沿，经 TRGO 给 TIM3 计数；TIM3 每十二个 PWM 周期产生一次 CC4 事件（ARR=CCR4=11），同时触发 ADC1/ADC2 注入转换，得到 **2 kHz 同步采样**。每路采样 13.5 周期、转换 12.5 周期，共约 2.17 μs。

由于是**低侧采样**，只有对应低侧 MOSFET 导通时读数才表示相电流。SVPWM 占空比限定为 0.08～0.82，且注入触发位于共同低侧窗口内；不能随意改成 0～1 或改为任意异步 ADC 轮询。零电压时三相 duty 同为 0.45，线电压为零。

ADC ISR 频率 2 kHz，每个采样都做电流过流/饱和检查并运行一次 **2 kHz FOC**；速度 PI 每两次 FOC 运行一次，即 **1 kHz**。PWM 载波与采样频率分离。软件三角函数用 1 KiB 查表/插值，母线倒数等常量在初始化计算。编码器在前台约 1 kHz 读取，检查磁铁状态和 ACK，带超时及总线恢复；控制时使用有限时间的速度外推，超过 5 ms 不再允许闭环。软件 I²C 不在电流 ISR 内运行。

ADC ISR 每次必须在下个采样之前返回，**2 kHz 下预算为 500 μs = 36000 CPU 周期**。原 4 kHz 在真实电流 PI 运行时出现积压，已据实测改为与电流环同频采样。此前保守 PI 捕获镜像的饱和工况 C 路径峰值约 270 μs，旧 Release 约 258 μs，属于历史参数记录；新参数的实时观测见 [电流环记录](docs/CURRENT_LOOP_BRINGUP.md)。BSP 保留严格超时关断；硬件异常入栈、编译器序言/尾声仍需另计，这不是全部工况的最坏耗时证明。GCC Release 使用 `-O2`，Debug/ARMCC 的实时表现需分别验证。

看门狗需要前台和 ADC 都有进展才能喂狗，名义超时约 101 ms。硬异常、时钟失效和检测到的硬件故障会撤销 EN。原理图的 DRV8313 `nFAULT` 没接 MCU，母线也没有测量通道，因此软件不能读取驱动器故障原因、实际母线电压或进行可靠的母线欠压/过压判定。

## 文件结构与验证

```text
build.ps1                        根目录构建主入口
software/project.psd1            GCC/Keil/测试共用工程布局
software/Config/board_config.h   板级时钟/PWM/duty 窗口
software/Config/motor_config.h   电机/采样/PI/首跑参数
software/Config/can_config.h     CAN 节点/波特率/期限
software/Core/Src/main.c         初始化与前台任务装配
software/Core/Src/system_stm32f1xx.c  CMSIS 系统初始化
software/Core/Startup/           GCC/ARMCC 启动汇编和向量表
software/Core/Linker/            GCC LD / ARMCC SCT 内存布局
software/App/Src/app.c            调试状态机、偏置、保护、模式联锁
software/Communication/Src/console.c       串口命令
software/Communication/Src/can_protocol.c  CAN 会话、执行确认和反馈
software/Control/Inc/foc.h        可移植控制 API
software/Control/Src/foc.c        Clarke/Park、双 PI、速度 PI、SVPWM
software/BSP/Src/board.c          引脚、时钟、PWM、ADC、编码器、串口
software/BSP/Src/board_can.c      CAN 滤波、RX 队列、非阻塞 TX、错误状态
software/MDK-ARM/                 Keil 工程
tests/                           控制仿真与 BSP stub 集成测试
docs/PROJECT_STRUCTURE.md        模块职责、依赖和扩展方式
docs/CAN_PROTOCOL.md             CAN 控制方案与逐字节协议
examples/gimbal_can.py            标准库上位机帧打包与反馈解码
```

本地应用集成可调用 `App_Submit(APP_CMD_CURRENT, iq_a, 0)` 更新电流目标，停止用 `APP_CMD_STOP`。CAN 已使用独立的 `App_SubmitCan` 入口，将真实收帧时间、100 ms 期限、控制源与执行确认关联，不能用本地入口代替，否则会套用 2 秒期限。CAN ISR 只搬运报文，前台验证格式/序号/新鲜度，FOC 仍在 ADC ISR 运行。没有沿用参考项目以“电压”为目标的 CAN 报文，旧报文不能直接控制新固件。

验证命令为 `tests/run_tests.ps1`。包含三角查表精度、全角度 SVPWM 边界与电压重建、开环斜坡、R/L 电流仿真、PMSM 串级速度仿真、PI 抗饱和、NaN/故障锁存、默认/采样已确认两种 App 配置，以及 CAN 长度/单位/序号/新鲜度/重握手/执行 ACK 测试。另有真实协议→App→FOC 的组合测试，验证电流目标抵达电流环、重复帧不续命和旧缓存无法重启。测试夹具使用 0.01 Ω、50 倍；用户后续确认的本板名义参数与之相同，但主机测试仍不能替代实物电流路径、极性和精度验证。

参考目录保持未修改。仅复用其必要 CMSIS 和启动模板，新控制和硬件采样实现独立编写，修正其 C6/C8 器件配置不一致的问题；没有复用原先的 Flash 参数写入或电压目标 CAN 协议。第三方许可见 `software/NOTICE.md`。

外设实现核对资料：[STM32F103C8 官方数据手册](https://www.st.com.cn/resource/en/datasheet/stm32f103c8.pdf)、[ST 官方 ADC 注入触发/双模式定义](https://github.com/STMicroelectronics/stm32f1xx-hal-driver/blob/master/Inc/stm32f1xx_hal_adc_ex.h)、[DRV8313 官方数据手册](https://www.ti.com/lit/ds/symlink/drv8313.pdf)。
