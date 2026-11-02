# 构建验证

## CAN 初始化恢复构建（2026-10-31，当前）

Release 已构建、烧录并校验。新增停机状态下的 500 ms 初始化失败重试，恢复后要求重新 STOP，保留既有 bus-off 和故障规则。GCC `Release -Test` 退出码 0，全部 C 测试通过；Python 94 项通过、6 项因构建 Python 缺少 Tkinter 跳过。Flash 33732 B，应用 RAM 2928 B，独立栈 2048 B。Keil 两目标已登记源码并校验 XML 路径，本轮未执行 ARMCC 构建。

当前 Release HEX SHA256：`7AA47364A781926E9162E34CFD2D45BD0846333AA456C52DE258B99EAF5B0317`。实板模拟未初始化状态后成功自动恢复，STOP/驱动关闭保持；真实 PCAN 经桌面 Controller 完成新 STOP 握手和 9 项参数查询，全程无运动指令。当前编码器为 NO_MAGNET，不代表电机闭环条件就绪。实现、日志与边界见 [CAN 自动恢复记录](../docs/CAN_INIT_RECOVERY_20261031.md)。

以下为 2026-10-25 的历史验证。正式控制配置为 **24 kHz PWM / 2 kHz ADC与电流环 / 1 kHz速度环**，本板采样确认默认开启，Iq目标限制±1 A、dq矢量限压5 V、相过流1.5 A，Kp=1/Ki=1000、Iq斜坡1 A/s，采用d轴优先电压分配。1 A尚无带载稳态结论；按用户要求已停止实板试验，交付CAN上位机。下方构建表、哈希和实测不代表当前源码的镜像容量或验证范围。

## 参数与配置整理构建（2026-10-25，历史）

本次只整理代码、运行离线测试和编译，**没有连接或操作实板，没有下载固件或试转电机**。9个运行参数的字段、wire ID和范围从生成的 `Config/parameter_catalog.def` 统一展开为 `Control/foc_parameters` 的只读描述表；默认值来自生成的 `motor_tuning_defaults.h`。`App` 保留控制中断执行、STOP/EN/故障门禁、固定过流、清积分及真实电气零点规则，模块只读写调用者的配置副本。原 `APP_PARAM_*` 名称、1..9的线协议及所有既有profile保留。

`motor_config.h` 继续作为统一入口，分别包含 `motor_setup.h`（电机、时序及固定保护）、`current_sense_config.h`（采样标定）、`bringup_config.h`（调试编译开关）和生成默认值。`#ifndef` 保留命令行profile覆盖；默认/profile单独的参数模块测试验证 Ki=1500 等覆盖有效。GCC/Keil共用的工程清单已登记新模块，Keil工程已同步源码和catalog文件。

| GCC配置 | 构建结果 | Flash | 应用RAM，另有独立2048 B栈 |
| --- | --- | ---: | ---: |
| Release | 退出码0，含最终BUS范围共用调整 | 32972 B | 2912 B |
| Debug | 退出码0 | 26300 B | 2912 B |
| OpenLoopTest | 退出码0 | 33460 B | 2920 B |
| OpenLoopSpin | 退出码0 | 33388 B | 2920 B |
| CurrentLoopTest | 退出码0 | 33244 B | 9576 B |
| CurrentLoopRunTest | 退出码0 | 33252 B | 16232 B |
| CurrentPulseTest | 退出码0 | 33288 B | 9576 B |
| CurrentLoop1500 | 退出码0 | 33248 B | 16232 B |

GCC Release `size` 为 text32864 / data108 / bss4848；BSS已含栈，不能重复加栈。以上8个目标均通过结构整理阶段的构建和零heap守卫，并放入64 KiB Flash/20 KiB SRAM。最终将UART BUS范围验证也接入同一catalog后，重编Release与Keil GimbalFOC；其他profile保留该微调前的结构整理构建记录，没有重复编译。

| Keil / ARMCC 5.06 update 7 build 960 | 全部重编结果 | Flash / ROM | RAM，已含2048 B栈 |
| --- | --- | ---: | ---: |
| GimbalFOC | 退出码0，0 errors / 0 warnings，含最终BUS调整 | Code18984+RO2100+RW236 = 21320 B | RW236+ZI4576 = 4812 B |
| OpenLoopTest | 结构整理阶段退出码0，0 errors / 0 warnings | Code19168+RO2316+RW236 = 21720 B | RW236+ZI4576 = 4812 B |

Keil继续使用 `Start-Process -WindowStyle Hidden -Wait -PassThru`，参数为 `-r <工程绝对路径> -t <目标> -j0 -o <日志绝对路径>`，没有使用下载参数。GimbalFOC callgraph主循环最大可见深度928 B、ADC IRQ260 B；沿此前保守中断嵌套估算约1420 B。callgraph仍带Unknown标记，不代表实板栈高水位，也不是GCC精确上界。

| 当前产物 | SHA256 |
| --- | --- |
| `software/build/Release/gimbal_foc.hex` | `9388493B76BCF5813C5B03CBF2CFFFE45472B003D2204462141B8EADB108D9A5` |
| `software/build/Release/gimbal_foc.elf` | `5526611D498508CA0E95D7BD8B54CADC5C2184745FD285ACAE7B06E2D2DFF0B3` |
| `software/build/CurrentLoop1500/gimbal_foc.hex` | `9A27957FEB400C9AED061E27E715596E195E2563C15A630BF1CE9579314C3951` |
| `software/MDK-ARM/Objects/gimbal_foc.hex` | `89B3B23191D5B4F6843D854A0BE9CA7B0E9C62BEFEC18DCDDB75B5EB4B99794E` |

完整 `tests/run_tests.ps1` 退出码0：FOC、参数模块default/profile、App、四类非有限采样参数拒绝、CAN协议/三组真实栈、自动测试及全部 **85项Python测试通过**。原有参数边界、运动拒写、故障只读、查询不续命、对齐门禁和有限邮箱优先回包回归继续通过。最终UART BUS也改用同一catalog后，补充上下边界和零点失效断言，分别重跑采样未确认/已确认两组App/console C回归，均通过。日志为 `build/host-tests/config-refactor-suite.log`、`config-refactor-release.log`、`config-refactor-1500.log`、`config-refactor-keil.log`、`config-refactor-keil-openloop.log`，其余GCC profile同目录以 `config-refactor-` 开头。

## 此前 CAN 参数与桌面上位机配套构建（历史）

此轮只编译和离线测试，没有连接调试器、下载固件或驱动电机。镜像新增 A1 参数读写（0x10/0x11）、参数读回和三页调参遥测，保留原运动协议；桌面调参必须使用支持这些扩展的固件。运行时参数仅存 RAM，固定相过流阈值不开放修改。硬件实测结论仍以[电流环记录](../docs/CURRENT_LOOP_BRINGUP.md)为准。

| 构建 | 结果 | Flash / ROM | 应用 RAM 与栈 |
| --- | --- | ---: | --- |
| GCC Release，O2 | 退出码 0，零 heap 守卫通过；未下载 | 32840 B | 2912 B + 独立 2048 B 栈 |
| ARMCC 5.06 update 7 build 960，Keil GimbalFOC | 全部重编，退出码 0，0 errors / 0 warnings；未下载 | Code18972 + RO1956 + RW236 = 21164 B | RW236 + ZI4576 = 4812 B，已含 2048 B 栈 |

GCC `size` 输出 text32732 / data108 / bss4848；其中 BSS 已含栈，不可再重复加栈。链接器区域统计为应用 RAM2912 B、STACK2048 B。Keil 继续使用 ARMCC5、Cortex-M3、MicroLIB，未切换至 ARMClang。

| 产物 | SHA256 |
| --- | --- |
| `software/build/Release/gimbal_foc.hex` | `FBCBEF160F829173BD7227DF5DBE4FC77AAB3DDC0CD4D38BDF5C8B75A724A6AF` |
| `software/build/Release/gimbal_foc.elf` | `96CC27D16AC8FB399C83094BC6BD04C0621E12F31621B64E4FA3A5DDAB10D814` |
| `software/MDK-ARM/Objects/gimbal_foc.hex` | `5704C777E198869AF03D4F8C5525B21664855B8964DF2F230DA4C43E67A7D7B4` |

Keil 命令使用 `D:\app\Keil_v5\UV4\UV4.exe -r D:\CODE\Gimbal_borad\software\MDK-ARM\GimbalFOC.uvprojx -t GimbalFOC -j0 -o D:\CODE\Gimbal_borad\build\host-tests\can-parameters-keil-build.log`，由 `Start-Process -WindowStyle Hidden -Wait -PassThru` 执行。没有使用下载参数。

`tests/run_tests.ps1` 完整通过：FOC、App、异常采样参数、CAN 协议、真实 CAN→App→FOC 栈和开环自动测试均成功；该次完整日志包含58项 Python。随后追加关闭窗口时等待 ESTOP 等桌面回归，最终全部 Python **60项通过**，包含参数浮点编码、9参数边界、运行拒写/故障只读、查询不续命、对齐进度见证、适配器模拟和桌面控制流程。三发送邮箱测试验证：遥测占满邮箱后 ACK/参数回应在下轮可用时重试；同轮回复与遥测到期时先发送 ACK/参数回应，允许丢低优先遥测。

日志位于 `build/host-tests/can-parameters-suite.log`、`can-parameters-python-final.log`、`can-parameters-release-build.log`、`can-parameters-keil-build.log`。Keil 新 callgraph 主循环最大可见深度928 B，ADC IRQ268 B、CAN RX56 B、CAN SCE40 B、USART/SysTick各16 B；沿旧保守中断嵌套估算 `928+16+56+16+268+128+16=1428 B`，低于2048 B，但 callgraph 仍含 `Unknown(Cycles, Untraceable Function Pointers)`，不代表实板栈高水位或 GCC 精确上界。

## 当前构建配置与验证边界

| 配置 | Iq上限 / 矢量限压 / 相过流阈值 | Kp / Ki | Iq斜坡 | 本地命令超时 | AppTrace |
| --- | --- | --- | --- | --- | --- |
| Release / Debug / Keil GimbalFOC | 1 A / 5 V / 1.5 A | 1 / 1000 | 1 A/s | 2 s | 关闭 |
| CurrentLoopTest（旧参数基线） | 0.1 A / 1 V / 0.5 A | 0.2 / 20 | 1 A/s | 2 s | 128条，每32拍，16 ms/条 |
| CurrentLoopRunTest | 0.2 A / 2 V / 0.5 A | 1 / 1000 | 1 A/s | 1 s | 256条，每8拍，4 ms/条 |
| CurrentPulseTest | 1 A / 5 V / 1.5 A | 1 / 1000 | 200 A/s | 20 ms | 128条，逐拍，0.5 ms/条 |

上述配置均无自动启动，必须显式设置方向、真实对齐并下发目标；保留ADC积压/超时、编码器、ADC电源轨及相过流保护。CurrentPulseTest用于本地短脉冲，从零到1 A斜坡约5 ms，停止由固件超时执行，避免在带电PWM期间暂停CPU。CAN运动期限仍独立为100 ms，不受本地20 ms定义影响。

```powershell
.\build.ps1 -Configuration Release
.\build.ps1 -Configuration CurrentLoopTest
.\build.ps1 -Configuration CurrentLoopRunTest
.\build.ps1 -Configuration CurrentPulseTest
```

产物分别在`software/build/<配置>/gimbal_foc.elf/.hex/.bin/.map`。CurrentLoopTest保留旧PI参数作为对照；CurrentLoopRunTest使用新PI和短时捕获；CurrentPulseTest用于1 A验证。当前源码的1 A目标上限不等于已经证明持续1 A输出。

实板连续切换+100/−100 mA已通过：Iq均值+100.855/−100.100 mA，目标误差RMS为2.71/3.46 mA，Id约1 mA。已完成的空载短脉冲尚不能用于报告1 A带载稳态通过；最新板上镜像、日志、故障与停机状态以[电流环实测记录](../docs/CURRENT_LOOP_BRINGUP.md)为准。

FOC主机测试已按`MOTOR_CURRENT_LIMIT_A=1.0f`、`MOTOR_VOLTAGE_LIMIT_V=5.0f`、`MOTOR_OVERCURRENT_TRIP_A=1.5f`、Kp=1/Ki=1000运行通过。2 kHz、R=0.6 Ω/L=2 mH植物在饱和释放后Iq为1.000005 A、峰值1.014441 A；动态PMSM饱和测试平均|Id|约1 µA，释放和直接反向恢复通过。新增同一CURRENT会话的双向10/20/50/100/200/500/1000 mA覆盖。理想植物不包含实板ADC噪声或未知绕组参数，不能替代实际1 A验证。

## 此前0.1 A / 1 V电流环交付（历史）

| 构建 | 结果 | Flash | 应用RAM与独立栈 |
| --- | --- | ---: | --- |
| GCC Release，O2 | 构建成功；实板下载、Flash匹配、对齐、正负电流切换、超时停机通过 | 30260 B | 2664 B + 2048 B栈 |
| GCC CurrentLoopTest，O2 | 构建成功；带AppTrace实板验证50/100 mA及电压饱和 | 30572 B | 9328 B + 2048 B栈 |
| GCC Debug，Og | 构建成功；未实板验证其实时性 | 24260 B | 2664 B + 2048 B栈 |
| GCC OpenLoopTest，O2 | 构建成功；本轮未下载此自动开环目标 | 30708 B | 2672 B + 2048 B栈 |
| GCC OpenLoopSpin，O2 | 构建成功；本轮未下载此自动开环目标 | 30720 B | 2672 B + 2048 B栈 |
| ARMCC 5.06u7，GimbalFOC | 重编译0 errors/0 warnings；未实板验证 | Code17656+RO1884+RW228=19768 B | RW228+ZI4340=4568 B，含栈 |

当时Release HEX SHA256：`F7C91A934204CE1BE46A097ED060A3095DF1CF5A3C8E2BFC5781D5A5856F3C2C`。当时下载此GCC Release，最终STOP、EN低、调试服务关闭。该版实板Release的ADC C路径峰值18602 cycles≈258.4 µs，捕获镜像限压工况19438 cycles≈270.0 µs，均小于500 µs周期，积压/超时/同步错误均0；旧4 kHz真实PI出现pending保护，因此没有保留250 µs截止时间。详见[完整电流环实测](../docs/CURRENT_LOOP_BRINGUP.md)。

新增主机回归覆盖真实2 kHz配置的正负电流、Id抑制、PWM驱动R/L植物及饱和恢复；App测试按采样/控制频率比推进，精确验证超时边界。CAN持续主机例程另有fake bus测试，实际CAN线缆收发和SPEED实板整定不在本次已验证范围。

最终`tests/run_tests.ps1`全套退出码0：Core、App（显式未确认/确认采样）、四种非有限采样参数拒绝、CAN协议、三组CAN→App→FOC栈、开环自动测试均通过；Python共31项通过（帧编码/解码12项、持续电流主机19项）。CAN边界验证±100 mA合法/±101 mA拒绝、1 V合法/1001 mV拒绝；未确认采样拒绝覆盖继续保留，不依赖已启用的生产默认值。

## 早期4 kHz版本构建记录（历史）

链接目标为 STM32F103C8：64 KiB Flash、20 KiB SRAM，独立保留 2 KiB 栈，heap 为 0。GCC 使用 Cortex-M3 Thumb、软件浮点；所有配置都能放入该容量。

## 早期实际构建（历史）

```powershell
# 从工程根目录执行；四个 GCC 配置分别生成独立产物。
.\build.ps1 -Configuration Release
.\software\build.ps1 -Configuration Debug
.\build.ps1 -Configuration OpenLoopTest
.\build.ps1 -Configuration OpenLoopSpin

foreach ($targetName in @('GimbalFOC', 'OpenLoopTest')) {
    $buildLog = if ($targetName -eq 'GimbalFOC') {
        'D:\CODE\Gimbal_borad\software\MDK-ARM\build.log'
    } else {
        'D:\CODE\Gimbal_borad\software\MDK-ARM\build_openloop.log'
    }
    $process = Start-Process -FilePath 'D:\app\Keil_v5\UV4\UV4.exe' -ArgumentList @(
        '-r', 'D:\CODE\Gimbal_borad\software\MDK-ARM\GimbalFOC.uvprojx',
        '-t', $targetName, '-j0', '-o', $buildLog
    ) -WindowStyle Hidden -Wait -PassThru
    if ($process.ExitCode -ne 0) { throw "Keil build failed: $targetName" }
}
```

| 工具链与配置 | 最终结果 | Flash / ROM | 静态 SRAM，含 2 KiB 栈与区域对齐 |
|---|---|---:|---:|
| GNU Arm Embedded 10.3.1，Release，`-O2` | 退出码 0，`-Wall -Wextra -Werror` | 30,300 B | 4,712 B |
| GNU Arm Embedded 10.3.1，Debug，`-Og` | 退出码 0，`-Wall -Wextra -Werror` | 23,860 B | 4,712 B |
| GNU Arm Embedded 10.3.1，OpenLoopTest，`-O2` | 退出码 0，`-Wall -Wextra -Werror` | 30,756 B | 4,720 B |
| GNU Arm Embedded 10.3.1，OpenLoopSpin，`-O2` | 退出码 0，`-Wall -Wextra -Werror` | 30,748 B | 4,720 B |
| ARMCC 5.06 update 7 build 960，GimbalFOC，MicroLIB，优化等级 2 | 全部重编，退出码 0，0 errors / 0 warnings | 19,596 B | 4,568 B |
| ARMCC 5.06 update 7 build 960，OpenLoopTest，MicroLIB，优化等级 2 | 全部重编，退出码 0，0 errors / 0 warnings | 19,996 B | 4,576 B |

GCC Release/Debug 的 `size` 为 data 108 B、BSS 4,600 B；OpenLoopTest/Spin 为 data 108 B、BSS 4,608 B。BSS 已含栈，不能再加 2 KiB。表中以链接器区域统计为准，分别为应用 RAM 2,664 / 2,672 B 加 STACK 2,048 B，另包含区域末尾对齐。Keil 普通固件为 Code 17,484 + RO 1,884 + RW 228，ZI 4,340；OpenLoopTest 为 Code 17,660 + RO 2,100 + RW 236，ZI 4,340。

本轮首次 Keil 构建发现 `foc.c` 的两个不可达 fallback 警告；控制核心已改成互斥预处理分支，随后以上两个目标重新全部编译，最终日志均无警告。

GCC 输出保持 `software/build/<配置>/gimbal_foc.elf/.hex/.bin/.map`，中间 `.o/.d` 在该配置的 `obj/`。Keil 普通目标在 `software/MDK-ARM/Objects/`、`Listings/`，OpenLoopTest 在 `ObjectsOpenLoopTest/`、`ListingsOpenLoopTest/`；两个目标独立。Keil 的普通 GimbalFOC 仍是默认目标。

## 早期4 kHz采样与启动方式（历史）

72 MHz TIM2 中心对齐 PWM：ARR=1500、CCR4=1425，OC4REF 作为 TRGO；TIM3 通过 ITR1、外部时钟模式 1 对其上升沿六分频。当前 TIM3 为 ARR=5、CCR4=5、SMCR=0x17、CCMR2=0x7800（CH4 PWM2 与 preload）、CCER=0x1000。ADC1 injected 选择 TIM3_CC4，ADC2 保持软件触发源，两者开启外部触发；实板记录 ADC1 CR2=0xC001。TIM3_CH4 仅用于内部事件，PB1 保持模拟输入。

此前 Frozen 比较模式虽然能置位 CC4IF，实板仍未产生 ADC 样本；切换到当前 PWM2 上升沿方案后才恢复采样。因此本报告不把 Frozen 内部比较事件当作已验证的 ADC 触发方案。TIM3 从 CNT=0 开始，首个匹配在第 5 个 PWM 上升沿，后续每 6 个周期一次。先使 ADC 与 TIM3 就绪，再启动 TIM2。

Release/Debug 默认不自动起转。OpenLoopTest 为 POR 门禁下等待 2 s、以 0.8 V / 20 电角 rad/s 开环运行 2 s 后停止的单次测试。OpenLoopSpin 是独立持续试转固件：允许任意复位后安排一次启动，使用 2.0 V / 40 电角 rad/s，直到 STOP 或故障；当前仅为未确认电流采样放宽偏置噪声门限，电源/偏置范围及故障门禁仍需满足。具体操作见 [首次开环](../docs/OPEN_LOOP_FIRST_RUN.md)。

## 早期开环实板结果与验证边界（历史）

本轮通过 SWD 实际下载并验证了 GCC OpenLoopSpin。其他配置的结果为构建/链接验证，不表示已经在电机上运行。详细过程见 [实板调试记录](../docs/BOARD_DEBUG.md)。

| 当前 24 kHz PWM / 4 kHz ADC 场景 | 采样与中断记录 | 故障记录 |
|---|---|---|
| 未使能功率输出的被动采样 | 2,007 ms 内 8,028 个样本，约 4,000 次/s；ADC ISR C 路径最大 12,949 cycles，约 179.8 μs | pending=0，elapsed overrun=0 |
| 带电 OPEN，目标 40 电角 rad/s、2.0 V | 5,015 ms 内 20,061 个样本，约 4,000 次/s；App 最大 13,077 cycles（181.625 μs），ADC ISR C 路径最大 13,385 cycles（185.903 μs） | pending=0，overrun=0，Board fault=0，App fault=0 |

带电记录使用 14 V 台式电源、0.6 A 电源限流。250 μs ADC 周期下，上述 C 路径最大值留出约 64.1 μs；硬件异常入退栈、编译器序言/尾声及部分诊断存储需另计。该数据只描述本次 OPEN 工况，不能当作所有模式的最坏执行时间。用户已确认 24 kHz 版本“啸叫明显减轻，转动顺滑”。最终 STOP 状态为 mode=STOP、owner=NONE、EN 低、Board/App fault=0，看门狗与采样保持运行，Flash 校验匹配，调试服务已关闭。

CURRENT 直接进入 d/q 电流 PI；SPEED 用速度 PI 生成 Iq 后进入同一电流环。CURRENT/SPEED 仅完成主机逻辑验证，尚未完成闭环电机实测。分流阻值与放大增益仍未确认，`MOTOR_CURRENT_SENSE_CONFIRMED=0`，这两个模式仍受门禁限制；当前 AS5600 读取报告 BUS_ERROR，也尚未证明编码器链路可用。开环运行结果不能用于证明电流校准、相序、电角零点或 PI 参数已正确。

## 早期链接、工程清单与栈（历史）

- 对当前四个 GCC ELF 的已定义符号检查，均无 `malloc/calloc/realloc/free`、newlib 分配器版本、`_sbrk/_sbrk_r`、`strtof/strtod/strtold` 或 printf 系列依赖；构建脚本的零 heap 守卫全部通过。两个 Keil map 同样未发现这些库依赖。
- GCC 栈符号为 `__StackLimit=0x20004800`、`__StackTop=0x20005000`。启动文件与链接器仍负责 `.data/.bss`、SystemInit、C 库初始化及 main；无 Flash 参数存储区。
- `software/project.psd1` 集中声明 include、源分组、测试角色、startup 和 linker；GCC、Keil 文件组更新器及主机测试脚本共用该清单。当前仍为 8 个 C 文件，Core 的 system C 只加入一次。模块位置为 Core、Config、App、Communication、BSP、Control、Drivers；CAN 参数与协议分别在 `Config/can_config.h`、`Communication/Inc/can_protocol.h`、`Communication/Src/can_protocol.c`。
- Keil 工程保留 STM32F103C8、CMSIS-DAP/SWD 配置及 64 KiB 下载范围。两目标本轮只重编，未执行下载。

两个 Keil 最终 callgraph 的主循环最大可见深度均为 772 B，ADC IRQ 为 260 B，CAN RX 为 56 B、CAN SCE 为 40 B，USART1 与 SysTick 均为 16 B。CAN 两个 IRQ 同优先级，不互相嵌套；按 SysTick → CAN → USART → ADC 加四层硬件异常帧，保守相加为 `772 + 16 + 56 + 16 + 260 + 128 = 1,248 B`，再为四层异常对齐最多计入 16 B，仍低于 2,048 B。

callgraph 继续标记 `Unknown(Cycles, Untraceable Function Pointers)`；以上静态计算不是实板栈高水位证明，也不是 GCC 的精确栈使用上界。报告来自两个 Keil `Objects*/gimbal_foc.htm`，修改局部缓冲区或中断后需要重新评估。

## 早期主机逻辑测试（历史）

最终源码已实际运行 `tests/run_tests.ps1`，退出码为 0：FOC、App 默认/采样参数确认配置、四种 NaN/Inf 配置拒绝、CAN 协议、三种 CAN→App→FOC 集成、自动首跑开/关及 12 项 Python 测试均通过。新增回归验证 4 kHz ADC 时间基准、2 kHz 控制固定 dt 与 250 μs 截止时间。也可通过 `build.ps1 -Test` 在构建后运行。主机模拟 BSP 与协议集成测试不作为 ADC 电气波形、实板闭环或声音改善的证据。
