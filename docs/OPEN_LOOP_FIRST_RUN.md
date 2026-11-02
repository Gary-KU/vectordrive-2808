# 没焊按键：上电开环首跑

使用独立的 **OpenLoopTest** 固件，无需按键、串口或 CAN 指令。

## 烧录哪一个文件

GCC 已生成的文件：`software/build/OpenLoopTest/gimbal_foc.hex`。

Keil 打开 `software/MDK-ARM/GimbalFOC.uvprojx`，在目标选择框选 **OpenLoopTest** 再编译；生成文件位于 `software/MDK-ARM/ObjectsOpenLoopTest/gimbal_foc.hex`。默认的 GimbalFOC 目标仍是普通控制固件，上电不会自动转动。

重新生成 GCC 测试固件：

```powershell
.\build.ps1 -Configuration OpenLoopTest
```

命令在仓库根目录运行。原来的 `.\software\build.ps1 -Configuration OpenLoopTest` 也保持兼容；两种入口输出同一个 `software/build/OpenLoopTest/` 目录。GCC 与 Keil 的源码分组、包含目录和启动/链接文件由 `software/project.psd1` 统一描述，目录迁移不改变首跑行为。

## 实际测试

1. 电机 U/V/W 接板上 U/V/W，让转轴可以自由转动，使用 **14 V 限流电源**。
2. 烧录 OpenLoopTest 的 HEX 后，将**整板完全断电再重新上电**。如果调试器或串口仍通过 3.3 V 给 MCU 供电，仅切换 14 V 不能保证触发一次新的上电测试。
3. 上电先保持驱动关闭并校准 ADC 偏置，等待 **2 秒**。
4. 条件正常时自动进入开环，以斜坡升到 **0.8 V / 20 电角 rad/s**。
5. 开环运行 **2 秒后自动停机**，不再自行重复。需要重测时再次整板断电重上电。

11 极对数对应的目标机械速度约为 17.4 rpm，但开环失步、绕组和负载都可能影响实际转速。0.8 V 是给电机的电压矢量幅值，供电依然使用已确认的 14 V。

普通构建的 `MOTOR_CURRENT_SENSE_CONFIRMED` 仍为 0，因此该开环配置没有安培软件过流保护，首试使用电源限流。用户后续确认的采样参数已填入配置，但最新向量检查发现 U 相无正常电流响应；应先排除该异常，见 [首测记录](CURRENT_LOOP_BRINGUP.md)。开环不依赖 AS5600 反馈，不启用电流环或速度环；ADC 偏置异常或其他已检测故障仍会阻止自动启动。当前 OpenLoopSpin 另有输出禁用设置，内部 OPEN 状态不代表 EN 已开启。

## 停机与未转动排查

- 如果接了串口，115200、8N1，发送 `stop` 立即停机。启动前发送任何有效本地/CAN 控制命令也会取消这次自动测试；查询 `status` 不取消。
- CAN 主控若在启动时发送 STOP 握手，也会取消自动测试。做这个独立首跑时，可以先让 CAN 主控不发指令。
- 2 秒到点条件不满足，就取消该次测试。后续清故障、重新校准或继续等待不会使它突然自动启动。
- 只有供电复位标志有效时才安排测试；单独按 NRST、看门狗或软件复位不会触发，防止恢复复位造成反复起转。下载器的复位通常也不属于整板重新上电，因此烧录后要整板重新上电。
- 若没有转动，可接串口看 `status`：偏置应 `offset_ok=1`、`fault=0`。不要因为没有转动就反复增大测试电压。

普通 Release/Debug/GimbalFOC 固件使用 `MOTOR_OPEN_AUTOTEST=0`；正常使用 CAN 电流控制时请换回普通固件。

## 当前连续开环调试固件

实板排查使用单独的 GCC `OpenLoopSpin` 配置：`.\build.ps1 -Configuration OpenLoopSpin`。输出为 `software/build/OpenLoopSpin/gimbal_foc.hex`。它在复位后等待 2 秒，以斜坡进入 **40 电角 rad/s、2 V** 连续开环，直到 STOP 或故障；与上述只运行 2 秒、仅 POR 启动的 OpenLoopTest 不同。11 极对对应约 34.7 rpm，实际开环速度会受失步和负载影响。

此调试配置保留硬件看门狗和 ADC/控制超时关断，只对未标定采样的偏置噪声门限作专用放宽，仍检查偏置均值范围。U 相采样异常和 AS5600 总线问题尚待检查，安培值、电流环及速度环不可用；目前按用户确认的 14 V、0.6 A 电源限流测试，不得将该调试配置作为闭环生产固件。连续测试前保持转轴自由，串口 `stop` 可停机，软件故障会撤销驱动使能。

当前载波为 24 kHz、同步 ADC 为 4 kHz、控制为 2 kHz。TIM3 内部六分频保持采样相位，避免直接提高 ADC 中断频率而再次漏采样；过程与测量见 [实板调试记录](BOARD_DEBUG.md)。

首跑电压、目标电角速度和上电等待时间在 `software/Config/motor_config.h`；板级 PWM/采样窗口常量在 `software/Config/board_config.h`。首跑状态机由 `software/App/Src/app.c` 实现，入口位于 `software/Core/Src/main.c`。这是 CMSIS 直接寄存器工程，采用常见 STM32 目录布局；更多职责说明见 [工程结构说明](PROJECT_STRUCTURE.md)。
