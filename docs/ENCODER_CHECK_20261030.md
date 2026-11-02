# AS5600 停机检查（2026-10-30）

通过当前连接的 DAPLink/CMSIS-DAP，以 1 MHz SWD 读取现场固件和运行内存。Flash 前 32972 字节与 `software/build/Release/gimbal_foc.bin` 完全一致，因此使用对应 ELF 的地址解释状态。没有烧录、复位、暂停 CPU 或下发运动指令。

约 4531 ms 内取得 40 次运行状态快照，结果如下：

| 项目 | 实测结果 |
| --- | --- |
| `BoardEncoderStatus` | 40/40 为 `BOARD_ENCODER_MAGNET_WEAK`（枚举值 4） |
| `status.sensor_valid` / `encoder.valid` | 40/40 为 false |
| 编码器内部有效数据时间戳 | 0，当前没有被固件接受的有效角度 |
| 电机模式 / 驱动状态 | 40/40 为 STOP / disabled，PA4 EN 输出低 |
| MCU 时间 | 持续推进，采集窗口约 4.5 秒 |

与已校验的现场驱动代码对应：该状态只在 AS5600 STATUS 寄存器读取成功、MD=1、ML=1 后设置，表示检测到了磁铁，但芯片报告磁场偏弱。它与此前 SCL 拉低导致的 BUS_ERROR 不同；现有证据不足以判断编码器芯片损坏。固件拒绝该状态下的角度反馈，内部的 0 rad 不能解释成实际转子位于零度。

建议先核实磁铁中心与编码器中心的相对位置，适当缩小间隙，再复测是否恢复 `BOARD_ENCODER_OK` 和 `sensor_valid=true`。恢复有效状态后，才能检查转动时的连续性、方向及静止抖动。本次没有测量 AGC、MAGNITUDE 或有效角度噪声。

状态位含义依据 [AS5600 数据手册，STATUS / AGC](https://look.ams-osram.com/m/7059eac7531a86fd/original/AS5600-DS000365.pdf)。磁铁位置或间隙是依据弱磁标志提出的排查方向，并非已定位的具体机械故障。

原始记录：`build/board-debug/encoder-20261030-connect.log`、`encoder-20261030-passive.log`、`encoder-20261030-summary.json`。OpenOCD 已正常关闭，电机保持停机。
