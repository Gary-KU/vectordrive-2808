# 调参与维护入口

日常调参从 CAN 上位机开始；修改掉电后的默认参数从 `config/parameters.json` 开始。电流环的执行频率、保护及协议保持原有行为，本次整理没有重新整定电机，也没有烧录或运行板子。

电机型号通过编译选项选择：`build.ps1 -Motor 2808` 对应 7 极对，`-Motor 4015` 对应 11 极对，默认 2808。切换后烧录带型号的固件并重新校准，详见 [电机选择](MOTOR_SELECTION.md)。这与下文 9 项运行时调参分开维护。

## 日常调参：不重新编译

1. 打开根目录 `start_can_workbench.cmd`，选择模拟器或对应 USB-CAN 接口，连接并完成 STOP 握手。
2. 停止驱动后点击“读取参数”。以“实际读回”判断板子的当前配置；如需从现有值开始调整，点击“读回填入编辑器”。输入框只是待写的编辑值。
3. 在参数页修改需要调整的项。每项都显示分组、单位、范围和说明；修改标记根据固件 float32 精度比较，不会把 `0.03` 与其二进制读回的尾数误判为改动。
4. 点击单项应用，或批量写入修改项。每项都要分别收到执行 ACK 和实际值读回才会确认成功。运行中禁止写入。
5. 显式开始电流测试，查看 Iq、Iq 参考、Id、Vd/Vq 与错误状态，结束后 STOP。曲线是 CAN 监视数据，不是 2 kHz 同步原始波形。
6. 将编辑值保存为 JSON 方案。下次导入方案只更新编辑器，不写 CAN、不运行电机；核对后再点击写入。

**运行时参数只保存在 MCU RAM 中，复位或掉电恢复编译默认值。** 方案文件保存在电脑中；导出方案、写入运行参数、修改固件默认值是三个明确的操作。

批量写入是逐项执行：所有值先在电脑预检查，再逐个发送、确认、读回。中途失败或 STOP 会终止后续项并报告“已确认 x/n”，已经生效的项保留，不会虚报整个方案成功，也不会自动回滚或重新启动。写入前请读取板上参数，记录需要保留的原值。

保存/导入文件时请先停机。文件对话框可能暂停界面心跳，已有的 500 ms 界面超时保护仍会自动请求停止。

## 修改固件默认值

唯一可维护的运行参数表是 [config/parameters.json](../config/parameters.json)。常改的字段：

| 字段 | 作用 |
|---|---|
| `default` | 复位后的默认值，同时用于上位机初始编辑值和模拟器默认值 |
| `min` / `max` | 参数写入范围，固件和 PC 共用；扩大范围前需要验证硬件与保护关系 |
| `step` | 上位机输入控件的增减步长 |
| `label` / `unit` / `description` | 参数名称、单位和操作说明 |

`id`、`symbol`、`name`、`field`、`default_macro` 属于稳定的协议/代码映射，日常调参不应修改。`invalidates_zero` 表示参数是否使电气零点失效；目前只有母线换算值如此。固定相过流阈值不在运行参数表中。

修改后在根目录执行：

```powershell
.\configure.ps1
.\build.ps1 -Configuration Release -Test
.\desktop\build_windows.ps1
```

根目录 `build.ps1` 会先同步参数；使用 Keil 或直接进入 `software/build.ps1` 前，修改过 JSON 后应先执行 `configure.ps1`。Keil 编译使用仓库内已经生成的 C 文件，不需要在编译机安装 Python。运行 `configure.ps1 -Check` 只检查生成物是否一致，不写文件。

生成物必须随源码一起保留，不手动编辑：

- `software/Config/motor_tuning_defaults.h`：九项编译默认值，保留专用构建的 `-D` 覆盖能力。
- `software/Config/parameter_catalog.def`：固件参数 ID、FocConfig 字段、范围和零点规则。
- `examples/parameter_catalog.py`：PC 协议、上位机与模拟器共同使用的参数定义。

## 代码职责

| 入口 | 修改内容 |
|---|---|
| `config/parameters.json` | 常用调参默认值、范围、单位、说明 |
| `software/Config/motor_setup.h` | 电机极对数、启动/对齐、控制时序与固定保护 |
| `software/Config/current_sense_config.h` | U/W 分流电阻、增益、极性、ADC 比例与偏置采集 |
| `software/Config/bringup_config.h` | 专用调试镜像的编译开关 |
| `software/Config/motor_config.h` | 兼容入口，组合以上配置 |
| `software/Control/Src/foc.c` | 坐标变换、电流/速度 PI、斜坡与 PWM 算法 |
| `software/Control/Src/foc_parameters.c` | 描述表、配置副本读写与参数范围验证 |
| `software/App/Src/app.c` | 校准、控制源、模式切换、参数应用及故障联锁 |
| `software/Communication/Src/can_protocol.c` | CAN 编码、序号、会话、ACK 与回复优先队列 |
| `desktop/commands.py` | 文本调试指令解析和类型检查，没有 CAN 副作用 |
| `desktop/controller.py` | 单线程 CAN 所有者、运动刷新、ACK/读回关联与停止 |
| `desktop/profiles.py` | 参数方案 JSON 的验证、加载和原子保存，不访问 CAN |
| `desktop/ui.py` | 界面、曲线和编辑值，不直接操作 CAN |
| `desktop/transports.py` | 真实适配器包装与明确标识的模拟节点 |

参数模块只读写调用方的 `FocConfig` 副本；只有 App 能在 STOP/EN 关闭/无故障条件下重建控制器、清积分并处理电气零点。通信前台和 UI 都不能直接修改 ISR 正在使用的控制状态。快速控制路径继续使用已有的 `FocConfig` 数值，不需要解析 JSON 或动态分配内存。

## 验证与交付

```powershell
.\configure.ps1 -Check
.\tests\run_tests.ps1
.\.venv-can\Scripts\python.exe desktop/launch.py --self-test --report build/can-workbench-selftest.json
```

测试覆盖生成物一致性、方案往返/非法文件、逐项与批量参数应用、命令解析、CAN 会话/ACK/超时、真实 App→FOC 的离线集成以及控制算法回归。模拟节点只用于界面和通讯验证。

接线、首次方向/对齐和实际适配器选择见 [CAN 上位机使用说明](CAN_WORKBENCH.md)，帧格式见 [CAN 协议](CAN_PROTOCOL.md)。
