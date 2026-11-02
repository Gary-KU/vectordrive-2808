# U相ADC软件与实板配置核查

日期：2026-10-24，Asia/Shanghai。用户要求检查U相ADC采样代码和引脚配置，排查是否为软件问题。

后续状态：用户修复DRV8313后，U相正反向反馈已恢复，两路电流极性和真实电流闭环均已验证；ADC与电流环调整为2 kHz。以下软件/PCB排查保留为历史证据，当前结果见[电流环实测](CURRENT_LOOP_BRINGUP.md)。

## 结论与范围

源码、当前ELF汇编和停机实板诊断均未发现U相通道选择、GPIO模拟模式、注入数据读取或raw_u传递错误。**不能据此排除模拟信号未到PB0、采样相位相对实际功率波形不合适，或其他尚未验证的动态问题；没有将结果写为“软件全部排除”或“运放必坏”。**

未改动或刷写固件，没有使能电机。诊断时通过board.initialized=false独立阻止Board_SetDriverEnabled(true)，EN保持低；ADC临时配置在结束前全部恢复。最终STOP、控制来源NONE、EN低，调试服务关闭。

## 源码和汇编链路

| 项目 | 证据 |
| --- | --- |
| 引脚 | STM32F103C8 LQFP48的PB0（18脚）支持ADC12_IN8，PB1（19脚）支持ADC12_IN9；[ST数据表](https://www.st.com/resource/en/datasheet/stm32f103c8.pdf) |
| GPIO | board.c gpio_init清零GPIOB_CRL低8位，PB0/PB1均为模拟输入；全工程无其他GPIOB_CRL写入路径。I2C只操作PB6/PB7，TIM3只启用内部CH4，不输出到PB0/PB1 |
| U通道 | ADC1_SMPR2的CH8=2，即13.5周期；JL=0，JSQ4=8，对应单通道注入 |
| W通道 | ADC2_SMPR2的CH9=2，同13.5周期；JL=0，JSQ4=9 |
| 同步 | ADC1_CR1 DUALMOD=5，双ADC注入同步；主ADC1 JEXTSEL=4为TIM3_CC4，ADC2选择软件触发源；两路JEXTTRIG均打开 |
| 结果 | ADC1/2注入完成后读取各自JDR1，分别传为App_OnAdcSample(raw_u,raw_w)；GCC反汇编确认分别从0x4001243C与0x4001283C加载至r0/r1，没有交换或常量替换 |
| 应用 | App保存raw_u/raw_w，U减offset_u并乘current_scale_u；当前两路系数均为−0.00161172159 A/count，U没有独立清零分支 |
| 单通道SCAN | SCAN关闭不构成缺陷。[ST官方HAL](https://raw.githubusercontent.com/STMicroelectronics/stm32f1xx-hal-driver/master/Src/stm32f1xx_hal_adc_ex.c)支持SCAN关闭的单通道注入，并使用JL0/JSQ4；结果读JDR1，不能因此改成读JDR4 |

## 实板读取

Flash .text范围0x0800010C～0x08007710与当前CurrentLoopTest ELF匹配。源文件/符号地址与板上代码一致后才进行变量和外设访问。

| 寄存器 | 实读 | 解释 |
| --- | --- | --- |
| GPIOB_CRL | 0x66484400 | PB0/PB1的MODE和CNF均0 |
| ADC1_CR1 / ADC2_CR1 | 0x00050080 / 0 | 主ADC双注入同步、JEOC中断；从ADC无独立中断 |
| ADC1_CR2 / ADC2_CR2 | 0x0000C001 / 0x0000F001 | ADON及两路触发使能正确 |
| ADC1_SMPR2 / ADC2_SMPR2 | 0x02000000 / 0x10000000 | CH8/CH9均13.5周期 |
| ADC1_JSQR / ADC2_JSQR | 0x00040000 / 0x00048000 | JL0，分别JSQ4=8/9 |
| ADC1/2 JOFR1～4 | 全0 | 没有硬件注入偏移抵消U信号 |
| RCC_CFGR | 0x001D840A | ADC时钟为APB2/6，即12 MHz |
| TIM3_SMCR / CCMR2 / CCER | 0x17 / 0x7800 / 0x1000 | 硬件分频/CH4触发，PB0的CH3没有启用 |

519 ms停机窗口新增2076次ADC回调=4 kHz，ADC同步错误/积压/超时均0。Board/App fault=0，AS5600 OK。

## 停机交叉与内部参考试验

为使输入有明确差异并验证整条读取路径，临时对换PB0/PB1，再让ADC1测内部Vref。每次重配置都先暂停CPU/冻结定时器，关两ADC的ADON/JEXTTRIG及主ADC双模式，设置序列后恢复双模式、上电稳定、清旧状态及ADC NVIC pending，再恢复触发。两路临时使用239.5周期采样，内部Vref开启TSVREFE且等待稳定。该长采样仅用于停机诊断，不是生产电流采样修改。

| 模式 | 本窗口新增ADC回调 | 应用raw_u | 应用raw_w | 外设结果 |
| --- | ---: | ---: | ---: | --- |
| ADC1采PB1，ADC2采PB0 | 452 | 2050 | 2052 | JDR1分别2050/2052 |
| ADC1采内部Vref CH17，ADC2采PB1 | 464 | 1491 | 2048 | ADC1 JDR1=1491，其余JDR为0；ADC2 JDR1=2048 |
| ADC1恢复PB0，ADC2恢复PB1（仍长采样） | 488 | 2049 | 2051 | JDR1分别2049/2051 |

1491计数按名义3.3 V换算约1.2015 V，符合[ST内部参考约1.20 V](https://www.st.com/resource/en/datasheet/stm32f103c8.pdf)的量级。关键证据是ADC1的JDR1与App raw_u在改变输入后共同变化，切回PB0后共同恢复；因此没有发现ADC1停转、读旧JDR、rank错位或raw_u被覆盖为常量的问题。

两路PB0/PB1静态电压本来都约1.65 V，所以交叉测量本身不能证明PCB网络标签或实际通道位置正确。内部参考也不能验证外部PB0模拟输入对交流电流信号的响应。

诊断结束复核ADC1_CR1/CR2/SMPR1/SMPR2/JSQR、ADC2_CR2/SMPR2/JSQR全部恢复原值，board.initialized=true；Board/App故障、ADC同步错误/积压/超时均0。OpenOCD在首次诊断前记录过外部复位，之后重新等待500ms并确认校准/STOP就绪才开始试验；该复位不作为动态电流故障结论。

日志在本地构建目录：u-adc-register-inspect.log、u-adc-crosscheck-stopped.log、u-adc-audit-final-stop.log。

## 尚待验证

下一步应把转动时U6输出及MCU PB0输入的波形，与ADC触发时刻一起比较。若U6输出变化但PB0不变化，检查中间连接；若PB0存在变化而当前同步读数仍近零，则进一步检查采样相位、输入建立时间与前端滤波。当前证据不足以直接归因某一运放或功率相损坏，也不应启用依赖U电流反馈的闭环。

## 当前嘉立创工程复核（2026-10-24）

通过嘉立创插件读取用户当前 `new_back_foc_small` 工程的 `STM32FOC_LCEDA_PCB`（UUID `52caa9452ee66dee`），及其关联的 `SimpleFOC103 / Sheet_1` 全工程原理图网表。网表共58个器件，只有这一页。该快照代表当前设计，不自动代表已生产PCB和实际焊装料。未修改PCB、原理图、BOM或固件，未进行新的电机试验。

| 分析项 | 当前设计的检查结果 |
| --- | --- |
| ① 电路功能概述 | STM32F103C8控制DRV8313三相桥；AS5600提供角度；U/W双INA181反馈电流；TJA1050提供CAN；U5降压产生5V，SPX3819产生3.3V，OPA333缓冲1.65V参考 |
| ② 器件清单与选型合理性 | U6/U8型号INA181A2IDBVT，DBV六脚映射符合数据表，R14/R15/R16标10mΩ。发现C14/C15/C16型号CL10A226MQ8NRNC为22µF/6.3V，C23型号0603ZD104KAT2A为100nF/10V，均接VCC/GND，不适合用户14V输入。C22的BOM器件封装0603与当前PCB封装C0402_L不一致，需核实实装兼容性；不据此归因U采样故障 |
| ③ 电源方案分析 | C24标10µF/50V；C25标22µF/25V，接5V，耐压标注可用。U5网表DeviceName为MP2359，SW网络仅U5.6、L2.1、C21.1，无外部续流二极管；若实装MP2359，应补阳极GND、阴极SW的合适肖特基。其49.9k/9.53k反馈比和100k输入拉EN符合该芯片推荐接法。U9的VIN/EN接5V、VOUT接3.3V，BYP空置本身不构成错误 |
| ④ 信号与连线检查 | 原理图和PCB均为U6.1→ADC_U→U3.18(PB0)，U6.3→ISENSE_1→R14.1/U4.6，U6.2/.4→GND，U6.5→1.65V，U6.6→3.3V；W为U8.1→PB1、U8.3→ISENSE_3。MCU PA0/1/2分别接DRV IN1/2/3，PA4接共用EN。AS5600供电方式、SDA/SCL上拉网络，以及CAN主信号映射未见明显错误 |
| ⑤ 保护与可靠性分析 | DRV nFAULT未接MCU，固件无法直接获知驱动器硬件故障；固定120Ω CAN终端只适用于总线端点。DRV nCOMPO为开漏输出，接GND不能当作推挽输出短路；NC21无内部连接，也不能作为U故障证据。实板焊接、母线尖峰、采样瞬时波形不由网表或DRC验证 |
| ⑥ 整体可用性评估 | PCB严格DRC check(true,false,true)最新返回空数组，当前规则未报错；原理图DRC仍有悬挂/网络标记/图框外对象警告，接口最多返回120条，不能写成原理图全部通过。已确认电容耐压选型错误及条件性的MP2359缺续流二极管问题；U反馈故障根因仍待实测 |

### U相走线证据

PCB焊盘网络、走线及过孔均已读取：

- U4.6 (PGND1) 位于 `(3981.6,-3754.2)`，ISENSE_1顶层走线经 `(3981.6468,-3797.0039)` 与 `(3982.0995,-3820.1733)` 过孔，到底层R14.1 `(3894.0,-3769.4)`；R14.2为GND。
- U6.3 `(3909.8,-3647.4)` 单独引线到R14.1，网络为ISENSE_1，与GND不同。
- U6.1 `(3909.8,-3572.6)` 的ADC_U底层线经 `(4008.2047,-3264.0)` 过孔转顶层，到U3.18 `(4010,-3199)`。没有发现接到PB1或参考电压的网络映射错误。
- U6/U8均为相同SOT-23-6 DBV封装和引脚编号，未发现一个使用SC70/DCK映射、另一个使用DBV映射的问题。

坐标按PCB源码/API的mil记录，POURED缓存坐标为其1/10。不能只凭大面积GND覆铜边框覆盖了采样区就断言分流电阻被短路，必须看填充后的隔离区和实板。

离线脚本 `build/board-debug/audit_pcb_copper.py` 进一步解析实际POURED缓存（坐标×10），对ARC按≤0.25°步长离散后进行局部几何核查，未出现解析或无效多边形警告。ISENSE_1的PGND1→双过孔→R14.1→U6.3为同一连通分量；ADC_U的U6.1→过孔→U3.18同样连续。未发现这两网与已解析GND、1.65V或其他网络铜皮交叠，不支持当前设计中“R14被铺铜旁路”或“ADC_U直接接参考电压”的判断。该几何检查不能代替加工板或焊接检查。

具体实物检查点：U6.3采样输入焊盘邻近GND过孔 `df02e62eb150eed4`，中心 `(3931.4,-3673.9)` mil、直径约23.622 mil，焊盘铜边到过孔铜边间隙约2.889 mil（0.0734 mm）。设计上仍有间隙，但这里应优先看是否存在焊锡桥。另一颗GND过孔 `797bbc08692bea77`、中心 `(3949.0,-3652.5)` mil，与同一焊盘间隙约5.739 mil。不能用10mΩ电阻两端蜂鸣证明短路，因为正常分流电阻本身即会让通断档蜂鸣。

### 优先处理与后续测量

先断开14V，核实C14/C15/C16/C23的实际装料耐压，以及U5实际型号和是否已另行补焊续流二极管。若与当前BOM一致，应更换母线电容（按14V及尖峰选25V或更高并校核有效容量，100nF旁路可选50V），补齐MP2359要求的续流路径，再恢复动力试验。当前只检查，未代用户修改元器件或重新选型。

电源问题处理后，仍需比较R14两端差分、U6.1输出、MCU PB0(18脚)与ADC触发时间。10mΩ×50倍意味着0.2A对应约2mV采样差分、约100mV输出偏移；符号随电流方向改变。U6输出不变应查其输入/供电/焊接；U6变而PB0不变查中间通路；PB0变而同步ADC不变才继续定位采样窗口。没有证据支持直接改ADC通道或继续提高试验电压。

依据：[TI INA181](https://www.ti.com/lit/ds/symlink/ina181.pdf)、[TI DRV8313](https://www.ti.com/lit/ds/symlink/drv8313.pdf)、[Samsung CL10A226MQ8NRNC](https://product.samsungsem.com/mlcc/CL10A226MQ8NRN.do)、[KYOCERA AVX X5R型号编码](https://datasheets.kyocera-avx.com/cx5r.pdf)、[MPS MP2359](https://www.monolithicpower.com/en/documentview/productdocument/index/version/2/document_type/Datasheet/lang/EN/sku/MP2359)。构建目录保留原理图网表 `schematic-live-netlist.json` 和实际PCB几何快照 `pcb-live-geometry.json`。
