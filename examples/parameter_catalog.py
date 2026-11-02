"""Generated from config/parameters.json by tools/generate_parameters.py; do not edit."""
from dataclasses import dataclass
from enum import IntEnum


class Param(IntEnum):
    CURRENT_KP = 1
    CURRENT_KI = 2
    IQ_LIMIT = 3
    VOLTAGE_LIMIT = 4
    IQ_SLEW = 5
    SPEED_KP = 6
    SPEED_KI = 7
    SPEED_LIMIT = 8
    BUS_VOLTAGE = 9


@dataclass(frozen=True)
class ParameterSpec:
    id: int
    name: str
    label: str
    group: str
    unit: str
    minimum: float
    maximum: float
    default: float
    step: float
    description: str
    invalidates_zero: bool


PARAMETER_SPECS = (
    ParameterSpec(1, 'current_kp', '电流环比例增益', 'current', 'V/A', 0.0, 5.0, 1.0, 0.05, 'Id 与 Iq 电流 PI 的比例增益；仅在停机状态修改。', False),
    ParameterSpec(2, 'current_ki', '电流环积分增益', 'current', 'V/A/s', 0.0, 2000.0, 1000.0, 10.0, 'Id 与 Iq 电流 PI 的积分增益；修改后积分状态清零。', False),
    ParameterSpec(3, 'iq_limit', 'Q 轴目标电流上限', 'current', 'A', 0.01, 3.3, 1.0, 0.01, '限制请求 Iq 的绝对值；板端按电机型号校验范围，实际电流仍受独立相过流保护和输出电压约束。', False),
    ParameterSpec(4, 'voltage_limit', 'dq 电压矢量上限', 'current', 'V', 0.1, 5.0, 5.0, 0.1, '限制电流控制输出电压；CAN 开环指令另受 2 V 协议上限约束。', False),
    ParameterSpec(5, 'iq_slew', '目标电流变化率', 'current', 'A/s', 0.01, 10.0, 1.0, 0.1, '限制目标 Iq 每秒的变化量，影响启动与正反向切换的斜坡。', False),
    ParameterSpec(6, 'speed_kp', '速度环比例增益', 'speed', 'A/(rad/s)', 0.0, 1.0, 0.03, 0.005, '速度外环将机械转速误差换算为目标 Iq 的比例增益。', False),
    ParameterSpec(7, 'speed_ki', '速度环积分增益', 'speed', 'A/rad', 0.0, 10.0, 0.1, 0.01, '速度外环的积分增益；电流模式不会运行速度外环。', False),
    ParameterSpec(8, 'speed_limit', '机械转速上限', 'speed', 'rad/s', 0.1, 30.0, 30.0, 0.1, '限制速度模式的机械转速目标绝对值，单位为弧度每秒。', False),
    ParameterSpec(9, 'bus_voltage', '母线电压', 'system', 'V', 8.0, 30.0, 14.0, 0.1, '填写电源实际电压以换算 PWM；板上没有母线电压采样，修改后需重新对齐。', True),
)

PARAMETERS = {
    spec.id: {
        'id': spec.id, 'name': spec.name, 'label': spec.label, 'group': spec.group,
        'unit': spec.unit, 'min': spec.minimum, 'max': spec.maximum,
        'default': spec.default, 'step': spec.step, 'description': spec.description,
        'invalidates_zero': spec.invalidates_zero,
    }
    for spec in PARAMETER_SPECS
}
DEFAULT_PARAMETERS = {spec.id: spec.default for spec in PARAMETER_SPECS}
