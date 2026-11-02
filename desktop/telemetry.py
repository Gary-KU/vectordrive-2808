"""Validity-aware display values for independent CAN status/trace pages."""
from dataclasses import dataclass
import math
import time

DISPLAY_TIMEOUT_S = 0.250

FAULT_NAMES = {
    0: "控制配置错误", 1: "控制输入无效", 2: "角度传感器无效",
    3: "相电流过流", 4: "控制命令无效", 5: "数值计算异常",
    16: "板级外设故障", 17: "电流偏置校准失败", 18: "ADC 接近量程边界",
    19: "编码器反馈异常", 20: "转子对齐失败", 21: "控制中断超时",
    22: "应用配置错误", 23: "CAN 通信故障", 24: "急停锁存",
}


def describe_faults(flags):
    """Names match FOC_FAULT_* / APP_FAULT_* in the firmware headers."""
    names = [name for bit, name in FAULT_NAMES.items() if flags & (1 << bit)]
    unknown = flags & ~sum(1 << bit for bit in FAULT_NAMES)
    if unknown:
        names.append(f"未知故障 0x{unknown:08X}")
    return "、".join(names)


def can_feedback_error(snapshot, now=None):
    """An open adapter is not proof that the selected node is responding."""
    if not snapshot.get("connected"):
        return ""
    now = time.monotonic() if now is None else now
    received = snapshot.get("received") or {}
    node = (snapshot.get("status") or {}).get("node", snapshot.get('node', 1))
    missing = []
    for kind, name, base in (("status", "状态", 0x180), ("diagnostic", "诊断", 0x280)):
        stamp = received.get(kind)
        if stamp is None or not 0 <= now - stamp <= DISPLAY_TIMEOUT_S:
            missing.append(f"{name}帧 0x{base + node:03X}")
    if not missing:
        return ""
    return (f"CAN 反馈中断：{'、'.join(missing)} 超过 250 ms 未更新。"
            "请检查板子主电源、CAN 接线、波特率和节点 ID。")


@dataclass(frozen=True)
class EncoderReadout:
    quality: str
    angle_rad: float | None = None
    angle_deg: float | None = None
    velocity_rad_s: float | None = None
    rpm: float | None = None
    age_ms: float | None = None

    @property
    def valid(self):
        return self.quality == "valid"


def encoder_readout(snapshot, now=None):
    """Never present cached, invalid or disconnected angles as live feedback.

    Prefer the fast mechanical frame when available, otherwise trace page 2.
    Neither field implies
    multi-turn position or electrical alignment; the displayed angle is wrapped.
    """
    now = time.monotonic() if now is None else now
    if not snapshot.get("connected"):
        return EncoderReadout("disconnected")
    received = snapshot.get("received") or {}
    status_time = received.get("status")
    if status_time is None:
        return EncoderReadout("waiting_status")
    if not 0 <= now - status_time <= DISPLAY_TIMEOUT_S:
        return EncoderReadout("status_stale")
    status = snapshot.get("status") or {}
    if not status.get("sensor_valid"):
        return EncoderReadout("sensor_invalid")
    fast_present = 'fast' in received
    values = (snapshot.get('fast') or {}) if fast_present else snapshot.get('telemetry') or {}
    if fast_present and not values.get('sensor_valid'):
        return EncoderReadout('sensor_invalid')
    speed = values.get('velocity_rad_s') if fast_present else status.get("velocity_rad_s")
    if not isinstance(speed, (int, float)) or not math.isfinite(speed):
        speed = None
    rpm = speed * 60 / math.tau if speed is not None else None
    trace_time = received.get('fast' if fast_present else 'trace_2')
    if trace_time is None:
        return EncoderReadout("waiting_angle", velocity_rad_s=speed, rpm=rpm)
    age = (now - trace_time) * 1000
    if not 0 <= age <= DISPLAY_TIMEOUT_S * 1000:
        return EncoderReadout("angle_stale", velocity_rad_s=speed, rpm=rpm, age_ms=age)
    angle = values.get("angle_rad")
    if not isinstance(angle, (int, float)) or not math.isfinite(angle):
        return EncoderReadout("waiting_angle", velocity_rad_s=speed, rpm=rpm, age_ms=age)
    angle = angle % math.tau
    return EncoderReadout("valid", angle, math.degrees(angle), speed, rpm, age)
