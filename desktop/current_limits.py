"""Host encoding capability and a separately confirmed board current limit."""
from __future__ import annotations

from collections.abc import Mapping
import math
import struct

from examples import gimbal_can as protocol
from examples.parameter_catalog import PARAMETERS


HOST_CURRENT_LIMIT_A = PARAMETERS[3]['max']


def _float32(value):
    return struct.unpack('<f', struct.pack('<f', value))[0]


def current_limit_readback(parameters) -> float | None:
    """Return a valid parameter-3 readback; editor/default values are not proof."""
    if not isinstance(parameters, Mapping):
        return None
    value = parameters.get(3)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    try:
        value = float(value)
        lower = min(PARAMETERS[3]['min'], _float32(PARAMETERS[3]['min']))
        upper = max(HOST_CURRENT_LIMIT_A, _float32(HOST_CURRENT_LIMIT_A))
        if not math.isfinite(value) or not lower <= value <= upper:
            return None
        return _float32(value)
    except (ValueError, OverflowError, struct.error):
        return None


def validate_current_target(value, parameters, *, require_readback=True) -> float:
    """Validate both requested and encoded Iq against an actual board readback.

    Tools may opt into readback-based validation. The desktop controller uses
    an empty parameter map and require_readback=False to expose the host wire
    range; execution limits and protection remain enforced by the firmware.
    """
    _, payload = protocol.pack_current(1, 0, value)
    number = float(value)
    limit = current_limit_readback(parameters)
    if limit is None:
        if require_readback:
            raise ValueError('尚未确认板端电流限幅；请在停机状态读取参数 3')
        return number
    wire_current = abs(struct.unpack_from('<h', payload, 4)[0]) / 1000.0
    if _float32(abs(number)) > limit or _float32(wire_current) > limit:
        raise ValueError(f'目标超过已读回的电流限幅 ±{limit:g} A')
    return number
