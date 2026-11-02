"""Generate/decode this board's classic CAN frames; never open or send to a bus.

Firmware defaults: node 1, 1 Mbit/s, standard 11-bit identifiers. After boot or
session loss, send STOP and wait for its execution ACK (result=0) before motion.
IQ requires verified shunts/gains/polarity, direction and electrical alignment.
Calibration can be explicitly saved after a valid unloaded alignment; matching
firmware restores it after ADC offset calibration without boot motion. This
helper can encode +/-3.3 A for IQ and 2 V for OPEN.
The 5 V closed-loop voltage limit does not raise the protocol's 2 V OPEN cap.
run_can_current.py provides bounded streaming with execution ACKs and STOP.

Motion has a 100 ms receive-time deadline. A real master should send a fresh
sequence approximately every 10 ms, modulo 256; increments 1..127 are forward.
This module does not schedule that stream, perform a STOP handshake, wait for
ACKs, change ownership, connect a CAN adapter or guarantee command execution.
Do not concatenate a STOP and a motion command in one queued burst: firmware
flushes queued traffic at STOP/ESTOP. Advance sequence yourself for each call.

External units are amperes, mechanical rad/s, electrical rad/s and volts.
Commands round to the nearest wire unit with Decimal ROUND_HALF_UP: exact
half-unit ties round away from zero. The original value must already be inside
this helper's encoding limits, even if rounding would bring it back inside.
The IQ helper can encode +/-3.3 A from the shared host parameter catalog. This
is encoding capacity, not board capability. Query LIMITS_READ for the board's
request ceiling and independent phase-current trip. The desktop exposes the
host encoding range; firmware enforces parameter 3 and reports execution ACKs.

Examples (frame generation only)::

    python examples/gimbal_can.py --node 1 --seq 1 stop
    python examples/gimbal_can.py --node 1 --seq 2 iq 0.05
    python examples/gimbal_can.py --node 1 --seq 3 open 20 0.8

Use decode_feedback(std_id, payload, node=1) for status/diagnostic/ACK bytes.
The documented wire format matches software/Communication/Src/can_protocol.c.
"""

from __future__ import annotations

import argparse
from decimal import Decimal, InvalidOperation, ROUND_HALF_UP, localcontext
from enum import IntEnum
import math
import struct
from typing import Any

try:
    from .parameter_catalog import Param, PARAMETERS, PARAMETER_SPECS, DEFAULT_PARAMETERS
except ImportError:
    # Direct execution sets sys.path[0] to examples/, while package callers use
    # examples.gimbal_can; path-based test loaders can have only the repo root.
    try:
        from examples.parameter_catalog import Param, PARAMETERS, PARAMETER_SPECS, DEFAULT_PARAMETERS
    except ImportError:
        from parameter_catalog import Param, PARAMETERS, PARAMETER_SPECS, DEFAULT_PARAMETERS

VERSION = 0xA1
BITRATE = 1_000_000
COMMAND_BASE = 0x200
STATUS_BASE = 0x180
DIAGNOSTIC_BASE = 0x280
ACK_BASE = 0x380
PARAMETER_BASE = 0x480
TRACE_BASE = 0x580
LIMITS_INFO_BASE = 0x600
CALIBRATION_INFO_BASE = 0x640
NODE_INFO_BASE = 0x680
FAST_BASE = 0x700
ESTOP_ID = 0x080


class Opcode(IntEnum):
    STOP = 0
    IQ = 1
    SPEED = 2
    OPEN = 3
    ALIGN = 4
    CLEAR = 5
    DIRECTION = 6
    CALIBRATE = 7
    PARAM_READ = 0x10
    PARAM_WRITE = 0x11
    NODE_READ = 0x12
    NODE_SET = 0x13
    LIMITS_READ = 0x14
    CALIBRATION_READ = 0x15
    CALIBRATION_SAVE = 0x16
    CALIBRATION_CLEAR = 0x17
    ESTOP = 0x7F


_MOTION = {Opcode.IQ, Opcode.SPEED, Opcode.OPEN, Opcode.ALIGN}
_MODES = {0: "STOP", 1: "OPEN", 2: "ALIGN", 3: "CURRENT", 4: "SPEED", 5: "FAULT"}
_RESULTS = {
    0: "OK", 1: "PENDING", 2: "BUSY", 3: "FAULT", 4: "CALIBRATING",
    5: "NEED_STOP", 6: "SENSE_UNKNOWN", 7: "NEED_DIRECTION", 8: "NEED_ZERO",
    9: "SENSOR", 10: "RANGE", 11: "CANCELLED", 12: "CONTROL_OWNED", 13: "STALE",
    0x80: "BAD_FRAME", 0x81: "RANGE", 0x82: "SEQUENCE", 0x83: "STALE",
    0x84: "REQUIRE_STOP", 0x85: "TRANSPORT", 0x86: "STORAGE", 0x87: "UNSUPPORTED",
}


def _integer(value: Any, minimum: int, maximum: int, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not minimum <= value <= maximum:
        raise ValueError(f"{name} must be an integer in {minimum}..{maximum}")
    return value


def _number(value: Any, name: str) -> Decimal:
    if isinstance(value, bool) or not isinstance(value, (Decimal, int, float, str)):
        raise ValueError(f"{name} must be a finite decimal number")
    try:
        number = value if isinstance(value, Decimal) else Decimal(str(value))
    except (InvalidOperation, ValueError) as error:
        raise ValueError(f"{name} must be a finite decimal number") from error
    if not number.is_finite():
        raise ValueError(f"{name} must be finite")
    return number


def _scaled(number: Decimal, scale: int, minimum: int, maximum: int) -> int:
    # Keep every supplied decimal digit through multiplication, including values
    # infinitesimally below a half-unit boundary; binary floats are not used.
    with localcontext() as context:
        context.prec = max(28, len(number.as_tuple().digits) + 10)
        result = int((number * scale).to_integral_value(rounding=ROUND_HALF_UP))
    if not minimum <= result <= maximum:
        raise ValueError("scaled value does not fit the wire field")
    return result


def pack_command(node: int, opcode: int, sequence: int,
                 value: Any = 0, auxiliary: Any = 0) -> tuple[int, bytes]:
    """Return (standard ID, 8 bytes); validate host encoding limits.

    IQ: value A (absolute limit from the host IQ_LIMIT catalog, +/-3.3);
    SPEED: mechanical rad/s (+/-30);
    OPEN: electrical rad/s (+/-100), auxiliary V (0..2);
    DIRECTION: value exactly +/-1. Other opcodes require zero target/auxiliary.
    Only OPEN accepts a nonzero auxiliary. ESTOP always uses broadcast ID 080.
    Board limits are independent; encoding success does not mean acceptance.
    """
    _integer(node, 1, 63, "node")
    _integer(sequence, 0, 255, "sequence")
    _integer(opcode, 0, 255, "opcode")
    try:
        operation = Opcode(opcode)
    except ValueError as error:
        raise ValueError("unknown opcode") from error
    if operation in (Opcode.PARAM_READ, Opcode.PARAM_WRITE):
        raise ValueError("use pack_param_read or pack_param_write for parameter commands")
    if operation in (Opcode.NODE_READ, Opcode.NODE_SET):
        raise ValueError("use pack_node_read or pack_node_set for node commands")
    if operation == Opcode.LIMITS_READ:
        raise ValueError("use pack_limits_read for read-only board protection information")
    if operation in (Opcode.CALIBRATION_READ, Opcode.CALIBRATION_SAVE,
                     Opcode.CALIBRATION_CLEAR):
        raise ValueError("use pack_calibration_read/save/clear for calibration storage commands")
    target = _number(value, "value")
    aux = _number(auxiliary, "auxiliary")
    if operation != Opcode.OPEN and aux != 0:
        raise ValueError("only OPEN permits a nonzero auxiliary")

    if operation == Opcode.IQ:
        current_limit = Decimal(str(PARAMETERS[Param.IQ_LIMIT]["max"]))
        if target.copy_abs() > current_limit:
            raise ValueError(f"IQ must be within +/-{current_limit} A (host encoding limit)")
        wire_target = _scaled(target, 1000, -32768, 32767)
    elif operation == Opcode.SPEED:
        if not Decimal(-30) <= target <= Decimal(30):
            raise ValueError("speed must be within +/-30 mechanical rad/s")
        wire_target = _scaled(target, 1000, -32768, 32767)
    elif operation == Opcode.OPEN:
        if not Decimal(-100) <= target <= Decimal(100) or not 0 <= aux <= Decimal(2):
            raise ValueError("OPEN requires +/-100 electrical rad/s and 0..2 V")
        wire_target = _scaled(target, 100, -32768, 32767)
    elif operation == Opcode.DIRECTION:
        if target not in (Decimal(-1), Decimal(1)):
            raise ValueError("direction must be exactly +1 or -1")
        wire_target = int(target)
    else:
        if target != 0:
            raise ValueError("this opcode requires value=0")
        wire_target = 0
    wire_aux = _scaled(aux, 1000, 0, 65535)
    flags = 1 if operation in _MOTION else 0
    payload = struct.pack("<BBBBhH", VERSION, operation, sequence, flags, wire_target, wire_aux)
    standard_id = ESTOP_ID if operation == Opcode.ESTOP else COMMAND_BASE + node
    return standard_id, payload


def pack_current(node: int, sequence: int, amperes: Any) -> tuple[int, bytes]:
    return pack_command(node, Opcode.IQ, sequence, amperes)


def pack_speed(node: int, sequence: int, radians_per_second: Any) -> tuple[int, bytes]:
    return pack_command(node, Opcode.SPEED, sequence, radians_per_second)


def pack_stop(node: int, sequence: int) -> tuple[int, bytes]:
    return pack_command(node, Opcode.STOP, sequence)


def pack_open(node: int, sequence: int, electrical_rad_s: Any,
              volts: Any = "0.8") -> tuple[int, bytes]:
    return pack_command(node, Opcode.OPEN, sequence, electrical_rad_s, volts)


def _parameter(param_id: int) -> dict[str, Any]:
    _integer(param_id, 1, 255, "param_id")
    if param_id not in PARAMETERS:
        raise ValueError("unknown parameter ID")
    return PARAMETERS[param_id]


def pack_param_read(node: int, sequence: int, param_id: int) -> tuple[int, bytes]:
    """Read one parameter; bytes 4..7 must be zero, rather than command flags."""
    _integer(node, 1, 63, "node")
    _integer(sequence, 0, 255, "sequence")
    _parameter(param_id)
    return COMMAND_BASE + node, struct.pack("<BBBBf", VERSION, Opcode.PARAM_READ,
                                           sequence, param_id, 0.0)


def pack_node_read(node: int, sequence: int) -> tuple[int, bytes]:
    _integer(node, 1, 63, "node")
    _integer(sequence, 0, 255, "sequence")
    return COMMAND_BASE + node, bytes((VERSION, Opcode.NODE_READ, sequence, 0, 0, 0, 0, 0))


def pack_limits_read(node: int, sequence: int) -> tuple[int, bytes]:
    """Read fixed board request/phase protection limits; does not change them."""
    _integer(node, 1, 63, "node")
    _integer(sequence, 0, 255, "sequence")
    return COMMAND_BASE + node, bytes((VERSION, Opcode.LIMITS_READ, sequence, 0, 0, 0, 0, 0))


def pack_calibration_read(node: int, sequence: int) -> tuple[int, bytes]:
    """Read persistent electrical calibration without changing motor state."""
    _integer(node, 1, 63, "node")
    _integer(sequence, 0, 255, "sequence")
    return COMMAND_BASE + node, bytes((VERSION, Opcode.CALIBRATION_READ,
                                       sequence, 0, 0, 0, 0, 0))


def pack_calibration_save(node: int, sequence: int) -> tuple[int, bytes]:
    """Persist the stopped board's current valid electrical zero and reboot."""
    _integer(node, 1, 63, "node")
    _integer(sequence, 0, 255, "sequence")
    return COMMAND_BASE + node, bytes((VERSION, Opcode.CALIBRATION_SAVE,
                                       sequence, 0)) + b'CAL1'


def pack_calibration_clear(node: int, sequence: int) -> tuple[int, bytes]:
    """Clear persistent electrical calibration and reboot stopped."""
    _integer(node, 1, 63, "node")
    _integer(sequence, 0, 255, "sequence")
    return COMMAND_BASE + node, bytes((VERSION, Opcode.CALIBRATION_CLEAR,
                                       sequence, 0)) + b'CLR1'


def pack_node_set(node: int, sequence: int, new_node: int) -> tuple[int, bytes]:
    """Persist one node ID and reboot stopped; configure one board at a time."""
    _integer(node, 1, 63, "node")
    _integer(sequence, 0, 255, "sequence")
    _integer(new_node, 1, 63, "new_node")
    return COMMAND_BASE + node, bytes((VERSION, Opcode.NODE_SET, sequence, new_node)) + b'CAN1'


def pack_param_write(node: int, sequence: int, param_id: int,
                     value: Any) -> tuple[int, bytes]:
    """Encode one finite, host-range float32 parameter; firmware requires STOP.

    Validate the original decimal before conversion, so rounding cannot turn an
    out-of-range request into an accepted boundary value. Host metadata can be
    wider than the firmware's independent parameter range; only a successful
    board reply confirms acceptance. The controller must honor board limits.
    """
    _integer(node, 1, 63, "node")
    _integer(sequence, 0, 255, "sequence")
    parameter = _parameter(param_id)
    number = _number(value, "value")
    if not Decimal(str(parameter["min"])) <= number <= Decimal(str(parameter["max"])):
        raise ValueError(f"{parameter['name']} must be in {parameter['min']}..{parameter['max']} {parameter['unit']}")
    try:
        wire_value = struct.pack("<f", float(number))
    except (OverflowError, ValueError, struct.error) as error:
        raise ValueError("value must fit a finite float32") from error
    if not math.isfinite(struct.unpack("<f", wire_value)[0]):
        raise ValueError("value must fit a finite float32")
    return COMMAND_BASE + node, bytes((VERSION, Opcode.PARAM_WRITE, sequence, param_id)) + wire_value


def _payload(data: Any) -> bytes:
    if not isinstance(data, (bytes, bytearray, memoryview)):
        raise ValueError("payload must contain exactly eight bytes")
    result = bytes(data)
    if len(result) != 8 or result[0] != VERSION:
        raise ValueError("expected eight bytes with version A1")
    return result


def _feedback_input(standard_id: int, data: Any, node: int, base: int) -> bytes:
    _integer(node, 1, 63, "node")
    _integer(standard_id, 0, 0x7FF, "standard_id")
    if standard_id != base + node:
        raise ValueError(f"expected standard ID {base + node:03X} for node {node}")
    return _payload(data)


def _result_name(result: int) -> str:
    return _RESULTS.get(result, f"UNKNOWN_0x{result:02X}")


def decode_ack(standard_id: int, data: Any, node: int = 1) -> dict[str, Any]:
    """Decode 0x380+node execution ACK; only result=0 reports execution success."""
    payload = _feedback_input(standard_id, data, node, ACK_BASE)
    _, opcode, sequence, result, faults = struct.unpack("<BBBBI", payload)
    try:
        operation = Opcode(opcode).name
    except ValueError:
        operation = f"UNKNOWN_0x{opcode:02X}"
    return {"type": "ack", "id": standard_id, "node": node, "opcode": opcode,
            "opcode_name": operation, "sequence": sequence, "result": result,
            "result_name": _result_name(result), "executed_ok": result == 0,
            "faults": faults}


def decode_status(standard_id: int, data: Any, node: int = 1) -> dict[str, Any]:
    """Decode 0x180+node. iq_a is meaningful only when current_available is true."""
    payload = _feedback_input(standard_id, data, node, STATUS_BASE)
    _, mode, flags, sequence, iq_ma, velocity_mrad_s = struct.unpack("<BBBBhh", payload)
    names = ("driver_enabled", "sense_confirmed", "offset_ready", "sensor_valid",
             "direction_confirmed", "zero_valid", "command_timed_out", "can_control")
    result = {"type": "status", "id": standard_id, "node": node, "mode": mode,
              "mode_name": _MODES.get(mode, f"UNKNOWN_{mode}"), "flags": flags,
              "last_sequence": sequence, "iq_a": iq_ma / 1000.0,
              "velocity_rad_s": velocity_mrad_s / 1000.0}
    result.update({name: bool(flags & (1 << bit)) for bit, name in enumerate(names)})
    result["current_available"] = result["sense_confirmed"]
    return result


def decode_diagnostic(standard_id: int, data: Any, node: int = 1) -> dict[str, Any]:
    """Decode 0x280+node; board_faults_low8 contains only its transmitted low byte."""
    payload = _feedback_input(standard_id, data, node, DIAGNOSTIC_BASE)
    _, execution_result, faults, board_low8, flags = struct.unpack("<BBIBB", payload)
    names = ("bus_off", "error_passive", "rx_loss", "require_stop", "request_pending",
             "not_initialized", "tx_loss")
    result = {"type": "diagnostic", "id": standard_id, "node": node,
              "execution_result": execution_result, "result_name": _result_name(execution_result),
              "faults": faults, "board_faults_low8": board_low8, "flags": flags}
    result.update({name: bool(flags & (1 << bit)) for bit, name in enumerate(names)})
    return result


def decode_parameter(standard_id: int, data: Any, node: int = 1) -> dict[str, Any]:
    """Decode 0x480+node parameter reply, including its execution result.

    A rejected request may report zero, outside that parameter's accepted write
    range. Check result=0 before treating the returned value as an applied setting.
    """
    payload = _feedback_input(standard_id, data, node, PARAMETER_BASE)
    _, param_id, sequence, result, value = struct.unpack("<BBBBf", payload)
    parameter = _parameter(param_id)
    if not math.isfinite(value):
        raise ValueError("parameter response must contain a finite float32")
    lower = struct.unpack("<f", struct.pack("<f", parameter["min"]))[0]
    upper = struct.unpack("<f", struct.pack("<f", parameter["max"]))[0]
    if result == 0 and not lower <= value <= upper:
        raise ValueError("successful parameter response is outside the parameter limits")
    return {"type": "parameter", "id": standard_id, "node": node,
            "param_id": param_id, "name": parameter["name"], "sequence": sequence,
            "result": result, "result_name": _result_name(result),
            "value": value, "unit": parameter["unit"]}


def decode_trace(standard_id: int, data: Any, node: int = 1) -> dict[str, Any]:
    """Decode one 0x580+node trace page; pages are independent observations."""
    payload = _feedback_input(standard_id, data, node, TRACE_BASE)
    _, page, first, second, third = struct.unpack("<BBhhh", payload)
    result = {"type": "trace", "id": standard_id, "node": node, "page": page}
    if page == 0:
        result.update(id_a=first / 1000.0, iq_reference_a=second / 1000.0,
                      vd_v=third / 1000.0)
    elif page == 1:
        result.update(vq_v=first / 1000.0, raw_u=second, raw_w=third)
    elif page == 2:
        result.update(angle_rad=first / 1000.0, max_isr_us=second,
                      bus_voltage_v=third / 1000.0)
    else:
        raise ValueError("unknown trace page")
    return result


def decode_limits_info(standard_id: int, data: Any, node: int = 1) -> dict[str, Any]:
    """Read-only capability metadata; independent of active Iq parameter 3."""
    payload = _feedback_input(standard_id, data, node, LIMITS_INFO_BASE)
    _, opcode, sequence, result, request_ma, phase_ma = struct.unpack('<BBBBHH', payload)
    if opcode != Opcode.LIMITS_READ:
        raise ValueError('unsupported limits information opcode')
    if result == 0 and (request_ma == 0 or phase_ma == 0):
        raise ValueError('successful limits information must contain positive limits')
    return dict(type='limits_info', id=standard_id, node=node, opcode=opcode,
                sequence=sequence, result=result, result_name=_result_name(result),
                request_max_a=request_ma / 1000.0, phase_trip_a=phase_ma / 1000.0)


def decode_node_info(standard_id: int, data: Any, node: int = 1) -> dict[str, Any]:
    payload = _feedback_input(standard_id, data, node, NODE_INFO_BASE)
    _, opcode, sequence, result, active, stored, version, flags = payload
    if opcode not in (Opcode.NODE_READ, Opcode.NODE_SET) or version != 1 or flags & ~7:
        raise ValueError("unsupported node information format")
    _integer(active, 1, 63, "active_node")
    _integer(stored, 1, 63, "stored_node")
    if active != node:
        raise ValueError("node information does not match its CAN identifier")
    return dict(type='node_info', id=standard_id, node=node, opcode=opcode, sequence=sequence,
                result=result, result_name=_result_name(result), active_node=active,
                stored_node=stored, version=version, flags=flags, stored_valid=bool(flags & 1),
                storage_error=bool(flags & 2), reset_pending=bool(flags & 4))


def decode_calibration_info(standard_id: int, data: Any, node: int = 1) -> dict[str, Any]:
    """Decode persistent electrical-zero state from 0x640+node."""
    payload = _feedback_input(standard_id, data, node, CALIBRATION_INFO_BASE)
    _, opcode, sequence, result, flags, direction, zero_mrad = struct.unpack('<BBBBBbH', payload)
    if opcode not in (Opcode.CALIBRATION_READ, Opcode.CALIBRATION_SAVE,
                      Opcode.CALIBRATION_CLEAR) or flags & ~0x1f:
        raise ValueError('unsupported calibration information format')
    stored = bool(flags & 1)
    if (stored and direction not in (-1, 1)) or (not stored and direction != 0):
        raise ValueError('calibration direction is inconsistent with stored state')
    if zero_mrad > 6283 or (not stored and zero_mrad != 0):
        raise ValueError('calibration electrical zero is outside one revolution')
    return dict(type='calibration_info', id=standard_id, node=node, opcode=opcode,
                sequence=sequence, result=result, result_name=_result_name(result),
                flags=flags, stored_valid=stored, storage_error=bool(flags & 2),
                profile_mismatch=bool(flags & 4), runtime_valid=bool(flags & 8),
                reset_pending=bool(flags & 16), sensor_direction=direction,
                electrical_zero_rad=zero_mrad / 1000.0)


def decode_fast(standard_id: int, data: Any, node: int = 1) -> dict[str, Any]:
    payload = _feedback_input(standard_id, data, node, FAST_BASE)
    _, flags, angle_mrad, speed_mrad_s, iq_ma = struct.unpack('<BBHhh', payload)
    if angle_mrad > 6283:
        raise ValueError('fast mechanical angle is outside one revolution')
    names = ('driver_enabled', 'sense_confirmed', 'offset_ready', 'sensor_valid',
             'direction_confirmed', 'zero_valid', 'command_timed_out', 'can_control')
    result = dict(type='fast', id=standard_id, node=node, flags=flags, angle_rad=angle_mrad/1000.0,
                  velocity_rad_s=speed_mrad_s/1000.0, iq_a=iq_ma/1000.0)
    result.update({name: bool(flags & (1 << bit)) for bit, name in enumerate(names)})
    return result


def decode_feedback(standard_id: int, data: Any, node: int = 1) -> dict[str, Any]:
    """Dispatch the selected node's status, diagnostic, ACK, parameter or trace ID."""
    _integer(node, 1, 63, "node")
    _integer(standard_id, 0, 0x7FF, "standard_id")
    decoders = {STATUS_BASE + node: decode_status, DIAGNOSTIC_BASE + node: decode_diagnostic,
                ACK_BASE + node: decode_ack, PARAMETER_BASE + node: decode_parameter,
                TRACE_BASE + node: decode_trace, LIMITS_INFO_BASE + node: decode_limits_info,
                CALIBRATION_INFO_BASE + node: decode_calibration_info,
                NODE_INFO_BASE + node: decode_node_info,
                FAST_BASE + node: decode_fast}
    if standard_id not in decoders:
        raise ValueError("identifier is not feedback from the selected node")
    return decoders[standard_id](standard_id, data, node)


def format_frame(standard_id: int, data: Any) -> str:
    """Return USB-CAN/candump-style ID#HEX text. This function transmits nothing."""
    _integer(standard_id, 0, 0x7FF, "standard_id")
    return f"{standard_id:03X}#{_payload(data).hex().upper()}"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--node", type=int, default=1, help="firmware node 1..63 (default: 1)")
    parser.add_argument("--seq", type=int, required=True, help="new sequence 0..255; advance it yourself")
    subparsers = parser.add_subparsers(dest="command", required=True)
    commands = {"stop": Opcode.STOP, "iq": Opcode.IQ, "speed": Opcode.SPEED,
                "open": Opcode.OPEN, "align": Opcode.ALIGN, "clear": Opcode.CLEAR,
                "dir": Opcode.DIRECTION, "cal": Opcode.CALIBRATE, "estop": Opcode.ESTOP}
    for name, opcode in commands.items():
        command_parser = subparsers.add_parser(name)
        if opcode in (Opcode.IQ, Opcode.SPEED, Opcode.OPEN, Opcode.DIRECTION):
            command_parser.add_argument("value")
        if opcode == Opcode.OPEN:
            command_parser.add_argument("volts", nargs="?", default="0.8")
    subparsers.add_parser('node-read')
    subparsers.add_parser('limits')
    subparsers.add_parser('calibration-read')
    subparsers.add_parser('calibration-save')
    subparsers.add_parser('calibration-clear')
    node_set = subparsers.add_parser('node-set')
    node_set.add_argument('new_node', type=int)
    args = parser.parse_args(argv)
    try:
        if args.command == 'node-read':
            standard_id, payload = pack_node_read(args.node, args.seq)
        elif args.command == 'limits':
            standard_id, payload = pack_limits_read(args.node, args.seq)
        elif args.command == 'calibration-read':
            standard_id, payload = pack_calibration_read(args.node, args.seq)
        elif args.command == 'calibration-save':
            standard_id, payload = pack_calibration_save(args.node, args.seq)
        elif args.command == 'calibration-clear':
            standard_id, payload = pack_calibration_clear(args.node, args.seq)
        elif args.command == 'node-set':
            standard_id, payload = pack_node_set(args.node, args.seq, args.new_node)
        else:
            standard_id, payload = pack_command(args.node, commands[args.command], args.seq,
                                                getattr(args, "value", 0), getattr(args, "volts", 0))
    except ValueError as error:
        parser.error(str(error))
    print(f"Standard ID: 0x{standard_id:03X}; DLC: 8; bus setting: {BITRATE} bit/s")
    print(" ".join(f"{byte:02X}" for byte in payload))
    print(format_frame(standard_id, payload))
    print("Generated only; no CAN frame transmitted and no execution ACK received.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
