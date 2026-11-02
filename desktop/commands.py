"""Parse the debug console independently of CAN I/O and control sessions.

Every accepted command has typed, finite arguments. Wire ranges remain owned
by the protocol codec; this module never opens a bus or changes motor state.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
import math
import shlex

from examples import gimbal_can as protocol


class CommandType(Enum):
    HELP = auto()
    STATUS = auto()
    READ_PARAMETERS = auto()
    WRITE_PARAMETER = auto()
    MOTION = auto()
    DIRECTION = auto()
    MAINTENANCE = auto()
    STOP = auto()
    ESTOP = auto()
    READ_NODE = auto()
    CHANGE_NODE = auto()
    READ_LIMITS = auto()
    READ_CALIBRATION = auto()
    SAVE_CALIBRATION = auto()
    CLEAR_CALIBRATION = auto()


@dataclass(frozen=True)
class ConsoleCommand:
    kind: CommandType
    opcode: protocol.Opcode | None = None
    value: float = 0.0
    auxiliary: float = 0.0
    parameter_ids: tuple[int, ...] = ()


HELP_TEXT = (
    "status | stop | estop | clear | cal | dir +/-1 | align | "
    "iq <A> | speed <机械rad/s> | open <电角rad/s> [V] | "
    "bus <V> | get [id/name] | set <id/name> <值> | limits | node | node <ID> single | "
    "zero | zero save | zero clear。"
    "zero 读取掉电零点；zero save 仅保存已完成且有效的对齐并重启；zero clear 清除后重启。"
)


def parameter_id(token: str) -> int:
    if token.isdecimal():
        result = int(token)
        if result in protocol.PARAMETERS:
            return result
    for number, metadata in protocol.PARAMETERS.items():
        if token.casefold() == metadata["name"].casefold():
            return number
    raise ValueError("未知参数；使用 get 查看参数 ID")


def finite_number(token: str) -> float:
    try:
        value = float(token)
    except (ValueError, OverflowError) as error:
        raise ValueError("参数必须是有限数值") from error
    if not math.isfinite(value):
        raise ValueError("参数必须是有限数值")
    return value


def parse_command(text: str) -> ConsoleCommand:
    words = shlex.split(text)
    if not words:
        raise ValueError("请输入命令；输入 help 查看")
    name, args = words[0].casefold(), words[1:]
    immediate = {"help": CommandType.HELP, "status": CommandType.STATUS,
                 "stop": CommandType.STOP, "estop": CommandType.ESTOP}
    if name in immediate and not args:
        return ConsoleCommand(immediate[name])
    if name == 'limits' and not args:
        return ConsoleCommand(CommandType.READ_LIMITS)
    if name == 'zero' and not args:
        return ConsoleCommand(CommandType.READ_CALIBRATION)
    if name == 'zero' and args == ['save']:
        return ConsoleCommand(CommandType.SAVE_CALIBRATION)
    if name == 'zero' and args == ['clear']:
        return ConsoleCommand(CommandType.CLEAR_CALIBRATION)
    if name == 'node' and not args:
        return ConsoleCommand(CommandType.READ_NODE)
    if name == 'node' and len(args) == 2 and args[1].casefold() == 'single' and args[0].isdecimal():
        new_node = int(args[0])
        protocol.pack_node_set(1, 0, new_node)
        return ConsoleCommand(CommandType.CHANGE_NODE, value=new_node)
    if name == "get" and len(args) <= 1:
        ids = (parameter_id(args[0]),) if args else tuple(protocol.PARAMETERS)
        return ConsoleCommand(CommandType.READ_PARAMETERS, parameter_ids=ids)
    if (name == "set" and len(args) == 2) or (name == "bus" and len(args) == 1):
        number = parameter_id(args[0]) if name == "set" else int(protocol.Param.BUS_VOLTAGE)
        value = finite_number(args[-1])
        protocol.pack_param_write(1, 0, number, value)
        return ConsoleCommand(CommandType.WRITE_PARAMETER, value=value, parameter_ids=(number,))

    opcode = None
    kind = CommandType.MOTION
    value = auxiliary = 0.0
    if name in ("iq", "speed") and len(args) == 1:
        opcode = protocol.Opcode.IQ if name == "iq" else protocol.Opcode.SPEED
        value = finite_number(args[0])
    elif name == "open" and 1 <= len(args) <= 2:
        opcode = protocol.Opcode.OPEN
        value = finite_number(args[0])
        auxiliary = finite_number(args[1]) if len(args) == 2 else 0.8
    elif name == "align" and not args:
        opcode = protocol.Opcode.ALIGN
    elif name == "dir" and len(args) == 1:
        kind, opcode = CommandType.DIRECTION, protocol.Opcode.DIRECTION
        value = finite_number(args[0])
    elif name in ("clear", "cal") and not args:
        kind = CommandType.MAINTENANCE
        opcode = protocol.Opcode.CLEAR if name == "clear" else protocol.Opcode.CALIBRATE
    if opcode is None:
        raise ValueError("命令格式错误；输入 help 查看")
    protocol.pack_command(1, int(opcode), 0, value, auxiliary)
    return ConsoleCommand(kind, opcode, value, auxiliary)
