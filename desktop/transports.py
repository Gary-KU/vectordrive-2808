"""CAN adapters and a protocol demonstrator; the simulator does not validate a motor.

The simulator uses first-order display dynamics, not the firmware's FOC algorithm
or measured motor parameters. It never opens an adapter or starts motion by itself.
Hardware dependencies are imported only when a hardware interface is requested.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import importlib
import math
import struct
import time

from examples import gimbal_can as protocol
from examples.parameter_catalog import DEFAULT_PARAMETERS as CATALOG_DEFAULT_PARAMETERS


@dataclass(frozen=True)
class Frame:
    arbitration_id: int
    data: bytes
    is_extended_id: bool = False
    is_remote_frame: bool = False
    is_error_frame: bool = False

    def __post_init__(self):
        object.__setattr__(self, "data", bytes(self.data))


def feedback_filters(node):
    """Observe shared-bus health without treating another node as our motor.

    Status and diagnostic ranges include every possible A1 node so an absent
    selected node cannot cause a healthy shared PCAN device to be restarted.
    Command replies and tuning/trace data stay specific to the selected node.
    The reserved node-zero identifiers admitted by each mask are rejected by
    decode_bus_health_frame.
    """
    if isinstance(node, bool) or not isinstance(node, int) or not 1 <= node <= 63:
        raise ValueError("CAN node must be 1..63")
    return [
        {"can_id": 0x180, "can_mask": 0x7C0, "extended": False},
        {"can_id": 0x280, "can_mask": 0x7C0, "extended": False},
        *({"can_id": base + node, "can_mask": 0x7FF, "extended": False}
          for base in (0x380, 0x480, 0x580, 0x600, 0x680, 0x700)),
    ]


def decode_bus_health_frame(frame):
    """Return validated A1 status/diagnostic data from any node, or None.

    This metadata proves that the shared adapter is receiving protocol traffic;
    it must never fill the selected motor's feedback cache or satisfy its ACK.
    """
    if frame.is_error_frame or frame.is_extended_id or frame.is_remote_frame:
        return None
    identifier = frame.arbitration_id
    if 0x181 <= identifier <= 0x1BF:
        node, decoder = identifier - 0x180, protocol.decode_status
    elif 0x281 <= identifier <= 0x2BF:
        node, decoder = identifier - 0x280, protocol.decode_diagnostic
    else:
        return None
    try:
        feedback = decoder(identifier, frame.data, node)
    except (ValueError, TypeError):
        return None
    if feedback["type"] == "status" and feedback["mode"] not in range(6):
        return None
    if feedback["type"] == "diagnostic" and frame.data[7] & 0x80:
        return None
    return feedback


def _float32(value):
    return struct.unpack("<f", struct.pack("<f", value))[0]


def _signed(value, scale=1000.0):
    return max(-32768, min(32767, round(value * scale)))


def _node(config):
    node = int(config.get("node", 1))
    if not 1 <= node <= 63:
        raise ValueError("CAN node must be 1..63")
    return node


class SimulatorTransport:
    """Single-owner, lazy-clock simulator for UI/protocol work only.

    Inject a clock/sleeper in unit tests; normal operation uses real time. Missed
    telemetry is not replayed in a burst: recv reports the latest observation.
    """
    simulation_only = True
    DEFAULT_PARAMETERS = CATALOG_DEFAULT_PARAMETERS

    def __init__(self, config=None, *, clock=time.monotonic, sleeper=time.sleep):
        config = {} if config is None else config
        self.node = _node(config)
        # Simulator board capability is separate from the host encoding range.
        # Default models the 3.3 A request ceiling shared by both motor profiles.
        self.iq_limit_max_a = float(config.get("iq_limit_max_a", 3.3))
        if not math.isfinite(self.iq_limit_max_a) or not 1.0 <= self.iq_limit_max_a <= protocol.PARAMETERS[3]["max"]:
            raise ValueError("simulator current capability is outside the supported host range")
        # Metadata only. This simple command/plant simulator does not model a
        # real motor's instantaneous phase-current protection or safe torque.
        self.phase_trip_a = 2.0
        self._node_store = config.get('_sim_node_store', {})
        self.node = self._node_store.get('node', self.node)
        self._node_store.setdefault('node', self.node)
        self._node_store.setdefault('valid', False)
        self._node_store.setdefault('calibration_valid', False)
        self._node_store.setdefault('calibration_error', False)
        self._node_store.setdefault('calibration_direction', 1)
        self._node_store.setdefault('calibration_zero', 0.0)
        self._clock, self._sleep = clock, sleeper
        self._closed = False
        self._frames = deque(maxlen=1024)
        now = clock()
        self._updated = now
        self._next_feedback = now + 0.010
        self._next_fast = now + .005
        self._calibration_ends = now + 0.512
        self._alignment_ends = None
        self._last_motion = None
        self._sequence_valid = False
        self.last_sequence = 0
        self.last_result = 0x84
        self.require_stop = True
        self.mode = 0
        self.driver_enabled = False
        self.faults = 0
        self.offset_ready = False
        self.sense_confirmed = bool(config.get("sense_confirmed", True))
        self.sensor_valid = True
        self.direction_confirmed = self.zero_valid = self.command_timed_out = False
        self.direction = 1
        self.electrical_zero_rad = 0.0
        self.parameters = {key: _float32(value) for key, value in self.DEFAULT_PARAMETERS.items()}
        self.iq = self.id = self.iq_reference = self.velocity = self.angle = 0.0
        self.vd = self.vq = self._target = self._open_voltage = self._speed_integral = 0.0
        self._trace_page = 0

    def _stop(self, timed_out=False):
        if self.mode == 2:
            self.zero_valid = False
        self.mode = 5 if self.faults else 0
        self.driver_enabled = False
        self._last_motion = self._alignment_ends = None
        self._target = self.iq_reference = self._speed_integral = 0.0
        self.vd = self.vq = 0.0
        if timed_out:
            self.command_timed_out = True
            self.require_stop = True
            self._sequence_valid = False

    def _plant(self, elapsed):
        if elapsed <= 0:
            return
        # Deliberately simple first-order plotting data. These constants do not
        # represent the user's motor, driver losses, stability or torque limits.
        if self.mode == 3:
            target = self._target
        elif self.mode == 4:
            error = self._target - self.velocity
            self._speed_integral = max(-self.parameters[3], min(self.parameters[3],
                self._speed_integral + self.parameters[7] * error * elapsed))
            target = self.parameters[6] * error + self._speed_integral
        elif self.mode == 1:
            target = math.copysign(min(self.parameters[3], self._open_voltage * 0.15), self._target)
        else:
            target = 0.0
        target = max(-self.parameters[3], min(self.parameters[3], target))
        slew = self.parameters[5] * elapsed
        self.iq_reference += max(-slew, min(slew, target - self.iq_reference))
        if self.driver_enabled and self.mode in (1, 3, 4):
            self.vq = max(-self.parameters[4], min(self.parameters[4],
                0.6 * self.iq_reference + 0.03 * self.velocity))
            achievable = (self.vq - 0.03 * self.velocity) / 0.6
            self.iq += (achievable - self.iq) * (1.0 - math.exp(-elapsed / 0.04))
        else:
            self.iq *= math.exp(-elapsed / 0.02)
            self.vq = 0.0
        self.id = 0.1 if self.mode == 2 else 0.0
        self.vd = 0.6 if self.mode == 2 else 0.0
        desired_speed = self._target / 11.0 if self.mode == 1 else self.iq * 10.0
        self.velocity += (desired_speed - self.velocity) * (1.0 - math.exp(-elapsed / 0.3))
        self.angle = (self.angle + self.velocity * elapsed) % (2.0 * math.pi)

    def _advance(self, now):
        # Process watchdog/alignment at their event time, before advancing the
        # stopped display dynamics through the rest of a delayed recv interval.
        end = now
        watchdog = None if self._last_motion is None else self._last_motion + 0.100
        if watchdog is not None:
            end = min(end, watchdog)
        if self._alignment_ends is not None:
            end = min(end, self._alignment_ends)
        self._plant(max(0.0, end - self._updated))
        self._updated = max(self._updated, end)
        if watchdog is not None and now + 1e-12 >= watchdog and (
                self._alignment_ends is None or watchdog < self._alignment_ends):
            self._stop(timed_out=True)
        elif self._alignment_ends is not None and now + 1e-12 >= self._alignment_ends:
            self._alignment_ends = None
            self.mode = 0
            self._stop()
            self.zero_valid = True
            self.electrical_zero_rad = (self.angle * 11.0) % (2.0 * math.pi)
        self._plant(max(0.0, now - self._updated))
        self._updated = now
        if self._calibration_ends is not None and now + 1e-12 >= self._calibration_ends:
            self.offset_ready = True
            self._calibration_ends = None
            if self._node_store.get('calibration_valid'):
                self.direction = int(self._node_store['calibration_direction'])
                self.electrical_zero_rad = float(self._node_store['calibration_zero'])
                self.direction_confirmed = self.zero_valid = True
        if now + 1e-12 >= self._next_feedback:
            self._publish()
            self._next_feedback = now + 0.010
        if now + 1e-12 >= self._next_fast:
            flags = sum(int(value) << bit for bit, value in enumerate((self.driver_enabled,
                self.sense_confirmed, self.offset_ready, self.sensor_valid, self.direction_confirmed,
                self.zero_valid, self.command_timed_out, self.driver_enabled)))
            self._frames.append(Frame(protocol.FAST_BASE + self.node, struct.pack('<BBHhh', 0xA1,
                flags, max(0, min(6283, round(self.angle*1000))), _signed(self.velocity), _signed(self.iq))))
            self._next_fast = now + .005

    def _publish(self):
        flags = sum(int(value) << bit for bit, value in enumerate((self.driver_enabled,
            self.sense_confirmed, self.offset_ready, self.sensor_valid, self.direction_confirmed,
            self.zero_valid, self.command_timed_out, self.driver_enabled)))
        self._frames.append(Frame(0x180 + self.node, struct.pack("<BBBBhh", 0xA1,
            self.mode, flags, self.last_sequence, _signed(self.iq), _signed(self.velocity))))
        self._frames.append(Frame(0x280 + self.node, struct.pack("<BBIBB", 0xA1,
            self.last_result, self.faults, 0, 8 if self.require_stop else 0)))
        if self._trace_page == 0:
            values = (_signed(self.id), _signed(self.iq_reference), _signed(self.vd))
        elif self._trace_page == 1:
            phase = self.angle * 11.0
            raw_u = round(2048 + self.iq * math.sin(phase) / 0.0016117216)
            raw_w = round(2048 + self.iq * math.sin(phase + 2 * math.pi / 3) / 0.0016117216)
            values = (_signed(self.vq), max(0, min(4095, raw_u)), max(0, min(4095, raw_w)))
        else:
            values = (_signed(self.angle), 0, _signed(self.parameters[9]))
        self._frames.append(Frame(0x580 + self.node, struct.pack("<BBhhh", 0xA1,
            self._trace_page, *values)))
        self._trace_page = (self._trace_page + 1) % 3

    def _reply(self, opcode, sequence, result, parameter=None):
        self.last_result = result
        self._frames.append(Frame(0x380 + self.node, struct.pack("<BBBBI", 0xA1,
            opcode, sequence, result, self.faults)))
        if parameter is not None:
            self._frames.append(Frame(0x480 + self.node, struct.pack("<BBBBf", 0xA1,
                parameter, sequence, result, self.parameters.get(parameter, 0.0))))

    def _execute(self, opcode, value, auxiliary, now, parameter):
        if opcode == 0x10:
            return 0
        if opcode == 0:
            self._stop()
            self.require_stop = False
            return 0
        if opcode == 0x7F:
            self.faults |= 1 << 24
            self.zero_valid = False
            self._stop()
            self.require_stop = True
            self._sequence_valid = False
            return 0
        stopped = self.mode == 0 and not self.driver_enabled
        if opcode == 5:
            if self.driver_enabled:
                return 5
            self.faults = 0
            self.mode = 0
            self.command_timed_out = False
            self.offset_ready = self.zero_valid = False
            self._calibration_ends = now + 0.512
            return 0
        if self.faults:
            return 3
        if opcode == 0x11:
            if not stopped:
                return 5
            metadata = protocol.PARAMETERS[parameter]
            if not _float32(metadata["min"]) <= value <= _float32(metadata["max"]):
                return 10
            if parameter == 3 and value > _float32(self.iq_limit_max_a):
                return 10
            self.parameters[parameter] = value
            if parameter == 9:
                self.zero_valid = False
            return 0
        if opcode == 7:
            if not stopped:
                return 5
            self.offset_ready = self.zero_valid = False
            self._calibration_ends = now + 0.512
            return 0
        if opcode == 6:
            if not stopped:
                return 5
            self.direction = int(value)
            self.direction_confirmed = True
            self.zero_valid = False
            return 0
        if not self.offset_ready:
            return 4
        wanted = {1: 3, 2: 4, 3: 1, 4: 2}[opcode]
        if not stopped and self.mode != wanted:
            return 5
        if opcode in (1, 2, 4):
            if opcode in (1, 2) and not self.sense_confirmed:
                return 6
            if not self.direction_confirmed:
                return 7
            if opcode in (1, 2) and not self.zero_valid:
                return 8
            if not self.sensor_valid:
                return 9
        if opcode == 1 and abs(value) > self.parameters[3] + 1e-7:
            return 10
        if opcode == 2 and abs(value) > self.parameters[8] + 1e-7:
            return 10
        if opcode == 3 and auxiliary > self.parameters[4] + 1e-7:
            return 10
        if opcode == 4 and stopped and self.zero_valid:
            return 0
        if opcode == 4 and self.mode != 2:
            self.zero_valid = False
            self._alignment_ends = now + 1.5
        self.mode = wanted
        self.driver_enabled = True
        self.command_timed_out = False
        self._last_motion = now
        self._target, self._open_voltage = value, auxiliary
        return 0

    def send(self, frame, timeout=0.01):
        if self._closed:
            raise RuntimeError("simulator is closed")
        now = self._clock()
        self._advance(now)
        if frame.arbitration_id not in (0x200 + self.node, 0x080):
            return
        data = bytes(frame.data)
        opcode = data[1] if len(data) > 1 else 0
        sequence = data[2] if len(data) > 2 else 0
        parameter = data[3] if len(data) > 3 and opcode in (0x10, 0x11) else None
        if (len(data) != 8 or data[0] != 0xA1 or frame.is_extended_id or
                frame.is_remote_frame or frame.is_error_frame):
            self._reply(opcode, sequence, 0x80, parameter)
            return
        if opcode == protocol.Opcode.LIMITS_READ:
            result = 0 if data[3:] == bytes(5) and frame.arbitration_id == 0x200 + self.node else 0x80
            if not result and self._sequence_valid and not 1 <= (sequence - self.last_sequence) % 256 <= 127:
                result = 0x82
            if not result:
                self.last_sequence, self._sequence_valid = sequence, True
            self._reply(opcode, sequence, result)
            self._frames.append(Frame(protocol.LIMITS_INFO_BASE + self.node,
                struct.pack('<BBBBHH', 0xA1, opcode, sequence, result,
                            round(self.iq_limit_max_a * 1000), round(self.phase_trip_a * 1000))))
            return
        if opcode in (protocol.Opcode.CALIBRATION_READ, protocol.Opcode.CALIBRATION_SAVE,
                      protocol.Opcode.CALIBRATION_CLEAR):
            save = opcode == protocol.Opcode.CALIBRATION_SAVE
            clear = opcode == protocol.Opcode.CALIBRATION_CLEAR
            valid = (data[3:] == bytes(5) if opcode == protocol.Opcode.CALIBRATION_READ else
                     data[3] == 0 and data[4:] == (b'CAL1' if save else b'CLR1'))
            result = 0 if valid else 0x80
            if not result and self._sequence_valid and not 1 <= (sequence - self.last_sequence) % 256 <= 127:
                result = 0x82
            if not result and (save or clear):
                if self.driver_enabled or self.mode not in (0, 5):
                    result = 5
                elif self.require_stop:
                    result = 0x84
                elif save and not self.offset_ready:
                    result = 4
                elif save and not self.sensor_valid:
                    result = 9
                elif save and not (self.direction_confirmed and self.zero_valid):
                    result = 8
            if not result and save:
                self._node_store.update(calibration_valid=True, calibration_error=False,
                    calibration_direction=self.direction,
                    calibration_zero=self.electrical_zero_rad)
            elif not result and clear:
                self._node_store.update(calibration_valid=False, calibration_error=False,
                    calibration_direction=1, calibration_zero=0.0)
            self._reply(opcode, sequence, result)
            stored = bool(self._node_store.get('calibration_valid'))
            flags = (int(stored) |
                     (2 if self._node_store.get('calibration_error') else 0) |
                     (8 if self.zero_valid else 0) |
                     (16 if not result and (save or clear) else 0))
            zero = max(0, min(6283, round(float(self._node_store.get('calibration_zero', 0.0)) * 1000))) if stored else 0
            direction = int(self._node_store.get('calibration_direction', 0)) if stored else 0
            self._frames.append(Frame(protocol.CALIBRATION_INFO_BASE + self.node,
                struct.pack('<BBBBBbH', 0xA1, opcode, sequence, result, flags, direction, zero)))
            if not result and (save or clear):
                self._stop()
                self.require_stop = True
                self._sequence_valid = self.direction_confirmed = self.zero_valid = self.offset_ready = False
                self._calibration_ends = now + .512
                self.parameters = {key: _float32(value) for key, value in self.DEFAULT_PARAMETERS.items()}
            elif not result:
                self.last_sequence, self._sequence_valid = sequence, True
            return
        if opcode in (protocol.Opcode.NODE_READ, protocol.Opcode.NODE_SET):
            is_set = opcode == protocol.Opcode.NODE_SET
            valid = (data[4:] == b'CAN1' and 1 <= data[3] <= 63) if is_set else data[3:] == bytes(5)
            result = 0 if valid else 0x80
            if is_set and not result:
                if self.faults:
                    result = 3
                elif self.driver_enabled or self.mode != 0:
                    result = 5
                elif self.require_stop:
                    result = 0x84
            old_node = self.node
            if not result and is_set:
                self._node_store.update(node=data[3], valid=True, error=False)
            self._reply(opcode, sequence, result)
            flags = int(self._node_store['valid']) | (2 if self._node_store.get('error') else 0) | (4 if is_set and not result else 0)
            self._frames.append(Frame(protocol.NODE_INFO_BASE + old_node,
                bytes((0xA1, opcode, sequence, result, old_node, self._node_store['node'], 1, flags))))
            if is_set and not result:
                self.node = self._node_store['node']
                self._stop()
                self.require_stop = True
                self._sequence_valid = self.direction_confirmed = self.zero_valid = self.offset_ready = False
                self._calibration_ends = now + .512
                self.parameters = {key: _float32(value) for key, value in self.DEFAULT_PARAMETERS.items()}
            elif not result:
                self.last_sequence, self._sequence_valid = sequence, True
            return
        value = auxiliary = 0.0
        malformed, out_of_range = False, False
        if opcode in (0x10, 0x11):
            value = struct.unpack("<f", data[4:])[0]
            out_of_range = parameter not in protocol.PARAMETERS or not math.isfinite(value)
            malformed = opcode == 0x10 and value != 0.0
        else:
            target, wire_aux = struct.unpack("<hH", data[4:])
            malformed = data[3] != int(opcode in (1, 2, 3, 4)) or (opcode != 3 and wire_aux != 0)
            if opcode == 1:
                value, out_of_range = target / 1000.0, abs(target) > round(self.iq_limit_max_a * 1000)
            elif opcode == 2:
                value, out_of_range = target / 1000.0, abs(target) > 30000
            elif opcode == 3:
                value, auxiliary = target / 100.0, wire_aux / 1000.0
                out_of_range = abs(target) > 10000 or wire_aux > 2000
            elif opcode == 6:
                value, out_of_range = target, target not in (-1, 1)
            elif opcode not in (0, 4, 5, 7, 0x7F) or target != 0:
                malformed = True
        if frame.arbitration_id == 0x080 and opcode != 0x7F:
            malformed = True
        if malformed or out_of_range:
            self._reply(opcode, sequence, 0x81 if out_of_range else 0x80, parameter)
            return
        safety = opcode in (0, 0x7F)
        if not safety and self._sequence_valid and not 1 <= (sequence - self.last_sequence) % 256 <= 127:
            self._reply(opcode, sequence, 0x82, parameter)
            return
        if opcode in (1, 2, 3, 4) and self.require_stop:
            self._reply(opcode, sequence, 0x84, parameter)
            return
        if opcode in (1, 2, 3, 4) and self._node_store.get('error') and not self._node_store['valid']:
            self._reply(opcode, sequence, 0x86, parameter)
            return
        self.last_sequence, self._sequence_valid = sequence, True
        result = self._execute(opcode, value, auxiliary, now, parameter)
        self._reply(opcode, sequence, result, parameter)

    def recv(self, timeout=0.005):
        if self._closed:
            return None
        deadline = self._clock() + max(0.0, timeout or 0.0)
        while True:
            now = self._clock()
            self._advance(now)
            if self._frames:
                return self._frames.popleft()
            if now >= deadline:
                return None
            self._sleep(min(deadline - now, max(0.0001, min(self._next_feedback, self._next_fast) - now)))

    def shutdown(self):
        self._stop()
        self._frames.clear()
        self._closed = True


class PythonCanTransport:
    """Lazy python-can wrapper; construction is the only adapter-open action."""
    simulation_only = False

    def __init__(self, config):
        node = _node(config)
        try:
            self._can = importlib.import_module("can")
        except ImportError as error:
            raise RuntimeError("Real CAN requires python-can; simulator needs only Python's standard library") from error
        channel = config.get("channel")
        if channel is None or channel == "":
            raise ValueError("a CAN adapter channel is required")
        if config["interface"] == "canalystii" and str(channel).isdigit():
            channel = int(channel)
        bitrate = int(config.get("bitrate", 1_000_000))
        if bitrate not in (125000, 250000, 500000, 1000000):
            raise ValueError("bitrate must be 125000, 250000, 500000 or 1000000")
        filters = feedback_filters(node)
        self._closed = False
        self._bus = self._can.Bus(interface=config["interface"], channel=channel,
            bitrate=bitrate, receive_own_messages=False, can_filters=filters, ignore_config=True)
        # A new PCAN session must not inherit disabled reception. Do not reset
        # the hardware or change controller mode during ongoing reception.
        if config["interface"] == "pcan":
            try:
                from can.interfaces.pcan import basic
                api, handle = self._bus.m_objPCANBasic, self._bus.m_PcanHandle
                code = api.SetValue(handle, basic.PCAN_RECEIVE_STATUS, basic.PCAN_PARAMETER_ON)
                if int(code):
                    raise RuntimeError(f"PCAN 接收启用失败：0x{int(code):X}")
            except (ImportError, AttributeError):
                # An injected test/alternate backend may lack the PCAN API.
                pass
            except Exception:
                self._bus.shutdown()
                self._closed = True
                raise

    def send(self, frame, timeout=0.01):
        if self._closed:
            raise RuntimeError("CAN transport is closed")
        message = self._can.Message(arbitration_id=frame.arbitration_id, data=frame.data,
            is_extended_id=frame.is_extended_id, is_remote_frame=frame.is_remote_frame,
            is_error_frame=frame.is_error_frame, check=True)
        self._bus.send(message, timeout=timeout)

    def recv(self, timeout=0.005):
        if self._closed:
            return None
        message = self._bus.recv(timeout=timeout)
        if message is None:
            return None
        return Frame(message.arbitration_id, bytes(message.data), message.is_extended_id,
                     message.is_remote_frame, message.is_error_frame)

    def shutdown(self):
        if not self._closed:
            self._closed = True
            self._bus.shutdown()


def create_transport(config=None):
    config = dict(config or {})
    interface = config.get("interface", "simulator")
    if interface == "simulator":
        return SimulatorTransport(config)
    if interface not in ("pcan", "slcan", "socketcan", "usb2can", "canalystii"):
        raise ValueError("unsupported CAN interface: %s" % interface)
    config["interface"] = interface
    return PythonCanTransport(config)
