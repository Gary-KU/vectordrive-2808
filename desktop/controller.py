"""One CAN owner thread; GUI never performs blocking I/O or renews motion itself."""
from __future__ import annotations

from collections import deque
import copy
import math
import queue
import struct
import threading
import time

from examples import gimbal_can as protocol
from .transports import Frame, create_transport, decode_bus_health_frame
from .commands import CommandType, HELP_TEXT, parse_command
from .telemetry import can_feedback_error, encoder_readout
from .recovery import restart_pcan_device
from .current_limits import current_limit_readback, validate_current_target


class ControlError(RuntimeError):
    pass


class Cancelled(ControlError):
    pass


class CommunicationError(ControlError):
    pass


class CommandTimeout(CommunicationError):
    pass


class CommandRejected(ControlError):
    def __init__(self, opcode, reply):
        self.opcode, self.result = int(opcode), reply['result']
        super().__init__(f"命令 0x{int(opcode):02X} 被拒绝：{reply['result_name']}")


class Controller:
    PERIOD = 0.010
    ACK_TIMEOUT = 0.040
    FEEDBACK_TIMEOUT = 0.050
    UI_TIMEOUT = 0.500
    MAX_RUN_SECONDS = 60.0
    CONNECT_ATTEMPTS = 3
    CONNECT_RETRY_DELAY = .4
    AUTO_RETRY_DELAYS = (.25, 2.0, 5.0, 10.0)
    DEVICE_RESET_COOLDOWN = 60.0
    DEVICE_RESET_WINDOW = 300.0
    DEVICE_RESET_LIMIT = 3
    MOTION_QUIET_SECONDS = .2  # MCU CAN watchdog is 100 ms.

    def __init__(self, transport_factory=create_transport, *, device_recovery=restart_pcan_device,
                 auto_reconnect=True):
        self._factory = transport_factory
        self._device_recovery = device_recovery
        self._auto_reconnect = auto_reconnect
        self._auto_target = None
        self._auto_due = None
        self._auto_failures = 0
        self._device_resets = deque()
        self._last_motion_send = float('-inf')
        self._last_node_receive = float('-inf')
        self._last_bus_receive = float('-inf')
        self._seen_bus_nodes = {}
        self._connection_config = None
        self._sim_node_store = {}
        self._lock = threading.RLock()
        self._events = deque(maxlen=10000)
        self._commands = queue.Queue(maxsize=64)
        self._stop = threading.Event()
        self._estop = threading.Event()
        self._closing = threading.Event()
        self._recovery_cancel = threading.Event()
        self._disconnect_requested = threading.Event()
        self._ui_tick = time.monotonic()
        self._state = dict(connected=False, interface="", session_ready=False,
                           running=False, mode="DISCONNECTED", status={}, diagnostic={},
                           telemetry={}, fast={}, parameters={}, received={}, error="", stop_confirmed=False,
                           tuning_busy=False, recovering=False, connecting=False, connection_stage="",
                           rx_frames=0, tx_frames=0, reconnect_pending=False,
                           auto_reconnect_enabled=False, recovery_count=0, device_reset_count=0,
                           node=1, node_info={}, limits_info={}, calibration_info={},
                           node_changing=False)
        self._bus = None
        self._node = 1
        self._sequence = 0
        self._pending = None
        self._ack = None
        self._param = None
        self._node_reply = None
        self._limits_reply = None
        self._calibration_reply = None
        self._current_limit_revision = 0
        self._status_time = self._diag_time = 0.0
        self._status_count = self._diag_count = 0
        self._rx_count = self._tx_count = 0
        self._align_seen = False
        self._motion = None
        self._motion_started = self._next_send = self._first_ack = 0.0
        self._thread = threading.Thread(target=self._worker, name="gimbal-can-owner", daemon=True)
        self._thread.start()

    def _update(self, **values):
        with self._lock:
            self._state.update(values)

    def _invalidate_current_limit(self):
        with self._lock:
            self._state['parameters'].pop(3, None)
            self._current_limit_revision = getattr(self, '_current_limit_revision', 0) + 1

    def _validate_iq(self, value):
        try:
            # Always encode within the host +/-3.3 A wire range. If parameter 3
            # has actually been read back, refuse a target above that real board
            # limit and name it instead of letting the board answer RANGE.
            with self._lock:
                parameters = dict(self._state['parameters'])
            return validate_current_target(value, parameters, require_readback=False)
        except ValueError as error:
            raise ControlError(str(error)) from error

    def _event(self, kind, **values):
        with self._lock:
            self._events.append(dict(type=kind, time=time.time(), **values))

    def _log(self, message):
        self._event("log", message=message)

    def snapshot(self):
        # A blocked/closed GUI must not leave the independent worker streaming forever.
        self._ui_tick = time.monotonic()
        with self._lock:
            return copy.deepcopy(self._state)

    def events(self):
        with self._lock:
            result = list(self._events)
            self._events.clear()
            return result

    def _queue(self, action, value=None):
        if self._closing.is_set():
            return
        with self._lock:
            if any(self._state.get(key) for key in ("recovering", "connecting", "reconnect_pending", "node_changing")):
                self._log("正在连接或恢复通信；请等待完成后重新提交操作。")
                return
            try:
                self._commands.put_nowait((action, value))
            except queue.Full:
                self._log("命令队列已满；请等待上一条命令完成。")

    def connect(self, config):
        self._queue("connect", dict(config))

    def recover(self, config):
        self._queue("recover", dict(config))

    def read_node(self):
        self._queue('read_node')

    def read_calibration(self):
        self._queue('read_calibration')

    def save_calibration(self):
        self._queue('save_calibration')

    def clear_calibration(self):
        self._queue('clear_calibration')

    def change_node(self, new_node, *, single_board=False):
        if not single_board:
            raise ControlError('修改 ID 时请只连接一块驱动板，并勾选单板确认。')
        protocol.pack_node_set(1, 0, new_node)
        self._queue('change_node', new_node)

    def disconnect(self):
        with self._lock:
            self._cancel_auto_reconnect()
            self.stop()
            self._disconnect_requested.set()

    def submit(self, text):
        text = text.strip()
        if text.lower() == "stop":
            self.stop()
        elif text.lower() == "estop":
            self.estop()
        else:
            self._queue("command", text)

    def stop(self):
        with self._lock:
            self._discard_commands()
            self._stop.set()
            if any(self._state.get(key) for key in ("recovering", "connecting", "reconnect_pending", "node_changing")):
                self._recovery_cancel.set()
                self._cancel_auto_reconnect()

    def estop(self):
        with self._lock:
            self._discard_commands()
            self._estop.set()
            self._stop.set()
            if any(self._state.get(key) for key in ("recovering", "connecting", "reconnect_pending", "node_changing")):
                self._recovery_cancel.set()
                self._cancel_auto_reconnect()

    def _take_stop(self):
        # Do not clear an ESTOP posted between reading and clearing the events.
        # The GUI and owner thread share this lock for the whole latch operation.
        with self._lock:
            pending, emergency = self._stop.is_set(), self._estop.is_set()
            self._stop.clear()
            self._estop.clear()
            return pending, emergency

    def set_parameter(self, param_id, value):
        self._queue("set", (param_id, value))

    def set_parameters(self, values):
        """Queue a stopped-only batch. Each item needs its own ACK and readback.

        This is an ordered batch, not a transaction: an interruption preserves
        already applied settings and reports exactly how many were confirmed.
        """
        self._queue("set_many", dict(values))

    def read_parameters(self):
        self._queue("read")

    def read_limits(self):
        """Queue a read-only capability/protection query; never raise Iq limits."""
        self._queue('read_limits')

    def close(self):
        self.stop()
        self._closing.set()

    def wait_closed(self, timeout=1.0):
        self._thread.join(timeout)
        return not self._thread.is_alive()

    def _discard_commands(self):
        while True:
            try:
                self._commands.get_nowait()
            except queue.Empty:
                return

    def _cancel_check(self):
        recovery_cancelled = getattr(self, "_recovery_cancel", None)
        if self._stop.is_set() or self._closing.is_set() or (recovery_cancelled and recovery_cancelled.is_set()):
            raise Cancelled("停止请求已接收")

    def _receive(self, timeout=0.0):
        try:
            frame = self._bus.recv(timeout=max(0.0, timeout))
        except Exception as error:
            raise CommunicationError(f"CAN 接收失败：{error}") from error
        if frame is None:
            return
        self._rx_count = getattr(self, "_rx_count", 0) + 1
        self._update(rx_frames=self._rx_count)
        self._event("frame", direction="RX", id=frame.arbitration_id,
                    data=bytes(frame.data).hex(" ").upper())
        if frame.is_error_frame:
            raise CommunicationError("CAN 适配器报告错误帧")
        bus_feedback = decode_bus_health_frame(frame)
        if bus_feedback is not None:
            now = time.monotonic()
            self._last_bus_receive = now
            self._seen_bus_nodes[bus_feedback['node']] = now
        if frame.is_extended_id or frame.is_remote_frame:
            return
        if frame.arbitration_id not in [base + self._node for base in
                (0x180, 0x280, 0x380, 0x480, 0x580, 0x600, 0x640, 0x680, 0x700)]:
            return
        try:
            feedback = protocol.decode_feedback(frame.arbitration_id, frame.data, self._node)
        except ValueError as error:
            raise ControlError("收到格式错误的节点反馈") from error
        now = time.monotonic()
        self._last_node_receive = now
        kind = feedback["type"]
        if kind == "ack":
            if self._pending and (feedback["opcode"], feedback["sequence"]) == self._pending[:2]:
                self._ack = feedback
        elif kind == "parameter":
            if self._pending and feedback["sequence"] == self._pending[1] and feedback["param_id"] == self._pending[2]:
                self._param = feedback
        elif kind == 'node_info':
            if self._pending and (feedback['opcode'], feedback['sequence']) == self._pending[:2]:
                self._node_reply = feedback
        elif kind == 'limits_info':
            if self._pending and (feedback['opcode'], feedback['sequence']) == self._pending[:2]:
                self._limits_reply = feedback
        elif kind == 'calibration_info':
            if self._pending and (feedback['opcode'], feedback['sequence']) == self._pending[:2]:
                self._calibration_reply = feedback
        elif kind == 'fast':
            with self._lock:
                self._state['fast'] = feedback
                self._state['received']['fast'] = now
        elif kind == "status":
            self._status_time = now
            self._status_count += 1
            self._update(status=feedback, mode=feedback["mode_name"])
            with self._lock:
                self._state["received"]["status"] = now
            if feedback["mode"] == 2 and feedback["driver_enabled"] and not feedback["zero_valid"]:
                self._align_seen = True
            with self._lock:
                trace = dict(self._state["telemetry"])
                encoder = encoder_readout(self._state, now)
            self._event("sample", timestamp=now, iq_a=feedback["iq_a"],
                        velocity_rad_s=feedback["velocity_rad_s"],
                        angle_rad=encoder.angle_rad, angle_deg=encoder.angle_deg,
                        rpm=encoder.rpm, angle_valid=encoder.valid, angle_age_ms=encoder.age_ms,
                        **{key: trace.get(key) for key in ("id_a", "iq_reference_a", "vd_v", "vq_v")})
        elif kind == "diagnostic":
            if feedback.get('require_stop') or feedback.get('not_initialized'):
                self._invalidate_current_limit()
                self._update(limits_info={})
            self._diag_time = now
            self._diag_count += 1
            self._update(diagnostic=feedback)
            with self._lock:
                self._state["received"]["diagnostic"] = now
        elif kind == "trace":
            with self._lock:
                self._state["telemetry"].update(feedback)
                self._state["received"][f"trace_{feedback['page']}"] = now

    def _exchange(self, opcode, value=0, auxiliary=0, param_id=None, safety=False):
        if self._bus is None:
            raise ControlError("请先连接 CAN 或模拟节点")
        if int(opcode) == protocol.Opcode.IQ:
            self._validate_iq(value)
        sequence = self._sequence
        self._sequence = (sequence + 1) & 255
        node_command = int(opcode) in (protocol.Opcode.NODE_READ, protocol.Opcode.NODE_SET)
        limits_command = int(opcode) == protocol.Opcode.LIMITS_READ
        calibration_command = int(opcode) in (protocol.Opcode.CALIBRATION_READ,
            protocol.Opcode.CALIBRATION_SAVE, protocol.Opcode.CALIBRATION_CLEAR)
        if int(opcode) == protocol.Opcode.NODE_READ:
            identifier, data = protocol.pack_node_read(self._node, sequence)
        elif int(opcode) == protocol.Opcode.NODE_SET:
            identifier, data = protocol.pack_node_set(self._node, sequence, value)
        elif limits_command:
            identifier, data = protocol.pack_limits_read(self._node, sequence)
        elif int(opcode) == protocol.Opcode.CALIBRATION_READ:
            identifier, data = protocol.pack_calibration_read(self._node, sequence)
        elif int(opcode) == protocol.Opcode.CALIBRATION_SAVE:
            identifier, data = protocol.pack_calibration_save(self._node, sequence)
        elif int(opcode) == protocol.Opcode.CALIBRATION_CLEAR:
            identifier, data = protocol.pack_calibration_clear(self._node, sequence)
        elif param_id is not None:
            pack = protocol.pack_param_read if int(opcode) == 0x10 else protocol.pack_param_write
            args = (self._node, sequence, param_id) if int(opcode) == 0x10 else (self._node, sequence, param_id, value)
            identifier, data = pack(*args)
        else:
            identifier, data = protocol.pack_command(self._node, int(opcode), sequence, value, auxiliary)
        if not safety:
            self._cancel_check()
        previous_limit = None
        if param_id == 3 and int(opcode) == protocol.Opcode.PARAM_WRITE:
            with self._lock:
                previous_limit = current_limit_readback(self._state['parameters'])
            self._invalidate_current_limit()
        limit_revision = getattr(self, '_current_limit_revision', 0)
        self._pending = (int(opcode), sequence, param_id)
        self._ack = self._param = None
        self._node_reply = None
        self._limits_reply = None
        self._calibration_reply = None
        terminal_store = int(opcode) in (protocol.Opcode.CALIBRATION_SAVE,
                                         protocol.Opcode.CALIBRATION_CLEAR)
        deadline = time.monotonic() + (1.5 if int(opcode) == protocol.Opcode.NODE_SET or terminal_store
                                       else .15 if safety else self.ACK_TIMEOUT)
        try:
            # Record before send: an adapter may accept a motion frame then
            # raise, so recovery must still wait out the MCU motion watchdog.
            if int(opcode) in (1, 2, 3, 4):
                self._last_motion_send = time.monotonic()
            try:
                self._bus.send(Frame(identifier, data), timeout=0.010)
            except Exception as error:
                raise CommunicationError(f"CAN 发送失败：{error}") from error
            self._tx_count = getattr(self, "_tx_count", 0) + 1
            self._update(tx_frames=self._tx_count)
            self._event("frame", direction="TX", id=identifier, data=data.hex(" ").upper())
            while time.monotonic() < deadline:
                if not safety:
                    self._cancel_check()
                    if self._motion is not None:
                        self._fresh()
                        if time.monotonic() - self._ui_tick > self.UI_TIMEOUT:
                            raise ControlError("界面心跳超过 500 ms，自动停止")
                if self._ack is not None:
                    if self._ack["result"] != 0:
                        raise CommandRejected(opcode, self._ack)
                    if not safety and not limits_command and self._ack["faults"]:
                        raise ControlError(f"执行 ACK 带故障 0x{self._ack['faults']:08X}")
                    if ((param_id is None or self._param is not None) and
                            (not node_command or self._node_reply is not None) and
                            (not limits_command or self._limits_reply is not None) and
                            (not calibration_command or self._calibration_reply is not None)):
                        break
                self._receive(min(0.003, max(0, deadline - time.monotonic())))
            if self._ack is None:
                hint = ("适配器已打开但没有收到帧；请检查供电、接线、波特率和接收通道。"
                        if not getattr(self, "_rx_count", 0) else
                        "已收到其他反馈，但没有匹配本次序号的执行 ACK。")
                if self._state.get("interface") == "pcan":
                    hint += "停机后可使用“恢复 PCAN”。"
                raise CommandTimeout(f"命令 0x{int(opcode):02X} ACK 超时：未收到节点 {self._node} "
                                     f"的 0x{0x380+self._node:03X} 应答。{hint}")
            if param_id is not None and self._param is None:
                raise CommandTimeout(f"参数 {param_id} 读回超时：已收到执行 ACK，未收到 "
                                     f"0x{0x480+self._node:03X} 参数反馈；未确认读回值。")
            if limits_command:
                if self._limits_reply is None:
                    raise CommandTimeout('保护信息读回超时；此固件可能尚未支持 limits 查询')
                if self._limits_reply['result']:
                    raise ControlError('板端拒绝保护信息查询；保护信息未知')
                self._update(limits_info=dict(self._limits_reply))
                return dict(self._limits_reply)
            if calibration_command:
                if self._calibration_reply is None:
                    raise CommandTimeout('电角度校准信息读回超时；请确认板端固件支持 Flash 校准。')
                if self._calibration_reply['result']:
                    raise ControlError('电角度校准存储操作失败；原保存值未确认。')
                self._update(calibration_info=dict(self._calibration_reply))
                return dict(self._calibration_reply)
            if node_command:
                if self._node_reply is None:
                    raise CommandTimeout('节点信息读回超时；请确认固件支持可调 ID。')
                if self._node_reply['result']:
                    raise ControlError('节点配置存储失败；未确认新 ID。')
                self._update(node_info=dict(self._node_reply))
                return dict(self._node_reply)
            if self._ack["result"] != 0 or (not safety and self._ack["faults"]):
                raise ControlError(f"执行失败：{self._ack['result_name']} / 0x{self._ack['faults']:08X}")
            if param_id is not None:
                if self._param["result"] != 0 or not math.isfinite(self._param["value"]):
                    raise ControlError("参数读回失败")
                if int(opcode) == 0x11:
                    expected = struct.unpack("<f", struct.pack("<f", float(value)))[0]
                    if self._param["value"] != expected:
                        raise ControlError("参数实际读回值与写入值不一致")
                if param_id == 3 and limit_revision != getattr(self, '_current_limit_revision', 0):
                    raise ControlError('CAN 会话已失效，电流限幅读回未确认')
                with self._lock:
                    self._state["parameters"][param_id] = self._param["value"]
                return self._param["value"]
            return self._ack
        except CommandRejected as error:
            # RANGE is an explicit no-write result. Missing replies, malformed
            # readback or session revocation cannot restore a previous limit.
            if (param_id == 3 and int(opcode) == protocol.Opcode.PARAM_WRITE and
                    error.result in (10, 0x81) and previous_limit is not None and
                    limit_revision == getattr(self, '_current_limit_revision', 0)):
                with self._lock:
                    self._state['parameters'][3] = previous_limit
            raise
        finally:
            self._pending = None

    def _stop_motion(self, emergency=False):
        self._motion = None
        self._update(running=False, session_ready=False, stop_confirmed=False)
        if self._bus is None:
            return
        try:
            self._exchange(protocol.Opcode.ESTOP if emergency else protocol.Opcode.STOP, safety=True)
            self._update(stop_confirmed=True, session_ready=not emergency)
            self._log("急停执行已确认；故障已锁存" if emergency else "STOP 执行已确认")
        except Exception as error:
            self._update(error=f"STOP 未确认：{error}")
            self._log(f"STOP 未确认：{error}；固件仍有 100 ms 运动超时保护")

    def _fresh(self):
        now = time.monotonic()
        if now - self._status_time > self.FEEDBACK_TIMEOUT or now - self._diag_time > self.FEEDBACK_TIMEOUT:
            raise CommunicationError("状态/诊断反馈超过 50 ms 未更新")
        with self._lock:
            status, diag = self._state["status"], self._state["diagnostic"]
        if diag.get("faults") or diag.get("board_faults_low8") or diag.get("bus_off") or diag.get("error_passive"):
            raise ControlError(f"节点故障：0x{diag.get('faults', 0):08X}")
        if diag.get("require_stop") or diag.get("not_initialized"):
            raise ControlError("CAN 会话已失效，需 STOP 重新握手")
        return status

    def _check_idle_feedback(self):
        # Motion retains its stricter 50 ms deadline. Stopped sessions must also
        # notice a silent node; recv(None) is not an adapter exception.
        if self._motion is not None:
            return
        with self._lock:
            problem = can_feedback_error(self._state)
        if problem:
            raise CommunicationError(problem)

    def _require_stopped(self):
        if self._bus is None:
            raise ControlError("请先连接")
        with self._lock:
            status = self._state["status"]
        if self._motion is not None or status.get("driver_enabled") or status.get("mode", 0) != 0:
            raise ControlError("请先 STOP，等待驱动停止后再配置参数")

    def _cancel_auto_reconnect(self):
        """Called with the state lock, also from the UI thread on cancellation."""
        self._auto_target = self._auto_due = None
        self._state.update(reconnect_pending=False, auto_reconnect_enabled=False)

    def _remember_connection(self, config):
        with self._lock:
            if self._connection_config is not None and any(
                    self._connection_config.get(key) != config.get(key) for key in ('interface', 'channel', 'bitrate')):
                self._seen_bus_nodes.clear()
                self._last_bus_receive = float('-inf')
            self._connection_config = dict(config)
            enabled = self._auto_reconnect and config.get("interface") == "pcan" and config.get("auto_recover", True)
            self._auto_target = dict(config) if enabled else None
            self._auto_due = None
            self._auto_failures = 0
            self._state.update(auto_reconnect_enabled=bool(enabled), reconnect_pending=False)

    def _schedule_reconnect(self, error):
        with self._lock:
            if self._auto_target is None or self._closing.is_set() or self._disconnect_requested.is_set():
                return
            self._discard_commands()
            delay = self.AUTO_RETRY_DELAYS[min(self._auto_failures, len(self.AUTO_RETRY_DELAYS)-1)]
            self._auto_failures += 1
            self._auto_due = max(time.monotonic() + delay, self._last_motion_send + self.MOTION_QUIET_SECONDS)
            self._state.update(reconnect_pending=True, session_ready=False,
                               connection_stage=f"自动重连 {self._auto_failures} · 等待 {delay:g} 秒")
        self._log(f"通信异常：{error}；{delay:g} 秒后自动恢复，旧运动/写参数请求已丢弃。")

    def _device_reset_allowed(self):
        now = time.monotonic()
        while self._device_resets and now - self._device_resets[0] >= self.DEVICE_RESET_WINDOW:
            self._device_resets.popleft()
        # Receiving valid node traffic means a missing command/page needs
        # diagnosis, not a USB device restart. Limit OS resets when power or
        # wiring remains absent; ordinary reconnect probes can keep waiting.
        return (now - max(self._last_node_receive, getattr(self, '_last_bus_receive', float('-inf'))) > .25 and
                len(self._device_resets) < self.DEVICE_RESET_LIMIT and
                (not self._device_resets or now - self._device_resets[-1] >= self.DEVICE_RESET_COOLDOWN))

    def _restart_device(self, config):
        self._device_resets.append(time.monotonic())
        self._device_recovery(config, self._cancel_check,
                              lambda stage: self._update(connection_stage=stage))
        with self._lock:
            self._state["device_reset_count"] += 1

    def _auto_restore(self):
        with self._lock:
            if self._auto_target is None:
                return
            config = dict(self._auto_target)
            self._auto_due = None
            self._recovery_cancel.clear()
            self._state.update(recovering=True, reconnect_pending=False, connection_stage="自动恢复 · 重新打开 CAN")
        try:
            # No motion producer remains. This wait also covers an unconfirmed
            # STOP after link loss before touching the USB device.
            while time.monotonic() - self._last_motion_send < self.MOTION_QUIET_SECONDS:
                self._cancel_check()
                time.sleep(.01)
            try:
                self._connect(config)
            except CommunicationError:
                self._cancel_check()
                if not self._device_reset_allowed():
                    raise
                self._log("重新打开仍无节点反馈，自动重启 PCAN-USB 后再做 STOP 握手。")
                self._restart_device(config)
                self._cancel_check()
                self._connect(config)
            with self._lock:
                self._auto_failures = 0
                self._state["recovery_count"] += 1
            self._log("通信已自动恢复，STOP 已确认；保持停机，请重新提交目标或参数操作。")
        finally:
            with self._lock:
                self._discard_commands()
                self._recovery_cancel.clear()
                self._state.update(recovering=False, connection_stage="")

    def _connect(self, config):
        attempts = self.CONNECT_ATTEMPTS if config.get("interface") == "pcan" else 1
        with self._lock:
            if not self._state.get("recovering") and not self._state.get('node_changing'):
                self._recovery_cancel.clear()
            self._state["connecting"] = True
            self._discard_commands()
        try:
            for attempt in range(1, attempts + 1):
                self._cancel_check()
                self._update(connection_stage=f"连接 {attempt}/{attempts}")
                try:
                    self._open_session(config)
                    return
                except (Cancelled, ValueError):
                    raise
                except Exception as error:
                    if self._bus is not None:
                        self._disconnect()
                    if attempt == attempts:
                        raise
                    self._log(f"连接 {attempt}/{attempts} 失败：{error}；重新打开适配器并做 STOP 握手")
                    deadline = time.monotonic() + self.CONNECT_RETRY_DELAY
                    while time.monotonic() < deadline:
                        self._cancel_check()
                        time.sleep(.01)
        finally:
            with self._lock:
                self._discard_commands()
                self._state.update(connecting=False, connection_stage="")
                if not self._state.get("recovering") and not self._state.get('node_changing'):
                    self._recovery_cancel.clear()

    def _open_session(self, config):
        if self._bus is not None:
            self._disconnect()
        node = int(config.get("node", 1))
        if not 1 <= node <= 63:
            raise ControlError("节点 ID 必须为 1..63")
        self._node, self._sequence = node, 0
        try:
            factory_config = dict(config, _sim_node_store=self._sim_node_store) if config.get('interface') == 'simulator' else config
            self._bus = self._factory(factory_config)
        except ValueError:
            raise
        except Exception as error:
            raise CommunicationError(f"打开 CAN 适配器失败：{error}") from error
        self._status_time = self._diag_time = 0.0
        self._status_count = self._diag_count = 0
        self._rx_count = self._tx_count = 0
        self._update(connected=True, interface=config.get("interface", "simulator"), error="",
                     status={}, diagnostic={}, telemetry={}, fast={}, parameters={}, received={}, mode="CONNECTING",
                     running=False, session_ready=False, stop_confirmed=False, rx_frames=0, tx_frames=0,
                     node=node, node_info={}, limits_info={}, calibration_info={})
        for _ in range(256):
            try:
                if self._bus.recv(timeout=0) is None:
                    break
            except Exception as error:
                raise CommunicationError(f"清理 CAN 接收队列失败：{error}") from error
        else:
            raise ControlError("适配器旧 RX 队列超过 256 帧")
        # STOP must be usable with a latched board fault. Its ACK reports that
        # fault for diagnosis; refusing the connection would hide CLEAR/telemetry.
        self._exchange(protocol.Opcode.STOP, safety=True)
        counts = self._status_count, self._diag_count
        deadline = time.monotonic() + 0.2
        while time.monotonic() < deadline:
            self._cancel_check()
            self._receive(0.003)
            if self._status_count > counts[0] and self._diag_count > counts[1]:
                with self._lock:
                    status = self._state["status"]
                    ready = (not self._state["diagnostic"].get("require_stop", True) and
                             not status.get("driver_enabled", True) and status.get("mode") in (0, 5))
                if ready:
                    self._update(session_ready=True, stop_confirmed=True)
                    self._log("连接成功，STOP 握手完成。参数不会自动写入，电机不会自动启动。")
                    return
        raise CommunicationError("未收到 STOP 后的新停机状态/诊断反馈")

    def _recover(self, config):
        if config.get("interface") != "pcan":
            raise ControlError("设备恢复仅支持 PCAN；其他适配器请断开后重新连接")
        with self._lock:
            active = self._motion is not None or (self._state["connected"] and
                     self._state["status"].get("driver_enabled"))
            if active:
                raise ControlError("请先停止输出，再恢复 PCAN")
            self._remember_connection(config)
            self._recovery_cancel.clear()
            self._state.update(recovering=True, connection_stage="关闭旧会话", error="")
            self._discard_commands()
        try:
            if self._bus is not None:
                self._disconnect()
            self._cancel_check()
            self._restart_device(config)
            self._cancel_check()
            self._log("PCAN 设备已重启；重新连接并执行 STOP，旧运动和参数写入请求已丢弃。")
            self._connect(config)
            self._log("PCAN 恢复完成，保持停机；需要重新提交运动或参数操作。")
        except BaseException:
            if self._bus is not None:
                self._disconnect()
            raise
        finally:
            with self._lock:
                self._discard_commands()
                self._recovery_cancel.clear()
                self._state.update(recovering=False, connection_stage="")

    def _read_node(self):
        if self._motion is not None or self._state['status'].get('driver_enabled'):
            raise ControlError('请先停止输出，再读取节点配置。')
        try:
            result = self._exchange(protocol.Opcode.NODE_READ, safety=True)
        except CommandRejected as error:
            if error.result in (0x80, 0x87):
                raise ControlError('当前固件不支持节点配置，请烧录支持可调 ID 的版本。') from error
            raise
        self._log(f"板上节点 ID {result['active_node']}；保存 ID {result['stored_node']}；"
                  + ('已掉电保存。' if result['stored_valid'] else '使用出厂默认值。'))
        return result

    def _read_calibration(self):
        if self._motion is not None or self._state['status'].get('driver_enabled'):
            raise ControlError('请先停止输出，再读取电角度校准。')
        try:
            result = self._exchange(protocol.Opcode.CALIBRATION_READ, safety=True)
        except CommandRejected as error:
            if error.result in (0x80, 0x87):
                raise ControlError('当前固件不支持电角度校准 Flash 存储。') from error
            raise
        if result['storage_error']:
            state = 'Flash 记录异常，禁止闭环；可在停机后清除或重新对齐保存'
        elif result['profile_mismatch']:
            state = '保存值与当前电机型号/极对数不匹配，禁止闭环'
        elif result['stored_valid']:
            state = (f"已保存：方向 {result['sensor_direction']:+d}，"
                     f"电角零点 {result['electrical_zero_rad']:.3f} rad")
        else:
            state = '尚未保存；掉电启动会执行一次自动对齐'
        self._log('电角度校准：' + state)
        return result

    def _commit_calibration(self, clear=False):
        self._require_stopped()
        status = self._fresh()
        if not clear and not all(status.get(key) for key in
                ('offset_ready', 'sensor_valid', 'direction_confirmed', 'zero_valid')):
            raise ControlError('保存前必须完成 ADC 偏置、编码器、方向和电角度对齐。')
        old_config = dict(self._connection_config)
        operation = (protocol.Opcode.CALIBRATION_CLEAR if clear else
                     protocol.Opcode.CALIBRATION_SAVE)
        verb = '清除' if clear else '保存'
        self._update(tuning_busy=True, connection_stage=f'{verb}电角度校准并重启')
        try:
            reply = self._exchange(operation, safety=True)
            if clear and reply['stored_valid']:
                raise ControlError('板端应答仍显示存在保存值')
            if not clear and not reply['stored_valid']:
                raise ControlError('板端应答未确认保存值')
            self._close_transport()
            deadline = time.monotonic() + .2
            while time.monotonic() < deadline:
                self._cancel_check()
                time.sleep(.01)
            self._connect(old_config)
            ready_deadline = time.monotonic() + 1.0
            while time.monotonic() < ready_deadline:
                self._cancel_check()
                with self._lock:
                    reboot_status = dict(self._state['status'])
                if reboot_status.get('offset_ready') and reboot_status.get('sensor_valid'):
                    break
                self._receive(.005)
            actual = self._read_calibration()
            if clear:
                if actual['stored_valid']:
                    raise ControlError('重启后仍检测到保存的电角度校准')
                self._log('电角度校准已清除并读回确认；下次真正掉电上电会重新自动对齐。')
            else:
                if not actual['stored_valid'] or actual['profile_mismatch'] or not actual['runtime_valid']:
                    raise ControlError('重启后未恢复有效的电角度校准')
                self._log('电角度校准已写入 Flash；以后上电只做 ADC 偏置，不再吸合对齐。')
            return actual
        finally:
            self._update(tuning_busy=False, connection_stage='')

    def _change_node(self, new_node):
        self._require_stopped()
        self._fresh()
        protocol.pack_node_set(self._node, 0, new_node)
        old_node = self._node
        old_config = dict(self._connection_config)
        set_sent = False
        restored_after_storage_failure = False
        with self._lock:
            self._cancel_auto_reconnect()
            self._recovery_cancel.clear()
            self._discard_commands()
            self._state.update(node_changing=True, connection_stage=f'读取节点 {old_node} 配置')
        try:
            before = self._read_node()
            self._cancel_check()
            if before['storage_error']:
                self._log('板上此前有未完整提交的 ID 记录；本次在停机/单板条件下重新保存并读回确认。')
            if new_node == old_node and before['stored_valid'] and before['stored_node'] == new_node:
                self._remember_connection(old_config)
                self._log('该 ID 已保存，不重复擦写 Flash。')
                return
            # Every healthy node broadcasts status/diagnostic. Listen without
            # sending to another motor. Offline duplicate nodes still require
            # the operator's explicit one-board configuration boundary.
            self._update(connection_stage=f'检查目标 ID {new_node} 是否占用')
            deadline = time.monotonic() + .25
            while time.monotonic() < deadline:
                self._cancel_check()
                self._receive(.005)
            if new_node != old_node and time.monotonic() - self._seen_bus_nodes.get(new_node, float('-inf')) < .5:
                raise ControlError(f'目标 ID {new_node} 已有节点反馈，拒绝修改。')
            self._update(connection_stage=f'保存 ID {new_node}，等待板子停机重启')
            self._cancel_check()
            set_sent = True
            try:
                saved = self._exchange(protocol.Opcode.NODE_SET, new_node)
                if not saved['stored_valid'] or saved['stored_node'] != new_node:
                    raise ControlError('旧 ID 应答的持久存储值不匹配')
            except CommandRejected as error:
                if error.result != 0x86:
                    set_sent = False  # a normal rejection did not commit/reset
                    raise
                self._close_transport()
                deadline = time.monotonic() + .2
                while time.monotonic() < deadline:
                    self._cancel_check()
                    time.sleep(.01)
                self._connect(old_config)
                self._cancel_check()
                self._read_node()
                self._remember_connection(old_config)
                restored_after_storage_failure = True
                raise ControlError(f'节点 ID 保存失败（Flash 存储错误），已重新连接原 ID {old_node}；方向和零点需重新校准。') from error
            except CommunicationError as error:
                # Flash may have committed before its old-ID reply was lost.
                # Never resend SET; resolve by reading the new address.
                self._log(f'旧 ID 回复未完整收到：{error}；仅探测新 ID，不重复写入。')
            self._close_transport()
            self._update(connection_stage=f'以新 ID {new_node} 重新 STOP 握手')
            deadline = time.monotonic() + .2
            while time.monotonic() < deadline:
                self._cancel_check()
                time.sleep(.01)
            new_config = dict(old_config, node=new_node)
            self._connect(new_config)
            self._cancel_check()
            actual = self._read_node()
            if actual['active_node'] != new_node or actual['stored_node'] != new_node or not actual['stored_valid']:
                raise ControlError('新 ID 下未确认实际保存值')
            self._remember_connection(new_config)
            self._event('node_changed', old_node=old_node, node=new_node)
            self._log(f'节点 ID {old_node} → {new_node} 已掉电保存并读回确认；驱动保持关闭，方向和零点需重新校准。')
        except BaseException as error:
            if set_sent and not restored_after_storage_failure:
                self._close_transport()
                with self._lock:
                    self._cancel_auto_reconnect()
                self._log(f'改 ID 结果未确认：旧 ID {old_node}，目标 ID {new_node}。未重发写入；请分别查询两地址。')
                if isinstance(error, CommunicationError):
                    raise ControlError(f'改 ID 结果未确认（{old_node} → {new_node}）：{error}') from error
            elif not isinstance(error, Cancelled):
                self._remember_connection(old_config)
            raise
        finally:
            with self._lock:
                self._discard_commands()
                self._recovery_cancel.clear()
                self._state.update(node_changing=False, connection_stage='')

    def _close_transport(self):
        """Close an already stopped/configuring link without sending old-ID STOP."""
        if self._bus is not None:
            try:
                self._bus.shutdown()
            except Exception as error:
                self._log(f"关闭旧 CAN 会话时设备已异常：{error}")
            finally:
                self._bus = None
        self._update(connected=False, session_ready=False, running=False, mode="DISCONNECTED", limits_info={})

    def _disconnect(self):
        if self._bus:
            _, emergency = self._take_stop()
            self._stop_motion(emergency)
            # Preserve a higher-priority request arriving during the STOP ACK wait.
            _, late_emergency = self._take_stop()
            if late_emergency:
                self._stop_motion(True)
        self._close_transport()

    def _start_motion(self, opcode, value=0, auxiliary=0):
        status = self._fresh()
        expected = {1: 3, 2: 4, 3: 1, 4: 2}[int(opcode)]
        if self._motion is not None and int(self._motion[0]) != int(opcode):
            raise ControlError("切换控制模式前必须先 STOP")
        if not status.get("offset_ready"):
            raise ControlError("ADC 零电流偏置尚未就绪")
        if int(opcode) in (1, 2, 4):
            if not status.get("sensor_valid") or not status.get("direction_confirmed"):
                raise ControlError("需要有效编码器和已确认方向（dir 1 / dir -1）")
        if int(opcode) in (1, 2):
            if not status.get("sense_confirmed") or not status.get("zero_valid"):
                raise ControlError("需要有效电流采样和已完成 align")
        if int(opcode) == 4 and status.get("zero_valid"):
            self._log("现有电气零点有效；如需重新对齐，先设置 dir 使旧零点失效。")
            return
        protocol.pack_command(self._node, int(opcode), self._sequence, value, auxiliary)
        if int(opcode) == protocol.Opcode.IQ:
            self._validate_iq(value)
        first = self._motion is None
        if first:
            self._motion_started = time.monotonic()
            self._first_ack = 0.0
            self._align_seen = False
        self._motion = (opcode, value, auxiliary, expected)
        self._next_send = time.monotonic()
        self._update(running=True, stop_confirmed=False, error="")
        self._log(f"持续 CAN 命令：{opcode.name} {value} {auxiliary}（最长 60 秒）")

    def _tick_motion(self):
        if self._motion is None:
            return
        now = time.monotonic()
        if now - self._ui_tick > self.UI_TIMEOUT:
            raise ControlError("界面心跳超过 500 ms，自动停止")
        status = self._fresh()
        opcode, value, auxiliary, expected = self._motion
        if int(opcode) == 4:
            if self._align_seen and status.get("zero_valid") and status.get("mode") == 0 and not status.get("driver_enabled"):
                self._stop_motion()
                self._log("对齐完成；电气零点有效")
                return
            if now - self._motion_started > 6.5:
                raise ControlError("对齐超时")
        else:
            if now - self._motion_started >= self.MAX_RUN_SECONDS:
                self._stop_motion()
                self._log("已到单次运行 60 秒上限")
                return
            if self._first_ack and self._status_time > self._first_ack:
                if status.get("mode") != expected or not status.get("driver_enabled") or status.get("command_timed_out"):
                    raise ControlError("固件离开请求的运行模式")
            if int(opcode) in (1, 2) and not all(status.get(key) for key in
                    ("sensor_valid", "sense_confirmed", "offset_ready", "direction_confirmed", "zero_valid")):
                raise ControlError("电流闭环前置条件失效")
        if now >= self._next_send:
            if now - self._next_send > 0.035:
                raise ControlError("CAN 命令刷新调度超时")
            self._exchange(opcode, value, auxiliary)
            if not self._first_ack:
                self._first_ack = time.monotonic()
            self._next_send = now + (0.05 if int(opcode) == 4 else self.PERIOD)

    def _parameter(self, param_id, value=None):
        self._require_stopped()
        result = self._exchange(0x10 if value is None else 0x11, value or 0, param_id=int(param_id))
        self._log(f"参数 {param_id} 实际读回 = {result:g}" + ("（仅 RAM，掉电恢复默认）" if value is not None else ""))

    def _read_limits(self):
        if self._bus is None:
            raise ControlError('请先连接，再读取板端保护信息')
        if self._motion is not None:
            raise ControlError('请先 STOP，再读取板端保护信息')
        with self._lock:
            was_busy = self._state.get('tuning_busy', False)
        self._update(tuning_busy=True, limits_info={})
        try:
            result = self._exchange(protocol.Opcode.LIMITS_READ)
            self._log(f"板端目标上限 ±{result['request_max_a']:g} A；"
                      f"独立相电流保护 {result['phase_trip_a']:g} A（只读，不修改实际 Iq 限幅）")
            return result
        except Cancelled:
            raise
        except (CommandRejected, CommandTimeout) as error:
            # Old firmware may reject an unknown opcode or ignore the query.
            # Keep its ordinary parameter/feedback session usable.
            self._log(f'板端保护信息未知：{error}；实际限流由板端执行 ACK 和保护判断')
            return None
        except CommunicationError:
            raise
        except ControlError as error:
            self._log(f'板端保护信息未知：{error}；实际限流由板端执行 ACK 和保护判断')
            return None
        finally:
            self._update(tuning_busy=was_busy)

    def _write_parameters(self, requested):
        self._require_stopped()
        # Validate every requested value before sending the first write. The
        # firmware still checks runtime constraints and fixed trip thresholds.
        for number, value in requested.items():
            protocol.pack_param_write(self._node, self._sequence, number, value)
        if not requested:
            self._log("没有需要写入的参数")
            return
        completed = 0
        self._update(tuning_busy=True, error="")
        try:
            for number, value in requested.items():
                self._cancel_check()
                self._parameter(number, value)
                completed += 1
        except Cancelled:
            self._log(f"写入已取消：{completed}/{len(requested)} 项已确认，已生效项保留在 RAM")
            raise
        except CommunicationError as error:
            raise type(error)(f"写入中止：{completed}/{len(requested)} 项已确认；{error}") from error
        except Exception as error:
            raise ControlError(f"写入中止：{completed}/{len(requested)} 项已确认；{error}") from error
        finally:
            self._update(tuning_busy=False)
        self._log(f"写入完成：{completed} 项均已执行并读回确认（RAM）")

    def _command(self, text):
        self._log("> " + text)
        command = parse_command(text)
        if command.kind == CommandType.READ_NODE:
            self._read_node()
            return
        if command.kind == CommandType.CHANGE_NODE:
            self._change_node(int(command.value))
            return
        if command.kind == CommandType.HELP:
            self._log(HELP_TEXT)
            return
        if command.kind == CommandType.STATUS:
            with self._lock:
                self._log(str(self._state))
            return
        if self._bus is None:
            raise ControlError("请先连接 CAN 或模拟模式")
        if command.kind == CommandType.READ_LIMITS:
            self._read_limits()
            return
        if command.kind == CommandType.READ_CALIBRATION:
            self._read_calibration()
            return
        if command.kind == CommandType.SAVE_CALIBRATION:
            self._commit_calibration(False)
            return
        if command.kind == CommandType.CLEAR_CALIBRATION:
            self._commit_calibration(True)
            return
        if command.kind in (CommandType.STOP, CommandType.ESTOP):
            self._stop_motion(command.kind == CommandType.ESTOP)
            return
        if command.kind == CommandType.READ_PARAMETERS:
            self._require_stopped()
            self._update(tuning_busy=True, error="")
            try:
                for number in command.parameter_ids:
                    self._cancel_check()
                    self._parameter(number)
                if command.parameter_ids == tuple(protocol.PARAMETERS):
                    self._read_limits()
            finally:
                self._update(tuning_busy=False)
            return
        if command.kind == CommandType.WRITE_PARAMETER:
            self._write_parameters({command.parameter_ids[0]: command.value})
            return
        if command.kind == CommandType.MOTION:
            self._start_motion(command.opcode, command.value, command.auxiliary)
            return
        if command.kind == CommandType.DIRECTION:
            self._require_stopped()
            self._exchange(command.opcode, command.value)
            return
        if command.kind == CommandType.MAINTENANCE:
            if self._motion is not None:
                raise ControlError("请先 STOP")
            self._exchange(command.opcode, safety=command.opcode == protocol.Opcode.CLEAR)
            return
        raise ControlError("命令格式错误；输入 help 查看")

    def _worker(self):
        try:
            while not self._closing.is_set():
                action = None
                try:
                    if self._disconnect_requested.is_set():
                        self._disconnect_requested.clear()
                        self._disconnect()
                        continue
                    pending_stop, emergency = self._take_stop()
                    if pending_stop:
                        self._stop_motion(emergency)
                    if self._auto_due is not None and time.monotonic() >= self._auto_due:
                        action = "auto"
                        self._auto_restore()
                        continue
                    try:
                        action, value = self._commands.get_nowait()
                    except queue.Empty:
                        action, value = None, None
                    if action == "connect":
                        self._remember_connection(value)
                        self._connect(value)
                    elif action == "recover":
                        self._recover(value)
                    elif action == "disconnect":
                        self._disconnect()
                    elif action == "command":
                        self._command(value)
                    elif action == "set":
                        self._write_parameters({value[0]: value[1]})
                    elif action == "set_many":
                        self._write_parameters(value)
                    elif action == "read":
                        self._command("get")
                    elif action == 'read_limits':
                        self._read_limits()
                    elif action == 'read_node':
                        self._read_node()
                    elif action == 'change_node':
                        self._change_node(value)
                    elif action == 'read_calibration':
                        self._read_calibration()
                    elif action == 'save_calibration':
                        self._commit_calibration(False)
                    elif action == 'clear_calibration':
                        self._commit_calibration(True)
                    if self._bus:
                        self._receive(0.001)
                        self._tick_motion()
                        self._check_idle_feedback()
                    else:
                        time.sleep(0.005)
                except Cancelled:
                    if action == "recover":
                        self._log("PCAN 恢复已取消，保持停机。")
                    continue
                except Exception as error:
                    message = str(error)
                    was_running = self._motion is not None
                    with self._lock:
                        status = self._state["status"]
                        local_active = status.get("driver_enabled") and not status.get("can_control") and not was_running
                        if local_active:
                            self._cancel_auto_reconnect()
                    communication_failure = isinstance(error, CommunicationError)
                    self._update(error=message)
                    self._log("错误：" + message)
                    if was_running:
                        self._discard_commands()
                        self._stop_motion()
                    if communication_failure or action in ("connect", "recover", "auto"):
                        self._discard_commands()
                        self._disconnect()
                    # A broken transport must not hot-loop while stopped.
                    if action is None and self._bus and not was_running:
                        self._discard_commands()
                        self._disconnect()
                    if not was_running or communication_failure:
                        # A failed STOP during disconnect must not hide the
                        # original feedback/adapter failure. Its own result is
                        # retained in stop_confirmed and the event log.
                        self._update(error=message)
                    if communication_failure or action == "auto":
                        self._schedule_reconnect(message)
        finally:
            self._disconnect()
