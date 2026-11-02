"""Bounded CAN Iq test for this repository's A1 protocol and +/-1 A limits.

This script DOES transmit when launched with an adapter. It never sends CLEAR,
OPEN, or ESTOP automatically. It accepts only +/-1.000 A, streams fresh sequence
numbers every 10 ms, and attempts one bounded STOP exchange on every exit.
An adapter send succeeding is not an application execution ACK.
Iq is nominal peak phase current, not torque in Nm. A steady 1 A test needs a
load; current unloaded verification supports only short current pulses.

First run (replace interface/channel with the actual python-can backend)::

    python examples/run_can_current.py --interface pcan --channel PCAN_USBBUS1 \
        --direction +1 --iq 0.05 --duration 2 --align

Later, only while the existing calibration is still valid::

    python examples/run_can_current.py --interface pcan --channel PCAN_USBBUS1 \
        --direction +1 --iq 0.05 --duration 2 --reuse-alignment

DIRECTION invalidates electrical zero. Without --align or --reuse-alignment,
the script sends DIRECTION, reports NEED_ZERO, and stops. --align explicitly
authorizes the firmware's alignment motion; completion requires real feedback,
not a fixed sleep. --reuse-alignment skips DIRECTION and requires valid zero and
direction flags. The protocol cannot read back the direction SIGN: the supplied
--direction is then an operator declaration that the existing sign is correct.

Requires python-can and the adapter's driver only for actual CLI operation.
Importing this module and running the fake-bus tests requires neither. Backend
send/receive timeout support is required; Python/USB timing is not hard real time.
Bus API reference: https://python-can.readthedocs.io/en/stable/bus.html
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from decimal import Decimal, InvalidOperation
import math
import sys
import time
from typing import Any, Callable

try:
    from . import gimbal_can as protocol
except ImportError:  # Direct execution: examples/ is on sys.path.
    import gimbal_can as protocol


class RunError(RuntimeError):
    """Protocol, feedback, prerequisite or deadline failure."""


class StopUnconfirmed(RunError):
    """The final STOP did not receive its successful execution ACK."""


@dataclass(frozen=True)
class Settings:
    direction: int
    iq_a: Any
    duration_s: float
    node: int = 1
    sequence: int = 0
    align: bool = False
    reuse_alignment: bool = False

    def validate(self) -> None:
        if isinstance(self.direction, bool) or not isinstance(self.direction, int) or self.direction not in (-1, 1):
            raise ValueError("direction must be +1 or -1, verified on this motor")
        try:
            current = Decimal(str(self.iq_a))
        except (ValueError, InvalidOperation) as error:
            raise ValueError("Iq must be finite and within +/-1.000 A") from error
        if not current.is_finite() or abs(current) > Decimal("1.000"):
            raise ValueError("this current runner accepts only +/-1.000 A")
        if isinstance(self.duration_s, bool) or not math.isfinite(self.duration_s) or not 0 < self.duration_s <= 60:
            raise ValueError("duration must be finite, greater than zero and at most 60 seconds")
        if self.align and self.reuse_alignment:
            raise ValueError("--align and --reuse-alignment are mutually exclusive")
        protocol.pack_current(self.node, self.sequence, self.iq_a)


class CurrentRunner:
    PERIOD_S = 0.010
    ALIGN_PERIOD_S = 0.050
    ACK_TIMEOUT_S = 0.040
    STOP_TIMEOUT_S = 0.100
    FEEDBACK_TIMEOUT_S = 0.050
    STARTUP_TIMEOUT_S = 2.0
    ALIGN_TIMEOUT_S = 4.0
    MAX_SEND_GAP_S = 0.050

    def __init__(self, bus: Any, settings: Settings, *, message_factory: Callable[..., Any],
                 # Windows monotonic can be 15.625 ms; QPC supports the 10 ms period.
                 clock: Callable[[], float] = time.perf_counter,
                 log: Callable[[str], None] = print):
        settings.validate()
        self.bus, self.settings = bus, settings
        self.message_factory, self.clock, self.log = message_factory, clock, log
        self.sequence = settings.sequence
        self.status: dict[str, Any] | None = None
        self.diagnostic: dict[str, Any] | None = None
        self.status_time = self.diagnostic_time = -math.inf
        self.status_count = self.diagnostic_count = 0
        self._last_alignment_active_status_count = 0
        self._pending: tuple[int, int] | None = None
        self._ack: dict[str, Any] | None = None
        self._session_ready = False
        self.stop_acknowledged = False
        self.iq_frames = 0

    def _fault_check(self) -> None:
        if self.status is not None and self.status["mode"] == 5:
            raise RunError("firmware reports FAULT mode")
        if self.diagnostic is not None:
            diagnostic = self.diagnostic
            if diagnostic["faults"] or diagnostic["board_faults_low8"]:
                raise RunError(f"firmware fault 0x{diagnostic['faults']:08X}, BSP 0x{diagnostic['board_faults_low8']:02X}")
            if diagnostic["bus_off"] or diagnostic["not_initialized"]:
                raise RunError("firmware CAN transport is unavailable")
            if self._session_ready and diagnostic["require_stop"]:
                raise RunError("CAN control session was revoked; a new STOP handshake is required")

    def _receive(self, timeout: float, *, guard: bool = True) -> None:
        message = self.bus.recv(timeout=max(0.0, timeout))
        if message is None:
            return
        if getattr(message, "is_error_frame", False):
            raise RunError("adapter reported a CAN error frame")
        if getattr(message, "is_extended_id", False) or getattr(message, "is_remote_frame", False):
            return
        identifier = message.arbitration_id
        if identifier not in (protocol.STATUS_BASE + self.settings.node,
                              protocol.DIAGNOSTIC_BASE + self.settings.node,
                              protocol.ACK_BASE + self.settings.node):
            return
        try:
            feedback = protocol.decode_feedback(identifier, message.data, self.settings.node)
        except ValueError as error:
            raise RunError("malformed feedback from selected node") from error
        if feedback["type"] == "status":
            self.status = feedback
            self.status_time = self.clock()
            self.status_count += 1
            if (feedback["mode"] == 2 and feedback["driver_enabled"] and
                    not feedback["zero_valid"] and feedback["direction_confirmed"] and
                    self._prerequisites()):
                self._last_alignment_active_status_count = self.status_count
        elif feedback["type"] == "diagnostic":
            self.diagnostic = feedback
            self.diagnostic_time = self.clock()
            self.diagnostic_count += 1
        elif (feedback["opcode"], feedback["sequence"]) == self._pending:
            self._ack = feedback
        if guard:
            self._fault_check()

    def _exchange(self, opcode: protocol.Opcode, value: Any = 0, *, guard: bool = True,
                  timeout: float | None = None) -> dict[str, Any]:
        sequence = self.sequence
        self.sequence = (sequence + 1) & 0xFF
        identifier, data = protocol.pack_command(self.settings.node, opcode, sequence, value)
        self._pending, self._ack = (int(opcode), sequence), None
        deadline = self.clock() + (self.ACK_TIMEOUT_S if timeout is None else timeout)
        try:
            message = self.message_factory(arbitration_id=identifier, data=data, is_extended_id=False)
            self.bus.send(message, timeout=0.010)
            while self._ack is None and self.clock() < deadline:
                self._receive(deadline - self.clock(), guard=guard)
            if self._ack is None:
                raise RunError(f"execution ACK timeout: {opcode.name}, sequence {sequence}")
            ack = self._ack
            if not ack["executed_ok"]:
                raise RunError(f"{opcode.name} rejected: {ack['result_name']} (sequence {sequence})")
            if guard and ack["faults"]:
                raise RunError(f"{opcode.name} ACK reports fault 0x{ack['faults']:08X}")
            return ack
        finally:
            self._pending = None

    def _freshness_check(self) -> None:
        now = self.clock()
        if now - self.status_time > self.FEEDBACK_TIMEOUT_S or now - self.diagnostic_time > self.FEEDBACK_TIMEOUT_S:
            raise RunError("status/diagnostic feedback missing for more than 50 ms")

    def _prerequisites(self, *, calibrated: bool = False) -> bool:
        if self.status is None:
            return False
        names = ["sense_confirmed", "offset_ready", "sensor_valid"]
        if calibrated:
            names += ["direction_confirmed", "zero_valid"]
        return all(self.status[name] for name in names)

    def _wait_startup(self) -> None:
        # Require new status AND diagnostics after the STOP ACK, not cached boot flags.
        status_count, diagnostic_count = self.status_count, self.diagnostic_count
        deadline = self.clock() + self.STARTUP_TIMEOUT_S
        while self.clock() < deadline:
            self._receive(min(0.010, deadline - self.clock()))
            fresh = self.status_count > status_count and self.diagnostic_count > diagnostic_count
            if fresh and self.status is not None and not self.status["sense_confirmed"]:
                raise RunError("SENSE_UNKNOWN: firmware has not enabled verified current conversion")
            if fresh and self._prerequisites() and not self.diagnostic["require_stop"]:
                self._session_ready = True
                return
        raise RunError("startup feedback not ready: need sense_confirmed, offset_ready, sensor_valid and STOP handshake")

    def _align(self) -> None:
        self.log("ALIGN authorized; waiting for firmware zero_valid feedback")
        # The A1 status has no request tag. A queued pre-DIRECTION zero=1/STOP
        # frame is not completion of this alignment. Witness active ALIGN with
        # zero=0 first, including telemetry received while awaiting its first ACK.
        before_alignment_status_count = self.status_count
        sent = self.clock()
        self._exchange(protocol.Opcode.ALIGN)
        next_send, deadline = sent + self.ALIGN_PERIOD_S, sent + self.ALIGN_TIMEOUT_S
        while self.clock() < deadline:
            self._fault_check()
            self._freshness_check()
            active_count = self._last_alignment_active_status_count
            witnessed_alignment = active_count > before_alignment_status_count
            if (witnessed_alignment and self.status_count > active_count and
                    self._prerequisites(calibrated=True)):
                if self.status["mode"] == 0 and not self.status["driver_enabled"]:
                    self.log("ALIGN completed: zero_valid=1, driver disabled")
                    return
            if self.clock() >= next_send:
                if self.clock() - next_send > self.MAX_SEND_GAP_S:
                    raise RunError("ALIGN scheduling deadline missed")
                sent = self.clock()
                self._exchange(protocol.Opcode.ALIGN)
                next_send = sent + self.ALIGN_PERIOD_S
            else:
                self._receive(min(next_send, deadline) - self.clock())
        raise RunError("ALIGN timeout: firmware did not report valid zero with driver disabled")

    def _stream_current(self) -> None:
        self.log(f"CURRENT Iq={self.settings.iq_a} A for {self.settings.duration_s:g} s; 10 ms fresh sequences")
        deadline = self.clock() + self.settings.duration_s
        next_send = self.clock()
        first_ack_time: float | None = None
        while self.clock() < deadline:
            self._fault_check()
            self._freshness_check()
            if not self._prerequisites(calibrated=True):
                raise RunError("closed-loop prerequisite lost")
            if first_ack_time is not None and self.status_time > first_ack_time:
                if self.status["mode"] != 3 or not self.status["driver_enabled"] or self.status["command_timed_out"]:
                    raise RunError("firmware left active CURRENT mode")
            if self.clock() >= next_send:
                if self.clock() - next_send > self.MAX_SEND_GAP_S:
                    raise RunError("IQ scheduling deadline missed")
                sent = self.clock()
                self._exchange(protocol.Opcode.IQ, self.settings.iq_a)
                self.iq_frames += 1
                if first_ack_time is None:
                    first_ack_time = self.clock()
                next_send = sent + self.PERIOD_S
            else:
                self._receive(min(next_send, deadline) - self.clock())

    def run(self) -> dict[str, Any]:
        """Run one bounded attempt; successful return includes a final STOP ACK.

        The caller owns bus shutdown. Do not run a second master concurrently.
        Even Iq=0 requires the final STOP. Failure to obtain that ACK raises
        StopUnconfirmed; it never prints or returns a claim that output stopped.
        """
        primary_error: BaseException | None = None
        self.stop_acknowledged = False
        try:
            # Discard old adapter RX data before establishing this STOP session.
            for _ in range(256):
                if self.bus.recv(timeout=0.0) is None:
                    break
            else:
                raise RunError("adapter RX backlog exceeds 256 frames")
            self._exchange(protocol.Opcode.STOP, guard=False)
            self._wait_startup()
            if self.settings.reuse_alignment:
                if not self._prerequisites(calibrated=True):
                    raise RunError("--reuse-alignment requires real direction_confirmed and zero_valid feedback")
                self.log(f"Reusing calibration; operator declares direction={self.settings.direction:+d}; sign is not readable over A1")
            else:
                self._exchange(protocol.Opcode.DIRECTION, self.settings.direction)
                if not self.settings.align:
                    raise RunError("NEED_ZERO: DIRECTION invalidates zero; explicitly use --align, or --reuse-alignment for an existing calibration")
                self._align()
            self._stream_current()
        except BaseException as error:
            primary_error = error
        finally:
            self._session_ready = False
            try:
                self._exchange(protocol.Opcode.STOP, guard=False, timeout=self.STOP_TIMEOUT_S)
                self.stop_acknowledged = True
                self.log("Final STOP execution acknowledged; driver disable requested successfully")
            except BaseException as error:
                detail = f"STOP UNCONFIRMED: {error}; no successful final STOP execution ACK"
                if primary_error is not None:
                    detail = f"{primary_error}; {detail}"
                raise StopUnconfirmed(detail) from primary_error or error
        if primary_error is not None:
            raise primary_error
        return {"iq_frames": self.iq_frames, "stop_acknowledged": self.stop_acknowledged}


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--interface", required=True, help="python-can backend, e.g. pcan, slcan or socketcan")
    parser.add_argument("--channel", required=True, help="backend channel, e.g. PCAN_USBBUS1, COM3 or can0")
    parser.add_argument("--direction", type=int, choices=(-1, 1), required=True, help="previously verified encoder sign")
    parser.add_argument("--iq", required=True, help="q-axis phase-current target in A; limited to +/-1.000")
    parser.add_argument("--duration", type=float, required=True, help="current command duration in seconds, >0 and <=60")
    parser.add_argument("--node", type=int, default=1)
    parser.add_argument("--seq", type=int, default=0, help="initial STOP sequence; following frames advance modulo 256")
    calibration = parser.add_mutually_exclusive_group()
    calibration.add_argument("--align", action="store_true", help="explicitly authorize firmware alignment motion")
    calibration.add_argument("--reuse-alignment", action="store_true", help="skip DIRECTION; require existing valid calibration")
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = make_parser()
    args = parser.parse_args(argv)
    settings = Settings(args.direction, args.iq, args.duration, args.node, args.seq,
                        args.align, args.reuse_alignment)
    try:
        settings.validate()
    except (ValueError, TypeError) as error:
        parser.error(str(error))
    try:
        import can  # Lazy: --help and all fake-bus tests work without python-can.
    except ImportError:
        print("python-can is required for actual CAN operation; no adapter was opened", file=sys.stderr)
        return 2
    bus = None
    control: CurrentRunner | None = None
    try:
        filters = [{"can_id": base + settings.node, "can_mask": 0x7FF, "extended": False}
                   for base in (protocol.STATUS_BASE, protocol.DIAGNOSTIC_BASE, protocol.ACK_BASE)]
        bus = can.Bus(interface=args.interface, channel=args.channel, bitrate=protocol.BITRATE,
                      receive_own_messages=False, can_filters=filters, ignore_config=True)
        control = CurrentRunner(bus, settings, message_factory=can.Message)
        control.run()
        return 0
    except KeyboardInterrupt:
        if control is None:
            print("Interrupted before current runner started; no control command sent", file=sys.stderr)
        elif control.stop_acknowledged:
            print("Interrupted; final STOP execution acknowledged", file=sys.stderr)
        else:
            print("Interrupted; final STOP UNCONFIRMED", file=sys.stderr)
        return 130
    except Exception as error:
        print(str(error), file=sys.stderr)
        return 2
    finally:
        if bus is not None:
            bus.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
