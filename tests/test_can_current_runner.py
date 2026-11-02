"""Exercise the transmitting current runner without python-can or hardware."""

import contextlib
import heapq
import io
from pathlib import Path
import struct
import sys
from types import SimpleNamespace
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "examples"))
import run_can_current as runner


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


class Frame:
    def __init__(self, arbitration_id, data, is_extended_id=False,
                 is_remote_frame=False, is_error_frame=False):
        self.arbitration_id = arbitration_id
        self.data = bytes(data)
        self.is_extended_id = is_extended_id
        self.is_remote_frame = is_remote_frame
        self.is_error_frame = is_error_frame


class FakeBus:
    """Small firmware model: ACKs are separate from send and 10 ms telemetry."""

    def __init__(self, clock, *, calibrated=False):
        self.clock = clock
        self.sent = []
        self.queue = []
        self.order = 0
        self.next_feedback = 0.010
        self.mode = 0
        self.driver = False
        self.sense = self.offset = self.sensor = True
        self.direction = self.zero = calibrated
        self.require_stop = True
        self.faults = 0
        self.align_started = None
        self.align_duration = 1.5
        self.alignment_starts = 0
        self.stop_count = 0
        self.iq_count = 0
        self.drop_ack = lambda opcode, sequence: False
        self.alter_ack = lambda opcode, sequence: (opcode, sequence)
        self.feedback_enabled = True
        self.feedback_hook = None
        self.send_hook = None

    def enqueue(self, identifier, payload, delay=0.0):
        self.order += 1
        heapq.heappush(self.queue, (self.clock() + delay, self.order, Frame(identifier, payload)))

    def send(self, message, timeout=None):
        assert timeout is not None and timeout <= 0.010
        _, opcode, sequence, flags, value, auxiliary = struct.unpack("<BBBBhH", message.data)
        self.sent.append((self.clock(), opcode, sequence, value, flags, auxiliary))
        if self.send_hook is not None:
            self.send_hook(opcode, sequence)
        result = 0
        if opcode == 0:
            self.stop_count += 1
            if self.mode == 2:
                self.zero = False
            self.mode, self.driver, self.require_stop = 0, False, False
            self.align_started = None
        elif opcode == 6:
            self.direction, self.zero = True, False
        elif opcode == 4:
            if self.require_stop:
                result = 0x84
            elif self.mode != 2 and not self.zero:
                self.mode, self.driver = 2, True
                self.align_started = self.clock()
                self.alignment_starts += 1
        elif opcode == 1:
            self.iq_count += 1
            if not (self.direction and self.zero and self.sense and self.offset and self.sensor):
                result = 8
            else:
                self.mode, self.driver = 3, True
        if not self.drop_ack(opcode, sequence):
            ack_opcode, ack_sequence = self.alter_ack(opcode, sequence)
            self.enqueue(0x381, struct.pack("<BBBBI", 0xA1, ack_opcode, ack_sequence, result, self.faults), 0.001)

    def _telemetry(self):
        if self.feedback_hook is not None:
            self.feedback_hook()
        if self.align_started is not None and self.mode == 2 and self.clock() - self.align_started >= self.align_duration:
            self.mode, self.driver, self.zero = 0, False, True
            self.align_started = None
        if not self.feedback_enabled:
            return
        flags = (int(self.driver) | (int(self.sense) << 1) | (int(self.offset) << 2) |
                 (int(self.sensor) << 3) | (int(self.direction) << 4) | (int(self.zero) << 5) |
                 (int(self.driver) << 7))
        self.enqueue(0x181, struct.pack("<BBBBhh", 0xA1, self.mode, flags, 0, 50, 0))
        self.enqueue(0x281, struct.pack("<BBIBB", 0xA1, 0, self.faults, 0, int(self.require_stop) << 3))

    def recv(self, timeout=None):
        assert timeout is not None and timeout >= 0
        deadline = self.clock() + timeout
        while True:
            queued_time = self.queue[0][0] if self.queue else float("inf")
            if queued_time <= min(self.next_feedback, deadline):
                when, _, frame = heapq.heappop(self.queue)
                self.clock.now = max(self.clock(), when)
                return frame
            if self.next_feedback <= deadline:
                self.clock.now = self.next_feedback
                self.next_feedback += 0.010
                self._telemetry()
                continue
            self.clock.now = deadline
            return None


class CurrentRunnerTests(unittest.TestCase):
    def make_runner(self, *, calibrated=False, **settings):
        clock = Clock()
        bus = FakeBus(clock, calibrated=calibrated)
        defaults = dict(direction=1, iq_a="0.05", duration_s=0.100, align=not calibrated,
                        reuse_alignment=calibrated)
        defaults.update(settings)
        control = runner.CurrentRunner(bus, runner.Settings(**defaults), message_factory=Frame,
                                       clock=clock, log=lambda text: None)
        return control, bus, clock

    def test_alignment_feedback_then_stream_and_final_stop(self):
        control, bus, clock = self.make_runner(sequence=250)
        result = control.run()
        operations = [frame[1] for frame in bus.sent]
        self.assertEqual(operations[:2], [0, 6])
        self.assertEqual(operations[-1], 0)
        self.assertEqual(bus.alignment_starts, 1)  # Keepalives do not restart alignment.
        self.assertGreaterEqual(operations.count(4), 30)
        iq = [frame for frame in bus.sent if frame[1] == 1]
        self.assertGreaterEqual(len(iq), 9)
        self.assertGreaterEqual(iq[0][0], 1.5)
        for previous, following in zip(iq, iq[1:]):
            self.assertAlmostEqual(following[0] - previous[0], 0.010, places=7)
            self.assertEqual(following[2], (previous[2] + 1) & 0xFF)
        for previous, following in zip(bus.sent, bus.sent[1:]):
            self.assertEqual(following[2], (previous[2] + 1) & 0xFF)
        self.assertTrue(result["stop_acknowledged"])
        self.assertFalse(bus.driver)
        self.assertEqual(result["iq_frames"], len(iq))

    def test_reuse_skips_direction_and_alignment(self):
        control, bus, _ = self.make_runner(calibrated=True)
        control.run()
        self.assertNotIn(6, [frame[1] for frame in bus.sent])
        self.assertNotIn(4, [frame[1] for frame in bus.sent])
        self.assertGreater(bus.iq_count, 0)
        self.assertEqual(bus.stop_count, 2)

    def test_zero_iq_still_enters_current_and_stops(self):
        control, bus, _ = self.make_runner(calibrated=True, iq_a="0")
        control.run()
        self.assertGreater(bus.iq_count, 0)
        self.assertTrue(all(frame[3] == 0 for frame in bus.sent if frame[1] == 1))
        self.assertEqual(bus.sent[-1][1], 0)
        self.assertTrue(control.stop_acknowledged)

    def test_one_amp_boundaries_and_low_current_remain_legal(self):
        for current, expected_ma in (("1", 1000), ("-1", -1000), ("0.001", 1),
                                     ("-0.001", -1), ("0.05", 50)):
            with self.subTest(current=current):
                control, bus, _ = self.make_runner(calibrated=True, iq_a=current)
                control.run()
                iq_frames = [frame for frame in bus.sent if frame[1] == 1]
                self.assertTrue(iq_frames)
                self.assertTrue(all(frame[3] == expected_ma for frame in iq_frames))
                self.assertTrue(control.stop_acknowledged)

    def test_generic_generator_supports_host_capacity_and_preserves_open_cap(self):
        for current, expected_ma in (("1", 1000), ("-1", -1000), ("0.05", 50),
                                     ("3.3", 3300), ("-3.3", -3300)):
            _, data = runner.protocol.pack_current(1, 0, current)
            self.assertEqual(struct.unpack_from("<h", data, 4)[0], expected_ma)
        for current in ("3.3001", "-3.3001", "3.300001", "-3.300001"):
            with self.subTest(current=current), self.assertRaises(ValueError):
                runner.protocol.pack_current(1, 0, current)
        with self.assertRaises(ValueError):
            runner.protocol.pack_open(1, 0, 20, "2.001")

    def test_no_implicit_alignment(self):
        control, bus, _ = self.make_runner(align=False)
        with self.assertRaisesRegex(runner.RunError, "NEED_ZERO"):
            control.run()
        self.assertEqual([frame[1] for frame in bus.sent], [0, 6, 0])
        self.assertTrue(control.stop_acknowledged)

    def test_reuse_requires_real_valid_zero(self):
        control, bus, _ = self.make_runner(calibrated=True)
        bus.zero = False
        with self.assertRaisesRegex(runner.RunError, "reuse-alignment"):
            control.run()
        self.assertEqual(bus.iq_count, 0)
        self.assertTrue(control.stop_acknowledged)

    def test_lost_initial_stop_ack_never_starts(self):
        control, bus, clock = self.make_runner()
        bus.drop_ack = lambda opcode, sequence: opcode == 0 and sequence == 0
        with self.assertRaisesRegex(runner.RunError, "ACK timeout: STOP"):
            control.run()
        self.assertEqual([frame[1] for frame in bus.sent], [0, 0])
        self.assertTrue(control.stop_acknowledged)  # Cleanup ACK is separate from handshake.
        self.assertLess(clock(), 0.2)

    def test_wrong_sequence_ack_is_not_handshake(self):
        control, bus, _ = self.make_runner()
        bus.alter_ack = lambda opcode, sequence: (opcode, sequence + 17) if sequence == 0 else (opcode, sequence)
        with self.assertRaisesRegex(runner.RunError, "ACK timeout: STOP"):
            control.run()
        self.assertEqual(bus.iq_count, 0)

    def test_alignment_requires_completion_feedback(self):
        control, bus, clock = self.make_runner()
        bus.align_duration = float("inf")
        with self.assertRaisesRegex(runner.RunError, "ALIGN timeout"):
            control.run()
        self.assertEqual(bus.iq_count, 0)
        self.assertTrue(control.stop_acknowledged)
        self.assertLess(clock(), 4.2)

    def test_stale_pre_direction_zero_does_not_complete_new_alignment(self):
        control, bus, _ = self.make_runner()
        injected = False
        def delayed_old_zero(opcode, sequence):
            nonlocal injected
            if opcode == 4 and not injected:
                # Old zero=1, STOP frame arrives after the first ALIGN ACK,
                # before any telemetry has witnessed this active alignment.
                bus.enqueue(0x181, struct.pack("<BBBBhh", 0xA1, 0, 0x3E, 0, 0, 0), 0.0015)
                injected = True
        bus.send_hook = delayed_old_zero
        control.run()
        iq = [frame for frame in bus.sent if frame[1] == 1]
        self.assertGreaterEqual(iq[0][0], 1.5)
        self.assertEqual(bus.alignment_starts, 1)
        self.assertTrue(control.stop_acknowledged)

    def test_active_alignment_before_first_ack_is_a_valid_witness(self):
        control, bus, _ = self.make_runner()
        bus.align_duration = 0.005
        def active_before_ack(opcode, sequence):
            if opcode == 4:
                bus.enqueue(0x181, struct.pack("<BBBBhh", 0xA1, 2, 0x9F, sequence, 0, 0), 0.0005)
        bus.send_hook = active_before_ack
        control.run()
        self.assertGreater(bus.iq_count, 0)
        self.assertEqual(bus.alignment_starts, 1)
        self.assertTrue(control.stop_acknowledged)

    def test_fault_during_current_stops_and_propagates(self):
        control, bus, _ = self.make_runner(calibrated=True)
        def fail_after_iq(opcode, sequence):
            if opcode == 1 and bus.iq_count >= 1:
                bus.faults = 1 << 3
        bus.send_hook = fail_after_iq
        with self.assertRaisesRegex(runner.RunError, "fault"):
            control.run()
        self.assertTrue(control.stop_acknowledged)
        self.assertEqual(bus.sent[-1][1], 0)

    def test_missing_iq_ack_stops_without_silent_retry(self):
        control, bus, _ = self.make_runner(calibrated=True)
        bus.drop_ack = lambda opcode, sequence: opcode == 1
        with self.assertRaisesRegex(runner.RunError, "ACK timeout: IQ"):
            control.run()
        self.assertEqual(bus.iq_count, 1)
        self.assertTrue(control.stop_acknowledged)

    def test_feedback_loss_stops_even_while_acks_arrive(self):
        control, bus, clock = self.make_runner(calibrated=True)
        def lose_feedback(opcode, sequence):
            if opcode == 1:
                bus.feedback_enabled = False
        bus.send_hook = lose_feedback
        with self.assertRaisesRegex(runner.RunError, "feedback missing"):
            control.run()
        self.assertTrue(control.stop_acknowledged)
        self.assertLess(clock(), 0.1)

    def test_interrupt_still_waits_for_stop_ack(self):
        control, bus, _ = self.make_runner(calibrated=True)
        def interrupt(opcode, sequence):
            if opcode == 1:
                raise KeyboardInterrupt()
        bus.send_hook = interrupt
        with self.assertRaises(KeyboardInterrupt):
            control.run()
        self.assertTrue(control.stop_acknowledged)
        self.assertFalse(bus.driver)

    def test_final_stop_ack_loss_is_explicit_and_bounded(self):
        control, bus, clock = self.make_runner(calibrated=True)
        bus.drop_ack = lambda opcode, sequence: opcode == 0 and bus.stop_count == 2
        with self.assertRaisesRegex(runner.StopUnconfirmed, "STOP UNCONFIRMED"):
            control.run()
        self.assertFalse(control.stop_acknowledged)
        self.assertEqual(bus.stop_count, 2)
        self.assertLess(clock(), 0.3)

    def test_sense_unknown_never_starts(self):
        control, bus, _ = self.make_runner()
        bus.sense = False
        with self.assertRaisesRegex(runner.RunError, "SENSE_UNKNOWN"):
            control.run()
        self.assertEqual(bus.iq_count, 0)
        self.assertEqual(bus.alignment_starts, 0)

    def test_limits_reject_before_any_transmission(self):
        for current in ("1.001", "-1.001", "1.000001", "-1.000001", "NaN", "Infinity"):
            with self.subTest(current=current), self.assertRaises(ValueError):
                self.make_runner(iq_a=current)
        for duration in (0, -1, 61, float("nan"), float("inf")):
            with self.subTest(duration=duration), self.assertRaises(ValueError):
                self.make_runner(duration_s=duration)

    def test_help_needs_no_can_library_and_required_arguments(self):
        with mock.patch.dict(sys.modules, {"can": None}), contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaises(SystemExit) as result:
                runner.main(["--help"])
            self.assertEqual(result.exception.code, 0)
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as result:
            runner.main(["--direction", "1", "--iq", "0.05", "--duration", "1"])
        self.assertEqual(result.exception.code, 2)

    def test_interrupt_opening_adapter_does_not_claim_stop_ack(self):
        can_module = SimpleNamespace(Bus=mock.Mock(side_effect=KeyboardInterrupt), Message=Frame)
        stderr = io.StringIO()
        with mock.patch.dict(sys.modules, {"can": can_module}), contextlib.redirect_stderr(stderr):
            result = runner.main(["--interface", "fake", "--channel", "test", "--direction", "1",
                                  "--iq", "0.05", "--duration", "1", "--align"])
        self.assertEqual(result, 130)
        self.assertIn("before current runner started", stderr.getvalue())
        self.assertNotIn("STOP execution acknowledged", stderr.getvalue())


if __name__ == "__main__":
    unittest.main()
