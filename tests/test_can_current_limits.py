"""Host exposes +/-3.3 A; execution limits belong to the simulated board."""
from pathlib import Path
import struct
import sys
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from desktop.controller import CommandRejected, CommandTimeout, ControlError, Controller
from desktop.current_limits import HOST_CURRENT_LIMIT_A, current_limit_readback, validate_current_target
from desktop.transports import Frame, SimulatorTransport


def float32(value):
    return struct.unpack('<f', struct.pack('<f', value))[0]


class CurrentLimitHelperTests(unittest.TestCase):
    def test_host_range_and_valid_float32_readback_boundaries(self):
        self.assertEqual(HOST_CURRENT_LIMIT_A, 3.3)
        for limit in (.01, 1.0, 3.3):
            with self.subTest(limit=limit):
                self.assertEqual(current_limit_readback({3: float32(limit)}), float32(limit))
                self.assertEqual(validate_current_target(limit, {3: float32(limit)}), limit)

    def test_unknown_and_invalid_readbacks_never_authorize_current(self):
        for parameters in (None, {}, {3: True}, {3: '3.3'}, {3: float('nan')},
                           {3: float('inf')}, {3: 0}, {3: -.5}, {3: .009}, {3: 3.301}):
            with self.subTest(parameters=parameters):
                self.assertIsNone(current_limit_readback(parameters))
                with self.assertRaises(ValueError):
                    validate_current_target(.1, parameters)
        self.assertEqual(validate_current_target(3.3, {}, require_readback=False), 3.3)

    def test_target_encoding_and_board_limits_are_independent(self):
        for target in (3.301, -3.301, True, float('nan'), float('inf')):
            with self.subTest(target=target), self.assertRaises(ValueError):
                validate_current_target(target, {3: float32(3.3)})
        for target in (3.3, -3.3, 1.001, -1.001):
            with self.subTest(target=target), self.assertRaisesRegex(ValueError, '已读回'):
                validate_current_target(target, {3: 1.0})
        self.assertEqual(validate_current_target(-1, {3: 1.0}), -1)

    def test_rounded_wire_current_cannot_exceed_readback(self):
        # .1006 A becomes 101 mA on the wire, above the board's .1006 A limit.
        with self.assertRaises(ValueError):
            validate_current_target(.1006, {3: float32(.1006)})
        self.assertEqual(validate_current_target(.1004, {3: float32(.1006)}), .1004)


class TrackedNode:
    def __init__(self, config):
        self.inner = SimulatorTransport(config)
        self.inner.offset_ready = self.inner.direction_confirmed = self.inner.zero_valid = True
        self.inner._calibration_ends = None
        self.inner.parameters[3] = float32(config.get('initial_iq_limit_a', 1.0))
        self.sent = []
        self.drop_ack = self.drop_parameter = self.wrong_sequence = False
        self.mismatch_write_readback = False

    def send(self, frame, timeout):
        self.sent.append(frame)
        self.inner.send(frame, timeout)

    def recv(self, timeout=0):
        frame = self.inner.recv(timeout)
        if frame is None:
            return None
        if frame.arbitration_id == 0x381 and self.drop_ack:
            return None
        if frame.arbitration_id == 0x481:
            if self.drop_parameter:
                return None
            data = bytearray(frame.data)
            if self.wrong_sequence:
                data[2] = (data[2] + 1) & 255
            if self.mismatch_write_readback and self.sent[-1].data[1] == 0x11:
                data[4:] = struct.pack('<f', .6)
            return Frame(frame.arbitration_id, bytes(data))
        return frame

    def shutdown(self):
        self.inner.shutdown()

    def iq_frames(self):
        return [frame for frame in self.sent if frame.data[1] == 1]


class ControllerCurrentLimitTests(unittest.TestCase):
    def setUp(self):
        self.controllers = []

    def tearDown(self):
        for controller in self.controllers:
            controller._disconnect()

    def make(self, *, capability=1.0, initial_limit=1.0):
        buses = []
        def factory(config):
            bus = TrackedNode(config)
            buses.append(bus)
            return bus
        # Keep the actual controller, codec and byte-level transport, while
        # stepping its owner methods synchronously instead of racing a thread.
        with patch('desktop.controller.threading.Thread.start'):
            controller = Controller(factory, auto_reconnect=False)
        self.controllers.append(controller)
        controller._connect(dict(interface='simulator', node=1, bitrate=1000000,
                                 iq_limit_max_a=capability, initial_iq_limit_a=initial_limit))
        return controller, buses[0]

    def drain_until(self, controller, predicate):
        deadline = time.monotonic() + .3
        while not predicate() and time.monotonic() < deadline:
            controller._receive(.003)
        self.assertTrue(predicate())

    def test_one_amp_board_receives_both_three_point_three_requests_and_rejects_in_ack(self):
        controller, bus = self.make()
        for target in (3.3, -3.3):
            controller._command(f'iq {target}')
            with self.subTest(target=target), self.assertRaisesRegex(CommandRejected, 'RANGE'):
                controller._tick_motion()
            controller._stop_motion()
        self.assertEqual([struct.unpack_from('<h', f.data, 4)[0] for f in bus.iq_frames()], [3300, -3300])
        self.assertEqual(controller.snapshot()['parameters'], {})
        self.assertFalse(any(f.data[1] == 0x10 for f in bus.sent))
        self.assertFalse(bus.inner.driver_enabled)

    def test_three_point_three_board_accepts_both_targets_without_host_parameter_read(self):
        controller, bus = self.make(capability=3.3, initial_limit=3.3)
        for target in (3.3, -3.3):
            controller._command(f'iq {target}')
            controller._tick_motion()
        self.assertEqual([struct.unpack_from('<h', frame.data, 4)[0] for frame in bus.iq_frames()], [3300, -3300])
        self.assertEqual(controller.snapshot()['parameters'], {})
        self.assertTrue(bus.inner.driver_enabled)

    def test_three_point_three_board_accepts_after_actual_parameter_write(self):
        controller, bus = self.make(capability=3.3)
        controller._write_parameters({3: 3.3})
        self.assertEqual(controller.snapshot()['parameters'][3], float32(3.3))
        self.assertEqual(bus.inner.parameters[3], float32(3.3))
        controller._command('iq 3.3')
        controller._tick_motion()
        self.assertEqual(struct.unpack_from('<h', bus.iq_frames()[0].data, 4)[0], 3300)

    def test_reconnection_clears_display_cache_and_new_board_owns_execution_limit(self):
        controller, old_bus = self.make(capability=3.3, initial_limit=3.3)
        controller._parameter(3)
        controller._connect(dict(interface='simulator', node=1, bitrate=1000000,
                                 iq_limit_max_a=1.0, initial_iq_limit_a=1.0))
        self.assertNotIn(3, controller.snapshot()['parameters'])
        controller._command('iq 3.3')
        with self.assertRaisesRegex(CommandRejected, 'RANGE'):
            controller._tick_motion()
        self.assertNotIn(3, controller.snapshot()['parameters'])
        self.assertEqual(struct.unpack_from('<h', controller._bus.iq_frames()[0].data, 4)[0], 3300)
        self.assertEqual(old_bus.iq_frames(), [])

    def test_unknown_limit_sends_iq_but_still_requires_execution_ack(self):
        controller, bus = self.make(capability=3.3, initial_limit=3.3)
        bus.drop_ack = True
        controller._command('iq .1')
        with self.assertRaises(CommandTimeout):
            controller._tick_motion()
        self.assertNotIn(3, controller.snapshot()['parameters'])
        self.assertEqual(len(bus.iq_frames()), 1)
        self.assertFalse(any(f.data[1] == 0x10 for f in bus.sent))
        bus.drop_ack = False

    def test_wrong_parameter_sequence_is_not_displayed_and_does_not_gate_motion(self):
        controller, bus = self.make(capability=3.3, initial_limit=3.3)
        bus.wrong_sequence = True
        with self.assertRaises(CommandTimeout):
            controller._parameter(3)
        self.assertNotIn(3, controller.snapshot()['parameters'])
        self.assertEqual(bus.iq_frames(), [])
        bus.wrong_sequence = False
        controller._command('iq 3.3')
        controller._tick_motion()
        self.assertEqual(struct.unpack_from('<h', bus.iq_frames()[0].data, 4)[0], 3300)

    def test_explicit_range_rejection_keeps_previous_actual_readback(self):
        controller, bus = self.make()
        controller._parameter(3)
        with self.assertRaisesRegex(ControlError, 'RANGE'):
            controller._write_parameters({3: 3.3})
        self.assertEqual(controller.snapshot()['parameters'][3], 1.0)
        self.assertEqual(bus.inner.parameters[3], 1.0)
        self.assertEqual(bus.iq_frames(), [])

    def test_applied_lower_limit_with_lost_parameter_reply_invalidates_old_cache(self):
        controller, bus = self.make(capability=3.3, initial_limit=3.3)
        controller._parameter(3)
        bus.drop_parameter = True
        with self.assertRaises(CommandTimeout):
            controller._write_parameters({3: .5})
        self.assertEqual(bus.inner.parameters[3], .5)
        self.assertNotIn(3, controller.snapshot()['parameters'])
        # No host readback veto: .6 A reaches the board and its actual .5 A
        # parameter rejects it through the mandatory execution ACK.
        controller._command('iq .6')
        with self.assertRaisesRegex(CommandRejected, 'RANGE'):
            controller._tick_motion()
        self.assertEqual(struct.unpack_from('<h', bus.iq_frames()[0].data, 4)[0], 600)
        controller._stop_motion()
        bus.drop_parameter = False
        self.assertNotIn(3, controller.snapshot()['parameters'])
        controller._command('iq .4')
        controller._tick_motion()
        self.assertEqual(struct.unpack_from('<h', bus.iq_frames()[-1].data, 4)[0], 400)

    def test_applied_lower_limit_with_lost_ack_cannot_reuse_old_cache(self):
        controller, bus = self.make(capability=3.3, initial_limit=3.3)
        controller._parameter(3)
        bus.drop_ack = True
        with self.assertRaises(CommandTimeout):
            controller._write_parameters({3: .5})
        self.assertNotIn(3, controller.snapshot()['parameters'])
        self.assertEqual(bus.inner.parameters[3], .5)
        self.assertEqual(bus.iq_frames(), [])
        bus.drop_ack = False

    def test_mismatched_write_readback_does_not_restore_previous_limit(self):
        controller, bus = self.make(capability=3.3, initial_limit=3.3)
        controller._parameter(3)
        bus.mismatch_write_readback = True
        with self.assertRaisesRegex(ControlError, '不一致'):
            controller._write_parameters({3: .5})
        self.assertNotIn(3, controller.snapshot()['parameters'])
        self.assertEqual(bus.iq_frames(), [])

    def test_reboot_diagnostic_clears_cache_and_still_requires_stop_handshake(self):
        controller, bus = self.make(capability=3.3, initial_limit=3.3)
        controller._parameter(3)
        bus.inner.parameters[3] = 1.0
        bus.inner.require_stop = True
        bus.inner._publish()
        self.drain_until(controller, lambda: controller.snapshot()['diagnostic'].get('require_stop'))
        self.assertNotIn(3, controller.snapshot()['parameters'])
        with self.assertRaisesRegex(ControlError, '会话已失效'):
            controller._command('iq 2')
        controller._stop_motion()
        self.drain_until(controller, lambda: not controller.snapshot()['diagnostic'].get('require_stop'))
        controller._command('iq 2')
        with self.assertRaisesRegex(CommandRejected, 'RANGE'):
            controller._tick_motion()
        self.assertNotIn(3, controller.snapshot()['parameters'])
        self.assertEqual(struct.unpack_from('<h', bus.iq_frames()[0].data, 4)[0], 2000)

    def test_iq_above_readback_limit_is_refused_before_transmission(self):
        controller, bus = self.make(capability=3.3, initial_limit=3.3)
        controller._command('iq 2')
        controller._tick_motion()
        before = len(bus.iq_frames())
        # A real read-back limit below the target must veto TX and name it.
        controller._state['parameters'][3] = .1
        with self.assertRaisesRegex(ControlError, '已读回的电流限幅'):
            controller._exchange(1, 2)
        self.assertEqual(len(bus.iq_frames()), before)
        with self.assertRaises(ControlError):
            controller._exchange(1, 3.301)
        self.assertEqual(len(bus.iq_frames()), before)

    def test_targets_outside_host_three_point_three_range_never_transmit(self):
        controller, bus = self.make(capability=3.3, initial_limit=3.3)
        for target in ('3.301', '-3.301', 'nan', 'inf'):
            with self.subTest(target=target), self.assertRaises(ValueError):
                controller._command(f'iq {target}')
        self.assertEqual(bus.iq_frames(), [])


if __name__ == '__main__':
    unittest.main()
