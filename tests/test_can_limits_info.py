"""Read-only protection metadata over simulated CAN; no motor/adapter access."""
from pathlib import Path
import struct
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from desktop.commands import CommandType, parse_command
from desktop.controller import CommandRejected, CommunicationError, ControlError, Controller
from desktop.transports import Frame, SimulatorTransport, feedback_filters
from examples import gimbal_can as codec


def limits_payload(sequence=7, *, request_ma=3300, phase_ma=1500, result=0):
    return struct.pack('<BBBBHH', 0xa1, 0x14, sequence, result, request_ma, phase_ma)


class LimitsCodecTests(unittest.TestCase):
    def test_read_request_bytes_and_terminal_command(self):
        self.assertEqual(codec.Opcode.LIMITS_READ, 0x14)
        for node in (1, 2, 63):
            self.assertEqual(codec.pack_limits_read(node, 255),
                             (0x200 + node, bytes.fromhex('A1 14 FF 00 00 00 00 00')))
        self.assertEqual(parse_command('limits').kind, CommandType.READ_LIMITS)
        with self.assertRaises(ValueError):
            parse_command('limits 3.3')
        with self.assertRaises(ValueError):
            codec.pack_command(1, codec.Opcode.LIMITS_READ, 1)
        for node, sequence in ((0, 0), (64, 0), (True, 0), (1, 256), (1, True)):
            with self.subTest(node=node, sequence=sequence), self.assertRaises(ValueError):
                codec.pack_limits_read(node, sequence)

    def test_request_and_phase_limits_are_independent_unsigned_little_endian(self):
        payload = limits_payload()
        self.assertEqual(payload, bytes.fromhex('A1 14 07 00 E4 0C DC 05'))
        for node in (1, 63):
            decoded = codec.decode_feedback(0x600 + node, payload, node)
            self.assertEqual(decoded['type'], 'limits_info')
            self.assertEqual(decoded['node'], node)
            self.assertEqual(decoded['opcode'], 0x14)
            self.assertEqual(decoded['sequence'], 7)
            self.assertEqual(decoded['request_max_a'], 3.3)
            self.assertEqual(decoded['phase_trip_a'], 1.5)
        decoded = codec.decode_limits_info(0x601, limits_payload(request_ma=1000))
        self.assertEqual(decoded['request_max_a'], 1)
        self.assertEqual(decoded['phase_trip_a'], 1.5)

    def test_invalid_response_does_not_become_successful_metadata(self):
        for identifier, payload in ((0x602, limits_payload()), (0x601, limits_payload()[:-1]),
                                    (0x601, b'\xA0' + limits_payload()[1:]),
                                    (0x601, b'\xA1\x13' + limits_payload()[2:]),
                                    (0x601, limits_payload(request_ma=0)),
                                    (0x601, limits_payload(phase_ma=0))):
            with self.subTest(identifier=identifier, payload=payload), self.assertRaises(ValueError):
                codec.decode_limits_info(identifier, payload)
        rejected = codec.decode_limits_info(0x601, limits_payload(request_ma=0, phase_ma=0, result=0x87))
        self.assertEqual(rejected['result_name'], 'UNSUPPORTED')

    def test_selected_node_only_limits_filter(self):
        filters = feedback_filters(2)
        def accepts(identifier):
            return any(identifier & f['can_mask'] == f['can_id'] & f['can_mask'] for f in filters)
        self.assertTrue(accepts(0x602))
        self.assertFalse(accepts(0x601))
        self.assertFalse(accepts(0x603))

    def test_simulator_query_does_not_handshake_or_renew_motion_watchdog(self):
        now = [100.0]
        bus = SimulatorTransport(clock=lambda: now[0], sleeper=lambda dt: now.__setitem__(0, now[0] + dt))
        bus.offset_ready = bus.direction_confirmed = bus.zero_valid = True
        bus._calibration_ends = None
        def send(packed):
            bus.send(Frame(*packed))
        send(codec.pack_limits_read(1, 0))
        self.assertTrue(bus.require_stop)
        send(codec.pack_stop(1, 1))
        send(codec.pack_current(1, 2, .05))
        self.assertTrue(bus.driver_enabled)
        now[0] += .060
        send(codec.pack_limits_read(1, 3))
        now[0] += .041
        bus.recv(timeout=0)
        self.assertFalse(bus.driver_enabled)
        self.assertTrue(bus.command_timed_out)
        bus.shutdown()


class LimitsNode:
    def __init__(self, config, behavior='normal'):
        self.inner = SimulatorTransport(config)
        self.inner.offset_ready = self.inner.direction_confirmed = self.inner.zero_valid = True
        self.inner._calibration_ends = None
        self.sent = []
        self.behavior = behavior

    def send(self, frame, timeout):
        self.sent.append(frame)
        if frame.data[1] == codec.Opcode.LIMITS_READ and self.behavior == 'send_failure':
            raise OSError('simulated adapter disconnected')
        if frame.data[1] == codec.Opcode.LIMITS_READ and self.behavior in ('bad_frame', 'unsupported', 'silent'):
            if self.behavior != 'silent':
                self.inner._reply(0x14, frame.data[2], 0x80 if self.behavior == 'bad_frame' else 0x87)
            return
        self.inner.send(frame, timeout)

    def recv(self, timeout=0):
        frame = self.inner.recv(timeout)
        if frame is None or frame.arbitration_id != 0x601:
            return frame
        if self.behavior == 'no_info':
            return None
        data = bytearray(frame.data)
        if self.behavior == 'wrong_sequence':
            data[2] = (data[2] + 1) & 255
        elif self.behavior == 'wrong_node':
            return Frame(0x602, data)
        elif self.behavior == 'bad_values':
            data[4:6] = b'\0\0'
        return Frame(frame.arbitration_id, data)

    def shutdown(self):
        self.inner.shutdown()


class LimitsControllerTests(unittest.TestCase):
    def setUp(self):
        self.controllers = []

    def tearDown(self):
        for controller in self.controllers:
            controller._disconnect()

    def make(self, *, capability=1.0, behavior='normal'):
        buses = []
        def factory(config):
            bus = LimitsNode(config, behavior)
            buses.append(bus)
            return bus
        with patch('desktop.controller.threading.Thread.start'):
            controller = Controller(factory, auto_reconnect=False)
        self.controllers.append(controller)
        controller._connect(dict(interface='simulator', node=1, bitrate=1000000,
                                 iq_limit_max_a=capability))
        return controller, buses[0]

    def test_full_parameter_read_automatically_reads_protection_without_motion_or_writes(self):
        controller, bus = self.make(capability=3.3)
        controller._command('get')
        state = controller.snapshot()
        self.assertEqual(len(state['parameters']), 9)
        self.assertEqual(state['parameters'][3], 1.0)
        self.assertEqual(state['limits_info']['request_max_a'], 3.3)
        self.assertEqual(state['limits_info']['phase_trip_a'], 2.0)
        self.assertEqual([f.data[1] for f in bus.sent], [0] + [0x10] * 9 + [0x14])
        self.assertFalse(state['running'])
        self.assertFalse(bus.inner.driver_enabled)
        self.assertFalse(state['tuning_busy'])

    def test_one_amp_simulator_reports_request_cap_and_independent_phase_trip(self):
        controller, _ = self.make()
        controller._command('limits')
        self.assertEqual(controller.snapshot()['limits_info']['request_max_a'], 1.0)
        self.assertEqual(controller.snapshot()['limits_info']['phase_trip_a'], 2.0)
        self.assertEqual(controller.snapshot()['parameters'], {})

    def test_bare_simulator_defaults_to_three_point_three_request_cap(self):
        with patch('desktop.controller.threading.Thread.start'):
            controller = Controller(lambda config: SimulatorTransport(config), auto_reconnect=False)
        self.controllers.append(controller)
        controller._connect(dict(interface='simulator', node=1, bitrate=1000000))
        controller._command('limits')
        self.assertEqual(controller.snapshot()['limits_info']['request_max_a'], 3.3)
        self.assertEqual(controller.snapshot()['limits_info']['phase_trip_a'], 2.0)

    def test_public_read_limits_queues_only_read_request(self):
        controller, bus = self.make()
        before = len(bus.sent)
        controller.read_limits()
        self.assertEqual(controller._commands.get_nowait(), ('read_limits', None))
        self.assertEqual(len(bus.sent), before)

    def test_capability_query_does_not_write_current_parameter_and_board_decides_execution(self):
        controller, bus = self.make(capability=3.3)
        controller._command('limits')
        self.assertNotIn(3, controller.snapshot()['parameters'])
        self.assertEqual(bus.inner.parameters[3], 1.0)
        self.assertFalse(any(f.data[1] == 0x11 for f in bus.sent))
        controller._command('iq 3.3')
        with self.assertRaisesRegex(CommandRejected, 'RANGE'):
            controller._tick_motion()
        self.assertEqual(controller.snapshot()['parameters'], {})
        current_frames = [f for f in bus.sent if f.data[1] == 1]
        self.assertEqual(struct.unpack_from('<h', current_frames[0].data, 4)[0], 3300)
        self.assertEqual(bus.inner.phase_trip_a, 2.0)

    def test_explicit_simulated_capability_and_real_parameter_write_can_encode_three_point_three(self):
        controller, bus = self.make(capability=3.3)
        controller._command('get')
        controller._write_parameters({3: 3.3})
        controller._command('iq 3.3')
        controller._tick_motion()
        current_frames = [f for f in bus.sent if f.data[1] == 1]
        self.assertEqual(struct.unpack_from('<h', current_frames[0].data, 4)[0], 3300)
        # A command/plant simulator does not validate real phase-current trips.
        self.assertTrue(bus.inner.simulation_only)
        self.assertEqual(controller.snapshot()['limits_info']['phase_trip_a'], 2.0)

    def test_legacy_firmware_rejection_or_missing_response_stays_connected(self):
        for behavior in ('bad_frame', 'unsupported', 'silent', 'no_info', 'wrong_sequence', 'wrong_node', 'bad_values'):
            with self.subTest(behavior=behavior):
                controller, bus = self.make(behavior=behavior)
                controller._command('get')
                state = controller.snapshot()
                self.assertTrue(state['connected'])
                self.assertTrue(state['session_ready'])
                self.assertEqual(state['limits_info'], {})
                self.assertEqual(state['parameters'][3], 1.0)
                self.assertFalse(state['tuning_busy'])
                self.assertFalse(state['error'])
                self.assertFalse(any(f.data[1] in (1, 4, 5, 0x11) for f in bus.sent))

    def test_partial_parameter_read_does_not_insert_additional_query(self):
        controller, bus = self.make()
        controller._command('get 3')
        self.assertEqual([f.data[1] for f in bus.sent], [0, 0x10])
        self.assertEqual(controller.snapshot()['limits_info'], {})

    def test_compatibility_handling_does_not_hide_an_actual_transport_failure(self):
        controller, _ = self.make(behavior='send_failure')
        with self.assertRaisesRegex(CommunicationError, '发送失败'):
            controller._command('limits')
        self.assertEqual(controller.snapshot()['limits_info'], {})
        self.assertFalse(controller.snapshot()['tuning_busy'])

    def test_stopped_faulted_board_still_reports_read_only_protection(self):
        controller, bus = self.make(capability=3.3)
        bus.inner.faults = 1 << 24
        bus.inner._stop()
        controller._command('limits')
        self.assertEqual(controller.snapshot()['limits_info']['request_max_a'], 3.3)
        self.assertEqual(bus.inner.faults, 1 << 24)
        self.assertFalse(bus.inner.driver_enabled)
        self.assertFalse(any(f.data[1] in (1, 4, 5, 0x11) for f in bus.sent))


if __name__ == '__main__':
    unittest.main()
