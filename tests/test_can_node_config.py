"""Node addressing codec and full host migration over byte-level fake CAN."""
from pathlib import Path
import struct
import sys
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from examples import gimbal_can as codec
from desktop.controller import Controller, ControlError
from desktop.transports import Frame, SimulatorTransport
from desktop.commands import CommandType, parse_command


class NodeCodecTests(unittest.TestCase):
    def test_fast_frame_units_flags_and_angle_bounds(self):
        fast = codec.decode_feedback(0x707, struct.pack('<BBHhh', 0xA1, 0x3F, 6283, -1234, -500), 7)
        self.assertAlmostEqual(fast['angle_rad'], 6.283)
        self.assertAlmostEqual(fast['velocity_rad_s'], -1.234)
        self.assertEqual(fast['iq_a'], -.5)
        self.assertTrue(fast['sensor_valid'] and fast['zero_valid'])
        with self.assertRaises(ValueError):
            codec.decode_fast(0x707, struct.pack('<BBHhh', 0xA1, 0x3F, 6284, 0, 0), 7)

    def test_wire_bytes_and_decoding(self):
        self.assertEqual(codec.pack_node_read(7, 18), (0x207, bytes.fromhex('A1 12 12 00 00 00 00 00')))
        self.assertEqual(codec.pack_node_set(1, 19, 7), (0x201, bytes.fromhex('A1 13 13 07 43 41 4E 31')))
        info = codec.decode_feedback(0x681, bytes.fromhex('A1 13 13 00 01 07 01 05'), 1)
        self.assertEqual((info['active_node'], info['stored_node']), (1, 7))
        self.assertTrue(info['stored_valid'] and info['reset_pending'])
        self.assertFalse(info['storage_error'])
        self.assertEqual(codec.decode_ack(0x381, bytes.fromhex('A1 13 13 86 00 00 00 00'))['result_name'], 'STORAGE')

    def test_invalid_ids_formats_and_confirmation_token(self):
        for value in (0, 64, -1, True, 3.0, '3'):
            with self.subTest(value=value), self.assertRaises(ValueError):
                codec.pack_node_set(1, 0, value)
        for payload in ('A1 12 00 00 02 02 01 01', 'A1 12 00 00 01 01 02 01',
                        'A1 12 00 00 01 01 01 08', 'A1 12 00 00 01 00 01 01'):
            with self.subTest(payload=payload), self.assertRaises(ValueError):
                codec.decode_node_info(0x681, bytes.fromhex(payload), 1)
        self.assertEqual(parse_command('node').kind, CommandType.READ_NODE)
        self.assertEqual(parse_command('node 7 single').kind, CommandType.CHANGE_NODE)
        with self.assertRaises(ValueError):
            parse_command('node 7')


class NodeBus:
    def __init__(self, config, commands, *, drop_reply=False, fail_storage=False):
        self.inner = SimulatorTransport(config)
        self.commands = commands
        self.drop_reply, self.fail_storage = drop_reply, fail_storage

    def send(self, frame, timeout):
        self.commands.append((frame.arbitration_id, bytes(frame.data)))
        if self.fail_storage and frame.data[1] == codec.Opcode.NODE_SET:
            self.inner._reply(codec.Opcode.NODE_SET, frame.data[2], 0x86)
            self.inner._frames.append(Frame(codec.NODE_INFO_BASE + self.inner.node,
                bytes((0xA1, 0x13, frame.data[2], 0x86, self.inner.node, self.inner.node, 1, 6))))
            return
        self.inner.send(frame, timeout)

    def recv(self, timeout):
        frame = self.inner.recv(timeout)
        if self.drop_reply and frame and frame.arbitration_id in (0x381, 0x681) and frame.data[1] == codec.Opcode.NODE_SET:
            return None
        return frame

    def shutdown(self):
        self.inner.shutdown()


class NodeControllerTests(unittest.TestCase):
    def setUp(self):
        self.buses, self.commands = [], []
        self.drop_reply = self.fail_storage = False
        def factory(config):
            bus = NodeBus(config, self.commands, drop_reply=self.drop_reply, fail_storage=self.fail_storage)
            self.buses.append(bus)
            return bus
        self.controller = Controller(factory)
        self.controller.CONNECT_RETRY_DELAY = .01
        self.config = dict(interface='simulator', channel='SIM', bitrate=1000000, node=1)
        self.controller.connect(self.config)
        self.wait(lambda s: s['session_ready'])

    def tearDown(self):
        self.controller.close()
        self.assertTrue(self.controller.wait_closed(3))

    def wait(self, predicate, timeout=5):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            state = self.controller.snapshot()
            if predicate(state):
                return state
            time.sleep(.005)
        self.fail(repr(self.controller.snapshot()))

    def change(self, node):
        self.controller.change_node(node, single_board=True)
        return self.wait(lambda s: s['node'] == node and s['session_ready'] and not s['node_changing'])

    def test_change_reconnect_readback_and_reopen_old_address_is_rejected(self):
        state = self.change(3)
        self.assertEqual(state['node_info']['stored_node'], 3)
        self.assertTrue(state['node_info']['stored_valid'])
        self.assertFalse(state['status']['driver_enabled'])
        self.assertFalse(state['status']['direction_confirmed'])
        self.assertFalse(state['status']['zero_valid'])
        self.assertEqual(self.controller._connection_config['node'], 3)
        self.assertEqual([data[1] for _,data in self.commands].count(0x13), 1)
        self.controller.disconnect()
        self.wait(lambda s: not s['connected'])
        self.controller.connect(self.config)
        self.wait(lambda s: bool(s['error']) and not s['connecting'])
        self.assertFalse(self.controller.snapshot()['session_ready'])
        self.controller.connect(dict(self.config, node=3))
        self.wait(lambda s: s['session_ready'])

    def test_same_saved_id_does_not_write_again(self):
        self.change(3)
        before = len(self.buses)
        self.controller.change_node(3, single_board=True)
        time.sleep(.15)
        self.assertEqual(len(self.buses), before)
        self.assertEqual([data[1] for _,data in self.commands].count(0x13), 1)

    def test_missing_old_reply_is_resolved_at_new_id_without_rewriting(self):
        self.buses[0].drop_reply = True
        state = self.change(7)
        self.assertEqual(state['node_info']['active_node'], 7)
        self.assertEqual([data[1] for _,data in self.commands].count(0x13), 1)

    def test_single_board_confirmation_and_online_collision(self):
        with self.assertRaises(ControlError):
            self.controller.change_node(3)
        self.buses[0].inner._frames.append(Frame(0x183, struct.pack('<BBBBhh', 0xA1, 0, 14, 0, 0, 0)))
        self.controller.change_node(3, single_board=True)
        self.wait(lambda s: '已' in s['error'] and not s['node_changing'])
        self.assertFalse(any(data[1] == 0x13 for _,data in self.commands))

    def test_stop_cancels_before_persisting(self):
        self.controller.change_node(3, single_board=True)
        self.wait(lambda s: s['node_changing'])
        self.controller.stop()
        state = self.wait(lambda s: not s['node_changing'] and s['stop_confirmed'])
        self.assertFalse(any(data[1] == 0x13 for _,data in self.commands))
        self.assertEqual(state['node'], 1)

    def test_storage_failure_reconnects_old_node_and_reports_failure(self):
        self.buses[0].fail_storage = True
        self.controller.change_node(3, single_board=True)
        state = self.wait(lambda s: '保存失败' in s['error'] and not s['node_changing'])
        self.assertTrue(state['connected'] and state['session_ready'])
        self.assertEqual(state['node'], 1)
        self.assertEqual([data[1] for _,data in self.commands].count(0x13), 1)

    def test_interrupted_first_save_can_be_repaired_without_swd_erase(self):
        self.controller._sim_node_store.update(valid=False, error=True)
        state = self.change(3)
        self.assertTrue(state['node_info']['stored_valid'])
        self.assertFalse(state['node_info']['storage_error'])

    def test_dirty_storage_without_valid_id_blocks_motion_until_repaired(self):
        self.wait(lambda s: s['status'].get('offset_ready'))
        self.controller._sim_node_store.update(valid=False, error=True)
        self.controller.submit('open 10 .4')
        state = self.wait(lambda s: 'STORAGE' in s['error'])
        self.assertFalse(state['status']['driver_enabled'])
        self.assertFalse(self.buses[0].inner.driver_enabled)

    def test_running_node_is_not_readdressed(self):
        self.wait(lambda s: s['status'].get('offset_ready'))
        self.controller.submit('open 10 .4')
        self.wait(lambda s: s['status'].get('driver_enabled'))
        self.controller.change_node(3, single_board=True)
        self.wait(lambda s: bool(s['error']))
        self.assertFalse(any(data[1] == 0x13 for _,data in self.commands))

    def test_auto_reconnect_tracks_new_node_and_never_reverts_to_old_address(self):
        from unittest.mock import Mock
        store, buses, commands = {}, [], []
        def factory(config):
            bus = NodeBus(dict(config, _sim_node_store=store), commands)
            buses.append(bus)
            return bus
        controller = Controller(factory, device_recovery=Mock())
        controller.CONNECT_RETRY_DELAY = .01
        controller.AUTO_RETRY_DELAYS = (.03,)
        try:
            controller.connect(dict(interface='pcan', channel='PCAN_USBBUS1', bitrate=1000000, node=1))
            deadline = time.monotonic()+5
            while not controller.snapshot()['session_ready']:
                self.assertLess(time.monotonic(), deadline)
                time.sleep(.005)
            controller.change_node(3, single_board=True)
            while not (controller.snapshot()['node'] == 3 and not controller.snapshot()['node_changing']):
                self.assertLess(time.monotonic(), deadline)
                time.sleep(.005)
            self.assertEqual(controller._auto_target['node'], 3)
            after_change = len(commands)
            original_recv = buses[-1].recv
            buses[-1].recv = lambda timeout: (original_recv(timeout) and None)
            while controller.snapshot()['recovery_count'] != 1:
                self.assertLess(time.monotonic(), deadline)
                time.sleep(.005)
            self.assertTrue(controller.snapshot()['session_ready'])
            self.assertTrue(all(identifier == 0x203 for identifier, _ in commands[after_change:]))
        finally:
            controller.close()
            self.assertTrue(controller.wait_closed(3))
