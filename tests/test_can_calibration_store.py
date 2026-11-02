"""Persistent electrical-zero CAN codec and desktop workflow tests."""
from pathlib import Path
import struct
import sys
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from examples import gimbal_can as codec
from desktop.commands import CommandType, parse_command
from desktop.controller import Controller
from desktop.transports import SimulatorTransport


class CalibrationCodecTests(unittest.TestCase):
    def test_wire_tokens_and_reply(self):
        self.assertEqual(codec.pack_calibration_read(3, 7),
                         (0x203, bytes.fromhex('A1 15 07 00 00 00 00 00')))
        self.assertEqual(codec.pack_calibration_save(3, 8),
                         (0x203, bytes.fromhex('A1 16 08 00 43 41 4C 31')))
        self.assertEqual(codec.pack_calibration_clear(3, 9),
                         (0x203, bytes.fromhex('A1 17 09 00 43 4C 52 31')))
        reply = codec.decode_feedback(0x643,
            struct.pack('<BBBBBbH', 0xA1, 0x15, 7, 0, 0x09, -1, 1234), 3)
        self.assertTrue(reply['stored_valid'] and reply['runtime_valid'])
        self.assertFalse(reply['storage_error'] or reply['profile_mismatch'])
        self.assertEqual(reply['sensor_direction'], -1)
        self.assertEqual(reply['electrical_zero_rad'], 1.234)

    def test_bad_info_and_generic_packer_are_rejected(self):
        for flags, direction, zero in ((0, 1, 0), (1, 0, 0), (0, 0, 1), (0x20, 0, 0),
                                       (1, 1, 6284)):
            with self.subTest(flags=flags, direction=direction, zero=zero), self.assertRaises(ValueError):
                codec.decode_calibration_info(0x641,
                    struct.pack('<BBBBBbH', 0xA1, 0x15, 0, 0, flags, direction, zero))
        for opcode in (codec.Opcode.CALIBRATION_READ, codec.Opcode.CALIBRATION_SAVE,
                       codec.Opcode.CALIBRATION_CLEAR):
            with self.assertRaisesRegex(ValueError, 'pack_calibration'):
                codec.pack_command(1, opcode, 0)

    def test_console_commands(self):
        self.assertEqual(parse_command('zero').kind, CommandType.READ_CALIBRATION)
        self.assertEqual(parse_command('zero save').kind, CommandType.SAVE_CALIBRATION)
        self.assertEqual(parse_command('zero clear').kind, CommandType.CLEAR_CALIBRATION)


class CalibrationControllerTests(unittest.TestCase):
    def setUp(self):
        self.controller = Controller(lambda config: SimulatorTransport(config))
        self.config = dict(interface='simulator', channel='SIM', bitrate=1000000, node=1)
        self.controller.connect(self.config)
        self.wait(lambda s: s['session_ready'] and s['status'].get('offset_ready'))

    def tearDown(self):
        self.controller.close()
        self.assertTrue(self.controller.wait_closed(3))

    def wait(self, predicate, timeout=8):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            state = self.controller.snapshot()
            if predicate(state):
                return state
            time.sleep(.005)
        self.fail(repr(self.controller.snapshot()))

    def test_align_save_reboot_restore_and_clear(self):
        self.controller.submit('dir 1')
        self.wait(lambda s: s['status'].get('direction_confirmed'))
        self.controller.submit('align')
        self.wait(lambda s: s['status'].get('zero_valid') and not s['status'].get('driver_enabled'))
        self.controller.save_calibration()
        saved = self.wait(lambda s: s['session_ready'] and
                          s['calibration_info'].get('stored_valid') and
                          s['calibration_info'].get('runtime_valid'))
        self.assertTrue(saved['status']['zero_valid'])
        self.controller.clear_calibration()
        cleared = self.wait(lambda s: s['session_ready'] and s['calibration_info'] and
                            not s['calibration_info'].get('stored_valid'))
        self.assertFalse(cleared['status']['zero_valid'])


if __name__ == '__main__':
    unittest.main()
