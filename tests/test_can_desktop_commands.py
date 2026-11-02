"""Console syntax and units are validated before any CAN side effects."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from desktop.commands import CommandType, parse_command
from examples.gimbal_can import Opcode


class ConsoleCommandTests(unittest.TestCase):
    def test_existing_motion_commands_preserve_units(self):
        current = parse_command("IQ -0.125")
        self.assertEqual((current.kind, current.opcode, current.value), (CommandType.MOTION, Opcode.IQ, -.125))
        self.assertEqual(parse_command("speed 3").value, 3)
        opened = parse_command("open -20 0.8")
        self.assertEqual((opened.value, opened.auxiliary), (-20, .8))
        self.assertEqual(parse_command("open 20").auxiliary, .8)
        self.assertEqual(parse_command("align").opcode, Opcode.ALIGN)
        self.assertEqual(parse_command("iq 3.3").value, 3.3)
        self.assertEqual(parse_command("iq -3.3").value, -3.3)

    def test_parameter_names_and_ids_share_one_path(self):
        named = parse_command("set current_ki 500")
        numbered = parse_command("set 2 500")
        self.assertEqual(named, numbered)
        self.assertEqual(parse_command("get CURRENT_KP").parameter_ids, (1,))
        self.assertEqual(parse_command("get").parameter_ids, tuple(range(1, 10)))
        self.assertEqual(parse_command("bus 14"), parse_command("set bus_voltage 14"))

    def test_maintenance_and_stops(self):
        self.assertEqual(parse_command("STOP").kind, CommandType.STOP)
        self.assertEqual(parse_command("estop").kind, CommandType.ESTOP)
        self.assertEqual(parse_command("dir -1").value, -1)
        self.assertEqual(parse_command("cal").opcode, Opcode.CALIBRATE)
        self.assertEqual(parse_command("clear").opcode, Opcode.CLEAR)

    def test_rejects_malformed_nonfinite_and_out_of_range_input(self):
        for text in ("", "unknown", "iq nan", "iq inf", "iq 3.3001", "iq .1 extra",
                     "speed 31", "open 1 2.001", "open 101 .1", "dir 0", "align now",
                     "get missing", "get 0", "get 1 extra", "set 1 nan", "set 2 2001",
                     "set 9 7.9", "set 1 .2 extra", 'iq "unterminated', "stop extra"):
            with self.subTest(command=text), self.assertRaises(ValueError):
                parse_command(text)


if __name__ == "__main__":
    unittest.main()
