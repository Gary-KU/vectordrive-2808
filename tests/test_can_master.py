"""No CAN hardware/library required: verify helper bytes against firmware layout."""
import contextlib
from decimal import Decimal
import importlib.util
import io
from pathlib import Path
import struct
import unittest
from unittest.mock import patch

_MODULE_PATH = Path(__file__).resolve().parents[1] / "examples" / "gimbal_can.py"
_SPEC = importlib.util.spec_from_file_location("gimbal_can", _MODULE_PATH)
can = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(can)


class CanMasterTests(unittest.TestCase):
    def test_positive_negative_current_and_little_endian(self):
        identifier, payload = can.pack_current(1, 1, "0.05")
        self.assertEqual(identifier, 0x201)
        self.assertEqual(payload, bytes.fromhex("A1 01 01 01 32 00 00 00"))
        self.assertEqual(can.pack_current(1, 255, "-0.5")[1],
                         bytes.fromhex("A1 01 FF 01 0C FE 00 00"))
        self.assertEqual(can.pack_current(1, 0, "0.5")[1][2:6], bytes.fromhex("00 01 F4 01"))
        self.assertEqual(can.pack_current(1, 1, "1")[1], bytes.fromhex("A1 01 01 01 E8 03 00 00"))
        self.assertEqual(can.pack_current(1, 2, "-1")[1], bytes.fromhex("A1 01 02 01 18 FC 00 00"))
        self.assertEqual(can.pack_current(1, 3, "3.3")[1], bytes.fromhex("A1 01 03 01 E4 0C 00 00"))
        self.assertEqual(can.pack_command(1, can.Opcode.IQ, 4, "-3.3")[1],
                         bytes.fromhex("A1 01 04 01 1C F3 00 00"))

    def test_iq_encoding_range_follows_shared_host_catalog(self):
        with patch.dict(can.PARAMETERS[can.Param.IQ_LIMIT], {"max": 1.2}):
            self.assertEqual(struct.unpack_from("<h", can.pack_current(1, 0, "1.2")[1], 4)[0], 1200)
            with self.assertRaises(ValueError):
                can.pack_command(1, can.Opcode.IQ, 0, "1.2001")

    def test_rounding_and_exact_decimal_near_tie(self):
        for target, expected in (("0.0005", 1), ("-0.0005", -1), ("0.0004999", 0),
                                 ("-0.0004999", 0), ("0.0015", 2), ("-0.0015", -2)):
            with self.subTest(target=target):
                self.assertEqual(struct.unpack_from("<h", can.pack_current(1, 0, target)[1], 4)[0], expected)
        almost_half = Decimal("0.0004999999999999999999999999999999999999999999999")
        self.assertEqual(can.pack_current(1, 0, almost_half)[1][4:6], b"\0\0")

    def test_speed_boundaries(self):
        self.assertEqual(can.pack_speed(63, 255, 30)[0], 0x23F)
        self.assertEqual(can.pack_speed(1, 0, 30)[1][4:6], bytes.fromhex("30 75"))
        self.assertEqual(can.pack_speed(1, 0, -30)[1][4:6], bytes.fromhex("D0 8A"))

    def test_open_units_direction_and_motion_flags(self):
        self.assertEqual(can.pack_open(1, 5, "-20.005", "0.8005")[1],
                         bytes.fromhex("A1 03 05 01 2F F8 21 03"))
        self.assertEqual(can.pack_open(1, 0, 100, 2)[1][4:], bytes.fromhex("10 27 D0 07"))
        self.assertEqual(can.pack_command(1, can.Opcode.DIRECTION, 0, -1)[1][3:],
                         bytes.fromhex("00 FF FF 00 00"))
        for operation in (can.Opcode.STOP, can.Opcode.CLEAR, can.Opcode.CALIBRATE, can.Opcode.ESTOP):
            self.assertEqual(can.pack_command(1, operation, 0)[1][3:], bytes(5))
        self.assertEqual(can.pack_command(1, can.Opcode.ALIGN, 0)[1][3], 1)
        self.assertEqual(can.pack_command(63, can.Opcode.ESTOP, 255)[0], 0x080)
        self.assertEqual(can.pack_stop(1, 0)[1], bytes.fromhex("A1 00 00 00 00 00 00 00"))

    def test_ranges_rejected_before_rounding(self):
        bad_calls = (
            lambda: can.pack_current(1, 0, "3.300001"),
            lambda: can.pack_current(1, 0, "-3.300001"),
            lambda: can.pack_current(1, 0, "3.3001"),
            lambda: can.pack_current(1, 0, "-3.3001"),
            lambda: can.pack_command(1, can.Opcode.IQ, 0, "3.3001"),
            lambda: can.pack_speed(1, 0, "30.0001"),
            lambda: can.pack_open(1, 0, "100.001", 1),
            lambda: can.pack_open(1, 0, 20, "2.00001"),
            lambda: can.pack_open(1, 0, 20, "-0.00001"),
            lambda: can.pack_command(1, can.Opcode.DIRECTION, 0, "1.0001"),
            lambda: can.pack_command(1, can.Opcode.STOP, 0, 1),
            lambda: can.pack_command(1, can.Opcode.IQ, 0, 0, 1),
            lambda: can.pack_command(1, 8, 0),
            lambda: can.pack_speed(1, 0, "1e9999"),
        )
        for bad_call in bad_calls:
            with self.subTest(call=bad_call), self.assertRaises(ValueError):
                bad_call()

    def test_nonfinite_and_illegal_types(self):
        for value in (float("nan"), float("inf"), -float("inf"), "NaN", "sNaN", "Infinity", "-Infinity"):
            with self.subTest(value=value), self.assertRaises(ValueError):
                can.pack_current(1, 0, value)
            with self.subTest(aux=value), self.assertRaises(ValueError):
                can.pack_open(1, 0, 20, value)
        for value in (True, None, object(), complex(1, 2), "nope"):
            with self.subTest(value=value), self.assertRaises(ValueError):
                can.pack_current(1, 0, value)
        for node in (0, 64, True, 1.0, "1"):
            with self.subTest(node=node), self.assertRaises(ValueError):
                can.pack_stop(node, 0)
        for sequence in (-1, 256, False, 1.0, "1"):
            with self.subTest(sequence=sequence), self.assertRaises(ValueError):
                can.pack_stop(1, sequence)

    def test_ack_faults_and_execution_result(self):
        result = can.decode_feedback(0x381, bytes.fromhex("A1 01 FF 00 78 56 34 12"))
        self.assertEqual(result["type"], "ack")
        self.assertEqual(result["sequence"], 255)
        self.assertEqual(result["faults"], 0x12345678)
        self.assertTrue(result["executed_ok"])
        self.assertEqual(result["opcode_name"], "IQ")
        result = can.decode_ack(0x381, bytes.fromhex("A1 01 00 84 00 00 00 00"))
        self.assertFalse(result["executed_ok"])
        self.assertEqual(result["result_name"], "REQUIRE_STOP")

    def test_status_signed_units_and_each_flag(self):
        names = ("driver_enabled", "sense_confirmed", "offset_ready", "sensor_valid",
                 "direction_confirmed", "zero_valid", "command_timed_out", "can_control")
        for bit, name in enumerate(names):
            payload = struct.pack("<BBBBhh", 0xA1, 4, 1 << bit, 0, -500, -30000)
            result = can.decode_status(0x181, payload)
            self.assertTrue(result[name])
            self.assertEqual(sum(result[key] for key in names), 1)
            self.assertEqual(result["iq_a"], -0.5)
            self.assertEqual(result["velocity_rad_s"], -30)
            self.assertEqual(result["mode_name"], "SPEED")
            self.assertEqual(result["current_available"], bit == 1)

    def test_diagnostic_fields_and_flags(self):
        result = can.decode_feedback(0x281, bytes.fromhex("A1 0B 78 56 34 12 AB 7F"))
        self.assertEqual(result["faults"], 0x12345678)
        self.assertEqual(result["board_faults_low8"], 0xAB)
        self.assertEqual(result["result_name"], "CANCELLED")
        for name in ("bus_off", "error_passive", "rx_loss", "require_stop", "request_pending",
                     "not_initialized", "tx_loss"):
            self.assertTrue(result[name])

    def test_parameter_metadata_and_all_write_boundaries(self):
        self.assertEqual(set(can.PARAMETERS), set(range(1, 10)))
        self.assertEqual(list(can.Param), [can.Param(index) for index in range(1, 10)])
        expected = (("current_kp", 0, 5, "V/A"), ("current_ki", 0, 2000, "V/A/s"),
                    ("iq_limit", 0.01, 3.3, "A"), ("voltage_limit", 0.1, 5, "V"),
                    ("iq_slew", 0.01, 10, "A/s"), ("speed_kp", 0, 1, "A/(rad/s)"),
                    ("speed_ki", 0, 10, "A/rad"), ("speed_limit", 0.1, 30, "rad/s"),
                    ("bus_voltage", 8, 30, "V"))
        for param_id, (name, lower, upper, unit) in enumerate(expected, 1):
            self.assertEqual({key: can.PARAMETERS[param_id][key]
                              for key in ("id", "name", "min", "max", "unit")},
                             {"id": param_id, "name": name, "min": lower, "max": upper, "unit": unit})
            for value in (lower, upper):
                with self.subTest(param_id=param_id, boundary=value):
                    identifier, data = can.pack_param_write(63, 255, param_id, value)
                    self.assertEqual(identifier, 0x23F)
                    self.assertEqual(data, struct.pack("<BBBBf", 0xA1, 0x11, 255, param_id, value))
                    decoded = can.decode_parameter(0x4BF,
                                                   struct.pack("<BBBBf", 0xA1, param_id, 255, 0, value), 63)
                    self.assertEqual(decoded["value"], struct.unpack("<f", data[4:])[0])
            for value in (Decimal(str(lower)) - Decimal("0.000000000000000000001"),
                          Decimal(str(upper)) + Decimal("0.000000000000000000001")):
                with self.subTest(param_id=param_id, outside=value), self.assertRaises(ValueError):
                    can.pack_param_write(1, 0, param_id, value)

    def test_parameter_read_and_float32_little_endian(self):
        self.assertEqual(can.pack_param_read(1, 8, can.Param.BUS_VOLTAGE),
                         (0x201, bytes.fromhex("A1 10 08 09 00 00 00 00")))
        self.assertEqual(can.pack_param_write(1, 9, can.Param.CURRENT_KP, "0.2"),
                         (0x201, bytes.fromhex("A1 11 09 01 CD CC 4C 3E")))
        decoded = can.decode_feedback(0x481, bytes.fromhex("A1 01 09 00 CD CC 4C 3E"))
        self.assertEqual(decoded["type"], "parameter")
        self.assertEqual(decoded["param_id"], 1)
        self.assertEqual(decoded["name"], "current_kp")
        self.assertEqual(decoded["sequence"], 9)
        self.assertEqual(decoded["result"], 0)
        self.assertEqual(decoded["result_name"], "OK")
        self.assertEqual(decoded["unit"], "V/A")
        self.assertEqual(decoded["value"], struct.unpack("<f", bytes.fromhex("CD CC 4C 3E"))[0])
        # A failed request may report zero even for a parameter whose minimum is positive.
        rejected = can.decode_parameter(0x481, struct.pack("<BBBBf", 0xA1, 9, 0, 0x84, 0.0))
        self.assertEqual(rejected["result_name"], "REQUIRE_STOP")
        self.assertEqual(rejected["value"], 0.0)
        unknown = can.decode_parameter(0x481, struct.pack("<BBBBf", 0xA1, 1, 0, 0xFF, 0.0))
        self.assertEqual(unknown["result_name"], "UNKNOWN_0xFF")

    def test_parameter_commands_reject_invalid_types_nonfinite_ids_and_ranges(self):
        for param_id in (0, 10, 255, -1, 256, True, 1.0, "1", None):
            with self.subTest(param_id=param_id), self.assertRaises(ValueError):
                can.pack_param_read(1, 0, param_id)
            with self.subTest(param_id=param_id), self.assertRaises(ValueError):
                can.pack_param_write(1, 0, param_id, 0.2)
        for value in (float("nan"), float("inf"), -float("inf"), "NaN", "sNaN", "Infinity",
                      "-Infinity", Decimal("NaN"), "1e10000", "-1e10000", True, None,
                      object(), complex(1, 2), "invalid"):
            with self.subTest(value=value), self.assertRaises(ValueError):
                can.pack_param_write(1, 0, can.Param.CURRENT_KP, value)
        for node, sequence in ((0, 0), (64, 0), (True, 0), (1, -1), (1, 256), (1, False)):
            with self.subTest(node=node, sequence=sequence), self.assertRaises(ValueError):
                can.pack_param_read(node, sequence, 1)
            with self.subTest(node=node, sequence=sequence), self.assertRaises(ValueError):
                can.pack_param_write(node, sequence, 1, 0.2)
        for opcode in (can.Opcode.PARAM_READ, can.Opcode.PARAM_WRITE):
            with self.subTest(opcode=opcode), self.assertRaisesRegex(ValueError, "pack_param_"):
                can.pack_command(1, opcode, 0)

    def test_parameter_responses_reject_unknown_id_nonfinite_and_success_outside_range(self):
        for param_id in (0, 10, 255):
            with self.subTest(param_id=param_id), self.assertRaises(ValueError):
                can.decode_parameter(0x481, struct.pack("<BBBBf", 0xA1, param_id, 0, 0, 0.0))
        for value in (float("nan"), float("inf"), -float("inf")):
            for result in (0, 0x84):
                with self.subTest(value=value, result=result), self.assertRaises(ValueError):
                    can.decode_parameter(0x481, struct.pack("<BBBBf", 0xA1, 1, 0, result, value))
        for param_id, value in ((1, -0.1), (1, 5.000001), (9, 0), (9, 30.00001)):
            with self.subTest(param_id=param_id, value=value), self.assertRaises(ValueError):
                can.decode_parameter(0x481, struct.pack("<BBBBf", 0xA1, param_id, 0, 0, value))

    def test_trace_page_fields_signed_values_and_units(self):
        page0 = can.decode_feedback(0x581, struct.pack("<BBhhh", 0xA1, 0, -123, 1000, -1234))
        self.assertEqual(page0, {"type": "trace", "id": 0x581, "node": 1, "page": 0,
                                 "id_a": -0.123, "iq_reference_a": 1.0, "vd_v": -1.234})
        page1 = can.decode_trace(0x581, struct.pack("<BBhhh", 0xA1, 1, -5000, 0, 4095))
        self.assertEqual(page1["vq_v"], -5.0)
        self.assertEqual(page1["raw_u"], 0)
        self.assertEqual(page1["raw_w"], 4095)
        page2 = can.decode_feedback(0x5BF, struct.pack("<BBhhh", 0xA1, 2, -3142, 249, 14000), 63)
        self.assertEqual(page2["angle_rad"], -3.142)
        self.assertEqual(page2["max_isr_us"], 249)
        self.assertEqual(page2["bus_voltage_v"], 14.0)
        for page in (3, 255):
            with self.subTest(page=page), self.assertRaises(ValueError):
                can.decode_trace(0x581, struct.pack("<BBhhh", 0xA1, page, 0, 0, 0))

    def test_parameter_trace_feedback_ids_length_and_magic(self):
        parameter = struct.pack("<BBBBf", 0xA1, 1, 0, 0, 0.2)
        trace = struct.pack("<BBhhh", 0xA1, 0, 0, 0, 0)
        for decoder, identifier, data in ((can.decode_parameter, 0x481, parameter),
                                          (can.decode_trace, 0x581, trace)):
            for bad_id in (identifier - 1, identifier + 1, 0x800, -1, True):
                with self.subTest(decoder=decoder, identifier=bad_id), self.assertRaises(ValueError):
                    decoder(bad_id, data)
            for payload in (b"", data[:-1], data + b"\0", b"\0" * 8, data.hex()):
                with self.subTest(decoder=decoder, payload=payload), self.assertRaises(ValueError):
                    decoder(identifier, payload)
        for wrong_node in (0, 64, True):
            with self.subTest(node=wrong_node), self.assertRaises(ValueError):
                can.decode_feedback(0x481, parameter, wrong_node)

    def test_feedback_ids_length_and_magic(self):
        data = bytes.fromhex("A1 00 00 00 00 00 00 00")
        self.assertEqual(can.decode_feedback(0x1BF, data, node=63)["type"], "status")
        for identifier in (0x201, 0x180, 0x182, 0x282, 0x382, 0x800, -1):
            with self.subTest(identifier=identifier), self.assertRaises(ValueError):
                can.decode_feedback(identifier, data)
        for payload in (b"", data[:-1], data + b"\0", b"\0" * 8, "A100000000000000"):
            with self.subTest(payload=payload), self.assertRaises(ValueError):
                can.decode_feedback(0x181, payload)

    def test_hex_output_and_cli_generates_only(self):
        identifier, data = can.pack_current(1, 1, "0.05")
        self.assertEqual(can.format_frame(identifier, data), "201#A101010132000000")
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            self.assertEqual(can.main(["--node", "1", "--seq", "1", "iq", "0.05"]), 0)
        self.assertIn("A1 01 01 01 32 00 00 00", output.getvalue())
        self.assertIn("201#A101010132000000", output.getvalue())
        self.assertIn("no CAN frame transmitted", output.getvalue())

    def test_cli_requires_sequence_and_rejects_invalid_values(self):
        for arguments in (["iq", "0.05"], ["--seq", "256", "stop"],
                          ["--seq", "0", "iq", "nan"], ["--seq", "0", "speed", "30.001"]):
            with self.subTest(arguments=arguments), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as error:
                    can.main(arguments)
                self.assertEqual(error.exception.code, 2)


if __name__ == "__main__":
    unittest.main()
