"""Protocol-level simulator tests and mocked adapters; no real CAN is opened."""
import struct
import types
import unittest
from unittest.mock import Mock, patch

from desktop.transports import Frame, SimulatorTransport, create_transport
from examples import gimbal_can as codec


class Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


class SimulatorTransportTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.bus = SimulatorTransport(clock=self.clock, sleeper=self.clock.advance)
        self.sequence = 0

    def tearDown(self):
        self.bus.shutdown()

    def drain(self):
        frames = []
        while True:
            frame = self.bus.recv(timeout=0)
            if frame is None:
                return frames
            frames.append(frame)

    def decode(self, frames, kind):
        return [decoded for frame in frames
                if (decoded := codec.decode_feedback(frame.arbitration_id, frame.data))["type"] == kind]

    def send(self, opcode, value=0, auxiliary=0, sequence=None):
        if sequence is None:
            sequence = self.sequence
            self.sequence = (self.sequence + 1) & 255
        identifier, payload = codec.pack_command(1, opcode, sequence, value, auxiliary)
        self.bus.send(Frame(identifier, payload))
        frames = self.drain()
        acknowledgements = self.decode(frames, "ack")
        self.assertEqual(len(acknowledgements), 1)
        return acknowledgements[0]

    def parameter(self, number, value=None):
        sequence = self.sequence
        self.sequence = (sequence + 1) & 255
        identifier, payload = (codec.pack_param_read(1, sequence, number) if value is None
                               else codec.pack_param_write(1, sequence, number, value))
        self.bus.send(Frame(identifier, payload))
        frames = self.drain()
        ack = self.decode(frames, "ack")
        response = self.decode(frames, "parameter")
        self.assertEqual(len(ack), 1)
        self.assertEqual(len(response), 1)
        self.assertEqual(ack[0]["sequence"], response[0]["sequence"])
        self.assertEqual(ack[0]["result"], response[0]["result"])
        return ack[0], response[0]

    def ready(self):
        self.clock.advance(0.513)
        self.drain()
        self.assertEqual(self.send(codec.Opcode.STOP)["result"], 0)

    def aligned(self):
        self.ready()
        self.assertEqual(self.send(codec.Opcode.DIRECTION, 1)["result"], 0)
        self.assertEqual(self.send(codec.Opcode.ALIGN)["result"], 0)
        for _ in range(29):
            self.clock.advance(0.05)
            self.assertEqual(self.send(codec.Opcode.ALIGN)["result"], 0)
        self.clock.advance(0.051)
        frames = self.drain()
        status = self.decode(frames, "status")[-1]
        self.assertEqual(status["mode_name"], "STOP")
        self.assertTrue(status["zero_valid"])
        self.assertFalse(status["driver_enabled"])

    def test_default_factory_is_lazy_and_never_starts_motion(self):
        with patch("desktop.transports.importlib.import_module") as importer:
            bus = create_transport({})
            self.assertTrue(bus.simulation_only)
            self.assertFalse(bus.driver_enabled)
            self.assertTrue(bus.require_stop)
            bus.shutdown()
            importer.assert_not_called()

    def test_boot_calibration_and_stop_handshake_gate_motion(self):
        self.assertEqual(self.send(codec.Opcode.OPEN, 20, 0.8)["result"], 0x84)
        self.assertEqual(self.send(codec.Opcode.STOP)["result"], 0)
        self.assertEqual(self.send(codec.Opcode.OPEN, 20, 0.8)["result"], 4)
        self.clock.advance(0.511)
        self.drain()
        self.assertFalse(self.bus.offset_ready)
        self.clock.advance(0.001)
        self.assertEqual(self.send(codec.Opcode.OPEN, 20, 0.8)["result"], 0)
        self.assertTrue(self.bus.driver_enabled)

    def test_rejected_sequence_does_not_renew_watchdog(self):
        self.ready()
        accepted = self.sequence
        self.assertEqual(self.send(codec.Opcode.OPEN, 20, 0.8)["result"], 0)
        self.clock.advance(0.050)
        self.assertEqual(self.send(codec.Opcode.OPEN, 20, 0.8, sequence=accepted)["result"], 0x82)
        self.clock.advance(0.049)
        self.drain()
        self.assertTrue(self.bus.driver_enabled)
        self.clock.advance(0.002)
        self.drain()
        self.assertFalse(self.bus.driver_enabled)
        self.clock.advance(0.010)  # Observe the next scheduled 10 ms feedback.
        frames = self.drain()
        status = self.decode(frames, "status")[-1]
        diagnostic = self.decode(frames, "diagnostic")[-1]
        self.assertFalse(status["driver_enabled"])
        self.assertTrue(status["command_timed_out"])
        self.assertTrue(diagnostic["require_stop"])
        self.assertEqual(self.send(codec.Opcode.OPEN, 20, 0.8)["result"], 0x84)
        self.assertEqual(self.send(codec.Opcode.STOP)["result"], 0)
        self.assertEqual(self.send(codec.Opcode.OPEN, 20, 0.8)["result"], 0)

    def test_forward_sequence_wrap_and_half_range_rejection(self):
        self.ready()
        self.assertEqual(self.send(codec.Opcode.STOP, sequence=254)["result"], 0)
        self.assertEqual(self.send(codec.Opcode.OPEN, 20, 0.8, sequence=255)["result"], 0)
        self.assertEqual(self.send(codec.Opcode.OPEN, 20, 0.8, sequence=0)["result"], 0)
        for sequence in (0, 255, 128):
            self.assertEqual(self.send(codec.Opcode.OPEN, 20, 0.8, sequence=sequence)["result"], 0x82)
        self.assertEqual(self.send(codec.Opcode.OPEN, 20, 0.8, sequence=1)["result"], 0)

    def test_direction_alignment_and_current_require_real_command_stream(self):
        self.ready()
        self.assertEqual(self.send(codec.Opcode.IQ, 0.2)["result"], 7)
        self.assertEqual(self.send(codec.Opcode.DIRECTION, -1)["result"], 0)
        self.assertEqual(self.send(codec.Opcode.IQ, 0.2)["result"], 8)
        self.assertEqual(self.send(codec.Opcode.ALIGN)["result"], 0)
        self.clock.advance(0.101)
        self.drain()
        self.assertFalse(self.bus.zero_valid)
        self.assertTrue(self.bus.require_stop)
        self.aligned()
        self.assertEqual(self.send(codec.Opcode.IQ, 0.2)["result"], 0)
        pages = set()
        for _ in range(6):
            self.clock.advance(0.02)
            frames = self.drain()
            for sample in self.decode(frames, "trace"):
                pages.add(sample["page"])
            self.assertEqual(self.send(codec.Opcode.IQ, 0.2)["result"], 0)
        self.assertEqual(pages, {0, 1, 2})
        self.assertGreater(self.bus.iq, 0)
        self.assertEqual(self.send(codec.Opcode.SPEED, 1)["result"], 5)

    def test_parameters_read_back_float32_and_write_only_stopped(self):
        self.ready()
        for number, expected in self.bus.DEFAULT_PARAMETERS.items():
            ack, response = self.parameter(number)
            self.assertEqual(ack["result"], 0)
            self.assertAlmostEqual(response["value"], expected, places=5)
        ack, response = self.parameter(3, 0.2)
        self.assertEqual(ack["result"], 0)
        self.assertEqual(response["value"], struct.unpack("<f", struct.pack("<f", 0.2))[0])
        self.assertEqual(self.send(codec.Opcode.OPEN, 20, 0.8)["result"], 0)
        ack, response = self.parameter(3, 0.5)
        self.assertEqual(ack["result"], 5)
        self.assertAlmostEqual(response["value"], 0.2, places=6)

    def test_parameter_query_and_rejected_write_are_not_keepalives(self):
        self.ready()
        self.assertEqual(self.send(codec.Opcode.OPEN, 20, 0.8)["result"], 0)
        self.clock.advance(0.040)
        self.assertEqual(self.parameter(1)[0]["result"], 0)
        self.clock.advance(0.040)
        self.assertEqual(self.parameter(1, 2)[0]["result"], 5)
        self.clock.advance(0.021)
        self.drain()
        self.assertFalse(self.bus.driver_enabled)
        self.assertTrue(self.bus.command_timed_out)

    def test_parameter_limits_and_zero_preservation(self):
        self.aligned()
        self.assertEqual(self.parameter(1, 2.0)[0]["result"], 0)
        self.assertTrue(self.bus.zero_valid)
        self.assertEqual(self.parameter(9, 15)[0]["result"], 0)
        self.assertFalse(self.bus.zero_valid)
        payload = struct.pack("<BBBBf", 0xA1, 0x11, self.sequence, 3, 3.4)
        self.bus.send(Frame(0x201, payload))
        self.assertEqual(self.decode(self.drain(), "ack")[-1]["result"], 10)
        self.assertEqual(self.bus.parameters[3], 1.0)

    def test_estop_latches_fault_and_clear_requires_calibration(self):
        self.ready()
        self.assertEqual(self.send(codec.Opcode.OPEN, 20, 0.8)["result"], 0)
        self.assertEqual(self.send(codec.Opcode.CALIBRATE)["result"], 5)
        self.assertEqual(self.send(codec.Opcode.ESTOP)["result"], 0)
        self.assertEqual(self.bus.faults, 1 << 24)
        self.assertEqual(self.bus.mode, 5)
        self.assertFalse(self.bus.driver_enabled)
        self.assertEqual(self.parameter(1)[0]["result"], 0)
        self.assertEqual(self.send(codec.Opcode.CLEAR)["result"], 0)
        self.assertFalse(self.bus.offset_ready)
        self.assertFalse(self.bus.zero_valid)
        self.assertEqual(self.bus.faults, 0)
        self.clock.advance(0.512)
        self.drain()
        self.assertTrue(self.bus.offset_ready)
        self.assertFalse(self.bus.driver_enabled)


class HardwareFactoryTests(unittest.TestCase):
    def test_python_can_is_lazy_and_adapter_is_mocked(self):
        fake_bus = Mock()
        fake_bus.m_objPCANBasic.SetValue.return_value = 0
        fake_message = types.SimpleNamespace(arbitration_id=0x181, data=b"12345678",
            is_extended_id=False, is_remote_frame=False, is_error_frame=False)
        fake_bus.recv.return_value = fake_message
        fake_can = types.SimpleNamespace(Bus=Mock(return_value=fake_bus),
                                        Message=Mock(return_value=fake_message))
        with patch("desktop.transports.importlib.import_module", return_value=fake_can) as importer:
            bus = create_transport(dict(interface="pcan", channel="PCAN_USBBUS1", node=2, bitrate=500000))
            importer.assert_called_once_with("can")
            options = fake_can.Bus.call_args.kwargs
            self.assertEqual(options["channel"], "PCAN_USBBUS1")
            self.assertEqual(options["bitrate"], 500000)
            self.assertFalse(options["receive_own_messages"])
            self.assertTrue(options["ignore_config"])
            self.assertEqual({(item["can_id"], item["can_mask"]) for item in options["can_filters"]},
                             {(0x180, 0x7C0), (0x280, 0x7C0), (0x382, 0x7FF),
                              (0x482, 0x7FF), (0x582, 0x7FF), (0x602, 0x7FF), (0x682, 0x7FF), (0x702, 0x7FF)})
            bus.send(Frame(0x202, b"abcdefgh"), timeout=0.01)
            fake_bus.send.assert_called_once_with(fake_message, timeout=0.01)
            self.assertEqual(bus.recv(timeout=0.005), Frame(0x181, b"12345678"))
            bus.shutdown()
            bus.shutdown()
            fake_bus.shutdown.assert_called_once()

    def test_missing_library_and_unknown_interface_never_open_an_adapter(self):
        with patch("desktop.transports.importlib.import_module", side_effect=ImportError) as importer:
            with self.assertRaisesRegex(RuntimeError, "python-can"):
                create_transport(dict(interface="slcan", channel="COM3"))
            self.assertEqual(importer.call_count, 1)
            with self.assertRaises(ValueError):
                create_transport(dict(interface="unknown"))
            self.assertEqual(importer.call_count, 1)


if __name__ == "__main__":
    unittest.main()
