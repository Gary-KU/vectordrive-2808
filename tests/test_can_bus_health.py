"""Shared-bus health remains distinct from selected-motor feedback."""
import struct
from collections import deque
import threading
import time
import unittest
from unittest.mock import Mock

from desktop.controller import Controller
from desktop.transports import Frame, SimulatorTransport, decode_bus_health_frame, feedback_filters


def status(node, mode=0):
    return Frame(0x180 + node, struct.pack('<BBBBhh', 0xA1, mode, 14, 0, 0, 0))


def diagnostic(node, flags=0):
    return Frame(0x280 + node, struct.pack('<BBIBB', 0xA1, 0, 0, 0, flags))


class BusHealthCodecTests(unittest.TestCase):
    def test_status_and_diagnostics_from_all_valid_node_addresses(self):
        for node in range(1, 64):
            for kind, frame in (('status', status(node)), ('diagnostic', diagnostic(node))):
                with self.subTest(node=node, kind=kind):
                    observation = decode_bus_health_frame(frame)
                    self.assertEqual(observation['node'], node)
                    self.assertEqual(observation['type'], kind)

    def test_error_extended_remote_or_non_a1_frames_do_not_prove_bus_health(self):
        valid = status(2)
        for frame in (Frame(valid.arbitration_id, valid.data, is_error_frame=True),
                      Frame(valid.arbitration_id, valid.data, is_extended_id=True),
                      Frame(valid.arbitration_id, valid.data, is_remote_frame=True),
                      Frame(valid.arbitration_id, b'\xA2' + valid.data[1:]),
                      Frame(valid.arbitration_id, valid.data[:-1]),
                      Frame(valid.arbitration_id, valid.data + b'\0')):
            with self.subTest(frame=frame):
                self.assertIsNone(decode_bus_health_frame(frame))

    def test_reserved_addresses_and_impossible_payload_fields_are_rejected(self):
        for frame in (status(0), diagnostic(0), status(64), diagnostic(64),
                      status(2, mode=6), status(2, mode=255), diagnostic(2, flags=0x80)):
            with self.subTest(frame=frame):
                self.assertIsNone(decode_bus_health_frame(frame))

    def test_ack_parameter_trace_and_node_info_are_not_bus_health_observations(self):
        for base in (0x380, 0x480, 0x580, 0x600, 0x680, 0x700):
            self.assertIsNone(decode_bus_health_frame(Frame(base + 2, b'\xA1' + bytes(7))))


class SharedBusFilterTests(unittest.TestCase):
    @staticmethod
    def accepts(filters, identifier):
        return any(identifier & item['can_mask'] == item['can_id'] & item['can_mask']
                   for item in filters)

    def test_filters_observe_all_nodes_status_and_diagnostic_only(self):
        for selected_node in (1, 2, 63):
            filters = feedback_filters(selected_node)
            self.assertEqual(len(filters), 8)
            self.assertTrue(all(item['extended'] is False for item in filters))
            for node in range(1, 64):
                self.assertTrue(self.accepts(filters, 0x180 + node))
                self.assertTrue(self.accepts(filters, 0x280 + node))
                for base in (0x380, 0x480, 0x580, 0x600, 0x680, 0x700):
                    self.assertEqual(self.accepts(filters, base + node), node == selected_node)
            for unrelated in (0x080, 0x200 + selected_node, 0x11, 0x12, 0x700, 0x7FF):
                self.assertFalse(self.accepts(filters, unrelated))

    def test_invalid_node_does_not_build_filters(self):
        for node in (0, 64, -1, True, 1.0, '1'):
            with self.subTest(node=node), self.assertRaises(ValueError):
                feedback_filters(node)


class ControllerBusHealthTests(unittest.TestCase):
    @staticmethod
    def receiver(frame):
        """Exercise the owner receive path without starting its worker or I/O."""
        controller = Controller.__new__(Controller)
        controller._lock = threading.RLock()
        controller._events = deque()
        controller._state = dict(connected=True, status={}, diagnostic={}, telemetry={},
                                 received={}, mode='DISCONNECTED')
        controller._node = 2
        controller._pending = (0, 7, None)
        controller._ack = controller._param = None
        controller._last_bus_receive = controller._last_node_receive = float('-inf')
        controller._seen_bus_nodes = {}
        controller._bus = Mock()
        controller._bus.recv.return_value = frame
        return controller

    def test_other_nodes_prove_bus_health_without_filling_selected_feedback(self):
        for frame in (status(1), diagnostic(63)):
            with self.subTest(frame=frame):
                controller = self.receiver(frame)
                controller._receive()
                observed = decode_bus_health_frame(frame)['node']
                self.assertIn(observed, controller._seen_bus_nodes)
                self.assertGreater(controller._last_bus_receive, float('-inf'))
                self.assertEqual(controller._last_node_receive, float('-inf'))
                self.assertEqual(controller._state['status'], {})
                self.assertEqual(controller._state['diagnostic'], {})
                self.assertEqual(controller._state['received'], {})
                self.assertIsNone(controller._ack)

    def test_another_nodes_matching_ack_sequence_never_acknowledges_our_command(self):
        controller = self.receiver(Frame(0x381, struct.pack('<BBBBI', 0xA1, 0, 7, 0, 0)))
        controller._receive()
        self.assertIsNone(controller._ack)
        self.assertEqual(controller._last_node_receive, float('-inf'))
        self.assertEqual(controller._last_bus_receive, float('-inf'))

    def test_malformed_foreign_status_does_not_refresh_bus_health(self):
        controller = self.receiver(status(1, mode=255))
        controller._receive()
        self.assertEqual(controller._last_bus_receive, float('-inf'))
        self.assertEqual(controller._seen_bus_nodes, {})

    def test_missing_selected_node_does_not_reset_adapter_used_by_healthy_node(self):
        class FastController(Controller):
            CONNECT_ATTEMPTS = 1
            AUTO_RETRY_DELAYS = (.02, .03, .04)

        helper = Mock()
        destinations, buses = [], []

        class OtherNodeBus:
            def __init__(self, config):
                # Node 1 is alive; the selected node 2 is absent throughout.
                self.inner = SimulatorTransport(dict(config, node=1))
                buses.append(self)

            def send(self, frame, timeout):
                destinations.append(frame.arbitration_id)
                self.inner.send(frame, timeout)

            def recv(self, timeout):
                return self.inner.recv(timeout)

            def shutdown(self):
                self.inner.shutdown()

        controller = FastController(OtherNodeBus, device_recovery=helper)
        try:
            controller.connect(dict(interface='pcan', channel='PCAN_USBBUS1', node=2, bitrate=1000000))
            deadline = time.monotonic() + 4
            while time.monotonic() < deadline:
                state = controller.snapshot()
                if controller._auto_failures >= 3:
                    break
                time.sleep(.005)
            else:
                self.fail(repr(controller.snapshot()))
            helper.assert_not_called()
            self.assertFalse(state['session_ready'])
            self.assertEqual(state['status'], {})
            self.assertEqual(set(destinations), {0x202})
            self.assertIn(1, controller._seen_bus_nodes)
            self.assertGreaterEqual(len(buses), 3)
        finally:
            controller.close()
            self.assertTrue(controller.wait_closed(2))


if __name__ == '__main__':
    unittest.main()
