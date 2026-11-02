"""End-to-end host sessions against byte-level simulation, never hardware."""
import time
import unittest
import threading
from collections import deque
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from desktop.controller import Controller
from desktop.transports import create_transport


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.controller = Controller()
        self.controller.connect({"interface": "simulator", "channel": "", "node": 1, "bitrate": 1000000})
        self.wait(lambda s: s["connected"] and s["session_ready"])

    def tearDown(self):
        self.controller.close()
        self.assertTrue(self.controller.wait_closed(1))

    def wait(self, condition, timeout=2):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            snapshot = self.controller.snapshot()
            if condition(snapshot):
                return snapshot
            time.sleep(.005)
        self.fail("condition timed out: " + repr(self.controller.snapshot()))

    def calibrated(self):
        self.wait(lambda s: s["status"].get("offset_ready"))
        self.controller.submit("dir 1")
        self.wait(lambda s: s["status"].get("direction_confirmed"))
        self.controller.submit("align")
        self.wait(lambda s: s["status"].get("zero_valid") and not s["running"], 3)

    def test_connect_never_starts_motor(self):
        state = self.controller.snapshot()
        self.assertFalse(state["running"])
        self.assertFalse(state["status"]["driver_enabled"])
        self.assertTrue(state["stop_confirmed"])

    def test_encoder_feedback_available_without_starting_or_alignment(self):
        from desktop.telemetry import encoder_readout
        state = self.wait(lambda s: "trace_2" in s["received"])
        encoder = encoder_readout(state)
        self.assertTrue(encoder.valid)
        self.assertEqual(encoder.angle_deg, 0)
        self.assertFalse(state["running"])
        self.controller.disconnect()
        state = self.wait(lambda s: not s["connected"])
        self.assertIsNone(encoder_readout(state).angle_deg)

    def test_parameter_read_write_verified(self):
        self.controller.read_parameters()
        self.wait(lambda s: len(s["parameters"]) == 9)
        self.controller.set_parameter(1, .7)
        state = self.wait(lambda s: abs(s["parameters"].get(1, 0) - .7) < 1e-6)
        self.assertFalse(state["status"]["driver_enabled"])
        self.controller.set_parameter(3, 3.4)
        self.wait(lambda s: bool(s["error"]))
        self.assertEqual(self.controller.snapshot()["parameters"][3], 1)

    def test_uncalibrated_current_does_not_start(self):
        self.controller.submit("iq 0.1")
        state = self.wait(lambda s: bool(s["error"]))
        self.assertFalse(state["running"])
        self.assertFalse(state["status"]["driver_enabled"])

    def test_parameter_batch_checks_all_values_before_first_write(self):
        self.controller.read_parameters()
        self.wait(lambda s: len(s["parameters"]) == 9)
        self.controller.set_parameters({1: .7, 2: 2001})
        state = self.wait(lambda s: bool(s["error"]))
        self.assertEqual(state["parameters"][1], 1)
        self.assertEqual(state["parameters"][2], 1000)

    def test_parameter_batch_confirms_each_value_and_stays_stopped(self):
        self.controller.set_parameters({1: .75, 2: 500})
        state = self.wait(lambda s: s["parameters"].get(1) == .75 and s["parameters"].get(2) == 500 and not s["tuning_busy"])
        self.assertFalse(state["running"])
        self.assertFalse(state["status"]["driver_enabled"])

    def test_stream_reverse_param_interlock_and_stop(self):
        self.calibrated()
        self.controller.submit("iq .1")
        self.wait(lambda s: s["status"].get("mode") == 3 and s["status"].get("iq_a", 0) > .05)
        self.controller.submit("iq -.1")
        self.wait(lambda s: s["status"].get("iq_a", 0) < -.05)
        self.controller.set_parameter(1, .8)
        # Invalid parameter change while running cancels the run conservatively.
        self.wait(lambda s: bool(s["error"]) and not s["running"] and s["stop_confirmed"])
        self.wait(lambda s: not s["status"].get("driver_enabled"))
        self.assertNotEqual(self.controller.snapshot()["parameters"].get(1), .8)

    def test_gui_heartbeat_expiry_stops_stream(self):
        self.calibrated()
        self.controller.submit("iq .05")
        self.wait(lambda s: s["status"].get("mode") == 3)
        time.sleep(.65)  # Deliberately do not renew GUI lease via snapshot().
        state = self.wait(lambda s: not s["running"] and s["stop_confirmed"])
        self.assertIn("500", state["error"])

    def test_estop_latches_and_clear_does_not_restart(self):
        self.controller.estop()
        self.wait(lambda s: bool(s["diagnostic"].get("faults")))
        self.controller.submit("clear")
        self.wait(lambda s: s["diagnostic"].get("faults") == 0)
        self.assertFalse(self.controller.snapshot()["running"])
        self.assertFalse(self.controller.snapshot()["status"]["driver_enabled"])

    def test_stop_cancels_queued_motion(self):
        for _ in range(20):
            self.controller.submit("open 20 .8")
        self.controller.stop()
        time.sleep(.08)
        state = self.controller.snapshot()
        self.assertFalse(state["running"])
        self.assertFalse(state["status"].get("driver_enabled"))


class NoAckBus:
    def __init__(self, config):
        self.inner = create_transport(config)
        self.drop = False

    def send(self, *args, **kwargs):
        return self.inner.send(*args, **kwargs)

    def recv(self, timeout=0):
        frame = self.inner.recv(timeout)
        if self.drop and frame and frame.arbitration_id == 0x381:
            return None
        return frame

    def shutdown(self):
        self.inner.shutdown()


class AckFailureTests(unittest.TestCase):
    def test_feedback_deadline_is_enforced_during_ack_wait(self):
        class SilentBus:
            def send(self, frame, timeout):
                pass
            def recv(self, timeout):
                time.sleep(timeout)
                return None
        # Start a request with feedback already 45 ms old. Missing an ACK must
        # not postpone the 50 ms feedback limit until the 40 ms ACK wait ends.
        controller = Controller.__new__(Controller)
        controller._lock = threading.RLock()
        controller._events = deque()
        controller._stop = threading.Event()
        controller._closing = threading.Event()
        controller._node, controller._sequence = 1, 0
        controller._bus = SilentBus()
        controller._motion = (1, .05, 0, 3)
        controller._ui_tick = time.monotonic()
        controller._status_time = controller._diag_time = time.monotonic() - .045
        controller._state = {"status": {}, "diagnostic": {}, "parameters": {3: 1.0}}
        started = time.monotonic()
        with self.assertRaisesRegex(RuntimeError, "50 ms"):
            controller._exchange(1, .05)
        self.assertLess(time.monotonic() - started, .030)

    def test_close_preserves_latched_estop(self):
        buses = []
        def factory(config):
            bus = create_transport(config)
            buses.append(bus)
            return bus
        controller = Controller(factory)
        controller.connect({"interface": "simulator", "node": 1})
        deadline = time.monotonic() + 1
        while not controller.snapshot()["session_ready"] and time.monotonic() < deadline:
            time.sleep(.005)
        self.assertTrue(controller.snapshot()["session_ready"])
        # Close may bypass the normal worker loop; the final disconnect must
        # still send the stronger, already requested ESTOP before shutting down.
        controller.estop()
        controller.close()
        self.assertTrue(controller.wait_closed(1))
        self.assertNotEqual(buses[0].faults, 0)
        self.assertFalse(buses[0].driver_enabled)

    def test_missing_stop_ack_is_not_confirmed(self):
        buses = []
        def factory(config):
            bus = NoAckBus(config)
            buses.append(bus)
            return bus
        controller = Controller(factory)
        try:
            controller.connect({"interface": "simulator", "node": 1})
            deadline = time.monotonic() + 1
            while not controller.snapshot()["session_ready"] and time.monotonic() < deadline:
                time.sleep(.005)
            self.assertTrue(controller.snapshot()["session_ready"])
            buses[0].drop = True
            controller.stop()
            time.sleep(.2)
            state = controller.snapshot()
            self.assertFalse(state["stop_confirmed"])
            self.assertIn("STOP 未确认", state["error"])
        finally:
            controller.close()
            self.assertTrue(controller.wait_closed(1))


class FeedbackLossTests(unittest.TestCase):
    class FilterBus:
        def __init__(self, config):
            self.inner = create_transport(config)
            self.drop_ids = set()
            self.sent_opcodes = []
        def send(self, frame, timeout):
            self.sent_opcodes.append(frame.data[1])
            self.inner.send(frame, timeout)
        def recv(self, timeout):
            frame = self.inner.recv(timeout)
            return None if frame and frame.arbitration_id in self.drop_ids else frame
        def shutdown(self):
            self.inner.shutdown()

    def wait(self, controller, predicate, timeout=2):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            state = controller.snapshot()
            if predicate(state):
                return state
            time.sleep(.005)
        self.fail(repr(controller.snapshot()))

    def test_stopped_node_silence_or_partial_feedback_loss_disconnects(self):
        for missing in ({0x181}, {0x281}, {0x181, 0x281, 0x381, 0x581}):
            with self.subTest(missing=missing):
                buses = []
                def factory(config):
                    bus = self.FilterBus(config)
                    buses.append(bus)
                    return bus
                controller = Controller(factory)
                try:
                    controller.connect(dict(interface='simulator', node=1))
                    self.wait(controller, lambda s: s['session_ready'])
                    buses[0].drop_ids = missing
                    state = self.wait(controller, lambda s: not s['connected'] and bool(s['error']))
                    self.assertIn('CAN 反馈中断', state['error'])
                    for identifier in missing & {0x181, 0x281}:
                        self.assertIn(f'0x{identifier:03X}', state['error'])
                    self.assertFalse(state['session_ready'])
                    self.assertFalse(state['running'])
                    self.assertFalse(buses[0].inner.driver_enabled)
                    self.assertTrue(all(opcode == 0 for opcode in buses[0].sent_opcodes))
                    # Reconnecting creates a new session; no automatic motion.
                    controller.connect(dict(interface='simulator', node=1))
                    state = self.wait(controller, lambda s: s['session_ready'])
                    self.assertEqual(state['error'], '')
                    self.assertFalse(state['status']['driver_enabled'])
                finally:
                    controller.close()
                    self.assertTrue(controller.wait_closed(1))

    def test_missing_angle_page_keeps_live_can_session(self):
        buses = []
        def factory(config):
            bus = self.FilterBus(config)
            buses.append(bus)
            return bus
        controller = Controller(factory)
        try:
            controller.connect(dict(interface='simulator', node=1))
            self.wait(controller, lambda s: s['session_ready'])
            buses[0].drop_ids = {0x581}
            deadline = time.monotonic() + .4
            while time.monotonic() < deadline:
                state = controller.snapshot()
                time.sleep(.005)
            self.assertTrue(state['connected'])
            self.assertEqual(state['error'], '')
            self.assertFalse(state['running'])
        finally:
            controller.close()
            self.assertTrue(controller.wait_closed(1))

    def test_connect_to_latched_fault_allows_monitoring_and_explicit_clear(self):
        buses = []
        def factory(config):
            bus = create_transport(config)
            bus.faults = 0x01080000
            bus.mode = 5
            buses.append(bus)
            return bus
        controller = Controller(factory)
        try:
            controller.connect(dict(interface='simulator', node=1))
            state = self.wait(controller, lambda s: s['connected'] and s['session_ready'])
            self.assertEqual(state['diagnostic']['faults'], 0x01080000)
            self.assertTrue(state['stop_confirmed'])
            self.assertFalse(state['status']['driver_enabled'])
            self.assertEqual(state['error'], '')
            # Connecting must preserve the fault. Only a separate CLEAR clears it.
            self.assertEqual(buses[0].faults, 0x01080000)
            controller.submit('clear')
            state = self.wait(controller, lambda s: s['diagnostic'].get('faults') == 0)
            self.assertFalse(state['running'])
            self.assertFalse(state['status']['driver_enabled'])
        finally:
            controller.close()
            self.assertTrue(controller.wait_closed(1))


if __name__ == "__main__":
    unittest.main()
