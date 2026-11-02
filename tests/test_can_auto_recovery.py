"""Automatic recovery over byte-level fake links; never opens real CAN."""
from collections import deque
import time
import unittest
from unittest.mock import Mock, patch
from test_can_recovery import CONFIG, TrackedBus
from desktop.controller import Controller


class FastController(Controller):
    CONNECT_ATTEMPTS = 1
    CONNECT_RETRY_DELAY = .005
    AUTO_RETRY_DELAYS = (.03, .05, .08, .1)


class AutoRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.controllers = []

    def tearDown(self):
        for controller in self.controllers:
            controller.close()
            self.assertTrue(controller.wait_closed(3))

    def make(self, factory, helper=None):
        controller = FastController(factory, device_recovery=helper or Mock())
        self.controllers.append(controller)
        controller.connect(CONFIG)
        return controller

    def wait(self, controller, predicate, timeout=4):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            state = controller.snapshot()
            if predicate(state):
                return state
            time.sleep(.005)
        self.fail(repr(controller.snapshot()))

    def test_open_failure_recovers_without_device_restart(self):
        commands, calls = [], []
        helper = Mock()
        def factory(config):
            calls.append(config)
            if len(calls) == 1:
                raise OSError('device not enumerated')
            return TrackedBus(config, commands)
        controller = self.make(factory, helper)
        state = self.wait(controller, lambda s: s['recovery_count'] == 1)
        self.assertTrue(state['session_ready'])
        self.assertFalse(state['status']['driver_enabled'])
        self.assertEqual(len(calls), 2)
        helper.assert_not_called()
        self.assertEqual(set(commands), {0})

    def test_silent_initial_connection_restarts_device_then_stops(self):
        commands, buses = [], []
        blocked = [True]
        def factory(config):
            bus = TrackedBus(config, commands)
            bus.silent = blocked[0]
            buses.append(bus)
            return bus
        def recover(config, cancel, progress):
            cancel()
            self.assertTrue(all(bus.inner._closed for bus in buses))
            blocked[0] = False
        controller = self.make(factory, recover)
        state = self.wait(controller, lambda s: s['recovery_count'] == 1)
        self.assertEqual(state['device_reset_count'], 1)
        self.assertTrue(state['stop_confirmed'])
        self.assertFalse(state['running'])
        self.assertEqual(set(commands), {0})

    def test_runtime_loss_discards_new_motion_and_parameter_requests(self):
        commands, buses = [], []
        def factory(config):
            bus = TrackedBus(config, commands)
            buses.append(bus)
            return bus
        controller = self.make(factory)
        controller.AUTO_RETRY_DELAYS = (.2,)
        self.wait(controller, lambda s: s['session_ready'])
        buses[0].silent = True
        self.wait(controller, lambda s: s['reconnect_pending'])
        after_loss = len(commands)
        controller.submit('open 20 .5')
        controller.set_parameter(1, .8)
        state = self.wait(controller, lambda s: s['recovery_count'] == 1)
        self.assertEqual(set(commands[after_loss:]), {0})
        self.assertEqual(state['parameters'], {})
        self.assertFalse(state['running'])

    def test_motion_loss_waits_watchdog_and_preserves_faults_without_replay(self):
        commands, buses, resets = [], [], []
        blocked, faults = [False], [0]
        def factory(config):
            bus = TrackedBus(config, commands, faults[0])
            bus.silent = blocked[0]
            buses.append(bus)
            return bus
        def recover(config, cancel, progress):
            resets.append(time.monotonic())
            blocked[0] = False
        controller = self.make(factory, recover)
        self.wait(controller, lambda s: s['status'].get('offset_ready'))
        controller.submit('open 20 .4')
        self.wait(controller, lambda s: s['status'].get('driver_enabled'))
        blocked[0], faults[0] = True, 0x1000000
        buses[0].silent = True
        self.wait(controller, lambda s: s['reconnect_pending'])
        last_motion = controller._last_motion_send
        after_loss = len(commands)
        state = self.wait(controller, lambda s: s['recovery_count'] == 1)
        self.assertGreaterEqual(resets[0] - last_motion, controller.MOTION_QUIET_SECONDS)
        self.assertEqual(state['diagnostic']['faults'], 0x1000000)
        self.assertFalse(state['running'])
        self.assertFalse(state['status']['driver_enabled'])
        self.assertEqual(set(commands[after_loss:]), {0})

    def test_stop_cancels_pending_reconnect(self):
        buses, commands = [], []
        def factory(config):
            bus = TrackedBus(config, commands)
            bus.silent = True
            buses.append(bus)
            return bus
        controller = self.make(factory)
        controller.AUTO_RETRY_DELAYS = (.5,)
        self.wait(controller, lambda s: s['reconnect_pending'])
        controller.stop()
        count = len(buses)
        time.sleep(.1)
        state = controller.snapshot()
        self.assertFalse(state['reconnect_pending'])
        self.assertFalse(state['auto_reconnect_enabled'])
        self.assertEqual(len(buses), count)

    def test_disconnect_cancels_active_automatic_device_restart(self):
        commands, buses = [], []
        import threading
        entered = threading.Event()
        def factory(config):
            bus = TrackedBus(config, commands)
            bus.silent = True
            buses.append(bus)
            return bus
        def recover(config, cancel, progress):
            entered.set()
            while True:
                cancel()
                time.sleep(.005)
        controller = self.make(factory, recover)
        self.assertTrue(entered.wait(3))
        controller.disconnect()
        self.wait(controller, lambda s: not s['recovering'] and not s['connected'])
        count = len(buses)
        time.sleep(.1)
        self.assertEqual(len(buses), count)
        self.assertFalse(controller.snapshot()['auto_reconnect_enabled'])

    def test_broken_native_close_does_not_kill_owner_thread(self):
        commands, buses = [], []
        def factory(config):
            bus = TrackedBus(config, commands)
            buses.append(bus)
            return bus
        controller = self.make(factory)
        self.wait(controller, lambda s: s['session_ready'])
        original_close = buses[0].shutdown
        def broken_close():
            original_close()
            raise OSError('USB handle invalid')
        buses[0].shutdown = broken_close
        buses[0].silent = True
        self.wait(controller, lambda s: s['recovery_count'] == 1)
        self.assertTrue(controller._thread.is_alive())

    def test_command_rejection_does_not_restart_or_reconnect(self):
        commands, buses = [], []
        helper = Mock()
        def factory(config):
            bus = TrackedBus(config, commands)
            buses.append(bus)
            return bus
        controller = self.make(factory, helper)
        self.wait(controller, lambda s: s['session_ready'])
        controller.submit('iq .1')  # direction/alignment not performed
        self.wait(controller, lambda s: bool(s['error']))
        time.sleep(.1)
        helper.assert_not_called()
        self.assertEqual(len(buses), 1)
        self.assertFalse(controller.snapshot()['reconnect_pending'])

    def test_persistent_silence_uses_backoff_and_limits_device_resets(self):
        commands = []
        helper = Mock()
        def factory(config):
            bus = TrackedBus(config, commands)
            bus.silent = True
            return bus
        controller = self.make(factory, helper)
        self.wait(controller, lambda s: controller._auto_failures >= 3)
        self.assertEqual(helper.call_count, 1)
        self.assertTrue(controller._thread.is_alive())
        self.assertEqual(set(commands), {0})

    def test_valid_node_feedback_without_command_ack_never_resets_usb(self):
        commands = []
        helper = Mock()
        def factory(config):
            bus = TrackedBus(config, commands)
            bus.drop_ack = True
            return bus
        controller = self.make(factory, helper)
        self.wait(controller, lambda s: controller._auto_failures >= 3)
        helper.assert_not_called()

    def test_local_active_motor_suspends_automatic_recovery(self):
        commands, buses = [], []
        helper = Mock()
        def factory(config):
            bus = TrackedBus(config, commands)
            buses.append(bus)
            return bus
        controller = self.make(factory, helper)
        state = self.wait(controller, lambda s: s['session_ready'])
        buses[0].silent = True
        controller._update(status=dict(state['status'], driver_enabled=True, can_control=False))
        state = self.wait(controller, lambda s: not s['connected'])
        self.assertFalse(state['auto_reconnect_enabled'])
        helper.assert_not_called()

    def test_reset_policy_has_cooldown_budget_and_requires_missing_feedback(self):
        controller = Controller.__new__(Controller)
        controller._device_resets = deque([1.0, 70.0, 140.0])
        controller._last_node_receive = 0
        with patch('desktop.controller.time.monotonic', return_value=205.0):
            self.assertFalse(controller._device_reset_allowed())  # 3 in 5 minutes
        with patch('desktop.controller.time.monotonic', return_value=302.0):
            self.assertTrue(controller._device_reset_allowed())
        controller._device_resets = deque([290.0])
        with patch('desktop.controller.time.monotonic', return_value=302.0):
            self.assertFalse(controller._device_reset_allowed())
        controller._device_resets.clear()
        controller._last_node_receive = 301.9
        with patch('desktop.controller.time.monotonic', return_value=302.0):
            self.assertFalse(controller._device_reset_allowed())
