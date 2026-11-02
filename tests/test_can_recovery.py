"""Recovery failure/cancellation tests; no real adapter or Windows device is used."""
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
import unittest
import uuid
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from desktop.controller import Cancelled, Controller
from desktop.recovery import RecoveryError, _run, restart_pcan_device
from desktop.transports import SimulatorTransport
from desktop.instance import acquire_gui_instance

CONFIG = dict(interface='pcan', channel='PCAN_USBBUS1', bitrate=1000000, node=1)
DEVICE = r'USB\VID_0C72&PID_000C\5&19F74E4B&0&3'


@unittest.skipUnless(os.name == 'nt', 'Windows named instance mutex')
class InstanceTests(unittest.TestCase):
    def test_duplicate_launch_is_refused_and_guard_can_be_reacquired(self):
        name = 'Local\\GimbalCanTest-' + uuid.uuid4().hex
        first = acquire_gui_instance(name)
        self.assertIsNotNone(first)
        try:
            self.assertIsNone(acquire_gui_instance(name))
        finally:
            first.close()
            first.close()
        again = acquire_gui_instance(name)
        self.assertIsNotNone(again)
        again.close()


class DeviceRecoveryTests(unittest.TestCase):
    @patch('desktop.recovery.os.name', 'nt')
    def test_only_unique_classic_pcan_instance_is_restarted(self):
        with patch('desktop.recovery._run', side_effect=[json.dumps([DEVICE]), 'success']) as run:
            progress = Mock()
            self.assertEqual(restart_pcan_device(CONFIG, lambda: None, progress), DEVICE)
        self.assertEqual(run.call_args_list[1].args[0], ['pnputil.exe', '/restart-device', DEVICE])
        self.assertEqual(progress.call_count, 2)

    @patch('desktop.recovery.os.name', 'nt')
    def test_missing_multiple_or_other_device_never_restarts_anything(self):
        for devices in ([], [DEVICE, DEVICE + '2'], [r'USB\VID_0D28&PID_0204\DAP'], [r'USB\VID_0C72&PID_0012\FD']):
            with self.subTest(devices=devices), patch('desktop.recovery._run', return_value=json.dumps(devices)) as run:
                with self.assertRaises(RecoveryError):
                    restart_pcan_device(CONFIG, lambda: None, lambda stage: None)
                self.assertEqual(run.call_count, 1)

    def test_unsupported_interface_or_channel_does_not_launch_process(self):
        for config in (dict(CONFIG, interface='slcan'), dict(CONFIG, channel='PCAN_USBBUS2')):
            with self.subTest(config=config), patch('desktop.recovery._run') as run:
                with self.assertRaises(RecoveryError):
                    restart_pcan_device(config, lambda: None, lambda stage: None)
                run.assert_not_called()

    @patch('desktop.recovery.os.name', 'nt')
    def test_device_restart_denied_is_reported(self):
        with patch('desktop.recovery._run', side_effect=[json.dumps([DEVICE]), RecoveryError('Access denied')]):
            with self.assertRaisesRegex(RecoveryError, 'Access denied'):
                restart_pcan_device(CONFIG, lambda: None, lambda stage: None)

    def test_cancel_terminates_pending_helper(self):
        process = Mock()
        process.communicate.side_effect = [subprocess.TimeoutExpired('helper', .1), ('', '')]
        process.poll.return_value = None
        cancel = Mock(side_effect=[None, Cancelled('stop')])
        with patch('desktop.recovery.subprocess.Popen', return_value=process):
            with self.assertRaises(Cancelled):
                _run(['helper'], cancel)
        process.terminate.assert_called_once()


class TrackedBus:
    def __init__(self, config, commands, faults=0):
        self.inner = SimulatorTransport(config)
        self.inner.faults = faults
        self.inner.mode = 5 if faults else 0
        self.commands = commands
        self.drop_ack = self.drop_parameter = self.silent = False
    def send(self, frame, timeout):
        self.commands.append(frame.data[1])
        self.inner.send(frame, timeout)
    def recv(self, timeout):
        frame = self.inner.recv(timeout)
        if frame is not None and (self.silent or self.drop_ack and frame.arbitration_id == 0x381 or
                                  self.drop_parameter and frame.arbitration_id == 0x481):
            return None
        return frame
    def shutdown(self):
        self.inner.shutdown()


class ControllerRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.controllers = []
    def tearDown(self):
        for controller in self.controllers:
            controller.close()
            self.assertTrue(controller.wait_closed(2))
    def wait(self, controller, predicate, timeout=3):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            state = controller.snapshot()
            if predicate(state):
                return state
            time.sleep(.005)
        self.fail(repr(controller.snapshot()))
    def make(self, helper=None, faults=0):
        buses, commands = [], []
        def factory(config):
            bus = TrackedBus(config, commands, faults)
            buses.append(bus)
            return bus
        controller = Controller(factory, device_recovery=helper or Mock(), auto_reconnect=False)
        controller.CONNECT_RETRY_DELAY = .01
        self.controllers.append(controller)
        controller.connect(CONFIG)
        self.wait(controller, lambda state: state['session_ready'] and not state['connecting'])
        return controller, buses, commands

    def test_recovery_discards_commands_and_preserves_fault_without_motion(self):
        entered, release = threading.Event(), threading.Event()
        def helper(config, cancel, progress):
            entered.set()
            while not release.wait(.005):
                cancel()
            progress('ready')
        controller, buses, commands = self.make(helper, faults=0x1000000)
        controller.recover(CONFIG)
        self.assertTrue(entered.wait(1))
        self.assertTrue(buses[0].inner._closed)
        controller.submit('open 20 .8')
        controller.set_parameter(1, .75)
        controller.read_parameters()
        release.set()
        state = self.wait(controller, lambda state: len(buses) == 2 and state['session_ready'] and not state['recovering'])
        self.assertEqual(state['diagnostic']['faults'], 0x1000000)
        self.assertFalse(state['status']['driver_enabled'])
        self.assertEqual(state['parameters'], {})
        self.assertTrue(all(opcode == 0 for opcode in commands))

    def test_stop_cancels_recovery_and_prevents_reconnect(self):
        entered = threading.Event()
        def helper(config, cancel, progress):
            entered.set()
            while True:
                cancel()
                time.sleep(.005)
        controller, buses, commands = self.make(helper)
        controller.recover(CONFIG)
        self.assertTrue(entered.wait(1))
        controller.stop()
        self.wait(controller, lambda state: not state['recovering'] and not state['connected'])
        self.assertEqual(len(buses), 1)
        self.assertTrue(all(opcode == 0 for opcode in commands))

    def test_running_recovery_is_refused_and_helper_is_never_called(self):
        helper = Mock()
        controller, buses, commands = self.make(helper)
        self.wait(controller, lambda state: state['status'].get('offset_ready'))
        controller.submit('open 20 .4')
        self.wait(controller, lambda state: state['status'].get('driver_enabled'))
        controller.recover(CONFIG)
        self.wait(controller, lambda state: not state['running'] and state['stop_confirmed'])
        helper.assert_not_called()
        self.assertEqual(len(buses), 1)

    def test_connection_retries_are_bounded_and_do_not_reset_device(self):
        calls, commands = [], []
        def factory(config):
            calls.append(config)
            if len(calls) < 3:
                raise RuntimeError('USB not ready')
            return TrackedBus(config, commands)
        helper = Mock()
        controller = Controller(factory, device_recovery=helper)
        controller.CONNECT_RETRY_DELAY = .01
        self.controllers.append(controller)
        controller.connect(CONFIG)
        state = self.wait(controller, lambda state: state['session_ready'] and not state['connecting'])
        self.assertEqual(len(calls), 3)
        helper.assert_not_called()
        self.assertFalse(state['running'])
        self.assertTrue(all(opcode == 0 for opcode in commands))

    def test_disconnect_cancels_connection_retry(self):
        attempts = []
        def factory(config):
            attempts.append(config)
            raise RuntimeError('unplugged')
        controller = Controller(factory)
        controller.CONNECT_RETRY_DELAY = .5
        self.controllers.append(controller)
        controller.connect(CONFIG)
        self.wait(controller, lambda state: state['connecting'])
        controller.disconnect()
        self.wait(controller, lambda state: not state['connecting'] and not state['connected'])
        time.sleep(.1)
        self.assertLessEqual(len(attempts), 1)

    def test_runtime_silence_does_not_automatically_reset_or_restart(self):
        helper = Mock()
        controller, buses, commands = self.make(helper)
        buses[0].silent = True
        state = self.wait(controller, lambda state: not state['connected'] and bool(state['error']))
        self.assertIn('反馈中断', state['error'])
        helper.assert_not_called()
        self.assertEqual(len(buses), 1)

    def test_stop_ack_timeout_does_not_claim_old_parameter_firmware(self):
        controller, buses, commands = self.make()
        buses[0].drop_ack = True
        controller.stop()
        state = self.wait(controller, lambda state: bool(state['error']))
        self.assertIn('ACK 超时', state['error'])
        self.assertIn('0x381', state['error'])
        self.assertNotIn('旧固件', state['error'])
        self.assertFalse(state['stop_confirmed'])

    def test_parameter_ack_and_readback_timeouts_are_distinct(self):
        for missing in ('ack', 'parameter'):
            with self.subTest(missing=missing):
                controller, buses, commands = self.make()
                setattr(buses[0], 'drop_' + missing, True)
                controller.read_parameters()
                state = self.wait(controller, lambda state: bool(state['error']))
                if missing == 'ack':
                    self.assertIn('ACK 超时', state['error'])
                else:
                    self.assertIn('已收到执行 ACK', state['error'])
                    self.assertIn('0x481', state['error'])
                self.assertEqual(state['parameters'], {})


if __name__ == '__main__':
    unittest.main()
