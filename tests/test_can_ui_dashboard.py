"""Native view tests with a fake controller: never opens a CAN adapter."""
import copy
import math
from pathlib import Path
import sys
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
try:
    import tkinter as tk
    from desktop.ui import FocDesktopUI
except ImportError:
    tk = None


class FakeController:
    def __init__(self):
        self.calls = []
        self.state = {}
    def snapshot(self):
        return copy.deepcopy(self.state)
    def events(self):
        return []
    def submit(self, command):
        self.calls.append(('submit', command))
    def connect(self, config):
        self.calls.append(('connect', config))
    def recover(self, config):
        self.calls.append(('recover', config))
    def read_node(self):
        self.calls.append(('read_node',))
    def change_node(self, node, *, single_board=False):
        self.calls.append(('change_node', node, single_board))
    def read_calibration(self):
        self.calls.append(('read_calibration',))
    def save_calibration(self):
        self.calls.append(('save_calibration',))
    def clear_calibration(self):
        self.calls.append(('clear_calibration',))
    def stop(self):
        self.calls.append(('stop',))
    def estop(self):
        self.calls.append(('estop',))
    def close(self):
        pass


@unittest.skipIf(tk is None, 'Tkinter not installed in this Python')
class DashboardViewTests(unittest.TestCase):
    def setUp(self):
        try:
            self.root = tk.Tk()
        except tk.TclError as error:
            self.skipTest(str(error))
        self.root.withdraw()
        self.controller = FakeController()
        self.ui = FocDesktopUI(self.root, self.controller)
        # Tests advance snapshots explicitly. A recurring live timer makes
        # root.update() depend on desktop load and unrelated window activity.
        for callback in self.root.tk.call('after', 'info'):
            self.root.after_cancel(callback)

    def tearDown(self):
        self.ui._destroying = True
        self.root.update_idletasks()
        for callback in self.root.tk.call('after', 'info'):
            self.root.after_cancel(callback)
        self.root.destroy()

    def ready(self):
        now = time.monotonic()
        return dict(connected=True, interface='simulator', session_ready=True, mode='STOP',
                    running=False, status=dict(mode=0, sensor_valid=True, offset_ready=True,
                    sense_confirmed=True, direction_confirmed=True, zero_valid=True,
                    driver_enabled=False, velocity_rad_s=math.tau, iq_a=.05), diagnostic=dict(faults=0),
                    telemetry=dict(angle_rad=math.pi/2), parameters={}, received=dict(
                        status=now, diagnostic=now, trace_0=now, trace_1=now, trace_2=now))

    def test_opens_disconnected_without_side_effects(self):
        self.assertEqual(self.controller.calls, [])
        self.assertEqual(self.ui.interface.get(), 'PCAN')
        self.assertEqual(self.ui.channel.get(), 'PCAN_USBBUS1')
        self.assertTrue(self.ui.start_button.instate(['disabled']))
        self.assertEqual(len(self.ui.notebook.tabs()), 4)
        self.assertFalse(self.ui.encoder_dial.readout.valid)

    def test_clear_saved_calibration_requires_explicit_confirmation(self):
        state = self.ready()
        state['calibration_info'] = dict(stored_valid=True, sensor_direction=1,
                                         electrical_zero_rad=1.25, runtime_valid=True)
        self.ui._render_snapshot(state)
        with patch('desktop.ui.messagebox.askyesno', return_value=False) as confirm:
            self.ui.clear_calibration_button.invoke()
        self.assertEqual(confirm.call_count, 1)
        self.assertEqual(self.controller.calls, [])
        with patch('desktop.ui.messagebox.askyesno', return_value=True):
            self.ui.clear_calibration_button.invoke()
        self.assertEqual(self.controller.calls, [('clear_calibration',)])

    def test_current_editor_sends_host_range_and_displays_board_limit(self):
        state = self.ready()
        state['parameters'] = {3: 1.0}
        self.ui._render_snapshot(state)
        self.assertAlmostEqual(float(self.ui.target_entry.cget('from')), -3.3)
        self.assertAlmostEqual(float(self.ui.target_entry.cget('to')), 3.3)
        self.assertIn('3.3', self.ui.target_unit.get())
        self.assertIn('±1 A', self.ui.current_limit_text.get())
        # A read-back 1 A limit must refuse a +/-3.3 A target and name the limit.
        with patch('desktop.ui.messagebox.showwarning') as warning:
            for target in ('3.3', '-3.3'):
                self.ui.iq_target.set(target)
                self.ui._start_target()
            self.assertEqual(self.controller.calls, [])
        self.assertEqual(warning.call_count, 2)
        self.assertIn('±1 A', warning.call_args[0][1])
        # After parameter 3 is raised to 3.3 A the same targets are accepted.
        state['parameters'] = {3: 3.3}
        self.ui._render_snapshot(state)
        for target in ('3.3', '-3.3'):
            self.ui.iq_target.set(target)
            self.ui._start_target()
        self.assertEqual(self.controller.calls, [('submit', 'iq 3.300000'), ('submit', 'iq -3.300000')])

    def test_target_above_readback_limit_is_refused_and_at_limit_allowed(self):
        state = self.ready()
        state['parameters'] = {3: 3.299999952316284}
        self.ui._render_snapshot(state)
        self.ui.iq_target.set('3.3')
        self.assertIn('±3.3 A', self.ui.current_limit_text.get())
        self.ui._start_target()
        self.assertEqual(self.controller.calls, [('submit', 'iq 3.300000')])
        self.controller.calls.clear()
        state['parameters'] = {3: .5}
        self.ui._render_snapshot(state)
        self.assertFalse(self.ui.start_button.instate(['disabled']))
        with patch('desktop.ui.messagebox.showwarning') as warning:
            self.ui._start_target()
        self.assertEqual(self.controller.calls, [])
        self.assertEqual(warning.call_count, 1)
        self.assertIn('±0.5 A', warning.call_args[0][1])
        self.assertTrue(warning.call_args[0][0])

    def test_parameter_editor_does_not_grant_current_limit_and_unknown_is_labeled(self):
        state = self.ready()
        state['parameters'] = {3: 1.0}
        self.ui._render_snapshot(state)
        self.ui.parameter_values[3].set('3.3')
        self.ui.iq_target.set('3.3')
        self.assertIn('±1 A', self.ui.current_limit_text.get())
        self.assertFalse(self.ui.start_button.instate(['disabled']))
        self.assertEqual(self.controller.calls, [])
        state['parameters'] = {}
        self.ui._render_snapshot(state)
        self.assertIn('未确认', self.ui.current_limit_text.get())
        self.assertNotIn('±3.3', self.ui.current_limit_text.get())
        self.assertFalse(self.ui.start_button.instate(['disabled']))

    def test_current_limit_note_is_hidden_in_other_modes_and_stop_remains_available(self):
        self.ui._render_snapshot(self.ready())
        self.ui.mode_choice.set('开环 · 电角速度')
        self.ui._select_mode()
        self.assertEqual(self.ui.current_limit_label.winfo_manager(), '')
        self.ui.mode_choice.set('电流环 · Iq')
        self.ui._select_mode()
        self.assertEqual(self.ui.current_limit_label.winfo_manager(), 'grid')
        self.ui.stop_button.invoke()
        self.assertEqual(self.controller.calls, [('stop',)])

    def test_request_limit_and_independent_phase_protection_are_shown_separately(self):
        state = self.ready()
        state['parameters'] = {3: 3.299999952316284}
        state['limits_info'] = dict(request_max_a=3.3, phase_trip_a=1.5)
        self.ui._render_snapshot(state)
        self.ui.iq_target.set('2')
        self.assertIn('目标上限：±3.3 A', self.ui.current_limit_text.get())
        self.assertIn('相电流保护：1.5 A', self.ui.current_limit_text.get())
        self.assertFalse(self.ui.start_button.instate(['disabled']))
        self.assertEqual(self.controller.calls, [])
        state['parameters'] = {3: 1.0}
        self.ui._render_snapshot(state)
        self.assertIn('目标上限：±1 A', self.ui.current_limit_text.get())
        self.assertFalse(self.ui.start_button.instate(['disabled']))

    def test_recovery_is_explicit_and_busy_state_blocks_controls(self):
        self.ui._render_snapshot({})
        self.assertFalse(self.ui.recover_button.instate(['disabled']))
        self.ui.recover_button.invoke()
        self.assertEqual(self.controller.calls, [('recover', dict(interface='pcan', channel='PCAN_USBBUS1', bitrate=1000000, node=1))])
        self.controller.calls.clear()
        self.ui._render_snapshot(dict(recovering=True, connecting=False, connection_stage='重启 PCAN-USB'))
        self.assertIn('重启 PCAN-USB', self.ui.banner.cget('text'))
        self.assertTrue(self.ui.recover_button.instate(['disabled']))
        self.assertTrue(self.ui.connect_button.instate(['disabled']))
        self.assertTrue(self.ui.start_button.instate(['disabled']))
        self.ui.stop_button.invoke()
        self.assertEqual(self.controller.calls, [('stop',)])
        state = self.ready()
        state.update(interface='pcan', running=True)
        state['status']['driver_enabled'] = True
        self.ui._render_snapshot(state)
        self.assertTrue(self.ui.recover_button.instate(['disabled']))

    def test_long_error_keeps_stop_controls_visible_in_small_window(self):
        self.root.geometry('1180x780')
        self.root.deiconify()
        self.controller.state = dict(error='PCAN 设备异常：' + '接收通道暂时不可用。' * 50)
        self.ui._render_snapshot(self.controller.state)
        self.root.update()
        self.assertLessEqual(len(self.ui.error_text.get().splitlines()), 3)
        self.assertIn('详见终端', self.ui.error_text.get())
        for button in (self.ui.stop_button, self.ui.estop_button):
            self.assertGreaterEqual(button.winfo_height(), 24)
            self.assertLessEqual(button.winfo_rooty() + button.winfo_height(),
                                 self.root.winfo_rooty() + self.root.winfo_height())

    def test_automatic_reconnect_wait_is_visible_and_can_be_disconnected(self):
        self.ui._render_snapshot(dict(reconnect_pending=True, auto_reconnect_enabled=True,
                                      connection_stage='自动重连 2 · 等待 2 秒', recovery_count=1))
        self.assertIn('自动重连', self.ui.banner.cget('text'))
        self.assertTrue(self.ui.start_button.instate(['disabled']))
        self.assertTrue(self.ui.connect_button.instate(['disabled']))
        self.assertFalse(self.ui.disconnect_button.instate(['disabled']))
        self.assertIn('自动恢复 开', self.ui.footer.get())
        self.ui.stop_button.invoke()
        self.assertEqual(self.controller.calls, [('stop',)])

    def test_node_id_controls_require_idle_single_board_and_update_connection_address(self):
        self.ui._render_snapshot(self.ready())
        self.assertTrue(self.ui.save_node_button.instate(['disabled']))
        self.ui.single_board.set(True)
        self.ui.new_node.set('7')
        self.ui._render_snapshot(self.ready())
        self.assertFalse(self.ui.save_node_button.instate(['disabled']))
        self.ui.save_node_button.invoke()
        self.assertEqual(self.controller.calls, [('change_node', 7, True)])
        self.ui._handle_events([dict(type='node_changed', old_node=1, node=7)])
        self.assertEqual(self.ui.node.get(), '7')
        self.assertFalse(self.ui.single_board.get())
        self.ui._render_snapshot(dict(node_changing=True, connection_stage='保存 ID'))
        self.assertIn('修改节点 ID', self.ui.banner.cget('text'))
        self.assertTrue(self.ui.save_node_button.instate(['disabled']))
        self.assertTrue(self.ui.start_button.instate(['disabled']))
        self.assertFalse(self.ui.disconnect_button.instate(['disabled']))

    def test_fast_encoder_frame_overrides_trace_and_invalid_fast_does_not_show_cached_angle(self):
        state = self.ready()
        state['interface'] = 'pcan'
        state['received']['fast'] = time.monotonic()
        state['fast'] = dict(angle_rad=1.0, velocity_rad_s=2.0, sensor_valid=True)
        self.ui._render_snapshot(state)
        self.assertAlmostEqual(self.ui.encoder_dial.readout.angle_rad, 1.0)
        self.assertIn('快速反馈', self.ui.angle_detail.get())
        state['fast']['sensor_valid'] = False
        self.ui._render_snapshot(state)
        self.assertFalse(self.ui.encoder_dial.readout.valid)

    def test_dropdown_focus_does_not_break_live_refresh(self):
        self.root.deiconify()
        self.root.update()
        selector = self.ui.mode_selector
        self.root.tk.call('ttk::combobox::Post', selector._w)
        try:
            self.root.update()
            popup = self.root.tk.call('ttk::combobox::PopdownWindow', selector._w)
            self.root.tk.call('focus', '-force', f'{popup}.f.l')
            self.root.update()
            self.assertIn('popdown', str(self.root.tk.call('focus')))
            self.controller.state = self.ready()
            self.ui._poll()
            self.assertEqual(self.ui._local_error, '')
            self.assertTrue(self.ui.encoder_dial.readout.valid)
            self.assertEqual(self.controller.calls, [])
        finally:
            self.root.tk.call('ttk::combobox::Unpost', selector._w)

    def test_valid_encoder_and_stale_sensor_states(self):
        state = self.ready()
        self.ui._render_snapshot(state)
        self.assertAlmostEqual(self.ui.encoder_dial.readout.angle_deg, 90)
        self.assertEqual(self.ui.metric_values['rpm'].get(), '60.0')
        state['status']['sensor_valid'] = False
        self.ui._render_snapshot(state)
        self.assertFalse(self.ui.encoder_dial.readout.valid)
        self.assertIn('无效', self.ui.angle_quality.get())
        self.assertEqual(self.ui.angle_radians.get(), '—')
        state['status']['sensor_valid'] = True
        state['received']['trace_2'] -= 1
        self.ui._render_snapshot(state)
        self.assertIn('过期', self.ui.angle_quality.get())

    def test_start_prerequisites_and_explicit_update(self):
        state = self.ready()
        state['status']['zero_valid'] = False
        self.ui._render_snapshot(state)
        self.assertTrue(self.ui.start_button.instate(['disabled']))
        self.assertIn('转子对齐', self.ui.start_hint.get())
        state['status']['zero_valid'] = True
        self.ui._render_snapshot(state)
        self.ui.iq_target.set('.2')
        self.assertEqual(self.controller.calls, [])
        self.ui.start_button.invoke()
        self.assertEqual(self.controller.calls, [('submit', 'iq 0.200000')])
        state['running'] = True
        self.ui._render_snapshot(state)
        self.assertEqual(self.ui.start_button.cget('text'), '更新目标')
        self.assertTrue(self.ui.mode_selector.instate(['disabled']))

    def test_open_mode_has_explicit_distinct_requirements(self):
        state = self.ready()
        state['status'].update(sensor_valid=False, direction_confirmed=False, zero_valid=False)
        self.ui._render_snapshot(state)
        self.assertTrue(self.ui.start_button.instate(['disabled']))
        self.ui.mode_choice.set('开环 · 电角速度')
        self.ui._select_mode()
        self.assertFalse(self.ui.start_button.instate(['disabled']))
        self.assertEqual(self.controller.calls, [])

    def test_fault_disables_start_and_parameter_write(self):
        state = self.ready()
        state['diagnostic']['faults'] = 0x20000
        self.ui._render_snapshot(state)
        self.assertTrue(self.ui.start_button.instate(['disabled']))
        self.assertTrue(self.ui.read_parameters_button.instate(['disabled']))
        self.assertFalse(self.ui.clear_button.instate(['disabled']))
        self.ui.stop_button.invoke()
        self.ui.estop_button.invoke()
        self.assertEqual(self.controller.calls, [('stop',), ('estop',)])

    def test_diagnostic_loss_is_visible_even_with_fresh_status(self):
        state = self.ready()
        state['interface'] = 'pcan'
        state['received']['diagnostic'] -= 1
        self.ui._render_snapshot(state)
        self.assertIn('反馈中断', self.ui.banner.cget('text'))
        self.assertIn('0x281', self.ui.error_text.get())
        self.assertTrue(self.ui.start_button.instate(['disabled']))
        self.assertTrue(self.ui.read_parameters_button.instate(['disabled']))
        state['received']['diagnostic'] = time.monotonic()
        self.ui._render_snapshot(state)
        self.assertIn('反馈正常', self.ui.banner.cget('text'))
        self.assertEqual(self.ui.error_text.get(), '')

    def test_sensor_failure_is_distinct_from_can_failure(self):
        state = self.ready()
        state['interface'] = 'pcan'
        state['status']['sensor_valid'] = False
        self.ui._render_snapshot(state)
        self.assertIn('反馈正常', self.ui.banner.cget('text'))
        self.assertIn('编码器反馈无效', self.ui.error_text.get())
        self.assertNotIn('CAN 反馈中断', self.ui.error_text.get())
        self.assertEqual(self.ui.angle_radians.get(), '—')

    def test_simulated_angle_is_explicitly_labeled(self):
        self.ui._render_snapshot(self.ready())
        self.assertIn('角度不来自板子', self.ui.banner.cget('text'))
        self.assertIn('模拟角度', self.ui.angle_quality.get())
        self.assertIn('SIM', self.ui.angle_detail.get())

    def test_angle_page_loss_does_not_claim_can_is_disconnected(self):
        state = self.ready()
        state['interface'] = 'pcan'
        state['received']['trace_2'] -= 1
        self.ui._render_snapshot(state)
        self.assertIn('反馈正常', self.ui.banner.cget('text'))
        self.assertIn('角度反馈页', self.ui.error_text.get())
        self.assertEqual(self.ui.angle_radians.get(), '—')

    def test_latched_faults_are_readable_and_revoked_session_is_not_green(self):
        state = self.ready()
        state['interface'] = 'pcan'
        state['diagnostic'].update(faults=0x01080000, require_stop=True)
        self.ui._render_snapshot(state)
        self.assertIn('编码器反馈异常', self.ui.error_text.get().replace('\n', ''))
        self.assertIn('急停锁存', self.ui.error_text.get().replace('\n', ''))
        self.assertIn('STOP 握手', self.ui.start_hint.get())
        self.assertTrue(self.ui.flag_labels['session_ready'][0].cget('text').startswith('○'))
        self.assertTrue(self.ui.start_button.instate(['disabled']))
