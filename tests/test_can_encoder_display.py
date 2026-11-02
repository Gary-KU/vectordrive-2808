"""A missing/invalid/stale encoder must never be displayed as a live zero."""
import math
import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from desktop.telemetry import can_feedback_error, describe_faults, encoder_readout


class EncoderDisplayTests(unittest.TestCase):
    def test_wire_endpoints_do_not_become_measured_rpm(self):
        for fast in (False, True):
            for speed in (-32.768, 32.767):
                with self.subTest(fast=fast, speed=speed):
                    state = self.state()
                    state['status']['velocity_rad_s'] = speed
                    if fast:
                        state['received']['fast'] = 100.
                        state['fast'] = dict(sensor_valid=True, angle_rad=1., velocity_rad_s=speed)
                    value = encoder_readout(state, 100.01)
                    self.assertTrue(value.valid)
                    self.assertTrue(value.velocity_saturated)
                    self.assertIsNone(value.rpm)
                    self.assertIsNone(value.velocity_rad_s)

    def test_inside_wire_boundary_still_displays_speed(self):
        state = self.state()
        state['status']['velocity_rad_s'] = 32.766
        value = encoder_readout(state, 100.01)
        self.assertFalse(value.velocity_saturated)
        self.assertAlmostEqual(value.rpm, 32.766 * 60 / math.tau)

    def state(self):
        return dict(connected=True, status=dict(sensor_valid=True, velocity_rad_s=math.tau),
                    telemetry=dict(angle_rad=math.pi/2), received=dict(status=100., trace_2=99.98))

    def test_valid_mechanical_angle_and_rpm_units(self):
        value = encoder_readout(self.state(), 100.01)
        self.assertTrue(value.valid)
        self.assertAlmostEqual(value.angle_deg, 90)
        self.assertAlmostEqual(value.rpm, 60)
        self.assertAlmostEqual(value.age_ms, 30)

    def test_disconnected_cache_not_shown_as_live(self):
        state = self.state()
        state['connected'] = False
        value = encoder_readout(state, 100.01)
        self.assertEqual(value.quality, 'disconnected')
        self.assertIsNone(value.angle_deg)
        self.assertIsNone(value.rpm)

    def test_invalid_sensor_zero_is_unavailable_not_zero_degrees(self):
        state = self.state()
        state['status']['sensor_valid'] = False
        state['telemetry']['angle_rad'] = 0
        value = encoder_readout(state, 100.01)
        self.assertEqual(value.quality, 'sensor_invalid')
        self.assertIsNone(value.angle_deg)

    def test_missing_angle_page_keeps_speed_but_no_angle(self):
        state = self.state()
        del state['received']['trace_2']
        state['received']['trace_0'] = 100
        value = encoder_readout(state, 100.01)
        self.assertEqual(value.quality, 'waiting_angle')
        self.assertIsNone(value.angle_deg)
        self.assertAlmostEqual(value.rpm, 60)

    def test_fresh_other_pages_do_not_renew_old_angle(self):
        state = self.state()
        state['received'].update(trace_2=99., trace_0=100., trace_1=100.)
        value = encoder_readout(state, 100.01)
        self.assertEqual(value.quality, 'angle_stale')
        self.assertIsNone(value.angle_deg)

    def test_stale_status_hides_angle_even_if_trace_is_fresh(self):
        state = self.state()
        state['received'].update(status=99., trace_2=100.)
        value = encoder_readout(state, 100.01)
        self.assertEqual(value.quality, 'status_stale')
        self.assertIsNone(value.angle_deg)
        self.assertIsNone(value.rpm)

    def test_actual_zero_and_signed_angle_wrap(self):
        for angle, degrees in ((0., 0.), (-math.pi/2, 270.), (math.tau, 0.)):
            with self.subTest(angle=angle):
                state = self.state()
                state['telemetry']['angle_rad'] = angle
                value = encoder_readout(state, 100.01)
                self.assertTrue(value.valid)
                self.assertAlmostEqual(value.angle_deg, degrees)

    def test_nonfinite_angle_and_missing_feedback(self):
        for angle in (None, math.nan, math.inf):
            state = self.state()
            state['telemetry']['angle_rad'] = angle
            self.assertIsNone(encoder_readout(state, 100.01).angle_deg)
        self.assertEqual(encoder_readout(dict(connected=True), 100).quality, 'waiting_status')


class FeedbackHealthTests(unittest.TestCase):
    def test_latched_encoder_and_estop_faults_are_named_individually(self):
        text = describe_faults(0x01080000)
        self.assertIn('编码器反馈异常', text)
        self.assertIn('急停锁存', text)
        self.assertNotIn('CAN', text)
        self.assertEqual(describe_faults(0), '')
        self.assertIn('0x80000000', describe_faults(0x80000000))

    def test_status_and_diagnostic_must_both_be_fresh(self):
        state = dict(connected=True, status=dict(node=2),
                     received=dict(status=100., diagnostic=100., trace_2=100.))
        self.assertEqual(can_feedback_error(state, 100.1), '')
        state['received']['diagnostic'] = 99.
        error = can_feedback_error(state, 100.1)
        self.assertIn('0x282', error)
        self.assertNotIn('0x182', error)
        state['received']['status'] = 99.
        self.assertIn('0x182', can_feedback_error(state, 100.1))

    def test_trace_frames_do_not_hide_a_silent_node(self):
        state = dict(connected=True, received=dict(trace_0=100., trace_1=100., trace_2=100.))
        self.assertIn('250 ms', can_feedback_error(state, 100.1))
        state['connected'] = False
        self.assertEqual(can_feedback_error(state, 100.1), '')


if __name__ == '__main__':
    unittest.main()
