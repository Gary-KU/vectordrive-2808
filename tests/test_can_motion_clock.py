"""Coarse UI ages must not quantize the 100 Hz motion scheduler."""
from types import SimpleNamespace
from unittest import TestCase, mock
from desktop.controller import Controller
from examples.gimbal_can import Opcode


class MotionClockTests(TestCase):
    def test_refresh_while_coarse_clock_has_not_advanced(self):
        status = dict(mode=3, driver_enabled=True, command_timed_out=False,
            sensor_valid=True, sense_confirmed=True, offset_ready=True,
            direction_confirmed=True, zero_valid=True)
        state = SimpleNamespace(_motion=(Opcode.IQ, .05, 0, 3), _ui_tick=100,
            _motion_started=100, _first_ack=0, _status_time=0, _next_send=1.,
            UI_TIMEOUT=.5, MAX_RUN_SECONDS=60, PERIOD=.01,
            _fresh=mock.Mock(return_value=status), _exchange=mock.Mock())
        with mock.patch('desktop.controller.time.monotonic', return_value=100), \
             mock.patch('desktop.controller.time.perf_counter', side_effect=[1.,1.009,1.010]):
            Controller._tick_motion(state)
            Controller._tick_motion(state)
            self.assertEqual(state._exchange.call_count,1)
            Controller._tick_motion(state)
            self.assertEqual(state._exchange.call_count,2)
