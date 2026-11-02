"""Short +/-50 and +/-100 mA unloaded checks using the guarded current runner."""
import json
from pathlib import Path
import sys
import time
import can
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from examples.run_can_current import CurrentRunner, Settings
from examples import gimbal_can as p


class TimeoutRunner(CurrentRunner):
    def _stream_current(self):
        super()._stream_current()
        last_send = self.clock()
        deadline = last_send+.3
        while self.clock()<deadline:
            self._receive(.005, guard=False)
            if self.status and self.status.get('command_timed_out') and not self.status['driver_enabled']:
                self.timeout_observed_ms = (self.clock()-last_send)*1000
                return
        raise RuntimeError('No disabled/timeout status within 300 ms')


class Recorder:
    def __init__(self, bus):
        self.bus, self.events = bus, []

    def send(self, message, **kwargs):
        self.events.append(dict(t=time.perf_counter(), direction='TX', id=message.arbitration_id, data=bytes(message.data).hex()))
        return self.bus.send(message, **kwargs)

    def recv(self, **kwargs):
        message = self.bus.recv(**kwargs)
        if message:
            self.events.append(dict(t=time.perf_counter(), direction='RX', id=message.arbitration_id, data=bytes(message.data).hex()))
        return message


def main():
    node = int(sys.argv[1])
    output = Path(sys.argv[2])
    bus = can.Bus(interface='pcan', channel='PCAN_USBBUS1', bitrate=1000000)
    recorder = Recorder(bus)
    result = dict(node=node, scope='Unloaded 0.8 s pulses; existing calibration; no parameter writes', runs=[])
    try:
        for index, target in enumerate((.05, -.05, .1, -.1, .05)):
            begin = len(recorder.events)
            runner_class = TimeoutRunner if index==4 else CurrentRunner
            runner = runner_class(recorder, Settings(direction=1, iq_a=target, duration_s=.8,
                node=node, reuse_alignment=True), message_factory=can.Message)
            run = dict(target_a=target, timeout_test=index==4)
            try:
                run.update(runner.run())
                if index==4:
                    run['timeout_observed_after_stream_ms'] = runner.timeout_observed_ms
                run['passed'] = True
            except Exception as exc:
                run.update(passed=False, error=repr(exc), stop_acknowledged=runner.stop_acknowledged)
            run['events'] = recorder.events[begin:]
            result['runs'].append(run)
            if not run['passed']:
                break
            # Drain feedback during stopped settling window.
            end = time.perf_counter()+.5
            while time.perf_counter()<end:
                recorder.recv(timeout=.01)
    finally:
        bus.shutdown()
        result['passed'] = len(result['runs'])==5 and all(r['passed'] for r in result['runs'])
        output.write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(json.dumps({**result, 'runs':[{k:v for k,v in r.items() if k!='events'} for r in result['runs']]}, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
