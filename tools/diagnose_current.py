"""Bounded low-duration tests to distinguish low-speed tracking from acceleration.

Explicit invocation permits motion. No firmware, tuning or protection writes.
"""
import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import statistics
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import can
from examples import gimbal_can as p
from examples.run_can_current import CurrentRunner, Settings
from tools.benchmark_current import Recorder
from tools.benchmark_board import summary


def summarize(events, node, target, duration):
    tx = [e for e in events if e['direction']=='TX' and bytes.fromhex(e['data'])[1]==1]
    begin = tx[0]['t']
    currents, ids, vd, vq, angles = [], [], [], [], []
    low = max(abs(target)+.02, duration*.5)  # 1 A/s slew, then a short settling margin.
    for e in events:
        if e['direction']!='RX':
            continue
        f = p.decode_feedback(e['id'],bytes.fromhex(e['data']),node)
        t = e['t']-begin
        if f['type']=='fast' and f['driver_enabled']:
            angles.append((t,f['angle_rad']))
            if low<t<duration:
                currents.append(f['iq_a'])
        if f['type']=='trace' and low<t<duration:
            for key, values in (('id_a',ids),('vd_v',vd),('vq_v',vq)):
                if key in f:
                    values.append(f[key])
    speeds=[((b-a+math.pi)%(2*math.pi)-math.pi)/(tb-ta)
            for (ta,a),(tb,b) in zip(angles,angles[1:]) if tb>ta and low<tb<duration]
    return dict(window_s=[low,duration], samples=len(currents),
        mean_iq_a=statistics.mean(currents) if currents else None,
        rmse_a=math.sqrt(statistics.mean((x-target)**2 for x in currents)) if currents else None,
        max_abs_id_a=max(map(abs,ids)) if ids else None,
        max_abs_vd_v=max(map(abs,vd)) if vd else None,
        max_abs_vq_v=max(map(abs,vq)) if vq else None,
        angle_difference_peak_velocity_rad_s=max(map(abs,speeds)) if speeds else None,
        command_interval_ms=summary([(b['t']-a['t'])*1000 for a,b in zip(tx,tx[1:])]))


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--node',type=int,required=True,choices=range(1,64))
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--legacy-clock',action='store_true',help='Baseline using coarse Windows GetTickCount64 clock')
    args=ap.parse_args()
    result=dict(checked_at=datetime.now().astimezone().isoformat(),node=args.node,
        high_resolution_clock=not args.legacy_clock, firmware_or_parameter_writes=False,runs=[])
    bus=can.Bus(interface='pcan',channel='PCAN_USBBUS1',bitrate=1000000,ignore_config=True)
    rec=Recorder(bus)
    duration=.22
    try:
        for repeat in range(3):
            for target in (.05,-.05,.1,-.1):
                begin=len(rec.events)
                runner=CurrentRunner(rec,Settings(direction=1,iq_a=target,duration_s=duration,
                    node=args.node,reuse_alignment=True),message_factory=can.Message,
                    clock=time.monotonic if args.legacy_clock else time.perf_counter)
                run=dict(repeat=repeat,target_a=target,duration_s=duration)
                try:
                    run.update(runner.run())
                    run['summary']=summarize(rec.events[begin:],args.node,target,duration)
                    run['passed']=True
                except Exception as error:
                    run.update(passed=False,error=repr(error),stop_acknowledged=runner.stop_acknowledged)
                run['events']=rec.events[begin:]
                result['runs'].append(run)
                if not run['passed']:
                    raise RuntimeError(run['error'])
                # Wait for actual stopped low-speed feedback before another pulse.
                quiet_since=None
                deadline=time.perf_counter()+5
                while time.perf_counter()<deadline:
                    message=rec.recv(timeout=.01)
                    if message is None or message.arbitration_id!=p.FAST_BASE+args.node:
                        continue
                    fast=p.decode_fast(message.arbitration_id,message.data,args.node)
                    quiet=fast['sensor_valid'] and not fast['driver_enabled'] and abs(fast['velocity_rad_s'])<1
                    quiet_since=(quiet_since or time.perf_counter()) if quiet else None
                    if quiet_since and time.perf_counter()-quiet_since>.3:
                        break
                else:
                    raise RuntimeError('Motor did not settle within 5 s; stopped before next pulse')
        result['passed']=True
    except Exception as error:
        result.update(passed=False,error=repr(error))
    finally:
        bus.shutdown()
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps({**result,'runs':[{k:v for k,v in r.items() if k!='events'} for r in result['runs']]},indent=2))
    return 0 if result['passed'] else 1


if __name__=='__main__':
    raise SystemExit(main())
