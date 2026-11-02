import json
from pathlib import Path
import statistics
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from examples import gimbal_can as p
from tools.benchmark_board import summary

root = Path(sys.argv[1])
d = json.loads((root/'current.json').read_text())
rows = []
for r in d['runs']:
    tx = [e for e in r['events'] if e['direction']=='TX' and bytes.fromhex(e['data'])[1]==1]
    start = tx[0]['t'] if tx else 0
    currents, speeds = [], []
    stop = None
    ack = None
    last_iq = tx[-1]['t'] if tx else None
    timeout = None
    for e in r['events']:
        b = bytes.fromhex(e['data'])
        if e['direction']=='TX' and b[1]==0:
            stop=e
        if e['direction']=='RX':
            f=p.decode_feedback(e['id'], b, d['node'])
            if f['type']=='fast' and f['driver_enabled'] and .25<e['t']-start<.75:
                currents.append(f['iq_a'])
                speeds.append(f['velocity_rad_s'])
            if f['type']=='ack' and f['opcode']==0 and stop:
                ack=(e['t']-stop['t'])*1000
            if f['type']=='status' and f['command_timed_out'] and not f['driver_enabled'] and timeout is None and last_iq:
                timeout=(e['t']-last_iq)*1000
    row={k:v for k,v in r.items() if k!='events'}
    row.update(samples=len(currents), mean_iq_a=statistics.mean(currents) if currents else None,
        rmse_a=statistics.mean([(x-r['target_a'])**2 for x in currents])**.5 if currents else None,
        peak_abs_velocity_rad_s=max(map(abs,speeds)) if speeds else None, stop_ack_ms=ack,
        timeout_from_last_tx_ms=timeout,
        command_interval_ms=summary([(b['t']-a['t'])*1000 for a,b in zip(tx,tx[1:])]))
    rows.append(row)
(root/'current-summary.json').write_text(json.dumps(rows,indent=2),encoding='utf-8')
print(json.dumps(rows,indent=2))
