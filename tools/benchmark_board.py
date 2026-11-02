"""Stopped-only PCAN benchmark; raw frames and host RTT are retained as evidence."""
import argparse
from collections import Counter, defaultdict
from datetime import datetime
import json
from pathlib import Path
import statistics
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from examples import gimbal_can as p
import can


def summary(values):
    values = sorted(values)
    if not values:
        return None
    return dict(n=len(values), min=min(values), median=statistics.median(values),
                p95=values[min(len(values)-1, int(.95*len(values)))], max=max(values))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--node', type=int, required=True, choices=range(1,64))
    ap.add_argument('--seconds', type=float, default=60)
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    bus = can.Bus(interface='pcan', channel='PCAN_USBBUS1', bitrate=p.BITRATE)
    events, replies, pending, errors = [], [], {}, []
    counts = Counter()
    stamps = defaultdict(list)
    latest = {}
    start = time.perf_counter()
    sequence = 0

    def send(frame):
        nonlocal sequence
        identifier, data = frame
        assert identifier == p.COMMAND_BASE + args.node
        assert data[1] in (0, 0x10, 0x12, 0x14, 0x15)
        now = time.perf_counter()
        bus.send(can.Message(arbitration_id=identifier, data=data, is_extended_id=False), timeout=.05)
        pending[(data[1], data[2])] = now
        events.append(dict(t=now-start, direction='TX', id=identifier, data=data.hex()))

    def receive(timeout):
        message = bus.recv(timeout)
        if message is None:
            return
        now = time.perf_counter()
        events.append(dict(t=now-start, direction='RX', id=message.arbitration_id,
                           data=message.data.hex(), adapter_timestamp=message.timestamp,
                           error=message.is_error_frame))
        if message.is_error_frame:
            errors.append('CAN error frame')
            return
        try:
            decoded = p.decode_feedback(message.arbitration_id, message.data, args.node)
        except ValueError:
            return
        kind = decoded['type']
        latest[kind] = decoded
        counts[kind] += 1
        stamps[kind].append(message.timestamp)
        if kind == 'ack':
            key = (decoded['opcode'], decoded['sequence'])
            sent = pending.pop(key, None)
            if sent is not None:
                replies.append(dict(opcode=key[0], sequence=key[1], rtt_ms=(now-sent)*1000,
                                    result=decoded['result']))
            if decoded['result'] or decoded['faults']:
                errors.append(decoded)
        if kind == 'status' and (decoded['mode'] != 0 or decoded['driver_enabled']):
            raise RuntimeError('Board did not remain stopped')
        if kind == 'diagnostic' and any(decoded[k] for k in
                ('faults','board_faults_low8','bus_off','error_passive','rx_loss','tx_loss','not_initialized')):
            errors.append(decoded)

    failure = None
    try:
        send(p.pack_stop(args.node, sequence))
        deadline = time.perf_counter()+2
        while pending and time.perf_counter()<deadline:
            receive(.01)
        if pending or not replies or replies[-1]['result']:
            raise RuntimeError('STOP handshake failed')
        next_query = time.perf_counter()
        index = 0
        while time.perf_counter()-start < args.seconds:
            now = time.perf_counter()
            if now >= next_query:
                # One outstanding request prevents ambiguous sequence-wrap pairing.
                if pending:
                    raise RuntimeError('Query ACK exceeded 100 ms')
                sequence = (sequence+1) % 256
                cycle = index % 12
                frame = (p.pack_param_read(args.node, sequence, cycle+1) if cycle<9 else
                         (p.pack_node_read, p.pack_limits_read, p.pack_calibration_read)[cycle-9](args.node, sequence))
                send(frame)
                index += 1
                next_query = now+.1
            receive(.005)
        deadline = time.perf_counter()+.2
        while pending and time.perf_counter()<deadline:
            receive(.005)
    except Exception as exc:
        failure = repr(exc)
    finally:
        try:
            sequence = (sequence+1)%256
            send(p.pack_stop(args.node, sequence))
            deadline = time.perf_counter()+1
            while pending and time.perf_counter()<deadline:
                receive(.01)
        finally:
            bus.shutdown()
    duration = time.perf_counter()-start
    timing = {}
    for kind, ts in stamps.items():
        timing[kind] = dict(frames=len(ts), observed_hz=(len(ts)-1)/(ts[-1]-ts[0]) if len(ts)>1 and ts[-1]>ts[0] else None,
            interval_ms=summary([(b-a)*1000 for a,b in zip(ts,ts[1:])]))
    passed = (not failure and not errors and not pending and
              all(counts[k] for k in ('status','diagnostic','fast','parameter','node_info','limits_info','calibration_info')))
    result = dict(passed=passed, checked_at=datetime.now().astimezone().isoformat(), node=args.node,
        duration_s=duration, scope='STOP and read-only queries; no motion, reset or flash writes',
        timing_basis='RTT: host perf_counter send-to-ACK including OS/USB; intervals: PCAN adapter timestamps',
        ack_rtt_ms=summary([r['rtt_ms'] for r in replies]), requests=len(replies)+len(pending),
        pending=list(pending), errors=errors, failure=failure, timing=timing, latest=latest,
        replies=replies, events=events)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding='utf-8')
    printable = {k:v for k,v in result.items() if k not in ('events','replies','errors')}
    printable['error_count'] = len(errors)
    printable['first_error'] = errors[0] if errors else None
    print(json.dumps(printable, indent=2, ensure_ascii=False))
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
