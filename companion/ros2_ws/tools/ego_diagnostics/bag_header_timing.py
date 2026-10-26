#!/usr/bin/env python3
"""只读 ROS 2 SQLite 原始话题的 CDR Header；并列报告接收与消息时钟。"""
from __future__ import annotations
import argparse
import json
import sqlite3
import struct
from pathlib import Path

RAW_TYPES = {'/boom_birds/stereo/left_raw': 'sensor_msgs/msg/Image',
             '/boom_birds/stereo/right_raw': 'sensor_msgs/msg/Image',
             '/boom_birds/imu': 'sensor_msgs/msg/Imu'}


def headerNs(prefix):
    if len(prefix) != 12 or prefix[:2] not in (b'\x00\x00', b'\x00\x01'):
        raise ValueError('expected CDR1 encapsulation and Header stamp')
    sec, nano = struct.unpack(('>' if prefix[1] == 0 else '<') + 'iI', prefix[4:12])
    if sec < 0 or nano >= 1_000_000_000 or sec == nano == 0:
        raise ValueError('invalid Header stamp')
    return sec * 1_000_000_000 + nano


def timingStats(pairs):
    receive = [r for r, h in pairs]
    header = [h for r, h in pairs]
    def series(values):
        gaps = [b-a for a, b in zip(values, values[1:])]
        return {'first_ns': values[0] if values else None,
                'last_ns': values[-1] if values else None,
                'max_forward_gap_s': max([0] + gaps) / 1e9,
                'backward_steps': sum(g < 0 for g in gaps),
                'duplicate_steps': sum(g == 0 for g in gaps)}
    offsets = [r-h for r, h in pairs]
    largest = sorted(zip(pairs, pairs[1:]), key=lambda p: p[1][0]-p[0][0], reverse=True)[:5]
    return {'messages': len(pairs), 'receive': series(receive), 'header': series(header),
            'receive_minus_header_range_s': [min(offsets)/1e9, max(offsets)/1e9] if offsets else None,
            'largest_header_gaps': [{'header_before_ns': a[1], 'header_after_ns': b[1],
                                    'header_gap_s': (b[1]-a[1])/1e9,
                                    'receive_gap_s': (b[0]-a[0])/1e9} for a,b in sorted(zip(pairs,pairs[1:]),key=lambda p:p[1][1]-p[0][1],reverse=True)[:5]],
            'largest_receive_gaps': [{'receive_before_ns': a[0], 'receive_after_ns': b[0],
                                     'receive_gap_s': (b[0]-a[0])/1e9,
                                     'header_gap_s': (b[1]-a[1])/1e9} for a,b in largest]}


def analyze(dbPath, eventsPath):
    events = json.loads(Path(eventsPath).read_text())
    markers = [r['capture_received_ns'] for r in events if r.get('phase') == 'STILL']
    if not markers:
        raise ValueError('STILL marker missing')
    marker = markers[0]
    lo, hi = marker-20_000_000_000, marker+10_000_000_000
    result = {'marker_ns': marker, 'window_ns': [lo,hi], 'topics': {},
              'changes_original_window_verdict': False, 'navigation_verified': False,
              'limits': 'Header is publisher time, not independently measured exposure. Receive minus Header includes clock offset; it is not transport latency.'}
    with sqlite3.connect(Path(dbPath).resolve().as_uri()+'?mode=ro', uri=True) as con:
        con.execute('pragma query_only=on')
        for name, kind in RAW_TYPES.items():
            topic = con.execute('select id,type,serialization_format from topics where name=?',(name,)).fetchone()
            if topic is None or topic[1:] != (kind,'cdr'):
                raise ValueError('missing topic or unsupported type/serialization: '+name)
            pairs, errors = [], 0
            for received, prefix in con.execute('select timestamp,substr(data,1,12) from messages where topic_id=? order by id',(topic[0],)):
                try:
                    pairs.append((received,headerNs(prefix)))
                except ValueError:
                    errors += 1
            before = [p for p in pairs if p[0] < lo]
            after = [p for p in pairs if p[0] > hi]
            window = ([before[-1]] if before else []) + [p for p in pairs if lo <= p[0] <= hi] + ([after[0]] if after else [])
            result['topics'][name] = {'invalid_headers': errors, 'full_bag': timingStats(pairs),
                                     'receive_window_with_bracketing_samples': timingStats(window),
                                     'receive_window_covered': bool(before and after)}
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('db', type=Path)
    parser.add_argument('--events', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    result = analyze(args.db,args.events)
    args.output.write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
    return 2 if any(r['invalid_headers'] or not r['receive_window_covered'] or r['full_bag']['header']['backward_steps'] for r in result['topics'].values()) else 0


if __name__ == '__main__':
    raise SystemExit(main())
