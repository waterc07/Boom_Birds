#!/usr/bin/env python3
"""核对原始 VIO bag 是否覆盖动作前后；只读 SQLite，不启动 ROS 节点。"""
from __future__ import annotations

import argparse
import json
import math
import sqlite3
from pathlib import Path

RAW_TOPICS = ('/boom_birds/stereo/left_raw', '/boom_birds/stereo/right_raw', '/boom_birds/imu')
STATE_TOPICS = ('/ov_msckf/state_poseimu', '/ov_msckf/state_odomimu')
STATUS_TOPIC = '/ov_msckf/status'


def assessWindow(topicStamps, markerNs, diagnosticsSeen, preS=20.0, postS=10.0, maxGapS=0.25):
    if not all(math.isfinite(v) and v > 0 for v in (preS, postS, maxGapS)):
        raise ValueError('window limits must be finite and positive')
    reasons = []
    topics = {}
    if markerNs is None:
        reasons.append('operator_marker_missing')
    for name in RAW_TOPICS:
        stamps = topicStamps.get(name, [])
        row = {'messages': len(stamps), 'first_ns': min(stamps) if stamps else None,
               'last_ns': max(stamps) if stamps else None}
        if not stamps:
            reasons.append('raw_topic_empty:' + name)
        elif markerNs is not None:
            lo, hi = markerNs - int(preS * 1e9), markerNs + int(postS * 1e9)
            # Include the immediate sample on each side of the requested interval.
            before = [s for s in stamps if s <= lo]
            after = [s for s in stamps if s >= hi]
            if not before or not after:
                reasons.append('raw_window_not_covered:' + name)
            window = sorted(s for s in stamps if lo <= s <= hi)
            if before: window.insert(0, max(before))
            if after: window.append(min(after))
            gap = max((b-a for a,b in zip(window, window[1:])), default=0) / 1e9
            row['window_max_receive_gap_s'] = gap
            if gap > maxGapS:
                reasons.append('raw_window_gap:' + name)
        topics[name] = row
    for name in STATE_TOPICS:
        row = topicStamps.get(name, [])
        inside = [t for t in row if markerNs is not None and markerNs-int(preS*1e9) <= t <= markerNs+int(postS*1e9)]
        topics[name] = {'messages':len(row), 'window_messages':len(inside)}
        if not inside:
            reasons.append('state_topic_empty:' + name)
    if not diagnosticsSeen:
        reasons.append('camera_diagnostics_missing')
    return {'diagnostic_window_ready':not reasons, 'reasons':reasons,
            'marker_ns':markerNs, 'pre_s':preS, 'post_s':postS, 'max_receive_gap_s':maxGapS,
            'clock_basis':'bag receive time vs operator capture_received_ns; not exposure/action ground truth',
            'camera_diagnostics_seen':diagnosticsSeen, 'topics':topics,
            'navigation_verified':False}


def verifyBag(bagPath, eventsPath, preS=20.0, postS=10.0):
    from rclpy.serialization import deserialize_message
    from std_msgs.msg import String
    bagPath = Path(bagPath)
    if not (bagPath/'metadata.yaml').is_file():
        raise ValueError('bag has no finalized metadata.yaml')
    dbs = sorted(bagPath.glob('*.db3'))
    if len(dbs) != 1:
        raise ValueError('expected one finalized SQLite bag')
    events = json.loads(Path(eventsPath).read_text())
    markers = [r['capture_received_ns'] for r in events if r.get('phase') == 'STILL']
    marker = markers[0] if markers else None
    with sqlite3.connect(dbs[0].resolve().as_uri() + '?mode=ro', uri=True) as con:
        integrity = con.execute('pragma integrity_check').fetchone()[0]
        ids = {name: tid for tid,name in con.execute('select id,name from topics')}
        stamps = {name:[s for s, in con.execute('select timestamp from messages where topic_id=? order by timestamp',(tid,))]
                  for name,tid in ids.items()}
        diagnosticsSeen = 0
        malformedStatus = 0
        for timestamp,data in con.execute('select timestamp,data from messages where topic_id=? order by timestamp',
                                         (ids.get(STATUS_TOPIC,-1),)):
            if marker is not None and not marker-int(preS*1e9) <= timestamp <= marker+int(postS*1e9):
                continue
            try:
                status = json.loads(deserialize_message(data,String).data)
                frame = status.get('camera_diagnostics')
                if isinstance(frame,dict) and frame.get('camera_frames',0)>0:
                    diagnosticsSeen += 1
            except (ValueError,TypeError,RuntimeError):
                malformedStatus += 1
    result = assessWindow(stamps, marker, diagnosticsSeen, preS, postS)
    result.update({'sqlite_integrity':integrity,'malformed_status_messages':malformedStatus})
    if integrity != 'ok' or malformedStatus:
        result['diagnostic_window_ready'] = False
        result['reasons'].append('sqlite_or_status_invalid')
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag',type=Path)
    parser.add_argument('--events',required=True,type=Path)
    parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args()
    result=verifyBag(args.bag,args.events)
    args.output.write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
    return 0 if result['diagnostic_window_ready'] else 2


if __name__=='__main__':
    raise SystemExit(main())
