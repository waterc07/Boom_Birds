#!/usr/bin/env python3
"""只订阅接入状态；结果不授权解锁，不代替标定、脱桨和系留验收。"""
import argparse
import json
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Image
from std_msgs.msg import String
from boom_birds_control.runtime_config import DEFAULTS

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--seconds',type=float,default=15.)
p.add_argument('--out',required=True)
a=p.parse_args()
if not 1<=a.seconds<=60: p.error('seconds 必须在 1–60 内')
out=Path(a.out)
if out.exists(): p.error('输出已存在，拒绝覆盖')
rclpy.init();node=Node('attitude_hardware_observer')
counts={};ages={};last={};status={};rows=[];status_at=-float("inf")
def sample(topic,msg):
    now=node.get_clock().now().nanoseconds*1e-9
    stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
    counts[topic]=counts.get(topic,0)+1
    ages.setdefault(topic,[]).append(now-stamp)
    last[topic]=time.monotonic()
def control(msg):
    global status, status_at
    status_at=time.monotonic()
    status=json.loads(msg.data)
    rows.append({'at':time.time(),'status':status})
topics={DEFAULTS.odom_topic:Odometry,DEFAULTS.imu_topic:Imu,DEFAULTS.depth_topic:Image}
def sample_callback(topic):
    def received(msg):
        sample(topic,msg)
    return received
for topic,t in topics.items():
    node.create_subscription(t,topic,sample_callback(topic),qos_profile_sensor_data)
node.create_subscription(String,DEFAULTS.control_status_topic,control,10)
start=time.monotonic()
try:
    while time.monotonic()-start<a.seconds: rclpy.spin_once(node,timeout_sec=.05)
    now=time.monotonic();attitude=status.get('attitude_controller') or {}
    backend=status.get('backend',{})
    checks={'fcu_connected':backend.get('connected') is True,
        'disarmed':backend.get('acknowledged_armed') is False,
        'status_fresh':now-status_at<=DEFAULTS.pose_timeout_s,
        'real_sample_clock':backend.get('sample_time_source')=='PX4_boot_timesync',
        'thrust_scaling':backend.get('attitude_scaling_ready') is True,
        'no_position_output':node.count_publishers('/mavros/setpoint_raw/local')==0,
        'timesync':backend.get('timesync_reason')=='ready' and backend.get('timesync_count',0)>=5,
        'single_attitude_output':node.count_publishers('/mavros/setpoint_raw/attitude')==1,
        'controller_mode':status.get('control_mode')=='companion_attitude',
        'reference_ready':attitude.get('ready') is True and not attitude.get('latched') and attitude.get('feedback_error')=='',
        'endpoint_verified':status.get('backend',{}).get('endpoint_verified') is True}
    limits={DEFAULTS.odom_topic:DEFAULTS.pose_timeout_s,DEFAULTS.imu_topic:DEFAULTS.imu_timeout_s,
            DEFAULTS.depth_topic:DEFAULTS.camera_timeout_s}
    for t in topics:
        checks[t]=(counts.get(t,0)>1 and now-last.get(t,-1e20)<=limits[t]
                   and ages[t][-1]>=0 and ages[t][-1]<=limits[t] and node.count_publishers(t)==1)
    report={'result':'READY_FOR_BENCH_REVIEW' if all(checks.values()) else 'NOT_READY',
        'checks':checks,'hardware_profile_verified':attitude.get('hardware_profile_verified',False),
        'topics':{t:{'samples':counts.get(t,0),'hz':counts.get(t,0)/a.seconds,
            'max_age_s':max(ages.get(t,[float('inf')])),'publishers':node.count_publishers(t)} for t in topics},
        'last_status':status,'samples':rows}
    out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(report,ensure_ascii=False,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k not in ('last_status','samples')},ensure_ascii=False,indent=2))
finally:
    node.destroy_node();rclpy.shutdown()

raise SystemExit(0 if all(checks.values()) else 2)
