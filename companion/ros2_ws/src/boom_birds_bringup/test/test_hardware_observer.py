"""只读接入报告的 ROS 回调回归；全部输入为测试替身，不连接飞控。"""
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Image
from std_msgs.msg import String
from mavros_msgs.msg import AttitudeTarget
from boom_birds_control.runtime_config import DEFAULTS


@pytest.mark.parametrize('fresh_status',[True,False])
def test_observer_counts_ros_samples_and_rejects_stale_status(tmp_path,fresh_status):
    tool=Path(__file__).resolve().parents[3]/'tools/monitor_attitude_hardware.py'
    out=tmp_path/'report.json'
    context=Context();rclpy.init(context=context)
    node=Node('hardware_observer_test',context=context)
    executor=SingleThreadedExecutor(context=context);executor.add_node(node)
    pubs={topic:node.create_publisher(kind,topic,qos_profile_sensor_data) for topic,kind in
        ((DEFAULTS.odom_topic,Odometry),(DEFAULTS.imu_topic,Imu),(DEFAULTS.depth_topic,Image))}
    node.create_publisher(AttitudeTarget,'/mavros/setpoint_raw/attitude',qos_profile_sensor_data)
    control=node.create_publisher(String,DEFAULTS.control_status_topic,10)
    status={'control_mode':'companion_attitude',
        'attitude_controller':{'ready':True,'latched':'','feedback_error':''},
        'backend':{'connected':True,'acknowledged_armed':False,'sample_time_source':'PX4_boot_timesync',
            'attitude_scaling_ready':True,'timesync_reason':'ready','timesync_count':10,'endpoint_verified':True}}
    proc=subprocess.Popen([sys.executable,str(tool),'--seconds','15','--out',str(out)],
        stdout=subprocess.PIPE,stderr=subprocess.STDOUT,env=os.environ.copy())
    start=time.monotonic();first_control=None
    try:
        while proc.poll() is None and time.monotonic()-start<20:
            for topic,pub in pubs.items():
                kind={DEFAULTS.odom_topic:Odometry,DEFAULTS.imu_topic:Imu,DEFAULTS.depth_topic:Image}[topic]
                msg=kind();msg.header.stamp=node.get_clock().now().to_msg();pub.publish(msg)
            # DDS 发现包含在观察窗口内；先等所有订阅匹配，再计状态停发时间。
            if (control.get_subscription_count()>0
                    and all(pub.get_subscription_count()>0 for pub in pubs.values())
                    and first_control is None):
                first_control=time.monotonic()
            if fresh_status or first_control is None or time.monotonic()-first_control<.4:
                control.publish(String(data=json.dumps(status)))
            executor.spin_once(timeout_sec=.02)
        stdout,_=proc.communicate(timeout=3)
        assert first_control is not None,stdout.decode()
        assert out.is_file(),stdout.decode()
        report=json.loads(out.read_text())
        assert all(item['samples']>1 for item in report['topics'].values())
        assert all(item['publishers']==1 for item in report['topics'].values())
        assert report['last_status']['control_mode']=='companion_attitude'
        assert report['checks']['status_fresh'] is fresh_status
        assert report['result']==('READY_FOR_BENCH_REVIEW' if fresh_status else 'NOT_READY')
        assert proc.returncode==(0 if fresh_status else 2)
    finally:
        if proc.poll() is None:
            proc.terminate();proc.wait(timeout=3)
        executor.shutdown();node.destroy_node();context.shutdown()
