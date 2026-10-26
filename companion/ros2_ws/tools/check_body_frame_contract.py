#!/usr/bin/env python3
"""合成输入核验发布 IMU、机体位姿与姿态控制轴向；不连接设备。"""
import argparse
import json
import math
from pathlib import Path
import struct
import time
from types import SimpleNamespace
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from mavros_msgs.msg import Mavlink, State, TimesyncStatus
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from boom_birds_sensing.mavros_imu_node import MavrosImuNode
from boom_birds_sensing.pose_adapter import PoseAdapter
from boom_birds_control.frames import quatToRot, rotToQuat
from boom_birds_control.attitude_control import AttitudeController, px4AttitudeEnuFlu


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--extrinsics-file', required=True)
    parser.add_argument('--calibration-file', required=True)
    parser.add_argument('--out', required=True)
    args = parser.parse_args()
    rclpy.init(args=['--ros-args', '-p', 'extrinsics_file:='+args.extrinsics_file,
                    '-p', 'calibration_file:='+args.calibration_file, '-p', 'use_depth_gate:=false'])
    collector = Node('body_contract_collector')
    imu = MavrosImuNode()
    adapter = PoseAdapter()
    executor = SingleThreadedExecutor()
    for node in (collector, imu, adapter): executor.add_node(node)
    samples, bodies = [], []
    collector.create_subscription(Imu, '/boom_birds/imu', samples.append, qos_profile_sensor_data)
    collector.create_subscription(Odometry, '/boom_birds/vio/odom_body', bodies.append, 10)
    report = dict(status='FAIL', scope='Synthetic axis contract, no VIO estimation or device access',
                  extrinsics_file=args.extrinsics_file, physical_hardware_started=False, flight_ready=False)
    try:
        deadline = time.monotonic()+5.
        while not (imu.pub.get_subscription_count() and adapter.pub_body.get_subscription_count()):
            if time.monotonic()>deadline: raise AssertionError('DDS discovery timeout')
            executor.spin_once(timeout_sec=.02)
        now = imu.get_clock().now()
        state = State(); state.header.stamp=now.to_msg(); state.connected=True
        imu._state(state)
        bootUs = 2000000
        sync = TimesyncStatus(); sync.header.stamp=now.to_msg()
        sync.remote_timestamp_ns=bootUs*1000
        sync.estimated_offset_ns=now.nanoseconds-bootUs*1000
        sync.observed_offset_ns=sync.estimated_offset_ns
        sync.round_trip_time_ms=.1
        for _ in range(5): imu._sync(sync)
        payload=struct.pack('<Q13fH',bootUs,1.,2.,-9.81,.1,.2,.3,0.,0.,0.,0.,0.,0.,20.,63)
        wire=Mavlink();wire.framing_status=1;wire.msgid=105;wire.sysid=1;wire.compid=1;wire.len=len(payload)
        wire.payload64=list(struct.unpack('<8Q',payload.ljust(64,b'\0')))
        imu._rawImu(wire)
        deadline=time.monotonic()+2.
        while not samples:
            if time.monotonic()>deadline: raise AssertionError('No IMU output')
            executor.spin_once(timeout_sec=.01)
        sample=samples[-1]
        acceleration=[sample.linear_acceleration.x,sample.linear_acceleration.y,sample.linear_acceleration.z]
        angular=[sample.angular_velocity.x,sample.angular_velocity.y,sample.angular_velocity.z]
        assert np.allclose(acceleration,[1.,2.,-9.81],atol=1e-5),acceleration
        assert np.allclose(angular,[.1,.2,.3],atol=1e-6),angular
        # 水平机体的 FRD 姿态在 z 向上世界中为绕 x 轴 180 度；body 输出须为 FLU。
        msg=Odometry();msg.header.stamp=adapter.get_clock().now().to_msg()
        msg.pose.pose.position.z=1.
        q=rotToQuat(np.diag([1.,-1.,-1.]))
        msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w=map(float,q)
        adapter.onOdom(msg);adapter.tick()
        deadline=time.monotonic()+2.
        while not bodies:
            if time.monotonic()>deadline: raise AssertionError('No body odometry output')
            executor.spin_once(timeout_sec=.01)
        body=bodies[-1];q=body.pose.pose.orientation;qBody=(q.x,q.y,q.z,q.w)
        bodyRotation=quatToRot(qBody)
        controller=AttitudeController()
        alignmentError=None
        try:
            for i in range(controller.config.alignment_samples):
                controller.observe((0.,0.,1.),(0.,0.,0.),qBody,px4AttitudeEnuFlu(0.,0.,0.),1.+i*.02,False)
        except ValueError as exc: alignmentError=str(exc)
        report.update(published_imu_axes='FRD',acceleration=acceleration,angular_velocity=angular,
                      body_rotation_world=bodyRotation.tolist(),body_up_z=float(bodyRotation[2,2]),
                      alignment_error=alignmentError,controller_ready=controller.ready)
        if alignmentError: raise AssertionError(alignmentError)
        desired=SimpleNamespace(position_m=(0.,0.,1.),velocity_m_s=(0.,0.,0.),acceleration_m_s2=(0.,0.,0.),yaw_rad=0.)
        command=controller.calculate(desired,(0.,0.,1.),(0.,0.,0.),qBody)
        assert np.allclose(bodyRotation,np.eye(3),atol=1e-9)
        assert math.isclose(command.thrust,.5,abs_tol=1e-9)
        report.update(status='PASS',synthetic_hover_thrust=command.thrust)
    except Exception as exc:
        report['error']=str(exc)
    finally:
        for node in (adapter,imu,collector):
            executor.remove_node(node);node.destroy_node()
        executor.shutdown();rclpy.shutdown()
        out=Path(args.out);out.parent.mkdir(parents=True,exist_ok=True)
        out.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(report,ensure_ascii=False))
    return 0 if report['status']=='PASS' else 1


if __name__=='__main__':raise SystemExit(main())
