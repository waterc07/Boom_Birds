"""真实 MAVROS + UDP 假 PX4 验证。pymavlink 仅用于测试对端。"""
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from pymavlink.dialects.v20 import common as mav
from boom_birds_control.mavros_backend import MavrosPx4Backend
from boom_birds_control.px4_frames import Px4LocalSetpoint, TypeMask
from boom_birds_sensing.mavros_imu_node import MavrosImuNode
from boom_birds_sensing.mavros_config_node import MavrosConfigNode
from sensor_msgs.msg import Imu
from rclpy.qos import qos_profile_sensor_data


def main():
    project = Path(__file__).resolve().parents[3]
    evidence = Path(os.environ["BB_MAVROS_EVID"])
    evidence.mkdir(parents=True, exist_ok=False)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 14580))
    sock.setblocking(False)
    out = evidence.joinpath("mavros.log").open("w")
    proc = subprocess.Popen(["/opt/ros/jazzy/lib/mavros/mavros_node", "--ros-args", "--params-file",
        str(project/"companion/ros2_ws/src/boom_birds_bringup/config/mavros.yaml"),
        "-p", "fcu_url:=udp://127.0.0.1:14540@127.0.0.1:14580", "-p", "tgt_system:=1", "-p", "tgt_component:=1"],
        stdout=out, stderr=out, start_new_session=True)
    rclpy.init()
    node = Node("mavros_loopback_check")
    backend = MavrosPx4Backend(node, dry_run=False, allow_arming=True, heartbeat_timeout_s=.6)
    backend.connect()
    imu_node = MavrosImuNode()
    config_node = MavrosConfigNode()
    received = []
    node.create_subscription(Imu, "/boom_birds/imu", received.append, qos_profile_sensor_data)
    executor = SingleThreadedExecutor()
    executor.add_node(node);executor.add_node(imu_node);executor.add_node(config_node)
    encoder = mav.MAVLink(None, srcSystem=1, srcComponent=1)
    parser = mav.MAVLink(None);parser.robust_parsing=True
    start = time.monotonic()
    heartbeat = True
    boot_offset = 60.
    armed = False
    imu_fields = 63
    packets=[]
    attitude_packets=[]
    def send(msg):
        sock.sendto(msg.pack(encoder), ("127.0.0.1",14540));encoder.seq=(encoder.seq+1)&255
    last_hb=last_sample=-100.
    def tick():
        nonlocal last_hb,last_sample,armed
        now=time.monotonic();boot=now-start+boot_offset
        if heartbeat and now-last_hb>.2:
            send(encoder.heartbeat_encode(2,12,128 if armed else 0,6<<16,4));last_hb=now
        if now-last_sample>.02:
            last_sample=now
            send(encoder.attitude_encode(round(boot*1000),.1,.2,.3,0.,0.,.4))
            send(encoder.local_position_ned_encode(round(boot*1000),1.,2.,-3.,.1,.2,-.3))
            send(encoder.highres_imu_encode(round(boot*1e6),1.,2.,-9.81,.1,.2,.3,0.,0.,0.,0.,0.,0.,20.,imu_fields))
            send(encoder.extended_sys_state_encode(0,2))
        while True:
            try:data,_=sock.recvfrom(8192)
            except BlockingIOError:break
            for msg in parser.parse_buffer(data) or []:
                if msg.get_type()=="TIMESYNC" and msg.tc1==0:
                    send(encoder.timesync_encode(round(boot*1e9),msg.ts1))
                elif msg.get_type()=="COMMAND_LONG":
                    if msg.command==400:armed=msg.param1==1
                    send(encoder.command_ack_encode(msg.command,0))
                elif msg.get_type()=="SET_POSITION_TARGET_LOCAL_NED":packets.append(msg)
                elif msg.get_type()=="SET_ATTITUDE_TARGET":attitude_packets.append(msg)
        executor.spin_once(timeout_sec=.002)
    def wait(predicate, seconds=10.):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            tick()
            if predicate():return
        raise AssertionError(str(imu_node.gate.__dict__) + str(backend.stream_diagnostics()))
    try:
        wait(lambda:backend.is_connected() and backend.read_vehicle_state().position_ned_m is not None and len(received)>60 and config_node.ready)
        assert backend.read_vehicle_state().position_ned_m==(1.,2.,-3.)
        sample=received[-1]
        assert abs(sample.linear_acceleration.z+9.81)<1e-4
        assert abs(sample.angular_velocity.y-.2)<1e-4
        assert sample.orientation_covariance[0]==-1.
        assert abs(time.time()-sample.header.stamp.sec-sample.header.stamp.nanosec*1e-9)<.1
        # 字段部分更新不能伪装成完整 IMU；MAVROS 原生 IMU 插件只检查任意更新位。
        imu_fields=1
        deadline=time.monotonic()+.15
        while time.monotonic()<deadline: tick()
        before_count=len(received)
        deadline=time.monotonic()+.2
        while time.monotonic()<deadline: tick()
        assert len(received)==before_count and imu_node.gate.reason=="imu_fields_unavailable"
        imu_fields=63
        wait(lambda:len(received)>before_count)
        point=Px4LocalSetpoint((1.2,-2.3,-3.4),(.2,-.3,-.4),(.5,-.6,-.7),.8,.9)
        mask=int(TypeMask.for_mode("position_velocity"))
        assert backend.send_setpoint(point,mask)
        wait(lambda:len(packets)>0)
        wire=packets[-1]
        for actual,expected in [(wire.x,1.2),(wire.y,-2.3),(wire.z,-3.4),(wire.vx,.2),(wire.vy,-.3),(wire.vz,-.4),(wire.yaw,.8)]:
            assert math.isclose(actual,expected,abs_tol=1e-5),(actual,expected)
        assert wire.type_mask==mask
        partial=1|16|64|1024  # 分轴 mask + yaw_rate，另外一组使用加速度前馈。
        assert backend.send_setpoint(point,partial)
        wait(lambda:len(packets)>1)
        wire=packets[-1]
        assert wire.type_mask==partial  # mask 由 MAVROS 原样转发为 NED 轴。
        assert math.isclose(wire.yaw_rate,.9,abs_tol=1e-5)
        assert math.isclose(wire.afy,-.6,abs_tol=1e-5)
        assert backend.arm(True)
        wait(lambda:backend.read_vehicle_state().armed)
        wait(lambda:backend.stream_diagnostics()["commands"]["400"].get("ack_result")==0)
        assert backend.set_mode("offboard")
        wait(lambda:backend.stream_diagnostics()["commands"]["176"].get("ack_result")==0)
        backend.dry_run=True;n=len(packets)
        assert backend.send_setpoint(point,mask)
        deadline=time.monotonic()+.3
        while time.monotonic()<deadline:tick()
        assert len(packets)==n and not backend.arm(True)
        backend.dry_run=False
        # 实际 MAVROS 插件须完成推力缩放回读和 ENU/FLU→NED/FRD 转换。
        import numpy as np
        from boom_birds_control.attitude_control import AttitudeSetpoint, rotation, yaw_rotation, NED_TO_ENU, FRD_TO_FLU
        from boom_birds_control.frames import rot_to_quat
        backend.close()
        backend=MavrosPx4Backend(node,dry_run=False,allow_arming=True,heartbeat_timeout_s=.6,control_mode='companion_attitude')
        backend.connect()
        wait(lambda:backend._attitude_scaling_ready and backend.is_connected())
        wait(lambda:bool(backend._attitude_history) and backend.attitude_at(backend._attitude_history[-1][0],.03) is not None)
        assert backend.stream_diagnostics()['sample_time_source']=='PX4_boot_timesync'
        assert np.allclose(backend.attitude_at(backend._attitude_history[-1][0],.03),(.1,.2,.3),atol=1e-5)
        requested=yaw_rotation(.8) @ rotation((math.sin(.1),0,0,math.cos(.1)))
        attitude=AttitudeSetpoint(tuple(rot_to_quat(requested)),.42,(0,0,0))
        assert not backend.send_setpoint(point,mask)
        assert backend.send_attitude_setpoint(attitude)
        wait(lambda:bool(attitude_packets))
        wire=attitude_packets[-1]
        q=wire.q
        assert np.allclose(rotation((q[1],q[2],q[3],q[0])),NED_TO_ENU @ requested @ FRD_TO_FLU,atol=1e-5)
        assert wire.type_mask==7 and math.isclose(wire.thrust,.42,abs_tol=1e-5)
        backend.dry_run=True; n=len(attitude_packets)
        assert backend.send_attitude_setpoint(attitude)
        deadline=time.monotonic()+.3
        while time.monotonic()<deadline:tick()
        assert len(attitude_packets)==n
        backend.dry_run=False
        heartbeat=False
        wait(lambda:not backend.is_connected(),2.)
        assert not backend.send_attitude_setpoint(attitude)
        heartbeat=True
        wait(lambda:backend.is_connected())
        before=backend.read_vehicle_state().restart_epoch
        boot_offset=.1
        wait(lambda:backend.read_vehicle_state().restart_epoch>before)
        result=dict(result="PASS", wire_setpoint_count=len(packets), attitude_packet_count=len(attitude_packets), imu_count=len(received),
            checked=["PX4 boot-to-ROS attitude timestamp pairing", "attitude quaternion ENU/FLU to NED/FRD", "attitude thrust scaling readback", "mutually exclusive output", "attitude dry-run and stale-heartbeat suppression", "NED/ENU vectors", "yaw and yaw_rate", "partial axis masks", "service ACK", "observed arm", "dry_run", "heartbeat timeout", "PX4 restart", "FRD IMU and synchronized timestamp", "partial IMU fields rejected", "sys_time parameter readback"])
        evidence.joinpath("report.json").write_text(json.dumps(result,indent=2))
        print(json.dumps(result))
    finally:
        backend.close();executor.remove_node(imu_node);executor.remove_node(node)
        config_node.destroy_node();imu_node.destroy_node();node.destroy_node();executor.shutdown();rclpy.shutdown()
        os.killpg(proc.pid,signal.SIGTERM)
        try:proc.wait(timeout=5.)
        except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        sock.close();out.close()


if __name__=="__main__":main()
