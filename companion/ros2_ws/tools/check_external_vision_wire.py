#!/usr/bin/env python3
"""TEST-ONLY：独占本机 UDP，核对 MAVROS ODOMETRY 转换；不连接飞控。"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import yaml
import rclpy
from nav_msgs.msg import Odometry
from pymavlink import mavutil
from boom_birds_control.external_vision import externalVisionOdometry


def freePort():
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sock:
        sock.bind(("127.0.0.1",0));return sock.getsockname()[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out",type=Path,required=True)
    args=parser.parse_args();args.out.mkdir(parents=True,exist_ok=False)
    receive=freePort();transmit=freePort()
    while transmit==receive:transmit=freePort()
    peer=mavutil.mavlink_connection("udpin:127.0.0.1:"+str(receive),source_system=1,source_component=1)
    config=args.out/"mavros.yaml"
    config.write_text(yaml.safe_dump({"/**":{"ros__parameters":{
        "plugin_allowlist":["sys_status","bb_odometry"],
        "fcu_url":f"udp://127.0.0.1:{transmit}@127.0.0.1:{receive}","gcs_url":""}}}))
    log=(args.out/"mavros.log").open("w")
    process=subprocess.Popen(["ros2","run","mavros","mavros_node","--ros-args","--params-file",str(config)],
        stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init();node=rclpy.create_node("external_vision_wire_probe")
    publisher=node.create_publisher(Odometry,"/mavros/odometry/out",10)
    report=dict(kind="TEST_ONLY_LOOPBACK_MAVROS_ODOMETRY",hardware_connected=False,result="FAIL")
    try:
        deadline=time.monotonic()+20.;sample=None;source=None
        while time.monotonic()<deadline and sample is None:
            rclpy.spin_once(node,timeout_sec=.03)
            message=Odometry();message.header.stamp=node.get_clock().now().to_msg()
            message.header.frame_id="global";message.child_frame_id="body"
            message.pose.pose.position.x=1.;message.pose.pose.position.y=2.;message.pose.pose.position.z=.8
            message.pose.pose.orientation.w=1.
            message.twist.twist.linear.x=.1;message.twist.twist.linear.y=.2;message.twist.twist.linear.z=.3
            if publisher.get_subscription_count():
                source=externalVisionOdometry(message,node.get_clock().now().nanoseconds*1e-9)
                publisher.publish(source)
            while True:
                packet=peer.recv_match(blocking=False)
                if packet is None:break
                if packet.get_type()=="HEARTBEAT":
                    peer.mav.heartbeat_send(2,12,0,0,3)
                if packet.get_type()=="COMMAND_LONG" and packet.command==400:
                    raise RuntimeError("unexpected_arm_command")
                if packet.get_type()=="ODOMETRY":sample=packet;break
        if sample is None:raise RuntimeError("no_wire_odometry: inspect mavros.log and static TF")
        assert sample.frame_id==20 and sample.child_frame_id==12 and sample.estimator_type==2
        assert max(abs(a-b) for a,b in zip((sample.x,sample.y,sample.z),(2.,1.,-.8)))<1e-5
        assert max(abs(a-b) for a,b in zip((sample.vx,sample.vy,sample.vz),(.1,-.2,-.3)))<1e-5
        assert abs(abs(sample.q[0])-math.sqrt(.5))<1e-5 and abs(abs(sample.q[3])-math.sqrt(.5))<1e-5
        assert abs(sample.pose_covariance[0]-.04)<1e-5 and abs(sample.velocity_covariance[0]-.04)<1e-5
        # 回读必须保留采样时间；允许前一条已发布样本在 socket 中排队。
        now=node.get_clock().now().nanoseconds*1e-9
        assert 0<=now-sample.time_usec*1e-6<.5
        report.update(result="PASS",wire=sample.to_dict(),sample_age_s=now-sample.time_usec*1e-6,
                      scope="MAVROS conversion only; no PX4 EKF fusion")
    except Exception as exc:
        report["error"]=str(exc);raise
    finally:
        node.destroy_node();rclpy.shutdown();peer.close();os.killpg(process.pid,signal.SIGINT)
        try:process.wait(timeout=10.)
        except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5.)
        log.close();(args.out/"report.json").write_text(json.dumps(report,indent=2)+"\n")
        print(json.dumps(report,indent=2))


if __name__=="__main__":main()
