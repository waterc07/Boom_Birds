"""Native ingress keeps reset/ACK packets intact and excludes unrelated telemetry."""
from pathlib import Path
import subprocess
import time
import uuid
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from ament_index_python.packages import get_package_prefix
from mavros_msgs.msg import Mavlink


def test_native_relay_preserves_original_reset_and_ack_fields():
    executable=Path(get_package_prefix('boom_birds_sensing'))/'lib/boom_birds_sensing/mavros_observation_relay'
    assert executable.is_file()
    name='/test_relay_'+uuid.uuid4().hex
    child=subprocess.Popen([str(executable),'--ros-args','-p','input_topic:='+name+'/in',
        '-p','output_topic:='+name+'/out'],stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
    context=Context();rclpy.init(context=context);node=Node('relay_test_observer',context=context)
    executor=SingleThreadedExecutor(context=context);executor.add_node(node)
    received={}
    imuSamples=[]
    sensorQos=QoSProfile(depth=1000,reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(Mavlink,name+'/out',lambda m:received.__setitem__(m.msgid,m),qos_profile_sensor_data)
    for mid in (30,32,230):
        node.create_subscription(Mavlink,name+'/out/latest_'+str(mid),
            lambda m:received.__setitem__(m.msgid,m),qos_profile_sensor_data)
    def sensor(m):
        received[m.msgid]=m
        if m.msgid==105 and m.payload64 and m.payload64[0]>=10000:
            imuSamples.append(m.payload64[0])
    for mid in (105,132):
        node.create_subscription(Mavlink,name+'/out/sensor_'+str(mid),sensor,sensorQos)
    pub=node.create_publisher(Mavlink,name+'/in',sensorQos)
    packets=[]
    for mid,framing in [(77,1),(331,1),(105,1),(132,1),(251,1),(30,0),(30,1),(32,1),(230,1),(0,1)]:
        m=Mavlink(msgid=mid,framing_status=framing,sysid=1,compid=1,seq=17 if framing==1 else 19,checksum=1234,
                  len=232 if mid==331 else 8,payload64=[0]*29 if mid==331 else [400])
        m.header.stamp.sec=123;m.header.frame_id='original_router'
        if mid==331:m.payload64[28]=7<<48
        packets.append(m)
    try:
        deadline=time.monotonic()+5.
        while time.monotonic()<deadline and not {0,30,32,77,105,132,230,331}.issubset(received):
            assert child.poll() is None
            if pub.get_subscription_count():
                for m in packets:pub.publish(m)
            executor.spin_once(timeout_sec=.02)
        assert set(received)=={0,30,32,77,105,132,230,331}
        for original in (m for m in packets if m.msgid in (30,32,77,105,132,230,331) and m.framing_status==1):
            m=received[original.msgid]
            assert m.header==original.header and m.sysid==m.compid==1 and m.seq==17
            assert m.checksum==1234 and m.len==original.len and list(m.payload64)==list(original.payload64)
        # 停止 spin 的发布窗口模拟 Python 接收者短时繁忙；IMU 中间样本不能合并为最新值。
        for i in range(64):
            pub.publish(Mavlink(msgid=105,framing_status=1,sysid=1,compid=1,
                               len=8,payload64=[10000+i],seq=i))
        deadline=time.monotonic()+3.
        while len(imuSamples)<64 and time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.02)
        assert imuSamples==list(range(10000,10064))
    finally:
        child.terminate();child.communicate(timeout=5.)
        executor.remove_node(node);executor.shutdown()
        node.destroy_node();context.shutdown()
