"""场地输入、真值隔离和启动阶段的回归。"""
import json
import math
from pathlib import Path
import pytest
import yaml
from boom_birds_sim.gazebo_arena import packagePath, bindLocalGoals, loadArena


def test_local_goals_use_measured_reference_and_reject_invalid_offsets():
    goals = bindLocalGoals([10, 20, 3], math.pi/2, [[2, 0, 0], [0, 0, 0]])
    assert goals[0] == pytest.approx([10, 22, 3])
    assert goals[1] == [10, 20, 3]
    for offsets in ([[1, 2]], [[float("nan"), 0, 0]]):
        with pytest.raises(ValueError): bindLocalGoals([0,0,0], 0, offsets)


def test_asset_path_rejects_parent_and_symlink_escape(tmp_path):
    root=tmp_path/'root'; root.mkdir()
    outside=tmp_path/'outside'; outside.write_text('x')
    (root/'escape').symlink_to(outside)
    for relative in ('../outside', 'escape'):
        with pytest.raises(ValueError): packagePath(root, relative)


def test_controlled_stereo_fixture_preserves_axis_calibration(tmp_path):
    from boom_birds_sim.gazebo_fixture import prepare, R_CAMERA_IMU
    import xml.etree.ElementTree as ET
    estimator=tmp_path/'est.yaml';imu=tmp_path/'imu.yaml';plugin=tmp_path/'motion.so'
    estimator.write_text('%YAML:1.0\nuse_stereo: true\nmax_cameras: 2\n')
    imu.write_text('%YAML:1.0\nimu0:\n  rostopic: /imu0\n');plugin.write_bytes(b'test')
    out=tmp_path/'out'
    manifest=prepare(out, estimator, imu, plugin, controlled_motion=True, vio_profile='stereo')
    assert 'use_stereo: true' in (out/'estimator_config.yaml').read_text()
    assert 'max_cameras: 2' in (out/'estimator_config.yaml').read_text()
    assert manifest['imu_convention'].startswith('FLU')
    tree=ET.parse(out/'sensor_world.sdf')
    assert tree.findtext(".//plugin[@name='boom_birds::SensorMotion']/manual_start")=='true'
    assert manifest['camera_to_imu_rotation']==R_CAMERA_IMU.tolist()


def test_controlled_capture_waits_for_epoch_and_separates_truth(tmp_path):
    import rclpy
    import rosbag2_py
    from sensor_msgs.msg import Image, Imu
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from boom_birds_sim.gazebo_capture import Capture, TRUTH, DEPTH_TRUTH
    rclpy.init()
    node=Capture(tmp_path, 1, controlled=True)
    try:
        clock=Clock();clock.clock.sec=10;node.clock(clock)
        assert not node.done
        for index in (0,1):
            image=Image();image.header.stamp.sec=10;image.width=640;image.height=480
            image.encoding='mono8';image.step=640;image.data=bytes(640*480)
            node.image(image,index)
        clock.clock.sec=20;node.clock(clock)
        assert not node.done  # 首帧已出现，阶段尚未明确开始。
        (tmp_path/'recording-start.json').write_text(json.dumps({'sim_ns':20000000000}))
        clock.clock.sec=20;clock.clock.nanosec=500000000;node.clock(clock)
        assert not node.done
        imu=Imu();imu.header.stamp.sec=20;node.forward('/boom_birds/imu',imu)
        odom=Odometry();odom.header.stamp.sec=20;node.forward(TRUTH,odom)
        depth=Image();depth.header.stamp.sec=20;node.forward(DEPTH_TRUTH,depth)
        clock.clock.sec=21;clock.clock.nanosec=0;node.clock(clock)
        assert node.done
        assert TRUTH not in node.pubs and DEPTH_TRUTH not in node.pubs
        node.finish()
        def topics(path):
            reader=rosbag2_py.SequentialReader()
            reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
            return {topic.name for topic in reader.get_all_topics_and_types()}
        assert not ({TRUTH,DEPTH_TRUTH}&topics(tmp_path/'inputs'))
        assert topics(tmp_path/'truth')=={TRUTH,DEPTH_TRUTH}
    finally:
        node.destroy_node();rclpy.shutdown()
