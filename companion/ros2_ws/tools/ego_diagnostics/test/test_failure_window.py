import importlib.util
from pathlib import Path
import pytest

spec=importlib.util.spec_from_file_location('failure_window',Path(__file__).parents[1]/'failure_window.py')
window=importlib.util.module_from_spec(spec)
spec.loader.exec_module(window)


def fixtureStamps():
    raw=[int(t*1e8) for t in range(0,501)]
    return {name:list(raw) for name in window.RAW_TOPICS+window.STATE_TOPICS}


def test_complete_window():
    assert window.assessWindow(fixtureStamps(),30_000_000_000,3)['diagnostic_window_ready']


def test_late_marker_is_not_coverage():
    assert not window.assessWindow(fixtureStamps(),50_000_000_000,3)['diagnostic_window_ready']


def test_no_marker_is_not_navigation_pass():
    result=window.assessWindow(fixtureStamps(),None,3)
    assert not result['diagnostic_window_ready'] and not result['navigation_verified']


def test_early_capture_gap_is_reported():
    data=fixtureStamps()
    data[window.RAW_TOPICS[0]]=[t for t in data[window.RAW_TOPICS[0]] if not 20e9<t<22e9]
    assert any(r.startswith('raw_window_gap:') for r in window.assessWindow(data,30_000_000_000,3)['reasons'])


def test_missing_diagnostics_or_state_is_not_ready():
    assert not window.assessWindow(fixtureStamps(),30_000_000_000,0)['diagnostic_window_ready']
    data=fixtureStamps();data[window.STATE_TOPICS[0]]=[]
    assert not window.assessWindow(data,30_000_000_000,3)['diagnostic_window_ready']


@pytest.mark.parametrize('value',[0,-1,float('nan'),float('inf')])
def test_invalid_limits(value):
    with pytest.raises(ValueError): window.assessWindow({},None,0,preS=value)


def test_finalized_rosbag_cdr(tmp_path):
    rosbag=pytest.importorskip('rosbag2_py')
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import Image,Imu
    from std_msgs.msg import String
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseWithCovarianceStamped
    import json
    writer=rosbag.SequentialWriter()
    bag=tmp_path/'bag'
    writer.open(rosbag.StorageOptions(uri=str(bag),storage_id='sqlite3'),rosbag.ConverterOptions('cdr','cdr'))
    types={window.RAW_TOPICS[0]:('sensor_msgs/msg/Image',Image),window.RAW_TOPICS[1]:('sensor_msgs/msg/Image',Image),
           window.RAW_TOPICS[2]:('sensor_msgs/msg/Imu',Imu),window.STATE_TOPICS[0]:('geometry_msgs/msg/PoseWithCovarianceStamped',PoseWithCovarianceStamped),
           window.STATE_TOPICS[1]:('nav_msgs/msg/Odometry',Odometry),window.STATUS_TOPIC:('std_msgs/msg/String',String)}
    for name,(kind,cls) in types.items():
        writer.create_topic(rosbag.TopicMetadata(id=0,name=name,type=kind,serialization_format='cdr'))
        for k in range(501):
            message=cls()
            if kind=='std_msgs/msg/String':message.data=json.dumps({'camera_diagnostics':{'camera_frames':10}})
            writer.write(name,serialize_message(message),10_000_000_000+k*100_000_000)
    del writer
    events=tmp_path/'events.json'
    events.write_text(json.dumps([{'phase':'STILL','capture_received_ns':40_000_000_000}]))
    result=window.verifyBag(bag,events)
    assert result['sqlite_integrity']=='ok'
    assert result['diagnostic_window_ready']
    events.write_text(json.dumps([{'phase':'STILL','capture_received_ns':60_000_000_000}]))
    assert not window.verifyBag(bag,events)['diagnostic_window_ready']
