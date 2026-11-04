"""三任务脚本的启动边界、配置锁定与取消后接地回读。无相机或飞控。"""
import importlib.util
import json
from pathlib import Path
import pytest

SCRIPT = Path(__file__).resolve().parents[3] / 'tools/tag_flight_test.py'
spec = importlib.util.spec_from_file_location('tag_flight_test', SCRIPT)
tool = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tool)


def ready():
    task = dict(task='tag_takeoff_land', state='IDLE', handover_enabled=False,
                landing_inputs_ready=True, landed_disarmed=True)
    control = dict(control_mode='px4_velocity',
        velocity_controller=dict(estimator_ready=True,ready=True, latched='', feedback_error='', hardware_profile_verified=True),
        backend=dict(connected=True, acknowledged_armed=False, endpoint_verified=True,
                     attitude_scaling_ready=True, timesync_reason='ready', timesync_count=10))
    return task, control


@pytest.mark.parametrize('change', ('stale', 'wrong_task', 'armed', 'latched', 'unverified', 'missing_inputs', 'handover'))
def test_ready_rejects_unsafe_start(change):
    task, control = ready()
    age = .1
    if change == 'stale': age = .51
    if change == 'wrong_task': task['task'] = 'tag_forward_land'
    if change == 'armed': control['backend']['acknowledged_armed'] = True
    if change == 'latched': control['velocity_controller']['latched'] = 'flight_controller_restart'
    if change == 'unverified': control['velocity_controller']['hardware_profile_verified'] = False
    if change == 'missing_inputs': task['landing_inputs_ready'] = False
    if change == 'handover': task['handover_enabled'] = True
    assert tool.readyReasons('tag_takeoff_land', task, control, age, .1)


def test_landed_requires_fresh_disarmed_feedback_from_both_nodes():
    task, control = ready()
    assert tool.landedFresh(task, control, .1, .1)
    for a, b in ((.51,.1), (.1,.51), (-.1,.1)):
        assert not tool.landedFresh(task, control, a, b)
    control['backend']['acknowledged_armed'] = True
    assert not tool.landedFresh(task, control, .1, .1)
    control['backend'].pop('acknowledged_armed')
    assert not tool.landedFresh(task, control, .1, .1)


@pytest.mark.parametrize('mode', ('start', 'dry_run', 'allow_arming', 'flight_task_file', 'tag_test_only'))
def test_config_cannot_override_flight_mode_or_task(tmp_path, mode):
    config = tmp_path/'config.json'
    config.write_text(json.dumps(dict(schema=1, launch={mode: 'true'})))
    with pytest.raises(ValueError, match='reserved'):
        tool.loadConfig(config)


def test_dry_run_cannot_arm_or_auto_start():
    args = tool.launchCommand(dict(launch={}, hardware_validation_note=''), Path('tag_takeoff_land.yaml'),
                              dict(synthetic=True), False)
    for value in ('dry_run:=true', 'start:=false', 'allow_arming:=false',
                  'allow_hardware_actions:=false', 'tag_test_only:=true', 'handover:=false'):
        assert value in args
    assert all(not arg.endswith(":=") for arg in args)
    live = tool.launchCommand(dict(launch={}, hardware_validation_note='measured'), Path('tag_takeoff_land.yaml'),
                              dict(synthetic=True), True)
    assert 'tag_test_only:=false' in live and 'start:=false' in live


def test_config_resolves_paths_and_requires_explicit_range_id(tmp_path):
    launch = {key: 'hardware/'+key for key in tool.FILES}
    launch.update(fcu_url='serial:///dev/ttyTEST:230400', range_sensor_id=0)
    config = tmp_path/'config.json'
    config.write_text(json.dumps(dict(schema=1, launch=launch)))
    assert tool.loadConfig(config)['launch']['tag_config_file'] == str(tmp_path/'hardware/tag_config_file')
    launch.pop('range_sensor_id')
    config.write_text(json.dumps(dict(schema=1, launch=launch)))
    with pytest.raises(ValueError, match='sensor_id'):
        tool.loadConfig(config)


@pytest.mark.parametrize('autoStart', (False, True))
def test_ros_console_start_once_and_cancel_keeps_process_until_landed(tmp_path, monkeypatch, autoStart):
    import os
    import pty
    import signal
    import sys
    import threading
    import time
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from std_msgs.msg import String
    from boom_birds_interfaces.srv import Mission
    from boom_birds_control.runtime_config import DEFAULTS
    context = Context()
    rclpy.init(context=context)
    fake = Node('tag_flight_script_test_peer', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(fake)
    task, control = ready()
    actions, errors = [], []
    changed = {'start': None, 'cancel': None}
    def mission(req, response):
        actions.append(req.action)
        if req.action == req.START:
            changed['start'] = time.monotonic()
            task.update(state='TAKEOFF', landed_disarmed=False)
            control['backend']['acknowledged_armed'] = True
        if req.action == req.CANCEL:
            changed['cancel'] = time.monotonic()
        response.accepted = True
        response.reason = 'test peer only'
        return response
    master, slave = pty.openpty()
    terminal = os.fdopen(slave, 'r')
    monkeypatch.setattr(sys, 'stdin', terminal)
    out = tmp_path/'console'
    done = threading.Event()
    def driver():
        try:
            begun = time.monotonic()
            sentStart = sentCancel = False
            taskPub = controlPub = None
            while not done.is_set() and time.monotonic()-begun < 20:
                elapsed = time.monotonic()-begun
                # 先让脚本完成互斥发现，再发布测试节点，模拟其启动的子进程。
                if elapsed > 2.5:
                    if taskPub is None:
                        taskPub = fake.create_publisher(String, '/boom_birds/tag_task/status', 10)
                        controlPub = fake.create_publisher(String, DEFAULTS.control_status_topic, 10)
                        fake.create_service(Mission, DEFAULTS.mission_service, mission)
                    taskPub.publish(String(data=json.dumps(task)))
                    controlPub.publish(String(data=json.dumps(control)))
                if not autoStart and elapsed > 6 and not sentStart:
                    os.write(master, b'START\nSTART\n'); sentStart = True
                if changed['start'] and time.monotonic()-changed['start'] > .5 and not sentCancel:
                    os.write(master, b'CANCEL\n'); sentCancel = True
                if changed['cancel']:
                    if time.monotonic()-changed['cancel'] < 1.:
                        assert not (out/'child-stopped').exists()
                    else:
                        task.update(landed_disarmed=True, state='FAULT_LATCHED')
                        control['backend']['acknowledged_armed'] = False
                executor.spin_once(timeout_sec=.02)
        except BaseException as exc:
            errors.append(exc)
    thread = threading.Thread(target=driver, daemon=True)
    thread.start()
    # 子进程仅休眠，SIGINT 时留下退出记录；不运行 ROS 控制器。
    command = [sys.executable, '-c',
        'import signal,time;from pathlib import Path;'
        + 'signal.signal(signal.SIGINT,lambda a,b:(Path('+repr(str(out/'child-stopped'))+').touch(),exit(0)));time.sleep(30)']
    try:
        result = tool.runStack({}, 'tag_takeoff_land', command, True, out, autoStart=autoStart)
        assert result == 2  # 取消不记为任务成功。
        assert actions == [Mission.Request.START, Mission.Request.CANCEL]
        assert (out/'child-stopped').exists()
        assert not errors
    finally:
        done.set();thread.join(timeout=3)
        terminal.close();os.close(master)
        executor.shutdown();fake.destroy_node();context.shutdown()


def test_prestart_latch_exit_requires_current_ground_and_disarmed_evidence():
    task,control=ready()
    control['velocity_controller']['latched']='vio_gap_reference_invalid'
    assert tool.prestartFault(task,control,.1,.1)=='vio_gap_reference_invalid'
    assert not tool.prestartFault(task,control,.6,.1)
    control['backend']['acknowledged_armed']=True
    assert not tool.prestartFault(task,control,.1,.1)
    control['backend']['acknowledged_armed']=False
    task['landed_disarmed']=False
    assert not tool.prestartFault(task,control,.1,.1)


def test_landing_input_diagnostic_does_not_require_optional_ground_range():
    task,control=ready()
    task.update(landing_inputs_ready=False,landing_input_reason='landing_sensor_link',
        landing_input_detail=dict(image_fresh=False,range_fresh=False,range_optional_near_ground=True))
    reasons=tool.readyReasons('tag_takeoff_land',task,control,.1,.1)
    assert 'landing_image_stale_or_missing' in reasons
    assert 'landing_range_stale_or_missing' not in reasons
