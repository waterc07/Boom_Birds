#!/usr/bin/env python3
"""独立 Tag 任务测试；配置检查不连接设备，fly 在连续就绪后自动请求一次启动。"""
import argparse
import json
import math
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import time

TASKS = ('tag_takeoff_land', 'tag_forward_land', 'tag_return_turn')
FILES = ('calibration_file', 'extrinsics_file', 'vio_config_file',
         'attitude_config_file', 'tag_config_file', 'obstacle_config_file')
# 这些参数由脚本固定，配置不能覆盖解锁模式、自动启动或任务文件。
INPUTS = set(FILES) | {'fcu_url', 'camera_device', 'capture_width', 'capture_height',
    'capture_fps', 'downward_input', 'downward_device', 'downward_pixel_format',
    'downward_capture_fps', 'downward_frame_id', 'downward_camera_config_file',
    'downward_native_config_file', 'downward_media_device', 'downward_sensor_entity',
    'downward_raw10_native_library', 'range_input', 'range_sensor_id',
    'range_target_system', 'range_target_component', 'fast_start'}


def loadConfig(path):
    import yaml
    path = Path(path).resolve()
    data = yaml.safe_load(path.read_text())
    if not isinstance(data, dict) or data.get('schema') != 1:
        raise ValueError('test_config_schema')
    args = data.get('launch', {})
    if not isinstance(args, dict) or set(args) - INPUTS:
        raise ValueError('unknown_or_reserved_launch_arguments')
    if any(type(v) not in (str, int, float) or isinstance(v, float) and not math.isfinite(v)
           for v in args.values()):
        raise ValueError('launch_argument_type')
    if args.get('fast_start','false') not in ('true','false'):
        raise ValueError('fast_start_must_be_true_or_false')
    for key in (*FILES, 'fcu_url'):
        if not isinstance(args.get(key), str) or not args[key].strip():
            raise ValueError('missing_' + key)
    for key in (*FILES, 'downward_camera_config_file', 'downward_native_config_file',
                'downward_raw10_native_library'):
        if args.get(key):
            file = Path(args[key]).expanduser()
            args[key] = str((path.parent / file).resolve())
    if args.get('range_input', 'mavros') == 'mavros':
        sensor = args.get('range_sensor_id')
        if type(sensor) is not int or not 0 <= sensor <= 255:
            raise ValueError('explicit_range_sensor_id_required')
    if type(data.get('prepare_dds_buffers', False)) is not bool:
        raise ValueError('prepare_dds_buffers_must_be_boolean')
    timeout = data.get('preflight_timeout_s', 180.)
    if type(timeout) not in (int, float) or not math.isfinite(timeout) or not 30 <= timeout <= 300:
        raise ValueError('preflight_timeout_s_must_be_30_to_300')
    return data


def inspectConfig(data, task):
    from ament_index_python.packages import get_package_share_directory
    from boom_birds_bringup.hardware_profile import validateProfile, readYaml
    from boom_birds_control.platform_model import validateProfile as validateTag
    from boom_birds_bringup.task_depth_guard import loadObstacleConfig
    from boom_birds_bringup.tag_flight_task import loadTask, taskFromProfile
    from boom_birds_control.runtime_config import DEFAULTS
    args = data['launch']
    taskFile = Path(get_package_share_directory('boom_birds_bringup')) / 'config/flight_tasks' / (task + '.yaml')
    definition = loadTask(taskFile)
    if definition['name'] != task or definition['height_m'] != .8:
        raise ValueError('expected_named_0_8m_task')
    board = readYaml(args['tag_config_file'])
    from boom_birds_control.flight_trial import validateTrialTask
    validateTrialTask([readYaml(args["attitude_config_file"]), board,
                       readYaml(args["obstacle_config_file"])], definition)
    checks = {}
    def check(name, action):
        try:
            action()
            checks[name] = 'OK'
        except (ValueError, KeyError, TypeError, OSError) as exc:
            checks[name] = str(exc)
    profileArgs = {key: args[key] for key in ('fcu_url', *FILES[:4])}
    def geometry():
        validateProfile(**profileArgs, control_mode="px4_velocity")
        vio = readYaml(args['vio_config_file'])
        if vio.get('max_cameras') != 2 or vio.get('use_stereo') is not True or vio.get('use_klt', True) is not True:
            raise ValueError('basic_requires_stereo_klt')
        validateTag(board, test_only=board.get('synthetic') is True)
        obstacle = loadObstacleConfig(args['obstacle_config_file'])
        taskFromProfile(DEFAULTS, readYaml(args['attitude_config_file']), definition, obstacle, handover=False)
    check('configuration', geometry)
    check('live_velocity_guidance', lambda: validateProfile(**profileArgs, live=True, control_mode="px4_velocity"))
    check('live_tag', lambda: validateTag(board, test_only=False))
    check('live_obstacle', lambda: loadObstacleConfig(args['obstacle_config_file'], live=True))
    note = data.get('hardware_validation_note')
    checks['validation_note'] = 'OK' if isinstance(note, str) and note.strip() else 'hardware_validation_note_missing'
    return taskFile, board, checks


def launchCommand(data, taskFile, board, live):
    args = dict(data['launch'])
    args.update(flight_task_file=str(taskFile), start=False, dry_run=not live,
        allow_arming=live, allow_non_loopback=True, allow_hardware_actions=live,
        hardware_validation_note=data.get('hardware_validation_note', ''),
        tag_test_only=bool(board.get('synthetic')) if not live else False,
        control_mode="px4_velocity", handover=False, tag_compute_profile='basic', start_sensing=True,
        depth_roi=False, shared_decode=True)
    return ['ros2', 'launch', 'boom_birds_bringup', taskFile.stem + '.launch.py'] + [
        key + ':=' + (str(value).lower() if type(value) is bool else str(value))
        for key, value in args.items() if value != ""]


def readyReasons(taskName, task, control, taskAge, controlAge):
    reasons = []
    if not 0 <= taskAge <= .5 or not 0 <= controlAge <= .5:
        return ['status_stale']
    if task.get('task') != taskName or task.get('state') != 'IDLE':
        reasons.append('task_not_idle_or_wrong_task')
    if task.get('handover_enabled') is not False:
        reasons.append('handover_not_disabled')
    if task.get('landing_inputs_ready') is not True:
        reasons.append(task.get('landing_input_reason') or 'landing_inputs_missing')
        detail = task.get('landing_input_detail') or {}
        if detail.get('image_fresh') is False:
            reasons.append('landing_image_stale_or_missing')
        if detail.get('range_fresh') is False and detail.get('range_optional_near_ground') is not True:
            reasons.append('landing_range_stale_or_missing')
    attitude = control.get('velocity_controller') or {}
    backend = control.get('backend') or {}
    if control.get('control_mode') != 'px4_velocity':
        reasons.append('wrong_control_mode')
    if attitude.get('ready') is not True or attitude.get('latched') or attitude.get('feedback_error'):
        reasons.append(attitude.get('latched') or attitude.get('feedback_error') or 'reference_not_ready')
    if attitude.get('hardware_profile_verified') is not True and attitude.get('flight_trial_authorized') is not True:
        reasons.append('velocity_guidance_profile_unverified')
    if backend.get('connected') is not True or backend.get('acknowledged_armed') is not False:
        reasons.append('fcu_not_connected_and_disarmed')
    if backend.get('endpoint_verified') is not True:
        reasons.append('fcu_endpoint_unverified')
    if attitude.get('estimator_ready') is not True:
        reasons.append(attitude.get('estimator_reason') or 'px4_velocity_estimate_missing')
    if backend.get('timesync_reason') != 'ready' or backend.get('timesync_count', 0) < 5:
        reasons.append('timesync_not_ready')
    return reasons


def landedFresh(task, control, taskAge, controlAge):
    return (0 <= taskAge <= .5 and 0 <= controlAge <= .5
        and task.get('landed_disarmed') is True
        and (control.get('backend') or {}).get('acknowledged_armed') is False)


def prestartFault(task, control, taskAge, controlAge):
    if not landedFresh(task, control, taskAge, controlAge):
        return ''
    return (control.get('velocity_controller') or {}).get('latched') or ''


def runStack(data, taskName, command, live, out, *, autoStart=True):
    if data.get("launch",{}).get("fast_start")=="true":
        from boom_birds_bringup.runtime_network import configureFastDds
        configureFastDds()
    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import String
    from boom_birds_interfaces.srv import Mission
    from boom_birds_control.runtime_config import DEFAULTS
    if live and not autoStart and not sys.stdin.isatty():
        raise ValueError('fly_requires_interactive_terminal')
    out.mkdir(parents=True, exist_ok=False)
    (out/'command.json').write_text(json.dumps(command, ensure_ascii=False, indent=2))
    log = (out/'launch.log').open('w')
    trace = (out/'status.jsonl').open('w')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('tag_flight_test_console')
    latest = {'task': {}, 'control': {}}
    received = {'task': -math.inf, 'control': -math.inf}
    def observe(key, msg):
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict):
                return
        except (ValueError, TypeError):
            return
        latest[key], received[key] = value, time.monotonic()
        trace.write(json.dumps({'time': time.time(), 'topic': key, 'value': value}) + '\n')
        trace.flush()
    node.create_subscription(String, '/boom_birds/tag_task/status', lambda m: observe('task', m), 10)
    node.create_subscription(String, DEFAULTS.control_status_topic, lambda m: observe('control', m), 10)
    client = node.create_client(Mission, DEFAULTS.mission_service)
    stop = False
    def stopping(number, frame):
        nonlocal stop
        stop = True
    signals = {n: signal.signal(n, stopping) for n in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
    from boom_birds_bringup.runtime_network import DdsSocketBuffers
    buffers = DdsSocketBuffers()
    child = None
    requested = cancelled = False
    future = None
    action = None
    prompt = False
    readyRecorded = False
    lastPrint = -math.inf
    result = 2
    cancelAt = math.inf
    inputBuffer = ""
    print('日志：' + str(out), flush=True)
    print(('fly：核心输入就绪后自动请求一次起飞；CANCEL 或 Ctrl+C 请求取消。' if autoStart
           else 'fly：就绪后输入 START 才请求一次起飞；CANCEL 或 Ctrl+C 请求取消。') if live
          else 'dry-run：不发送 START、不允许解锁；Ctrl+C 结束。', flush=True)
    if live:
        print('上方 fly_ready 是配置许可；实时输入仍需通过下面的就绪检查。', flush=True)
    if data.get('prepare_dds_buffers', False):
        print(('快速准备：等待时钟收敛、连续静止 VIO 和 PX4 实际速度反馈；尽量缩短准备时间，不满足输入条件不会 START。' if data.get('launch',{}).get('fast_start') == 'true' else '准备阶段等待 500 个合格时钟同步样本与 VIO 初始化，通常约 1–2 分钟；准备预算不计入起飞后的 60 s。'), flush=True)
    try:
        discoveryEnd = time.monotonic()+2.
        while time.monotonic() < discoveryEnd:
            rclpy.spin_once(node, timeout_sec=.05)
        if (node.count_publishers('/boom_birds/tag_task/status')
                or node.count_publishers(DEFAULTS.control_status_topic)
                or client.service_is_ready()):
            raise ValueError('existing_task_or_controller_stop_it_before_launch')
        if data.get('prepare_dds_buffers', False):
            buffers.prepare()
            (out/'network-buffers.json').write_text(json.dumps(dict(original=buffers.original, minimum=buffers.minimum)))
        child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        preflightStarted = time.monotonic()
        while True:
            rclpy.spin_once(node, timeout_sec=.05)
            now = time.monotonic()
            task, control = latest['task'], latest['control']
            ages = now-received['task'], now-received['control']
            if child.poll() is not None:
                print('任务进程退出，请查看 launch.log；若仍在空中请遥控接管。', flush=True)
                break
            reasons = readyReasons(taskName, task, control, *ages)
            ready = not reasons and client.service_is_ready()
            if ready and not readyRecorded:
                (out/'ready.json').write_text(json.dumps(dict(wall_time=time.time(),
                    preflight_elapsed_s=now-preflightStarted, start_sent=False),indent=2))
                readyRecorded = True
                if not live:
                    print('READY：只读就绪检查通过，未发送 START。', flush=True)
            latched = prestartFault(task, control, *ages)
            if (live and not requested and now-preflightStarted >= data.get('preflight_timeout_s', 180.)
                    and landedFresh(task, control, *ages)):
                print('起飞前等待超时；未发送 START，结束本轮。' + ','.join(str(x) for x in reasons), flush=True)
                break
            if live and not requested and latched:
                print('起飞前参考已锁定：' + str(latched)
                      + '；未发送 START，结束本轮，请查看日志后重新运行。', flush=True)
                break
            if now-lastPrint >= 5.:
                print('state=' + str(task.get('state', 'waiting')) + ' phase=' + str(task.get('phase', ''))
                      + ' reason=' + ','.join(str(x) for x in reasons), flush=True)
                lastPrint = now
            if live and autoStart and ready and not requested and not stop and future is None:
                requested = True
                action = Mission.Request.START
                future = client.call_async(Mission.Request(action=action))
                print('READY：已自动发送一次 START。', flush=True)
            if live and not autoStart and ready and not prompt and not requested:
                print('READY：确认起飞区域和遥控器就绪，输入 START 后回车。', flush=True)
                prompt = True
            if live and sys.stdin.isatty() and select.select([sys.stdin], [], [], 0)[0]:
                chunk = os.read(sys.stdin.fileno(), 4096).decode('utf-8', errors='replace')
                if not chunk:
                    stop = True
                inputBuffer += chunk
            while '\n' in inputBuffer:
                line, inputBuffer = inputBuffer.split('\n', 1)
                if line.strip() == 'CANCEL':
                    stop = True
                elif line.strip() == 'START' and not requested and not stop:
                    if ready and future is None:
                        requested = True  # 服务超时或响应丢失时也不重发 START。
                        action = Mission.Request.START
                        future = client.call_async(Mission.Request(action=action))
                        print('START 已发送，等待服务响应。', flush=True)
                    else:
                        print('尚未就绪，未发送 START。' + ','.join(str(x) for x in reasons), flush=True)
            if future is not None and future.done():
                response = future.result()
                print('mission response: ' + str(response), flush=True)
                if action == Mission.Request.START and not response.accepted:
                    stop = True
                future = None
            if stop:
                if not live or not requested:
                    break
                if not cancelled and client.service_is_ready():
                    future = client.call_async(Mission.Request(action=Mission.Request.CANCEL))
                    action = Mission.Request.CANCEL
                    cancelled = True
                    cancelAt = now
                    print('已请求取消；保留控制进程，等待接地且锁定。可立即遥控接管。', flush=True)
                # 取消请求之后必须收到新的状态，不能用请求前的地面回读结束。
                if cancelled and future is None and received['task'] > cancelAt and received['control'] > cancelAt and landedFresh(task, control, *ages):
                    break
            if requested and task.get('complete') is True and landedFresh(task, control, *ages):
                result = 0
                print('COMPLETE：已接地且锁定。', flush=True)
                break
    except Exception as exc:
        print('测试控制台异常：' + str(exc), file=sys.stderr, flush=True)
        if live and requested and child is not None and child.poll() is None:
            print('尝试取消并等待接地；请准备遥控接管。', flush=True)
            try:
                if client.service_is_ready():
                    client.call_async(Mission.Request(action=Mission.Request.CANCEL))
                while child.poll() is None:
                    rclpy.spin_once(node, timeout_sec=.1)
                    now = time.monotonic()
                    if landedFresh(latest['task'], latest['control'],
                            now-received['task'], now-received['control']):
                        break
            except Exception as waitError:
                print('无法确认接地：' + str(waitError), file=sys.stderr, flush=True)
        result = 2
    finally:
        # 异常退出时也不能在未确认接地的情况下停止仍存活的控制进程。
        now = time.monotonic()
        safeStop = (not live or not requested or landedFresh(latest['task'], latest['control'],
            now-received['task'], now-received['control']))
        if child is not None and child.poll() is None and not safeStop:
            print('未确认接地，保留控制进程组 pid=' + str(child.pid) + '；请遥控接管。', flush=True)
        if child is not None and child.poll() is None and safeStop:
            os.killpg(child.pid, signal.SIGINT)
            end = time.monotonic()+15
            while child.poll() is None and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.05)
            if child.poll() is None:
                print('进程尚未退出，保留进程组 pid=' + str(child.pid), flush=True)
        if child is None or child.poll() is not None:
            try:
                buffers.restore()
            except Exception as exc:
                print('DDS 缓冲恢复失败：' + str(exc), file=sys.stderr, flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for number, handler in signals.items():
            signal.signal(number, handler)
        log.close()
        trace.close()
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--task', required=True, choices=TASKS)
    parser.add_argument('mode', nargs='?', choices=('check', 'dry-run', 'fly'), default='check')
    parser.add_argument('--config', type=Path, default=Path(os.environ.get('BB_TAG_TEST_CONFIG', Path(__file__).with_name('tag_flight_test.yaml'))))
    parser.add_argument('--out', type=Path)
    parser.add_argument('--manual-start', action='store_true', help='就绪后等待终端 START；fly 默认自动开始')
    args = parser.parse_args(argv)
    try:
        data = loadConfig(args.config)
        taskFile, board, checks = inspectConfig(data, args.task)
        report = {'task': args.task, 'height_m': .8, 'checks': checks,
                  'fly_ready': all(value == 'OK' for value in checks.values())}
        print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
        if args.mode == 'check':
            return 0 if report['fly_ready'] else 2
        if checks['configuration'] != 'OK' or args.mode == 'fly' and not report['fly_ready']:
            print('配置检查未通过，未启动节点、未请求解锁。', file=sys.stderr)
            return 2
        live = args.mode == 'fly'
        out = args.out or Path.home()/'boombirds/evidence'/('test-'+args.task+'-'+time.strftime('%Y%m%d-%H%M%S'))
        return runStack(data, args.task, launchCommand(data, taskFile, board, live), live, out, autoStart=not args.manual_start)
    except (ValueError, OSError, KeyError, TypeError) as exc:
        print(str(exc), file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
