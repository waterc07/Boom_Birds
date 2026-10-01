"""真实 C++ traj_server 的会话封装/取消回归，不连接 PX4。"""
import copy
import subprocess
import time
from pathlib import Path
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from ament_index_python.packages import get_package_prefix
from boom_birds_interfaces.msg import ExecutionStatus, ControlCommand, PlannerStatus
from traj_utils.msg import SessionBspline
from geometry_msgs.msg import Point
from conftest import child_env


def test_session_trajectory_cancel_and_old_session(tmp_path):
    context = Context()
    rclpy.init(context=context)
    node = rclpy.create_node("session_trajectory_test", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    status_pub = node.create_publisher(ExecutionStatus, "/boom_birds/control/execution_status", 50)
    spline_pub = node.create_publisher(SessionBspline, "planning/session_bspline", 50)
    commands = []
    instances = []
    instance_sub = node.create_subscription(PlannerStatus, "/boom_birds/planner/executor_status", instances.append, 10)
    sub = node.create_subscription(ControlCommand, "/boom_birds/planner/command", lambda msg: commands.append(msg), 50)
    status = ExecutionStatus(session_id="new-session", offboard_confirmed=True)
    binary = Path(get_package_prefix("ego_planner")) / "lib/ego_planner/traj_server"
    log = (tmp_path / "traj_server.log").open("w")
    process = subprocess.Popen([str(binary), "--ros-args", "-p", "project/require_session:=true", "-p", "traj_server/time_forward:=1.0"], env=child_env(), stdout=log, stderr=subprocess.STDOUT)
    def spin(duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            assert process.poll() is None
            status.header.stamp = node.get_clock().now().to_msg()
            status_pub.publish(status)
            executor.spin_once(timeout_sec=.01)
    def spline(session, seq):
        msg = SessionBspline(session_id=session, producer_session_id="planner-instance", sequence=seq)
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = "global"
        msg.trajectory.order = 3
        msg.trajectory.traj_id = 1
        msg.trajectory.start_time = node.get_clock().now().to_msg()
        msg.trajectory.pos_pts = [Point(x=1., y=2., z=1.5) for _ in range(6)]
        msg.trajectory.knots = [float(i) * .2 for i in range(10)]
        return msg
    try:
        spin(1.5)
        assert spline_pub.get_subscription_count() > 0
        assert instances and all(m.session_id == status.session_id and m.producer_session_id for m in instances)
        wrong_frame = spline(status.session_id, 1)
        wrong_frame.header.frame_id = "other-world"
        spline_pub.publish(wrong_frame)
        spin(.15)
        assert not commands
        missing = spline(status.session_id, 1)
        missing.producer_session_id = ""
        spline_pub.publish(missing)
        spin(.15)
        assert not commands
        spline_pub.publish(spline("old-session", 99))
        spin(.15)
        assert not commands
        spline_pub.publish(spline(status.session_id, 1))
        spin(.2)
        assert commands and all(m.session_id == status.session_id for m in commands)
        assert all(m.planner_session_id == "planner-instance" and m.executor_session_id for m in commands)
        assert len({m.executor_session_id for m in commands}) == 1
        assert any(m.command_type == m.EXECUTE for m in commands)
        invalid = spline(status.session_id, 2)
        invalid.trajectory.order = 0
        invalid.trajectory.pos_pts = []
        spline_pub.publish(invalid)
        spin(.15)
        cancelled = next(i for i, m in enumerate(commands) if m.command_type == m.CANCEL)
        assert not any(m.command_type == m.EXECUTE for m in commands[cancelled + 1:])
        spline_pub.publish(spline(status.session_id, 1))
        spin(.15)
        assert not any(m.command_type == m.EXECUTE for m in commands[cancelled + 1:])
        spline_pub.publish(spline(status.session_id, 3))
        spin(.15)
        assert commands[-1].command_type == ControlCommand.EXECUTE
        previous_seq = commands[-1].sequence
        changed = spline(status.session_id, 1000)
        changed.producer_session_id = "restarted-planner"
        spline_pub.publish(changed)
        # 跨进程订阅处理前仍可能收到在途旧命令；先收齐，再验证持续停止。
        spin(.15)
        stopped_seq = commands[-1].sequence
        assert stopped_seq >= previous_seq
        assert all(m.planner_session_id == "planner-instance" for m in commands)
        spin(.15)
        assert commands[-1].sequence == stopped_seq
        status.offboard_confirmed = False
        spin(.15)
        last = commands[-1].sequence
        spin(.15)
        assert commands[-1].sequence == last
        assert last >= previous_seq
    finally:
        process.terminate()
        try: process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill(); process.wait()
        log.close()
        executor.shutdown()
        node.destroy_node()
        context.shutdown()
