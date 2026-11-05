"""通过插件参数服务配置 MAVROS 时间同步，并回读结果。"""
import rclpy
import math
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient

class MavrosConfigNode(Node):
    def __init__(self):
        super().__init__("boom_birds_mavros_config")
        self.client = AsyncParameterClient(self, "/mavros/time")
        self.declare_parameter("timesync_rate_hz",10.)
        self.declare_parameter("convergence_samples",500)
        rate=float(self.get_parameter("timesync_rate_hz").value)
        samples=self.get_parameter("convergence_samples").value
        if not math.isfinite(rate) or not 1<=rate<=50 or type(samples) is not int or not 50<=samples<=500:
            raise ValueError("timesync_startup_limits")
        self.expected = dict(timesync_mode="MAVLINK", timesync_rate=rate,
            convergence_window=samples, max_rtt_sample=20, max_deviation_sample=50)
        self.future = None
        self.ready = False
        self.create_timer(1., self.poll)

    def poll(self):
        if self.future is not None or not self.client.services_are_ready(): return
        self.future = self.client.get_parameters(list(self.expected))
        def checked(f):
            self.future = None
            try:
                from rclpy.parameter import parameter_value_to_python
                values = [parameter_value_to_python(v) for v in f.result().values]
                wasReady = self.ready
                self.ready = values == list(self.expected.values())
                if self.ready and not wasReady:
                    self.get_logger().info("MAVROS sys_time 参数回读通过")
                if self.ready: return
                self.future = self.client.set_parameters([Parameter(k, value=v) for k,v in self.expected.items()])
                self.future.add_done_callback(lambda _: setattr(self, "future", None))
            except Exception as exc: self.get_logger().error(str(exc))
        self.future.add_done_callback(checked)

def main(args=None):
    rclpy.init(args=args)
    node = MavrosConfigNode()
    try: rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException): pass
    finally: node.destroy_node(); rclpy.try_shutdown()
