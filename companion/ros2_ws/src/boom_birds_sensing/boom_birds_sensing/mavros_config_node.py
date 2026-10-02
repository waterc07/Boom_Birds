"""通过插件参数服务配置 MAVROS 时间同步，并回读结果。"""
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient

class MavrosConfigNode(Node):
    def __init__(self):
        super().__init__("boom_birds_mavros_config")
        self.client = AsyncParameterClient(self, "/mavros/time")
        self.expected = dict(timesync_mode="MAVLINK", timesync_rate=10.0,
            convergence_window=5, max_rtt_sample=20, max_deviation_sample=50)
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
                was_ready = self.ready
                self.ready = values == list(self.expected.values())
                if self.ready and not was_ready:
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
    finally: node.destroy_node(); rclpy.shutdown()
