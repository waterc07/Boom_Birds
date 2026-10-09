"""单进程采集＋深度；解码数组直接交给有界深度工作线程。"""
import signal

import cv2

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions

from boom_birds_sensing.depth_node import DepthNode
from boom_birds_sensing.stereo_source import StereoSourceNode


def main(argv=None):
    rclpy.init(args=argv, signal_handler_options=SignalHandlerOptions.NO)
    handlers = {sig: signal.signal(sig, signal.default_int_handler)
                for sig in (signal.SIGINT, signal.SIGTERM)}
    executor = SingleThreadedExecutor()
    nodes = []
    try:
        depth = DepthNode(local_input=True)
        nodes.append(depth)
        source = StereoSourceNode()
        nodes.append(source)
        if source.mode not in ("v4l2", "replay"):
            raise ValueError("共享解码仅支持 v4l2/replay")
        width = int(source.get_parameter("capture_width").value) if source.mode == "v4l2" else source.frame_source.stitched_width
        height = int(source.get_parameter("capture_height").value) if source.mode == "v4l2" else source.frame_source.height
        if width // (2 * source.raw_decode_divisor) < 320 or height // source.raw_decode_divisor < 240:
            raise ValueError("共享解码分辨率不得小于每目 320×240")
        # OpenCV 线程数是进程全局值，两个节点共用深度配置。
        threads = int(depth.get_parameter("opencv_threads").value)
        if threads:
            cv2.setNumThreads(threads)
        source.decoded_sink = depth.queue_decoded_pair
        for node in nodes:
            executor.add_node(node)
        source.start_reader()
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        error = None
        for node in reversed(nodes):
            try:
                node.shutdown()
            except Exception as exc:
                error = error or exc
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
        for sig, handler in handlers.items():
            signal.signal(sig, handler)
        if error:
            raise error


if __name__ == "__main__":
    main()
