import json
import subprocess
import sys
import time
from types import SimpleNamespace as NS
import rclpy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from boom_birds_bringup.compute_release_node import ComputeReleaseNode


def test_compute_release_service_requires_matching_confirmed_lease_and_stopped_processes():
    context=rclpy.context.Context();rclpy.init(context=context)
    children=[subprocess.Popen([sys.executable,"-c","import time;time.sleep(60)"]) for _ in range(2)]
    node=ComputeReleaseNode(children[:1],children[1:],lambda:"session",context=context)
    response=Trigger.Response()
    output=dict(session="old",release_compute=True,vio_required=True,owner="platform",state="ALIGN",token="lease")
    try:
        node.on_status(String(data=json.dumps(output)))
        assert not node.release(Trigger.Request(),response).success
        assert all(p.poll() is None for p in children)
        output["session"]="session";output["state"]="PREPARE"
        node.on_status(String(data=json.dumps(output)))
        assert not node.release(Trigger.Request(),Trigger.Response()).success
        output["state"]="ALIGN"
        node.on_status(String(data=json.dumps(output)))
        assert node.release(Trigger.Request(),Trigger.Response()).success
        assert all(p.poll() is not None for p in children)
    finally:
        for p in children:
            if p.poll() is None:p.terminate()
            p.wait(timeout=3.)
        node.destroy_node();rclpy.shutdown(context=context)
