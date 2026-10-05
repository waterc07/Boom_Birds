from types import SimpleNamespace as NS
from boom_birds_sensing.pose_adapter import PoseAdapter


def test_vio_producer_restart_revokes_map_pose_buffers():
    adapter=NS(_vio_publisher_gid=b'old',pose_buffer=[object()],pending_depth=object(),
        latest_pose=object(),reset_latched=False,get_logger=lambda:NS(warn=lambda _:None))
    adapter.on_reset=lambda req,res:PoseAdapter.on_reset(adapter,req,res)
    PoseAdapter.on_odom(adapter,NS(),NS(publisher_gid=b'new'))
    assert adapter.reset_latched and adapter.blocked_by=='vio_producer_changed'
    assert adapter.pose_buffer==[] and adapter.pending_depth is None and adapter.latest_pose is None
    assert not adapter.vio_valid and not adapter.depth_valid
