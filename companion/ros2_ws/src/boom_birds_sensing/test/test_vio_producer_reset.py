from types import SimpleNamespace as NS
from boom_birds_sensing.pose_adapter import PoseAdapter


def test_vio_producer_restart_revokes_map_pose_buffers():
    adapter=NS(_vioPublisherGid=b'old',pose_buffer=[object()],pending_depth=object(),
        latest_pose=object(),reset_latched=False,get_logger=lambda:NS(warn=lambda _:None))
    adapter.onReset=lambda req,res:PoseAdapter.onReset(adapter,req,res)
    PoseAdapter.onOdom(adapter,NS(),NS(publisher_gid=b'new'))
    assert adapter.reset_latched and adapter.blockedBy=='vio_producer_changed'
    assert adapter.pose_buffer==[] and adapter.pending_depth is None and adapter.latest_pose is None
    assert not adapter.vio_valid and not adapter.depth_valid
