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


def test_body_odom_is_not_republished_at_an_unchanged_source_stamp():
    import numpy as np
    from builtin_interfaces.msg import Time
    now=[100.01];body=[];ego=[]
    a=NS(reset_latched=False,latest_pose=(100.,np.eye(4),np.zeros(3),np.zeros(3)),
         poseTimeout=.15,depthTimeout=1.,_lastPairWall=100.01,
         get_clock=lambda:NS(now=lambda:NS(nanoseconds=round(now[0]*1e9))),
         _pairPending=lambda _:None,T_i_b=np.eye(4),pIB=np.zeros(3),
         frame_world="global",frame_body="body",inputCov6=None,
         pub_body=NS(publish=body.append),pub_ego=NS(publish=ego.append))
    a._stampFromSeconds=lambda t:Time(sec=int(t),nanosec=round((t-int(t))*1e9))
    a._odomMsg=lambda *args:PoseAdapter._odomMsg(a,*args)
    PoseAdapter.tick(a);PoseAdapter.tick(a)
    assert len(body)==len(ego)==1
    now[0]=100.03;a.latest_pose=(100.02,np.eye(4),np.zeros(3),np.zeros(3))
    PoseAdapter.tick(a)
    assert len(body)==len(ego)==2
    assert body[-1].header.stamp.nanosec==20000000
