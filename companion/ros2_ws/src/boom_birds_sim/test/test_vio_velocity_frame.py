from types import SimpleNamespace as NS
import math
import numpy as np
from rclpy.time import Time
from boom_birds_sim.vio_source import SyntheticVioSource
from boom_birds_control.frames import quatToRot
from boom_birds_sim.synthetic import defaultScene


def test_synthetic_vio_twist_reconstructs_world_position_derivative():
    output = []
    params = {'y_motion_amplitude_m': .2, 'y_motion_period_s': 10.,
              'pose_lag_s': 0., 'frame_world': 'global'}
    node = NS(scene=defaultScene(),originOffset=0.,_tMotion=lambda:0.,
              get_parameter=lambda name:NS(value=params[name]),
              get_clock=lambda:NS(now=lambda:Time(seconds=100.)),
              pub_odom=NS(publish=output.append))
    SyntheticVioSource.tickOdom(node)
    msg = output[0]
    q=msg.pose.pose.orientation;v=msg.twist.twist.linear
    world=quatToRot((q.x,q.y,q.z,q.w)) @ (v.x,v.y,v.z)
    np.testing.assert_allclose(world,(0.,.2*2*math.pi/10,0.),atol=1e-12)
