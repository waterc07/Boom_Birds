import time
from types import SimpleNamespace
import numpy as np
import pytest
from boom_birds_sim.synthetic_stereo_source import SyntheticStereoSourceNode as StereoSourceNode


def test_startup_altitude_uses_agl_and_does_not_stop_images_during_descent(monkeypatch):
    import boom_birds_sim.synthetic as synth
    monkeypatch.setattr(synth, "renderStereo", lambda scene: (np.zeros((2, 2)), np.zeros((2, 2)), None))
    params = {"synth_pose_topic": "pose", "synth_pose_timeout_s": .5,
              "synth_min_altitude_m": 1.3, "synth_ground_z_m": .1, "synth_map_topic": ""}
    node = SimpleNamespace(mode="synth", _synthPoseMono=time.monotonic(),
        _synthPoseStampS=100., get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=100_010_000_000)),
        _synthPose=(0., 0., 1.35), _synthRotation=np.eye(3), _synthAltitudeStarted=False,
        _synthetic=SimpleNamespace(wall_x=10.),
        get_parameter=lambda name: SimpleNamespace(value=params[name]))
    with pytest.raises(RuntimeError, match="起飞高度"):
        StereoSourceNode._loadOffline(node)
    node._synthPose = (0., 0., 1.5)
    assert StereoSourceNode._loadOffline(node)[0].shape == (2, 2)
    node._synthPose = (0., 0., 1.0)
    assert StereoSourceNode._loadOffline(node)[0].shape == (2, 2)
    node._synthPoseMono = time.monotonic() - 1.
    with pytest.raises(RuntimeError, match="过期"):
        StereoSourceNode._loadOffline(node)


def test_synthetic_frame_uses_render_pose_time_instead_of_completion_time(monkeypatch):
    import boom_birds_sim.synthetic as synth
    rendered = []
    def render(scene):
        rendered.append(scene.camera_x)
        return np.zeros((2, 2)), np.zeros((2, 2)), None
    monkeypatch.setattr(synth, "renderStereo", render)
    params = {"synth_pose_topic": "pose", "synth_pose_timeout_s": .15,
              "synth_min_altitude_m": .3, "synth_ground_z_m": 0., "synth_map_topic": ""}
    node = SimpleNamespace(mode="synth", _synthPoseMono=time.monotonic(),
        _synthPoseStampS=100., _synthPose=(2., 0., 1.5), _synthRotation=np.eye(3),
        _synthAltitudeStarted=True, _synthetic=SimpleNamespace(wall_x=10.),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=100_020_000_000)),
        get_parameter=lambda name: SimpleNamespace(value=params[name]))
    StereoSourceNode._loadOffline(node)
    assert rendered == [2.]
    assert node._offlineCaptureRosS == 100.
    # 旧位姿刚收到也不能被视为新鲜；未来时钟同样拒绝。
    for stamp in (99., 101., 0.):
        node._synthPoseStampS = stamp
        with pytest.raises(RuntimeError, match="过期"):
            StereoSourceNode._loadOffline(node)
    assert rendered == [2.]
