from pathlib import Path
import importlib.util
import numpy as np
import pytest
import yaml
from boom_birds_bringup.hardware_profile import validate_profile
from boom_birds_control.runtime_config import DEFAULTS


def bundle(tmp_path):
    k=np.array([[400.,0,320.],[0,400.,240.],[0,0,1.]])
    np.savez(tmp_path/'stereo.npz',K1=k,K2=k,D1=np.zeros(4),D2=np.zeros(4),R=np.eye(3),T=np.array([-.1,0,0]),image_size=[640,480])
    t=np.eye(4);r=t.copy();r[0,3]=.1
    cam={}
    for i,trans in enumerate((t,r)):
        cam['cam'+str(i)]={'camera_model':'pinhole','distortion_model':'radtan','distortion_coeffs':[0.,0.,0.,0.],
            'intrinsics':[400.,400.,320.,240.],'resolution':[640,480],'T_imu_cam':trans.tolist(),
            'rostopic':(DEFAULTS.stereo_left_topic,DEFAULTS.stereo_right_topic)[i]}
    files={'extrinsics.yaml':{'T_I_C0':t.tolist(),'T_I_B':t.tolist(),'source':'unit test fixture'},
        'camera.yaml':cam,'imu.yaml':{'imu0':{'rostopic':DEFAULTS.imu_topic}},
        'estimator.yaml':{'max_cameras':2,'use_stereo':True,'relative_config_imucam':'camera.yaml','relative_config_imu':'imu.yaml'},
        'attitude.yaml':{'hardware_verified':False,'evidence':'','controller':{},'mission':{}}}
    for name,cfg in files.items(): (tmp_path/name).write_text(yaml.safe_dump(cfg))
    return dict(fcu_url='serial:///dev/test:921600',calibration_file=str(tmp_path/'stereo.npz'),
        extrinsics_file=str(tmp_path/'extrinsics.yaml'),vio_config_file=str(tmp_path/'estimator.yaml'),
        attitude_config_file=str(tmp_path/'attitude.yaml'))


def test_dryrun_can_review_unverified_profile_but_live_is_blocked(tmp_path):
    a=bundle(tmp_path)
    result=validate_profile(**a)
    assert not result['hardware_verified'] and len(result['files'])==6
    with pytest.raises(ValueError,match='只允许 dry-run'): validate_profile(**a,live=True)


def test_camera_imu_calibration_mismatch_is_rejected(tmp_path):
    a=bundle(tmp_path)
    p=tmp_path/'extrinsics.yaml';cfg=yaml.safe_load(p.read_text());cfg['T_I_C0'][0][3]=.02;p.write_text(yaml.safe_dump(cfg))
    with pytest.raises(ValueError,match='相机外参不同'):validate_profile(**a)


def test_stereo_intrinsics_and_baseline_are_checked(tmp_path):
    a=bundle(tmp_path)
    p=tmp_path/'camera.yaml';cfg=yaml.safe_load(p.read_text());cfg['cam1']['intrinsics'][0]=450;p.write_text(yaml.safe_dump(cfg))
    with pytest.raises(ValueError,match='内参不同'):validate_profile(**a)


def test_hardware_launch_builds_actions_without_opening_devices(tmp_path,monkeypatch):
    from launch import LaunchContext
    path=Path(__file__).resolve().parents[1]/'launch/attitude_hardware.launch.py'
    spec=importlib.util.spec_from_file_location('hardware_launch_probe',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    # 动作组合测试不依赖 CI 未构建的可选算法；实际入口发现由完整安装检查覆盖。
    share=module.get_package_share_directory
    monkeypatch.setattr(module,'get_package_share_directory',lambda name:
        str(tmp_path/name) if name in ('ego_planner','ov_msckf') else share(name))
    ctx=LaunchContext();ctx.launch_configurations.update(bundle(tmp_path))
    ctx.launch_configurations.update(camera_device='/dev/test',capture_width='1280',capture_height='480',capture_fps='60',
        output_scale='.75',compute_profile='calibrated',dry_run='true',allow_arming='false',allow_non_loopback='true',allow_hardware_actions='false',hardware_validation_note='')
    assert len(module.nodes(ctx))==9
    ctx.launch_configurations['dry_run']='typo'
    with pytest.raises(ValueError,match='true/false'):module.nodes(ctx)


def test_inconsistent_camera_mode_or_different_camera_clocks_are_rejected(tmp_path):
    a=bundle(tmp_path)
    p=tmp_path/'estimator.yaml';cfg=yaml.safe_load(p.read_text());cfg['max_cameras']=1;p.write_text(yaml.safe_dump(cfg))
    with pytest.raises(ValueError,match='OpenVINS 相机配置'):validate_profile(**a)
    a=bundle(tmp_path)
    p=tmp_path/'camera.yaml';cfg=yaml.safe_load(p.read_text());cfg['cam1']['timeshift_cam_imu']=.01;p.write_text(yaml.safe_dump(cfg))
    with pytest.raises(ValueError,match='时间偏移'):validate_profile(**a)
