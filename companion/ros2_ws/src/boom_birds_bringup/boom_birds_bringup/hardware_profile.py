"""实机启动文件静态核验；不连接设备、不写 PX4 参数。"""
from pathlib import Path
import hashlib
import numpy as np
import yaml
from boom_birds_control.attitude_control import AttitudeConfig
from boom_birds_sensing.config_io import load_extrinsics
from boom_birds_bringup.attitude_lifecycle import AttitudeMissionConfig
from boom_birds_control.runtime_config import DEFAULTS


def read_yaml(path):
    text=Path(path).read_text(encoding="utf-8")
    return yaml.safe_load("\n".join(line for line in text.splitlines() if not line.startswith("%YAML")))


def validate_profile(*, fcu_url, calibration_file, extrinsics_file, vio_config_file, attitude_config_file, live=False):
    if not fcu_url or fcu_url.startswith("udp://127.0.0.1"):
        raise ValueError("实机入口必须提供实际 fcu_url；SIH 使用单独入口")
    files={k:Path(v).resolve() for k,v in dict(calibration_file=calibration_file,
        extrinsics_file=extrinsics_file,vio_config_file=vio_config_file,attitude_config_file=attitude_config_file).items()}
    for k,p in files.items():
        if not p.is_file(): raise ValueError(k+" 不存在")
        if 'boombirds_synthetic' in str(p) or 'boom_birds_synth' in str(p):
            raise ValueError(k+" 是合成配置")
    extr=load_extrinsics(extrinsics_file)
    source=extr['source']
    if not source.strip() or source=='未注明来源' or 'TEST-ONLY' in source or 'T_I_B' not in read_yaml(extrinsics_file):
        raise ValueError("实机外参必须给出 T_I_B 和实测 source")
    with np.load(calibration_file,allow_pickle=False) as data:
        for k in ('K1','K2','D1','D2','R','T','image_size'):
            if k not in data or not np.isfinite(data[k]).all(): raise ValueError("双目标定缺少有限字段 "+k)
    vio=read_yaml(vio_config_file)
    count = vio.get("max_cameras")
    if type(count) is not int or count not in (1, 2) or vio.get("use_stereo") is not (count == 2):
        raise ValueError("OpenVINS 相机配置必须为单目(1/false)或双目(2/true)")
    if vio.get("boombirds_test_only") is True:
        raise ValueError("实机入口拒绝派生的合成配置")
    if vio.get("downsample_cameras") is True:
        raise ValueError("原图已按 chain 尺寸发布，不允许 OpenVINS 再次缩图")
    for k in ('calib_cam_extrinsics','calib_cam_intrinsics','calib_cam_timeoffset','calib_imu_intrinsics','calib_imu_g_sensitivity'):
        if vio.get(k) is True: raise ValueError('实机首轮禁止在线变更标定 '+k)
    chain_path=files['vio_config_file'].parent/vio['relative_config_imucam']
    chain=read_yaml(chain_path)
    for i,topic in enumerate((DEFAULTS.stereo_left_topic,DEFAULTS.stereo_right_topic)[:count]):
        if chain['cam'+str(i)]['rostopic'] != topic: raise ValueError("OpenVINS 双目 rostopic 不匹配")
    shifts=[float(chain['cam'+str(i)].get('timeshift_cam_imu',0.)) for i in range(count)]
    if not np.isfinite(shifts).all() or (count == 2 and abs(shifts[0]-shifts[1])>1e-9):
        raise ValueError('双目必须使用同一有限相机—IMU时间偏移')
    if not np.allclose(np.asarray(chain['cam0']['T_imu_cam']),extr['T_I_C0'],atol=1e-6):
        raise ValueError("OpenVINS 与 pose_adapter 的相机外参不同")
    with np.load(calibration_file,allow_pickle=False) as data:
        for i in range(count):
            cam=chain['cam'+str(i)]; k=data['K'+str(i+1)]
            if cam['camera_model'] != 'pinhole' or cam['distortion_model'] != 'radtan':
                raise ValueError('首轮双目入口只接受 pinhole/radtan')
            size = np.asarray(cam['resolution'], dtype=float)
            scale = size / data['image_size']
            if (size.shape != (2,) or not np.isfinite(size).all() or np.any(size < 1)
                    or not np.equal(size, np.floor(size)).all() or not 0 < scale[0] <= 1
                    or not np.isclose(scale[0], scale[1], atol=1e-9, rtol=0)):
                raise ValueError('OpenVINS 与双目尺寸必须等比例缩小')
            if not np.allclose(cam['intrinsics'],np.array([k[0,0],k[1,1],k[0,2],k[1,2]]) * scale[0],atol=1e-3):
                raise ValueError('OpenVINS 与双目内参不同')
            d=np.asarray(data['D'+str(i+1)]).reshape(-1)
            if len(d)!=4 and (len(d)!=5 or abs(d[4])>1e-12): raise ValueError('OpenVINS radtan 四项畸变与 NPZ 模型不一致')
            if not np.allclose(cam['distortion_coeffs'],d[:4],atol=1e-6): raise ValueError('双目畸变不同')
        if count == 2:
            if chain['cam0']['resolution'] != chain['cam1']['resolution']:
                raise ValueError('OpenVINS 左右图尺寸不同')
            relative=np.linalg.inv(np.asarray(chain['cam1']['T_imu_cam'])) @ np.asarray(chain['cam0']['T_imu_cam'])
            if not np.allclose(relative[:3,:3],data['R'],atol=1e-5) or not np.allclose(relative[:3,3],np.asarray(data['T']).reshape(3),atol=1e-5):
                raise ValueError('OpenVINS 与 NPZ 双目基线或旋转不同')
    imu_path=files['vio_config_file'].parent/vio['relative_config_imu']
    if read_yaml(imu_path)['imu0']['rostopic'] != DEFAULTS.imu_topic: raise ValueError("OpenVINS IMU rostopic 不匹配")
    profile=read_yaml(attitude_config_file)
    AttitudeConfig(**profile['controller']); AttitudeMissionConfig(**profile['mission'])
    verified=profile.get('hardware_verified') is True and bool(str(profile.get('evidence','')).strip())
    if live and not verified: raise ValueError("姿态控制参数尚未实测确认；只允许 dry-run")
    files.update(vio_camera_chain=chain_path,vio_imu_chain=imu_path)
    return {'hardware_verified':verified,'extrinsics_source':source,
        'files':{k:{'path':str(p),'sha256':hashlib.sha256(p.read_bytes()).hexdigest()} for k,p in files.items()}}
