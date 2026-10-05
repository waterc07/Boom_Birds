#!/usr/bin/env python3
"""实机配置核验；只读文件，不连接设备。"""
import argparse
import json
from boom_birds_bringup.hardware_profile import validate_profile

p=argparse.ArgumentParser(description=__doc__)
for key in ('fcu_url','calibration_file','extrinsics_file','vio_config_file','attitude_config_file'):
    p.add_argument('--'+key.replace('_','-'),required=True)
p.add_argument('--live',action='store_true',help='额外要求悬停推力等机体参数已有实测记录')
a=p.parse_args()
print(json.dumps(validate_profile(**vars(a)),ensure_ascii=False,indent=2))
