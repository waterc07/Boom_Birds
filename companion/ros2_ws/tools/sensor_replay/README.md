# 传感器离线回放

在仓库根加载开发环境后运行；输入文件保持原位，输出使用新路径：

```bash
source companion/ros2_ws/tools/activate_development_env.sh
python3 -m tools.sensor_replay.mavlink_imu_replay /absolute/path/record.tlog --out /absolute/path/new-report.json
python3 -m tools.sensor_replay.platform_replay \
  --config companion/ros2_ws/src/boom_birds_sim/config/platform_landing_test.yaml \
  --manifest /absolute/path/images.jsonl --test-only --out /absolute/path/new-observations.jsonl
```

`platform_replay` 支持图像清单或 rosbag2，按样本时间戳生成板观测；`render_board` 仅供合成回归。MAVLink 回放只验证接收、时间映射和拒绝路径，不证明设备实时性。

工具和测试不进入运行安装树。平台配置与门控见[控制包](../../src/boom_birds_control/README.md)。

旧原始 UART 节点保存在 `mavlink_imu_node.py`，参数在 `config/mavlink_imu.yaml`，供历史协议回归；正式 ROS 入口 `ros2 run boom_birds_sensing mavlink_imu_node` 仍转发至 MAVROS IMU。不要将旧节点的 connection/baud 参数传给正式入口。
