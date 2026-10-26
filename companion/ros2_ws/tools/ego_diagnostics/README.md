# EGO 失败几何记录

仅用于开发分析，不进入运行安装树。主工程当前状态见 [STATUS](../../../../docs/STATUS.md)。

EGO 原生入口增加 `failure_diagnostics:=true` 可记录 `BB_OCCUPANCY`；默认 false。每秒最多一条，包含失败位置、地图边界、原始/膨胀占用、未知标记、当前深度与最近融合深度时间、当前相机位姿及最多 16 个贡献体素。超过 16 个保留总数和截断标记。无需开启 DEBUG。此开关须使用本次重建的 EGO，不能只把新 launch 覆盖到旧二进制。

```bash
# 只在已选定的现场启动命令中追加参数；本工具不启动设备。
python3 companion/ros2_ws/tools/ego_diagnostics/analyze_occupancy_log.py \
  /path/to/ego.log --output /path/to/occupancy.json
```

分类区分原始占用、邻格膨胀、虚拟顶棚、无当前贡献体素的膨胀、地图外和未知空间。没有记录、记录不完整或 JSON 非有限值时退出 2，不把旧日志的零记录解释为地图正常。

贡献体素只解释当前占用几何。当前相机位姿和最近融合帧不能证明某个历史体素来自哪个深度像素；真实占用来源仍需同步深度、位姿和占用快照。此工具不清地图，不修改碰撞或定位失效判据。

```bash
python3 -m pytest companion/ros2_ws/tools/ego_diagnostics/test -q
```

## VIO 原始失效窗

`failure_window.py` 只读已收尾的单文件 SQLite bag，核对操作员 STILL 标记前 20 s、后 10 s 的原始双目/IMU 覆盖，窗口内接收间隔不得超过 0.25 s，并要求窗口内有 state pose/odom 和新相机诊断字段。该间隔仅用于证据窗口检查，不替换运行期 1 s 保护。缺标记、缺话题或缺诊断时退出 2；`diagnostic_window_ready` 与采集完成、文件完整、导航验收分别记录。

```bash
source /opt/ros/jazzy/setup.bash
python3 companion/ros2_ws/tools/ego_diagnostics/failure_window.py /path/to/init-bag \
  --events /path/to/observer/events.json --output /path/to/failure-window.json
```

窗口按 bag 接收时间与操作员标记核对，不证明曝光时间或实际放回时刻。原始 bag 应在初始化前启动；目标未生成也应保留。下一轮候选与构建状态见 [STATUS](../../../../docs/STATUS.md)。

## bag 接收时钟与 Header 对照

`bag_header_timing.py` 只读 SQLite 的 Image/Imu CDR1 Header 前缀，不依赖 ROS，不启动设备。报告原始记录顺序下的两种间隔、倒序/重复计数及窗口覆盖；不修改 `failure_window.py` 的结果或 0.25 s 判据。暂不支持 CDR2，话题类型/序列化不符时拒绝读取。

```bash
python3 companion/ros2_ws/tools/ego_diagnostics/bag_header_timing.py /path/to/init-bag/init-bag_0.db3 \
  --events /path/to/observer/events.json --output /path/to/header-timing.json
```

Header 是发布者时间，不是独立曝光测量。接收时间减 Header 混有时钟偏差，不能直接当传输延迟。bag Header 有缺口而独立观察器持有中间帧时，只能证明 bag 未保留全部已发布帧；多个订阅者同时缺帧仍不能单凭 Header 定位相机、发布或调度根因。此工具不输出导航或输入连续性 PASS。
