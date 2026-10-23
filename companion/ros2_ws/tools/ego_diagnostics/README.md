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
