# RMUC2027 Gazebo 场地

下载与转换：2026-10-28。适用 Gazebo Harmonic（Gazebo Sim 8）。

## 启动

Windows 双击 `打开Gazebo.cmd`，通过 Ubuntu-24.04 / WSLg 打开场地。
也可以在 WSL 中执行：

```bash
bash /mnt/d/Users/Admin/Desktop/RMUC2027_Gazebo/start_gazebo.sh
```

首次加载需要解析约 400 万个三角面，可能等待数秒。此入口只启动场地，不启动 PX4、ROS 或无人机。
运行模型不需要 Python、MuJoCo 或该上游库。

## 放入已有 world

将 `models/rmuc2027_terrain` 放到你的 Gazebo 模型目录，并将其父目录加入 `GZ_SIM_RESOURCE_PATH`：

```bash
export GZ_SIM_RESOURCE_PATH="/mnt/d/Users/Admin/Desktop/RMUC2027_Gazebo/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
```

在 world 中加入：

```xml
<include>
  <uri>model://rmuc2027_terrain</uri>
  <pose>0 0 0 0 0 0</pose>
</include>
```

单位 m，Z 向上。保留上游世界坐标，没有把最低点平移到 z=0；原点处的地面高度不能当成全场地面高度。已有 world 中的无限地平面可能遮住或填实低处，需检查两者是否重叠。

## 内容与转换

- `models/rmuc2027_terrain/`：独立静态场地；46 个视觉网格、34 个分区碰撞网格。
- `worlds/preview.sdf`：只包含场地和光照的预览 world。
- `gui.config`：Gazebo 默认 GUI 配置的副本，调整了初始观察位置。
- `source/arena_mjcf/`：下载的原始场地、来源记录及作者说明。
- `source/upstream.tar.gz`：原仓库快照；SHA256 和归档中的提交标识见 `conversion.json`。
- `validation/`：本次检查结果和日志。
- `tools/`：本次执行脚本的记录，包含构建机绝对路径；不是通用安装入口。

使用 MuJoCo 3.17.0 编译原 MJCF 后导出世界坐标，避免遗漏网格内部中心和旋转补偿。原视觉三角面按 RGBA 合并为二进制 STL，颜色保存在 SDF material 中；未减面。STL 不保留原网格的平滑法线或 UV，显示可能更有棱角。

碰撞网格取原碰撞 mesh 的凸包，并保留 box；两个波纹路高度场转换为三角网格，补齐侧面与底面。按原几何体中心的 4 m XY 分区合并三角面；没有对整个场地取凸包，隧道和空洞不被整体包住。Gazebo 和 MuJoCo 的求解器、接触参数及高度场三角划分不同，不能视为接触行为完全等价。MuJoCo 的 solref、solimp、priority 和扭转/滚动摩擦未转换，滑动摩擦系数保留为 0.8。

保留上游自行增加的侧面封口和两个四槽停靠台；它们不代表官方场地结构。详见 `source/arena_mjcf/README.md`。

## 本次验证

- 模型与 world 的 `gz sdf -k` 检查通过；world 检查需将 models 加入 SDF_PATH。
- Gazebo Harmonic 无界面加载并运行 20 步，退出码 0。
- 12 个半径 30 mm 球在转换场地上自由下落，运行 1200 步（2.4 s 仿真时间）；12/12 的末态球心高度与原 MJCF 对应位置的地面高度 + 半径相差小于 10 mm。详见 `validation/contact_check.json`。
- Gazebo Ogre2 无界面相机渲染成功，1280×960 图像见 `preview.png`，已查看确认场地整体可见。
- 落球只检查采样位置的接触；不证明全部隧道、薄壁、高速碰撞或无人机任务通过。
- 本包尚未接入 Boom_Birds 的无人机、传感器或任务链。

## 来源与授权记录

来源：https://github.com/CosmosMount/rmuc2027_mjarena

仓库附带 MIT LICENSE，保存在 `source/LICENSE`。但 `source/arena_mjcf/provenance.json` 另行记录了原始 CAD 的权属及授权待确认事项。其记录指出来源文章为 CC BY-NC-SA 4.0，适用于下载图纸及衍生资产的范围仍需确认；不要仅凭仓库 MIT 文件判断整套地形资产的再分发或商用权限。
