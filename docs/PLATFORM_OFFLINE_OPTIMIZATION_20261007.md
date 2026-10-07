# 平台降落脱机优化记录

日期：2026-10-07。证据根：`/home/waterc/bb_build/platform-opt-20261007/`。运行范围为 WSL、本机 PX4 SIH；不连接真实设备，不写 PX4 参数。

## 代码改动

- 控制历史、SIH 回读历史和图像耗时窗口均限制为 1000 条。控制记录通过有界队列写入分段 JSONL，队满/写入失败有计数。
- 在线图像检测使用单槽待处理输入；新图替换待处理旧图。回放逐帧处理。
- 计算释放改为异步确认、有界重试和超时。取消、epoch 改变或超时后的响应不能确认释放；本周期先处理这些条件，再接受服务结果。
- 平台状态增加生产者 ID、独立状态序号、单调采样时间和飞控 epoch。任务门控、释放服务拒绝过期、倒序、旧会话和错误 token。
- IPPE 的 NaN 候选和无效最终变换拒绝输出有效位姿。ROS 观测输入增加类型/有限值检查。
- 原生 Land 只有新鲜 landed=true 和 armed=false 回读才进入 COMPLETE。
- 实际进程退出确认在失标期间也更新资源状态，不依赖下一张有效 Tag 图像。
- 局部特征原型仅用于脱机像素目标延续，未接入控制。接口和尺寸见 [局部跟踪说明](PLATFORM_FEATURE_TRACKING.md)。

## 软件、图像与耐久

| 项目 | 结果 | 证据 |
| --- | --- | --- |
| control/sensing/bringup/nav 构建 | PASS，4 包 | build-final.console |
| 回归 | PASS，1060 项，零跳过 | regressions-final；132/47/28/853 |
| 图像配置扫描 | 已执行，1152 组合 | scan-delivery；4 布局 × 8 高度 × 3 偏移 × 3 倾斜 × 2 模糊 × 2 遮挡 |
| 标定扰动 | 已执行，9 组合 | scan-delivery/sensitivity.json；焦距 ±3%、相机平移 ±20 mm |
| 引导参数/延迟 | PASS，18 组合，无未经许可下降 | scan-delivery/guidance.json；简化速度模型 |
| 虚拟耐久 | PASS，180000 周期、3600 s 虚拟输入 | soak-delivery；实际控制循环约 36.7 s，非一小时运动闭环 |
| 图像过载 | PASS，实际 60 s、500 Hz 提交 | 30000 输入，29806 处理，194 替换，0 错误；仅 WSL |
| 局部纹理延续 | PASS，60 合成帧 | feature-delivery；完整解码失败时仍延续像素目标，空白帧清空跟踪 |

耐久的控制历史为 1000 条、发送关联历史 256 条，预热后 RSS 范围 308 KiB；1800 条采样日志写入，零丢弃/错误。该测试重复给定合成观测，不证明实际动态控制或 Pi 共载性能。分段日志没有目录总容量上限。

扫描存在 board_not_visible、planar_ambiguity、tag_too_small 和 pnp_failed。记录有效/无效比例用于比较配置，不能把“执行了扫描”当作所有场景识别通过。保留 kp_xy=0.6；简化模型中的更快收敛不足以更改真实控制增益。0.18 s 延迟超过当前 0.15 s 观测门限，撤销下降是预期行为。

## ROS/MAVROS/SIH

运行实际控制节点、Mission.LAND、图像节点、MAVROS 和释放服务。导航为受控 HOLD，VIO/IMU/depth 由两个实际受管的合成输入进程提供；不运行 OpenVINS/EGO。释放回读检查这些进程退出。联合脚本记录实际 MAVROS 输出、uORB、ULog、输入、配置和源码哈希。

| 用例/目录 | 配置起飞高度 | 结果 |
| --- | --- | --- |
| ros-reviewed/origin | 1.5 m | PASS |
| ros-reviewed/return | 1.5 m | PASS，位置导航外出返航 |
| ros-reviewed/service_late | 1.5 m | PASS，服务晚到后释放 |
| ros-reviewed/tag_loss | 1.5 m | PASS，撤销下降后原生 Land |
| ros-reviewed/range_jump | 1.5 m | FAIL，观测中断、交接超时；未进入测距故障阶段 |
| ros-reviewed/range_jump-1m | 1.0 m | PASS，测距突变撤销下降 |
| ros-reviewed/old_session-1m | 1.0 m | FAIL，旧请求被拒绝，但板观测中断使交接超时 |
| ros-attitude-feedback | 1.5 m | PASS，姿态导航转速度降落 |
| ros-reviewed/attitude-return-1m | 1.0 m | FAIL，导航到达等待超时 |
| ros-reviewed/attitude-return-arrival | 1.0 m | PASS，姿态导航外出返航后交接 |

七项通过均确认交接，实际停止两个受管输入进程，最终 landed/disarmed；MAVROS 输出互斥违规为零。原生 Land 失败兜底也核对最终 landed/disarmed。以上不是一次全部通过的矩阵，不给出跨源码/配置的成功率。

姿态导航原到达条件使用三维误差 <0.08 m，稳态高度误差使其不能结束等待。测试入口改为横向 <0.08 m、高度 <0.12 m，并记录到报告；没有修改控制器、落地对准门限或 PX4 参数。姿态导航时 trajectory_setpoint 不刷新，速度估计准入改为检查新鲜 vehicle_local_position；交接确认仍要求新鲜速度目标及控制标志。

每次报告记录自己的源码哈希；较早用例在服务结果处理顺序调整前运行。最终回归和 attitude-return-arrival 验证当前顺序。其它初次失败保存在证据根，未删除。

## 尚未验证

- 0.5～1.5 m 全捕获区间：NOT PASS。部分近正视合成图像有平面位姿歧义，实际相机精度尚未测量。
- 局部特征 → 测距/姿态补偿 → 速度引导 → SIH 闭环：NOT RUN。现有控制仍要求有效板位姿。
- 完整 OpenVINS/EGO 任务与实际 VIO/双目资源释放：NOT RUN。
- 真实标定、光流/测距融合、传感器时延、近地可见范围、落点精度、Pi 实时性和飞行：NOT RUN。

候选打印图为 [tag36h11 ID7](assets/tag36h11-id7-black300mm.svg)：黑框 300 × 300 mm，不计白边；含白边 375 × 375 mm。SVG 栅格化回读 ID7，通过尺寸检查，打印后仍须实测黑框。
