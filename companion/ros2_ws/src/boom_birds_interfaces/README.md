# boom_birds_interfaces：公共接口与数据契约

本包定义任务、规划和控制使用的 ROS 2 消息与服务，保存话题、坐标、时间及无效值契约。本包只生成接口类型，不启动节点或执行控制。

## 内部结构

- `msg/ControlCommand.msg`：带会话、序号和有效期的控制命令。
- `msg/ExecutionStatus.msg`：控制接收结果、飞控观测和执行状态。
- `msg/PlannerRequest.msg`、`msg/PlannerStatus.msg`：规划目标请求和规划状态。
- `srv/Mission.srv`：启动、取消或降落任务的请求与响应。
- `srv/VehicleAction.srv`：控制会话和飞行动作的请求与响应。
- `config/contract.yaml`：话题类型、坐标系、时间语义、单位和无效值等跨模块约定。
- `CMakeLists.txt`：通过 ROS 2 接口生成器生成消息与服务，并安装配置。

## 模块关系

任务/规划侧通过消息或服务表达请求，控制侧消费命令并回传状态；感知及其他节点的数据约定集中在 `config/contract.yaml`。消息字段的准确含义以对应 `.msg`、`.srv` 文件为准，运行实现见 [控制包](../boom_birds_control/README.md)和[编排包](../boom_birds_bringup/README.md)。深度主数据使用米和 NaN 表示无效值，详见契约文件。

构建方式见 [ROS 工作空间](../../README.md)；实现、默认入口与验收状态见 [STATUS](../../../../docs/STATUS.md#实现默认入口部署与验收)。
