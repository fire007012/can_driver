# can_driver：ROS1 到 ROS2 迁移计划

## 1. 文档目的

本文档是 `can_driver` 从 ROS1 迁移到 ROS2 的执行基线。目标是在保留 CAN 协议行为、安全策略、生命周期语义和现有配置能力的前提下，交付一个可由 `ros2_control` 管理、可独立启动、可测试和可回滚的 ROS2 包。

ROS1 代码和归档文档仍作为行为回归参考；当前包已经切换为 ROS2 `ament_cmake` 构建。

## 当前实施状态（2026-10-24）

已完成：

- 消息、服务和 Action 已迁移到 `rosidl`，`colcon build` 与现有无硬件 gtest 通过。
- 核心协议/runtime 已从 ROS1 时间和日志 API 解耦，新增 `can_driver::CanDriverSystem` 插件。
- `can_driver_node`、Python 工具、Python launch、控制器 YAML、ros2_control URDF 和 plugin XML 已加入安装规则。
- 插件已经被 `ros2_control_node` 成功发现并完成 `on_init`；真实 SocketCAN 配置的 configure/activate 仍需要目标 CAN 设备和权限。
- Shell 联调脚本已改用 `ros2 service/topic/param` 入口，YAML 关节配置由共享适配函数读取。

当前剩余工作：

- 使用 vcan/UDP loopback 或 fake hardware 完成 `configure -> activate -> read/write` 和控制器运行验证。
- 在真实 CAN 设备上验证 PP/CSP、DM、MT、ECB 报文、限位、恢复和启动位置同步。
- 将历史操作手册中的 ROS1 命令继续迁移或明确标记为归档内容。
- 根据硬件回归结果确认 `CanDriverSystem` 的启动同步、接口单位和 controller manager 生命周期语义。

## 2. 当前基线

### 2.1 架构和构建现状

| 领域 | 当前实现 | 主要位置 | ROS2 迁移影响 |
| --- | --- | --- | --- |
| 构建 | `catkin`、`message_generation`、`catkin_add_gtest` | `package.xml`、`CMakeLists.txt` | 改为 `ament_cmake`、`rosidl_generate_interfaces`、`ament_add_gtest` |
| 节点入口 | `ros::init`、私有 `NodeHandle`、`AsyncSpinner`、`ros::Rate` | `src/can_driver_node.cpp` | 改为 `rclcpp` executor、timer/wall clock 或控制器管理器循环 |
| 硬件接口 | `hardware_interface::RobotHW`，注册位置和速度接口 | `include/can_driver/CanDriverHW.h`、`src/CanDriverHW.cpp` | 改为 `hardware_interface::SystemInterface` 插件 |
| 控制器 | ROS1 `controller_manager`、`joint_state_controller`、轨迹控制器、差速控制器 | `config/ros_controllers*.yaml`、`launch/*.launch` | 改为 ROS2 `controller_manager`、`joint_state_broadcaster`、ROS2 控制器参数格式 |
| 参数 | ROS 参数服务器 + `XmlRpc::XmlRpcValue` 解析 `joints` 列表 | `src/JointConfigParser.cpp`、`config/*.yaml` | 声明 ROS2 参数；重点处理“列表中嵌套字典”不适合直接作为 ROS2 参数值的问题 |
| 话题 | `std_msgs/Float64` 直通命令、`can_driver/MotorState`、`std_msgs/String` 状态 | `src/driver_ros_endpoints.cpp`、`CanDriverHW.cpp` | 改为 ROS2 消息类型和明确 QoS |
| 服务 | 7 个自定义 service，另有 `std_srvs/Trigger` 生命周期入口 | `src/lifecycle_service_gateway.cpp`、`src/motor_maintenance_service.cpp` | 接口定义迁移到 `rosidl`；回调改为 ROS2 service callback |
| Action | `MoveMotor.action`、`HomeMotor.action` 作为接口合同；当前默认入口没有实际 Action server | `action/*.action`、`README.md` | 先保留接口；只有确认上层使用后再实现 `rclcpp_action` server |
| CAN/协议 | SocketCAN、UDP、MT、PP、DM、ECB，以及设备刷新线程和命令门 | `src/*Can.cpp`、`src/*Transport.cpp`、`DeviceManager` | 尽量保留；先清除 ROS 日志和时间类型泄漏，再接 ROS2 |
| 测试 | 大量协议/运行时 gtest，硬件 smoke/rostest 使用 ROS1 API | `tests/`、`CMakeLists.txt` | 纯逻辑测试保留；ROS 集成测试改为 `ament`/`launch_testing` |
| 工具 | Python 使用 `rospy`，Shell 使用 `rosservice`、`rostopic`、`rosparam` | `scripts/` | 改为 `rclpy` 和 `ros2` CLI，保留同样的安全检查 |

### 2.2 对外接口基线

当前默认节点名为 `can_driver_node`，接口位于该节点的私有命名空间：

| 类型 | 当前接口 | 类型/语义 |
| --- | --- | --- |
| 订阅 | `~/motor/<joint>/cmd_velocity` | `std_msgs/Float64`，关节速度单位 |
| 订阅 | `~/motor/<joint>/cmd_position` | `std_msgs/Float64`，关节位置单位 |
| 发布 | `~/motor_states` | `can_driver/MotorState`，含原始反馈、关节 SI 值、有效性和新鲜度 |
| 发布 | `~/lifecycle_state` | `std_msgs/String`，当前 `OperationalCoordinator` 状态 |
| 服务 | `~/init`、`~/enable`、`~/disable`、`~/halt`、`~/resume`、`~/recover`、`~/shutdown` | 初始化、使能、停止、恢复和关闭 |
| 服务 | `~/motor_command`、`~/set_zero`、`~/apply_limits`、`~/set_zero_limit` | 维护和单电机命令 |
| 控制链 | `/joint_states`、轨迹 action、`/cmd_vel` | 由 ROS1 controller manager 和控制器插件提供 |

自定义接口文件为 1 个消息、7 个服务、2 个 Action。字段名、枚举常量、单位和错误消息属于兼容合同，除非在迁移评审中明确批准，不应顺手改名或改单位。

### 2.3 已有的迁移基础

- `DeviceManager`、各协议实现、SocketCAN/UDP transport、`CanDriverRuntime`、`CommandGate`、`OperationalCoordinator` 已拆出相对独立的运行时职责。
- 协议和安全逻辑有较完整的 gtest 覆盖，适合先建立 ROS2 无关的回归门槛。
- `CanDriverHW` 已把 ROS endpoint 和硬件运行时部分拆开，`DriverRosEndpoints`、`LifecycleServiceGateway`、`MotorMaintenanceService` 可作为 ROS2 适配层的切分依据。
- 配置已经覆盖 PP/CSP、DM、MT、ECB、零点持久化、限位、安全停机和掉线恢复，这些行为必须纳入迁移验收。

## 3. 迁移原则和目标架构

1. **先冻结行为，再替换中间件。** 先记录 ROS1 的消息、服务、参数、控制器和生命周期行为，再改 API。
2. **协议层不跟着 ROS2 重写。** CAN 帧格式、反馈转换、命令限幅、故障停机和掉线恢复保持原实现，仅移除其对 `ros::Time` 和 `rosconsole` 的直接依赖。
3. **ROS2 控制链使用 `ros2_control`。** 最终由 `controller_manager` 加载 `can_driver` 的 `SystemInterface` 插件；节点不再手工实例化 ROS1 `RobotHW` 和旧控制器。
4. **自定义运行状态与 ROS2 lifecycle 分层。** ROS2 lifecycle 负责组件配置、激活和清理；`OperationalCoordinator` 继续维护 `Configured/Standby/Armed/Running/Faulted/Recovering` 等业务状态，不能把这些状态简单压扁成 `active/inactive`。
5. **先做接口兼容，再做接口优化。** 第一版 ROS2 保持 topic/service/action 名称和字段不变；QoS、参数结构和控制器类型只在有明确兼容方案时调整。
6. **硬件联调以 vcan/UDP loopback 起步。** 先通过无硬件测试验证状态机、命令门和报文路径，再接真实 CAN 设备。

目标分层如下：

```text
ROS2 launch / rclcpp node / rclcpp_action
        │
        ├── custom topics, services, actions, lifecycle bridge
        │
ros2_control controller_manager
        │  load/start controllers
        ▼
can_driver::CanDriverSystem  (hardware_interface::SystemInterface)
        │  read() / write() / command and state interfaces
        ▼
CanDriverRuntime + OperationalCoordinator + CommandGate
        │
        ▼
DeviceManager → SocketCAN/UDP/ECB transport → MT/PP/DM/ECB protocol
```

## 4. ROS1 到 ROS2 映射表

| ROS1 | ROS2 目标 | 实施要点 |
| --- | --- | --- |
| `catkin` | `ament_cmake` | 重写 `find_package`、安装规则和测试宏；删除 `catkin_package` |
| `roscpp` | `rclcpp` | Node、Publisher、Subscription、Service、Timer、Executor、Clock |
| `rospy` | `rclpy` | UI、MT 工具和临时测试脚本逐个迁移 |
| `ros::Time/Duration` | `rclcpp::Time/Duration` 或 `std::chrono` | 协议/运行时优先使用 `std::chrono`，ROS 适配层再转换时间戳 |
| `ROS_INFO/WARN/ERROR` | `RCLCPP_INFO/WARN/ERROR` | 运行时通过 logger 注入，避免 transport 层依赖节点全局 logger |
| `rosparam + XmlRpcValue` | 声明参数 + YAML 配置读取 | `joints` 结构需要单独设计，见第 6 节 |
| `hardware_interface::RobotHW` | `hardware_interface::SystemInterface` | 实现生命周期回调、接口导出、`read/write` 返回值和 pluginlib 导出 |
| ROS1 `controller_manager` | ROS2 `controller_manager` | 使用 `ros2_control_node` 或等价组合；控制器由 spawner 加载 |
| `joint_state_controller` | `joint_state_broadcaster` | 参数和启动命令不同 |
| `JointTrajectoryController` | `joint_trajectory_controller/JointTrajectoryController` | 明确 `command_interfaces`、`state_interfaces` 和 action 名称 |
| `DiffDriveController` | `diff_drive_controller/DiffDriveController` | 迁移 `left_wheel_names/right_wheel_names`、轮距/半径、QoS 和 odom 配置 |
| `message_generation` | `rosidl_default_generators` | 生成 C++/Python 消息、服务和 Action |
| `actionlib` | `rclcpp_action` | 只迁移真实使用的 Action server/client |
| `roslaunch` XML | `launch` + `launch_ros` Python | 使用 `get_package_share_directory`，替换 `$(find ...)`、`<rosparam>`、`<node>` |
| `rostest` | `launch_testing`、`ament_add_gtest` | ROS graph 和服务测试与纯 C++ 测试分开 |
| `std_msgs/Float64` | `std_msgs/msg/Float64` | 第一版保留字段；长期可评估 `Float64MultiArray` 或 ros2_control 控制器输入 |
| `std_srvs/Trigger` | `std_srvs/srv/Trigger` | 保留 `success/message` 语义 |

## 5. 分阶段执行计划

### 阶段 0：冻结 ROS1 基线和接口合同

**目标：** 在改代码前，建立可比较的 ROS1 行为基线。

**任务：**

- 固定当前 ROS1 分支/标签和构建环境（ROS 发行版、Ubuntu、编译器、Eigen/SDK 版本、SocketCAN 权限）。
- 记录默认启动、`tracks_only.launch`、机械臂配置和直通调试的完整 topic/service/parameter 列表。
- 用 vcan/UDP loopback 跑完协议单测、硬件 smoke、生命周期调用和安全保护脚本，保存日志和报文样本。
- 为每个 joint 记录 `name/motor_id/protocol/can_device/control_mode/scale/direction_sign`，确认 URDF joint 名一致。
- 记录控制周期、反馈查询频率、状态发布周期、QoS 等价要求和控制器更新频率。
- 明确 `MoveMotor/HomeMotor` 目前只是接口合同还是已有外部调用者；没有调用者时第一版只生成接口，不实现未使用的 server。

**交付物和出口条件：**

- `ros1-baseline` 测试记录、接口清单、参数快照和硬件回归日志。
- ROS1 版本可从干净工作区构建，核心 gtest 和无硬件 smoke 全部通过。
- 任何 ROS2 版本的行为差异都能在这份基线中找到对照。

### 阶段 1：创建 ROS2 包骨架和接口包

**目标：** 先让 ROS2 工作区能生成同名消息、服务和 Action。

**任务：**

- 新建 ROS2 分支或目录，重写 `package.xml` 为 format 3，使用 `ament_cmake`。
- 迁移 `msg/MotorState.msg`、`srv/*.srv`、`action/*.action`，使用 `rosidl_default_generators`、`action_msgs`、`std_msgs`、`std_srvs` 声明依赖。
- 在 `CMakeLists.txt` 中加入 `rosidl_generate_interfaces`、目标依赖和 `rosidl_default_runtime` 导出。
- 生成并验证 C++、Python 类型；检查常量、数组、浮点和布尔字段与 ROS1 MD5 对应的语义一致。
- 将接口生成作为独立目标，避免硬件库在接口生成前编译。

**出口条件：**

- `colcon build --packages-select can_driver` 能生成全部接口。
- C++/Python 最小 publisher、service client、action client 可以导入并实例化。
- 接口字段和单位对照表经过评审。

### 阶段 2：清理 ROS 依赖，提取可复用运行时

**目标：** 让 CAN、协议、调度和安全逻辑可以在没有 ROS 节点的情况下编译和测试。

**任务：**

- 将 `CanDriverHwTypes.h`、`CanDriverIoRuntime.h`、`AxisCommandSemantics.h` 等内部时间字段改为 `std::chrono` 或抽象时钟；只在 ROS endpoint 处转换为 `rclcpp::Time`。
- 替换 `SocketCanController.cpp`、`UdpCanTransport.cpp`、各协议实现、`DeviceManager.cpp`、`DeviceRuntime.cpp` 中直接使用的 ROS 日志宏，采用 logger 接口或统一回调。
- 把 `CanDriverIoRuntime` 的“采集反馈、准备命令、限位、故障停机、掉线恢复 hold”保持原逻辑，补上无 ROS 单测。
- 将 `JointConfigParser` 从 `XmlRpc::XmlRpcValue` 解耦：定义与 ROS 无关的 `ParsedJointConfig` 输入，ROS2 参数/YAML 适配层负责填充它。
- 把 URDF 限位读取、参数限位覆盖、命令 saturation 的规则整理成独立函数，保留现有优先级和告警行为。
- 维护 `OperationalCoordinator` 的状态转换和 `DriverOps`，把日志和状态发布从状态机中移出。

**出口条件：**

- 协议、transport、runtime、parser 相关 gtest 在 ROS2 工作区通过。
- 不链接 `roscpp`、`rospy`、`xmlrpcpp` 的核心库可以独立编译。
- 针对故障、命令对齐、限位、恢复 hold、零点持久化的回归结果与阶段 0 一致。

### 阶段 3：实现 ros2_control 硬件插件

**目标：** 用一个标准 `SystemInterface` 替代 ROS1 `RobotHW`。

**建议实现：** 新建 `CanDriverSystem`（名称可在实现时确定），由它持有 `CanDriverRuntime`/`DeviceManager`，不要把 ROS publisher/service 直接塞进硬件插件。

**任务：**

- 实现 `on_init`：读取 hardware info，校验 joint 名、command/state interface 和协议配置；完成参数解析但不启动 CAN 线程。
- 实现 `on_configure`：构造设备组、加载限位和零点持久化、建立 transport/protocol；失败时回到可重试状态。
- 实现 `on_activate`/`on_deactivate`：分别对接 enable/disable、命令 hold 和新命令 latch；激活前完成启动位置同步和故障检查。
- 实现 `on_cleanup`/`on_shutdown`/`on_error`：停止刷新线程、关闭 transport、保持 stop/disable 安全动作，保证重复配置不会遗留线程或 socket。
- 导出每个 joint 的 `position`、`velocity`、必要的 `effort` state interface，以及 `position`/`velocity` command interface；不支持的组合在 `on_init` 明确报错。
- 将 `read()` 对接反馈快照，将 `write()` 对接命令准备和发送；返回 `OK/ERROR`，不能吞掉 CAN 错误。
- 增加 plugin XML 和 `pluginlib_export_plugin_description_file(hardware_interface ...)`。
- 在 URDF 的 `<ros2_control>` 段声明插件、joint 和接口；ROS1 URDF 不直接作为 ROS2 控制器配置使用。

**生命周期决策：**

- ROS2 lifecycle `unconfigured/inactive/active/finalized` 只表示硬件组件可配置、可运行和已清理。
- `OperationalCoordinator` 的 `Configured/Standby/Armed/Running/Faulted/Recovering` 继续作为业务状态，并通过 `~/lifecycle_state` 或诊断接口发布。
- `~/init`、`~/recover`、`~/halt` 等兼容服务继续调用同一个 coordinator，不在 callback 中复制状态转换。
- 在阶段 3 结束时评审一次 `init/enable/resume` 与 `configure/activate` 的映射；如果上层只需要标准 ros2_control 生命周期，可在阶段 5 再逐步减少兼容服务。

**出口条件：**

- `ros2_control_node` 能加载插件并通过 `on_configure`/`on_activate`。
- 无硬件模式下 `read/write` 周期、接口数量和单位与 ROS1 一致。
- 重复 configure、activate、deactivate、cleanup 不泄漏线程、文件描述符或命令。

### 阶段 4：迁移 rclcpp endpoint 和业务服务

**目标：** 恢复直通命令、状态发布、生命周期服务和维护服务。

**任务：**

- 用 `rclcpp::Node` 或 `rclcpp_lifecycle::LifecycleNode` 重写 `DriverRosEndpoints`，发布 `MotorState` 和 lifecycle state。
- 每个 joint 创建 `std_msgs::msg::Float64` subscription；保留命名规则 `motor/<joint>/cmd_velocity` 和 `motor/<joint>/cmd_position`。
- 用 timer 发布电机状态和生命周期状态；状态发布 timer 不得阻塞控制循环。
- 用 ROS2 service callback 重写 `LifecycleServiceGateway` 和 `MotorMaintenanceService`，所有业务操作仍调用 `CanDriverHW`/runtime/coordinator 的单一入口。
- 明确 callback group：控制服务、状态 timer 和低频维护服务使用互不阻塞的 callback group；CAN 发送不在 executor callback 中等待长时间响应。
- 设置 QoS：生命周期状态使用 `transient_local + reliable` 近似 ROS1 latch；电机状态按现有订阅者需求选择 `reliable`；直通命令使用小队列并记录丢包策略。
- 将参数声明、日志、clock 和节点关闭顺序集中在一个 ROS2 adapter 中。

**出口条件：**

- ROS2 CLI 可以完成 init/enable/disable/halt/resume/recover/shutdown 和四类维护服务。
- 直通速度/位置命令经过同样的命令门、限位和故障保护。
- 新订阅者可以拿到最近一次 lifecycle 状态，状态消息字段和单位与 ROS1 对齐。

### 阶段 5：迁移参数、控制器和 launch

**目标：** 用 ROS2 配置启动机械臂和履带两条主要链路。

#### 5.1 参数结构

当前 `config/can_driver*.yaml` 的 `joints` 是“数组中的字典”，而 ROS2 参数服务器不适合直接表达这类任意嵌套结构。选择并固定一种格式后再改 parser，建议使用下面的 ROS2 原生结构：

```yaml
can_driver_node:
  ros__parameters:
    control_frequency: 100.0
    joints: [elbow_pitch_joint, left_front_wheel_joint]
    joints.elbow_pitch_joint.motor_id: 19
    joints.elbow_pitch_joint.protocol: PP
    joints.elbow_pitch_joint.can_device: canable1
    joints.elbow_pitch_joint.control_mode: csp
    joints.left_front_wheel_joint.motor_id: 7
    joints.left_front_wheel_joint.protocol: DM
    joints.left_front_wheel_joint.can_device: canable1
    joints.left_front_wheel_joint.control_mode: velocity
```

若需要继续保留现有数组写法，则使用显式的 `yaml-cpp` 配置文件，由节点在 `config_file` 参数中读取，并把 ROS 参数只用于标量覆盖。两种方式不能混用；应在阶段 0 的配置快照上做自动转换器并逐项比较解析结果。

#### 5.2 控制器

- `joint_state_controller` 改为 `joint_state_broadcaster`。
- 位置轨迹控制器改为 ROS2 `joint_trajectory_controller`，声明 `position` command interface 和对应 state interfaces。
- 履带控制器改为 ROS2 `diff_drive_controller`，迁移轮关节列表、轮距、半径、速度限制、odom/TF 和 `cmd_vel` 的 QoS。
- 在 `controller_manager.ros__parameters` 中声明 `update_rate`，控制器类型放到每个 controller 的 `type` 参数。
- 统一硬件 `control_frequency` 与 controller manager `update_rate`；启动时检测不一致并告警。

#### 5.3 launch 和安装

- 将 `launch/can_driver.launch`、`launch/tracks_only.launch` 改为 `launch/can_driver.launch.py`、`launch/tracks_only.launch.py`。
- 使用 `DeclareLaunchArgument`、`ParameterFile`、`Node`、`ExecuteProcess` 或 ROS2 controller spawner；替换 `$(find can_driver)`、`<rosparam>`、`<node if=...>`。
- 将 ECB `ACTUATOR_TOOL_PATH` 改为 launch 的环境变量设置，并验证 SDK 动态库 `RPATH/LD_LIBRARY_PATH`。
- 安装 `launch`、`config`、plugin XML、URDF 片段、Python entry points 和测试资源；当前 ROS1 CMake 中部分头文件/库的安装规则是注释状态，迁移时一并补齐。
- 保留 `use_controllers`、`config_file`、`controllers_file`、`start_controllers` 等启动参数的语义，控制器加载命令改为 ROS2 方式。

**出口条件：**

- 默认启动可只启动硬件和 endpoint，履带启动可加载差速控制器，机械臂启动可加载轨迹控制器。
- `ros2 control list_hardware_components`、`ros2 control list_controllers` 显示预期状态。
- 参数 dump 后重新启动，joint 映射和限位结果不变。

### 阶段 6：迁移 Python 工具、Shell 工具和文档

**Python：**

- `scripts/mt_motor_interface.py`、`scripts/realtime_motor_ui.py` 改为 `rclpy`，消息/服务导入改为 `can_driver.msg`、`can_driver.srv`。
- `rospy.get_param` 改为节点参数声明/读取；`wait_for_service`、publisher/subscriber、timer、日志和 shutdown 分别迁移。
- GUI 线程与 ROS executor 分离，确保界面刷新不会阻塞服务或状态订阅；启动时检查服务和参数存在性。

**Shell 和操作手册：**

- `rosservice`、`rostopic`、`rosparam`、`roslaunch`、`catkin_make`、`devel/setup.bash` 全部替换为 ROS2 CLI、Python launch、`colcon build`、`install/setup.bash`。
- 脚本中使用 `ros2 service call`/`ros2 topic pub` 时固定 YAML 输入格式，并处理 ROS2 CLI 的异步输出。
- 更新 `docs/使用指南.md`、`docs/单电机控制说明.md`、`docs/达妙履带接入说明.md`、`docs/实时控制UI使用说明.md`，保留 ROS1 文档到归档并注明适用分支。
- 更新 `docs/README.md`，将本文档列入迁移/架构入口。

**出口条件：** Python UI 能接收 `MotorState` 并调用维护服务；安全测试脚本能够在 vcan/UDP loopback 下完成启停、故障和限位验证。

### 阶段 7：测试、实机联调和性能验收

**测试分层：**

1. **纯 C++ 单测：** `SafeCommand`、协议编解码、transport、刷新调度、设备 runtime、配置 parser、coordinator、命令门。使用 `ament_add_gtest`，不启动 ROS graph。
2. **ROS2 节点集成测试：** 用 `launch_testing` 验证参数加载、topic/service、QoS、生命周期状态和重复启动/关闭。
3. **ros2_control 测试：** 使用 fake/mock hardware 验证接口导出、控制器加载、位置/速度命令路由、控制器停止时的命令 hold。
4. **vcan/UDP loopback：** 验证 MT/PP/DM 报文、反馈新鲜度、掉线恢复、启动位置同步、零点持久化和故障停机。
5. **真实硬件：** 先单电机，再四电机 ECB，再 DM 履带，再 PP/CSP 机械臂；每一步只启用已验证的协议和 joint 配置。

**关键验收项：**

- 100 Hz 控制周期抖动、CAN 总线负载和 ROS1 基线相比不恶化；测量控制循环实际周期和 write 超时。
- 电机未使能、反馈故障、掉线恢复、模式切换未对齐、超限位置和非法参数均进入原有安全路径。
- `MotorState` 的 `*_valid`、`feedback_fresh`、`joint_position`、`joint_velocity` 语义一致。
- `init → enable → resume → halt → disable → shutdown` 以及 `fault → recover` 的状态迁移与阶段 0 结果一致。
- 关闭节点后无残留刷新线程、socket、controller 或未处理的 executor callback。
- 机械臂轨迹和履带 `cmd_vel` 在明确的限速和急停测试中通过。

### 阶段 8：切换、并行运行和回滚

**切换步骤：**

1. 发布 ROS2 包和配置转换工具，先在无硬件环境验证。
2. 在同一 CAN 设备上只允许 ROS1 或 ROS2 一个驱动实例持有总线；并行运行只用于不同 vcan/仿真设备。
3. 先切换监控和状态读取，再切换单电机命令，最后切换 controller manager、轨迹和底盘闭环。
4. 保留 ROS1 分支、旧配置和已验证二进制，至少覆盖一个完整现场回归周期。

**回滚触发条件：** 报文内容异常、控制周期超时、状态机错误迁移、故障停机失效、限位失效、线程/socket 泄漏或任一真实硬件验收项失败。

**回滚动作：** 停止 ROS2 controller 和 node，确认电机进入 stop/disable 安全状态，释放 CAN 设备，再启动已验证的 ROS1 版本；保存 ROS2 日志、参数 dump 和 CAN 抓包后再修复。

## 6. 需要提前做出的设计决策

以下决策会影响实现量，建议在阶段 0 结束前固定：

| 决策 | 推荐结论 | 原因 |
| --- | --- | --- |
| 是否同一源码同时支持 ROS1/ROS2 | 不在同一包内双构建；使用分支或迁移目录 | `catkin`/`ament`、接口生成和控制器 API 差异大，双构建会放大维护和测试成本 |
| `joints` 配置格式 | ROS2 原生参数扁平化，或独立 YAML 文件二选一 | ROS2 参数不等价支持 ROS1 XmlRpc 的任意数组字典 |
| 硬件插件边界 | `SystemInterface` 只负责硬件接口和 runtime；endpoint 单独由 node 持有 | 避免 controller manager 生命周期和 rclcpp executor 互相持有，便于无 ROS 测试 |
| lifecycle 权威 | ROS2 lifecycle 管组件生命周期，`OperationalCoordinator` 管业务状态 | 现有状态多于 ROS2 lifecycle，直接合并会丢失安全语义 |
| Action 迁移 | 先生成接口，确认调用者后实现 server | 当前默认入口没有 Action server，避免迁移未使用功能引入新状态机 |
| QoS | lifecycle 状态 transient local/reliable；命令小队列；反馈按消费者验证 | ROS1 latch 和队列语义不会自动映射到 ROS2 QoS |
| URDF 限位 | 保留 URDF + 参数覆盖优先级，并在 ros2_control 配置声明接口 | 现有命令限幅和校准流程依赖这套规则 |
| ECB SDK | 继续作为可选后端，ROS2 包安装时验证动态库路径 | SDK 是运行时依赖，不应因为 ROS API 迁移改变协议行为 |

## 7. 风险和应对

| 风险 | 影响 | 应对 |
| --- | --- | --- |
| ROS2 参数无法直接承载当前 `joints` 数组 | 启动后 joint 映射为空或类型错误 | 先写转换器和 parser 单测，禁止直接把旧 YAML 当 ROS2 参数加载 |
| `RobotHW` 到 `SystemInterface` 接口模型变化 | 控制器加载成功但命令未下发 | 用 fake hardware 验证每个 interface 的名称、方向、单位和生命周期，再接 CAN |
| ROS2 controller 参数和 plugin 名称变化 | 控制器无法加载或接口不匹配 | 逐个迁移 joint state、trajectory、diff drive，并用 `ros2 control list_controllers` 做出口检查 |
| 业务生命周期多于 ROS2 lifecycle | 状态被错误压缩，可能绕过安全门 | 保留 `OperationalCoordinator` 单一状态入口，所有服务和 lifecycle callback 都调用它 |
| ROS2 executor 与 CAN/维护服务阻塞 | 控制周期抖动或服务死锁 | callback group 分离，长操作放 runtime worker，控制循环不等待服务 callback |
| QoS 不匹配 | 看不到状态或收不到命令 | 为每个 topic 写 QoS 集成测试，并在 CLI 中检查 offered/requested QoS |
| Python UI 和 Shell 工具仍依赖 ROS1 CLI | 现场工具不可用 | 工具迁移列为正式阶段，不以节点能启动作为完成条件 |
| ECB 专有 SDK/路径差异 | 仅 ECB 配置在现场失败 | 在无 ECB 主机上做可选依赖启动测试，在 ECB 主机做动态库和网络发现回归 |
| ROS1/ROS2 同时打开真实 CAN | 两个主站争用总线 | 运行手册明确单实例锁和切换顺序，实机只允许一个驱动进程 |

## 8. 建议的提交拆分

每个提交保持可编译、可回滚，建议顺序如下：

1. `docs: freeze ros1 interface and baseline`
2. `build: add ros2 ament package and rosidl interfaces`
3. `refactor: decouple runtime from ros time and logging`
4. `ros2_control: add can driver system interface plugin`
5. `ros2: add rclcpp endpoints and lifecycle bridge`
6. `config: convert parameters controllers and launch files`
7. `tools: migrate python and shell utilities to ros2`
8. `test: add launch testing ros2_control and loopback coverage`
9. `docs: update ros2 operation and rollback guide`

每个提交的最低检查：

```bash
colcon build --packages-select can_driver --symlink-install
colcon test --packages-select can_driver --event-handlers console_direct+
colcon test-result --verbose
```

有真实硬件的提交还应附上 CAN 抓包、控制周期统计、状态机日志和使用的配置文件摘要。

## 9. 完成定义

迁移在以下条件全部满足后才算完成：

- ROS2 包能从干净工作区构建、安装和启动，接口生成、pluginlib 导出和配置安装完整。
- 默认、机械臂、履带三种配置都能启动；控制器能被加载并进入预期状态。
- 协议/安全/生命周期/限位/零点/恢复行为通过 ROS2 单测和 loopback 回归。
- 直通 topic、MotorState、维护服务、controller manager、轨迹和 `cmd_vel` 的接口合同已验证。
- Python UI、MT 工具和现场 Shell 检查脚本不再依赖 ROS1 命令。
- 至少完成一次真实硬件单电机回归和一次多电机回归，并有可执行回滚步骤。
- 所有用户文档明确 ROS2 启动命令、参数格式、控制器名称、QoS 约束和故障排查方法。

