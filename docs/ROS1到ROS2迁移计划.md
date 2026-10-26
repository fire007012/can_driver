# can_driver ROS2 实施状态

当前包使用 ament_cmake、rclcpp 与 ros2_control SystemInterface，已完成协议/runtime 解耦、生命周期、会话命令门控、分组权限及首次使能同步。正式整车由父工作区 robot_hardware 的唯一 controller_manager 管理，与 CANopen 后端共享 robot_safety。

旧 CanDriverHW、XmlRpc 配置解析、ROS1 launch/rostest/控制器配置及原迁移计划已移到父工作区 reference/ros1/can_driver/，不参与构建和安装，也不作为 ROS1 回退交付。

ROS2 协议/分组单测与模拟双后端整链已通过；实际 CAN 的身份/方向/量纲、PP/CSP、DM/MT/ECB、限位/恢复、模式与真实停止行为待现场确认。当前软件工作树有配套未提交补丁，发布必须记录父仓库与两个驱动的匹配版本。

完整架构与实机操作见 [项目文档](../../docs/README.md)、[硬件入口](../../robot_hardware/README.md) 和 [现场验收](../../docs/28_CANopen软件交付与实机验收.md)。
