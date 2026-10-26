# can_driver 文档导航

主入口是 [`../README.md`](../README.md)。先看包根 README，就能完成启动、生命周期调用和基础调试。

## 配置教程

- [`配置文件字段详解与从零配置指南.md`](配置文件字段详解与从零配置指南.md)
  - `can_driver.yaml` / `ros2_controllers.yaml` 字段说明
- [`电机配置变更指南.md`](电机配置变更指南.md)
  - 新增、修改、停用、删除电机时必须同步的文件和校验规则

## ROS2 迁移

- [`ROS1到ROS2迁移计划.md`](ROS1到ROS2迁移计划.md)
  - ROS2 现状、历史代码位置及实机验收边界

## API 参考

- [`../README.md`](../README.md)
  - 包职责、结构图、快速开始、接口速查
- [`意优行星关节CAN通讯协议.md`](意优行星关节CAN通讯协议.md)
- [`脉塔协议.md`](脉塔协议.md)

## 使用教程

- [`使用指南.md`](使用指南.md)
  - 上机部署、运行步骤、基础排障
- [`单电机控制说明.md`](单电机控制说明.md)
  - 单轴调试链路
- [`独立电机控制指南.md`](独立电机控制指南.md)
  - 不依赖完整系统时的独立 bringup
- [`达妙履带接入说明.md`](达妙履带接入说明.md)
  - 达妙履带接入与底盘链路
- [`CSP模式使用指南.md`](CSP模式使用指南.md)
  - CSP 配置与联调要点
- [`实时控制UI使用说明.md`](实时控制UI使用说明.md)
  - `realtime_motor_ui.py` 的使用方式
- [`ECB网络接入与排障指南.md`](ECB网络接入与排障指南.md)
- [`ECB四电机联调指南.md`](ECB四电机联调指南.md)
- [`MT_MIT模式接入说明.md`](MT_MIT模式接入说明.md)

## 历史参考与验证

父工作区 `reference/ros1/can_driver/` 保存旧 ROS1 接口、rostest、Noetic CI 和原归档说明；本包只维护 ROS2，不再安排 ROS1 回退。整车构建与回归使用父工作区 `tools/validate_ros2.sh`，根 `.github/workflows/ci.yml` 提供 Humble 集成检查。

## 相关包

- [`../../robot_hardware/README.md`](../../robot_hardware/README.md)
  - 统一外观层 / 混合后端正式入口
- [`../../mobility_control/README.md`](../../mobility_control/README.md)
  - 底盘北向控制入口
