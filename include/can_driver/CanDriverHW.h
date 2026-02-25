#ifndef CAN_DRIVER_CAN_DRIVER_HW_H
#define CAN_DRIVER_CAN_DRIVER_HW_H

#include "can_driver/CanProtocol.h"
#include "can_driver/CanType.h"
#include "can_driver/EyouCan.h"
#include "can_driver/MotorID.h"
#include "can_driver/MtCan.h"
#include "can_driver/SocketCanController.h"

#include <can_driver/Init.h>
#include <can_driver/MotorCommand.h>
#include <can_driver/MotorState.h>
#include <can_driver/Recover.h>
#include <can_driver/Shutdown.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

/**
 * @brief hardware_interface::RobotHW 实现，将 MtCan/EyouCan 协议层桥接到 ros_control。
 *
 * 每个 joint 在 YAML 中独立指定 can_device 和 protocol，节点按需创建传输实例：
 *   - 同一 can_device 上的所有 joint 共享一个 SocketCanController
 *   - 同一 can_device 上的 MT/PP joint 分别共享一个 MtCan / EyouCan 实例
 */
class CanDriverHW : public hardware_interface::RobotHW {
public:
    CanDriverHW() = default;
    ~CanDriverHW() override;

    /**
     * @brief 从 rosparam 读取 joints 配置，创建传输/协议实例，注册 ros_control 接口。
     * @param nh   节点 NodeHandle（用于 topic/service）
     * @param pnh  私有 NodeHandle（读取参数）
     */
    bool init(ros::NodeHandle &nh, ros::NodeHandle &pnh);

    /**
     * @brief 从协议缓存拉取最新 pos/vel/eff，写入 ros_control 状态缓存。
     */
    void read(const ros::Time &time, const ros::Duration &period) override;

    /**
     * @brief 从 ros_control 命令缓存读取目标值，下发给协议层。
     */
    void write(const ros::Time &time, const ros::Duration &period) override;

private:
    // -----------------------------------------------------------------------
    // 关节配置
    // -----------------------------------------------------------------------
    struct JointConfig {
        std::string name;
        MotorID     motorId{MotorID::LeftWheel};
        CanType     protocol{CanType::MT};
        std::string canDevice;
        std::string controlMode;   // "velocity" | "position"

        // 单位换算：协议原始整数值 × scale = SI 单位（rad 或 rad/s）
        // 从 YAML 的 position_scale / velocity_scale 读取，默认 1.0（不换算）
        double positionScale{1.0};
        double velocityScale{1.0};

        // ros_control 状态/命令缓存（由 hardware_interface handle 指向这里）
        double pos{0.0};
        double vel{0.0};
        double eff{0.0};
        double posCmd{0.0};
        double velCmd{0.0};
    };

    std::vector<JointConfig> joints_;

    // -----------------------------------------------------------------------
    // 传输 / 协议实例（按 can_device 分组）
    // -----------------------------------------------------------------------
    std::map<std::string, std::shared_ptr<SocketCanController>> transports_;
    std::map<std::string, std::shared_ptr<MtCan>>               mtProtocols_;
    std::map<std::string, std::shared_ptr<EyouCan>>             eyouProtocols_;

    // -----------------------------------------------------------------------
    // ros_control 接口对象
    // -----------------------------------------------------------------------
    hardware_interface::JointStateInterface    jntStateIface_;
    hardware_interface::VelocityJointInterface velIface_;
    hardware_interface::PositionJointInterface posIface_;

    // -----------------------------------------------------------------------
    // ROS 通信
    // -----------------------------------------------------------------------
    ros::ServiceServer initSrv_;
    ros::ServiceServer shutdownSrv_;
    ros::ServiceServer recoverSrv_;
    ros::ServiceServer motorCmdSrv_;

    // 直接命令订阅者（per joint，绕过控制器，测试/调试用）
    std::map<std::string, ros::Subscriber> cmdVelSubs_;
    std::map<std::string, ros::Subscriber> cmdPosSubs_;

    ros::Publisher motorStatesPub_;
    ros::Timer     stateTimer_;

    // -----------------------------------------------------------------------
    // 内部辅助
    // -----------------------------------------------------------------------

    /**
     * @brief 初始化（或重新初始化）指定 CAN 通道。
     *        若 transport 尚不存在则创建；若已存在则先 shutdown 再重新 initialize。
     */
    bool initDevice(const std::string &device, bool loopback = false);

    /**
     * @brief 返回指定通道和协议类型对应的 CanProtocol 指针，不存在时返回 nullptr。
     */
    CanProtocol *getProtocol(const std::string &device, CanType type);

    /**
     * @brief 定时发布 ~/motor_states（10 Hz）。
     */
    void publishMotorStates(const ros::TimerEvent &);

    // -----------------------------------------------------------------------
    // Service 回调
    // -----------------------------------------------------------------------
    bool onInit(can_driver::Init::Request &req, can_driver::Init::Response &res);
    bool onShutdown(can_driver::Shutdown::Request &req, can_driver::Shutdown::Response &res);
    bool onRecover(can_driver::Recover::Request &req, can_driver::Recover::Response &res);
    bool onMotorCommand(can_driver::MotorCommand::Request &req,
                        can_driver::MotorCommand::Response &res);
};

#endif // CAN_DRIVER_CAN_DRIVER_HW_H
