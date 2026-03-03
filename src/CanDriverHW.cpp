#include "can_driver/CanDriverHW.h"

#include <algorithm>
#include <ros/ros.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

// ---------------------------------------------------------------------------
// 析构
// ---------------------------------------------------------------------------
CanDriverHW::~CanDriverHW()
{
    active_.store(false, std::memory_order_release);
    stateTimer_.stop();

    for (auto &kv : cmdVelSubs_) {
        kv.second.shutdown();
    }
    for (auto &kv : cmdPosSubs_) {
        kv.second.shutdown();
    }
    initSrv_.shutdown();
    shutdownSrv_.shutdown();
    recoverSrv_.shutdown();
    motorCmdSrv_.shutdown();

    std::unique_lock<std::shared_mutex> lock(protocolMutex_);

    // 协议实例析构时会停止刷新线程，transport 析构时调用 shutdown()
    // 清空顺序：先协议层，再传输层
    mtProtocols_.clear();
    eyouProtocols_.clear();
    transports_.clear();
    deviceCmdMutexes_.clear();
}

// ---------------------------------------------------------------------------
// init
// ---------------------------------------------------------------------------
bool CanDriverHW::init(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
    (void)nh;
    active_.store(false, std::memory_order_release);

    {
        std::unique_lock<std::shared_mutex> lock(protocolMutex_);
        joints_.clear();
        jointIndexByName_.clear();
        mtProtocols_.clear();
        eyouProtocols_.clear();
        transports_.clear();
        deviceCmdMutexes_.clear();
    }

    // 读取 joints 列表
    XmlRpc::XmlRpcValue jointList;
    if (!pnh.getParam("joints", jointList)) {
        ROS_ERROR("[CanDriverHW] Parameter 'joints' not found under %s",
                  pnh.getNamespace().c_str());
        return false;
    }
    if (jointList.getType() != XmlRpc::XmlRpcValue::TypeArray || jointList.size() == 0) {
        ROS_ERROR("[CanDriverHW] 'joints' must be a non-empty list.");
        return false;
    }

    if (!pnh.getParam("direct_cmd_timeout_sec", directCmdTimeoutSec_)) {
        directCmdTimeoutSec_ = 0.5;
    }
    if (!std::isfinite(directCmdTimeoutSec_) || directCmdTimeoutSec_ < 0.0) {
        ROS_WARN("[CanDriverHW] Invalid direct_cmd_timeout_sec=%.9g, fallback to 0.5s.",
                 directCmdTimeoutSec_);
        directCmdTimeoutSec_ = 0.5;
    }

    // 遍历 joint 配置
    for (int i = 0; i < jointList.size(); ++i) {
        XmlRpc::XmlRpcValue &jv = jointList[i];

        JointConfig jc;

        // --- 必填字段 ---
        if (!jv.hasMember("name") || !jv.hasMember("motor_id") ||
            !jv.hasMember("protocol") || !jv.hasMember("can_device") ||
            !jv.hasMember("control_mode")) {
            ROS_ERROR("[CanDriverHW] Joint [%d] missing required field "
                      "(name/motor_id/protocol/can_device/control_mode).", i);
            return false;
        }

        jc.name        = static_cast<std::string>(jv["name"]);
        jc.canDevice   = static_cast<std::string>(jv["can_device"]);
        jc.controlMode = static_cast<std::string>(jv["control_mode"]);

        // --- 可选换算系数（默认 1.0，不换算）---
        if (jv.hasMember("position_scale"))
            jc.positionScale = static_cast<double>(jv["position_scale"]);
        if (jv.hasMember("velocity_scale"))
            jc.velocityScale = static_cast<double>(jv["velocity_scale"]);
        if (!std::isfinite(jc.positionScale) || jc.positionScale <= 0.0) {
            ROS_ERROR("[CanDriverHW] Joint '%s': invalid position_scale=%.9g (must be > 0).",
                      jc.name.c_str(), jc.positionScale);
            return false;
        }
        if (!std::isfinite(jc.velocityScale) || jc.velocityScale <= 0.0) {
            ROS_ERROR("[CanDriverHW] Joint '%s': invalid velocity_scale=%.9g (must be > 0).",
                      jc.name.c_str(), jc.velocityScale);
            return false;
        }

        // motor_id: YAML 中用十六进制字符串（"0x141"）或整数均可
        int rawId = 0;
        const auto motorIdType = jv["motor_id"].getType();
        if (motorIdType == XmlRpc::XmlRpcValue::TypeInt) {
            rawId = static_cast<int>(jv["motor_id"]);
        } else if (motorIdType == XmlRpc::XmlRpcValue::TypeString) {
            // 字符串形式 "0x141"
            try {
                rawId = static_cast<int>(
                    std::stoul(static_cast<std::string>(jv["motor_id"]), nullptr, 0));
            } catch (const std::exception &e) {
                ROS_ERROR("[CanDriverHW] Joint '%s': invalid motor_id '%s' (%s).",
                          jc.name.c_str(),
                          static_cast<std::string>(jv["motor_id"]).c_str(),
                          e.what());
                return false;
            }
        } else {
            ROS_ERROR("[CanDriverHW] Joint '%s': motor_id must be int or string.",
                      jc.name.c_str());
            return false;
        }
        if (rawId < 0 || rawId > static_cast<int>(std::numeric_limits<uint16_t>::max())) {
            ROS_ERROR("[CanDriverHW] Joint '%s': motor_id out of range [0, 65535]: %d.",
                      jc.name.c_str(), rawId);
            return false;
        }
        jc.motorId = static_cast<MotorID>(static_cast<uint16_t>(rawId));

        // protocol: "MT" or "PP"
        std::string protoStr = static_cast<std::string>(jv["protocol"]);
        if (protoStr == "MT") {
            jc.protocol = CanType::MT;
        } else if (protoStr == "PP") {
            jc.protocol = CanType::PP;
        } else {
            ROS_ERROR("[CanDriverHW] Joint '%s': unknown protocol '%s' (use MT or PP).",
                      jc.name.c_str(), protoStr.c_str());
            return false;
        }

        joints_.push_back(jc);
        jointIndexByName_[jc.name] = joints_.size() - 1;

        // --- 按 can_device 按需创建传输和协议实例 ---
        if (transports_.find(jc.canDevice) == transports_.end()) {
            auto transport = std::make_shared<SocketCanController>();
            if (!transport->initialize(jc.canDevice)) {
                ROS_ERROR("[CanDriverHW] Failed to initialize CAN device '%s'.",
                          jc.canDevice.c_str());
                return false;
            }
            transports_[jc.canDevice] = transport;
            deviceCmdMutexes_[jc.canDevice] = std::make_shared<std::mutex>();
            ROS_INFO("[CanDriverHW] Opened CAN device '%s'.", jc.canDevice.c_str());
        }

        auto transport = transports_[jc.canDevice];
        if (jc.protocol == CanType::MT &&
            mtProtocols_.find(jc.canDevice) == mtProtocols_.end()) {
            mtProtocols_[jc.canDevice] = std::make_shared<MtCan>(transport);
        }
        if (jc.protocol == CanType::PP &&
            eyouProtocols_.find(jc.canDevice) == eyouProtocols_.end()) {
            eyouProtocols_[jc.canDevice] = std::make_shared<EyouCan>(transport);
        }
    }

    // --- 注册 ros_control 接口 ---
    for (auto &jc : joints_) {
        hardware_interface::JointStateHandle stateHandle(
            jc.name, &jc.pos, &jc.vel, &jc.eff);
        jntStateIface_.registerHandle(stateHandle);

        if (jc.controlMode == "velocity") {
            hardware_interface::JointHandle velHandle(stateHandle, &jc.velCmd);
            velIface_.registerHandle(velHandle);
        } else {
            hardware_interface::JointHandle posHandle(stateHandle, &jc.posCmd);
            posIface_.registerHandle(posHandle);
        }
    }

    registerInterface(&jntStateIface_);
    registerInterface(&velIface_);
    registerInterface(&posIface_);

    // --- 加载关节限位（从 URDF 和 rosparam）---
    urdf::Model urdf;
    bool urdfLoaded = urdf.initParam("robot_description");
    if (!urdfLoaded) {
        ROS_WARN("[CanDriverHW] Failed to load URDF from 'robot_description'. "
                 "Joint limits will not be enforced.");
    }

    for (auto &jc : joints_) {
        joint_limits_interface::JointLimits limits;
        joint_limits_interface::SoftJointLimits soft_limits;
        bool hasLimits = false;

        // 1. 从 URDF 读取硬限位
        if (urdfLoaded) {
            urdf::JointConstSharedPtr urdfJoint = urdf.getJoint(jc.name);
            if (urdfJoint) {
                hasLimits = joint_limits_interface::getJointLimits(urdfJoint, limits);
                if (hasLimits) {
                    ROS_INFO("[CanDriverHW] Joint '%s': URDF limits [%.3f, %.3f] rad, "
                             "max_vel=%.3f rad/s, max_effort=%.3f",
                             jc.name.c_str(), limits.min_position, limits.max_position,
                             limits.max_velocity, limits.max_effort);
                }
            } else {
                ROS_WARN("[CanDriverHW] Joint '%s' not found in URDF.", jc.name.c_str());
            }
        }

        // 2. 从 rosparam 读取软限位（可选，会覆盖 URDF）
        if (joint_limits_interface::getJointLimits(jc.name, pnh, limits)) {
            hasLimits = true;
            ROS_INFO("[CanDriverHW] Joint '%s': rosparam overrides limits.", jc.name.c_str());
        }
        joint_limits_interface::getSoftJointLimits(jc.name, pnh, soft_limits);

        // 3. 注册限位接口
        if (hasLimits) {
            jc.limits = limits;
            jc.hasLimits = true;
            if (jc.controlMode == "velocity") {
                joint_limits_interface::VelocityJointSaturationHandle handle(
                    velIface_.getHandle(jc.name), limits);
                velLimitsIface_.registerHandle(handle);
            } else {
                joint_limits_interface::PositionJointSaturationHandle handle(
                    posIface_.getHandle(jc.name), limits);
                posLimitsIface_.registerHandle(handle);
            }
        } else {
            ROS_WARN("[CanDriverHW] Joint '%s': no limits found, commands will not be clamped.",
                     jc.name.c_str());
        }
    }

    // --- 启动电机状态刷新线程 ---
    // 按 (device, protocol) 分组收集 motor ID
    std::map<std::string, std::vector<MotorID>> mtIds, ppIds;
    for (const auto &jc : joints_) {
        if (jc.protocol == CanType::MT)
            mtIds[jc.canDevice].push_back(jc.motorId);
        else
            ppIds[jc.canDevice].push_back(jc.motorId);
    }
    for (auto &kv : mtIds) {
        mtProtocols_[kv.first]->initializeMotorRefresh(kv.second);
    }
    for (auto &kv : ppIds) {
        eyouProtocols_[kv.first]->initializeMotorRefresh(kv.second);
    }

    // --- ROS Services（私有命名空间：~/...） ---
    initSrv_     = pnh.advertiseService("init",          &CanDriverHW::onInit,         this);
    shutdownSrv_ = pnh.advertiseService("shutdown",      &CanDriverHW::onShutdown,     this);
    recoverSrv_  = pnh.advertiseService("recover",       &CanDriverHW::onRecover,      this);
    motorCmdSrv_ = pnh.advertiseService("motor_command", &CanDriverHW::onMotorCommand, this);

    // --- 直接命令 subscribers（per joint） ---
    for (auto &jc : joints_) {
        const std::string velTopic = "motor/" + jc.name + "/cmd_velocity";
        const std::string posTopic = "motor/" + jc.name + "/cmd_position";

        const std::size_t idx = jointIndexByName_[jc.name];
        cmdVelSubs_[jc.name] = pnh.subscribe<std_msgs::Float64>(
            velTopic, 1,
            [this, idx](const std_msgs::Float64::ConstPtr &msg) {
                if (!active_.load(std::memory_order_acquire)) {
                    return;
                }
                std::lock_guard<std::mutex> lock(jointStateMutex_);
                joints_[idx].directVelCmd = msg->data;
                joints_[idx].hasDirectVelCmd = true;
                joints_[idx].lastDirectVelTime = ros::Time::now();
            });

        cmdPosSubs_[jc.name] = pnh.subscribe<std_msgs::Float64>(
            posTopic, 1,
            [this, idx](const std_msgs::Float64::ConstPtr &msg) {
                if (!active_.load(std::memory_order_acquire)) {
                    return;
                }
                std::lock_guard<std::mutex> lock(jointStateMutex_);
                joints_[idx].directPosCmd = msg->data;
                joints_[idx].hasDirectPosCmd = true;
                joints_[idx].lastDirectPosTime = ros::Time::now();
            });
    }

    // --- 电机状态发布定时器（10 Hz） ---
    motorStatesPub_ = pnh.advertise<can_driver::MotorState>("motor_states", 10);
    stateTimer_ = pnh.createTimer(ros::Duration(0.1),
                                 &CanDriverHW::publishMotorStates, this);

    ROS_INFO("[CanDriverHW] Initialized with %zu joints on %zu CAN device(s).",
             joints_.size(), transports_.size());
    active_.store(true, std::memory_order_release);
    return true;
}

// ---------------------------------------------------------------------------
// read
// ---------------------------------------------------------------------------
void CanDriverHW::read(const ros::Time & /*time*/, const ros::Duration & /*period*/)
{
    if (!active_.load(std::memory_order_acquire)) {
        return;
    }

    struct JointSnapshot {
        double pos{0.0};
        double vel{0.0};
        double eff{0.0};
        bool   valid{false};
    };
    std::vector<JointSnapshot> snapshots(joints_.size());

    std::map<std::pair<std::string, CanType>, std::vector<std::size_t>> groups;
    for (std::size_t i = 0; i < joints_.size(); ++i) {
        groups[{joints_[i].canDevice, joints_[i].protocol}].push_back(i);
    }

    for (const auto &group : groups) {
        const std::string &device = group.first.first;
        const CanType protocol = group.first.second;
        auto proto = getProtocol(device, protocol);
        auto devMutex = getDeviceMutex(device);
        if (!proto || !devMutex) {
            continue;
        }

        std::lock_guard<std::mutex> devLock(*devMutex);
        for (const std::size_t i : group.second) {
            const auto &jc = joints_[i];
            snapshots[i].pos = static_cast<double>(proto->getPosition(jc.motorId)) * jc.positionScale;
            snapshots[i].vel = static_cast<double>(proto->getVelocity(jc.motorId)) * jc.velocityScale;
            snapshots[i].eff = static_cast<double>(proto->getCurrent(jc.motorId));
            snapshots[i].valid = true;
        }
    }

    std::lock_guard<std::mutex> lock(jointStateMutex_);
    for (std::size_t i = 0; i < joints_.size(); ++i) {
        if (!snapshots[i].valid) {
            continue;
        }
        joints_[i].pos = snapshots[i].pos;
        joints_[i].vel = snapshots[i].vel;
        joints_[i].eff = snapshots[i].eff;
    }
}

// ---------------------------------------------------------------------------
// write
// ---------------------------------------------------------------------------
void CanDriverHW::write(const ros::Time & /*time*/, const ros::Duration &period)
{
    if (!active_.load(std::memory_order_acquire)) {
        return;
    }

#if SOFTWARE_LOOPBACK_MODE
    // ========== 软件回环模式 ==========
    // 不发送 CAN 帧，命令值已经在 write() 被 ros_control 写入 posCmd/velCmd
    // read() 会直接读取这些值作为反馈
#else
    // ========== 真实 CAN 模式 ==========

    // 应用关节限位（钳制命令值到安全范围）
    posLimitsIface_.enforceLimits(period);
    velLimitsIface_.enforceLimits(period);

    struct JointCommand {
        MotorID motorId{MotorID::LeftWheel};
        CanType protocol{CanType::MT};
        std::string canDevice;
        std::string controlMode;
        int32_t rawValue{0};
        bool valid{false};
    };
    std::vector<JointCommand> commands;
    commands.reserve(joints_.size());

    {
        std::lock_guard<std::mutex> lock(jointStateMutex_);
        const ros::Time now = ros::Time::now();
        const auto clampWithJointLimits = [](const JointConfig &jc, double cmdValue) -> double {
            if (!jc.hasLimits || !std::isfinite(cmdValue)) {
                return cmdValue;
            }

            if (jc.controlMode == "velocity" && jc.limits.has_velocity_limits) {
                return std::clamp(cmdValue, -jc.limits.max_velocity, jc.limits.max_velocity);
            }
            if (jc.controlMode == "position" && jc.limits.has_position_limits) {
                return std::clamp(cmdValue, jc.limits.min_position, jc.limits.max_position);
            }
            return cmdValue;
        };

        for (auto &jc : joints_) {
            JointCommand cmd;
            cmd.motorId = jc.motorId;
            cmd.protocol = jc.protocol;
            cmd.canDevice = jc.canDevice;
            cmd.controlMode = jc.controlMode;

            bool useDirect = false;
            double cmdValue = 0.0;
            if (jc.controlMode == "velocity") {
                if (jc.hasDirectVelCmd) {
                    const double age = (now - jc.lastDirectVelTime).toSec();
                    if (age <= directCmdTimeoutSec_) {
                        useDirect = true;
                        cmdValue = jc.directVelCmd;
                    } else {
                        jc.hasDirectVelCmd = false;
                    }
                }
                if (!useDirect) {
                    cmdValue = jc.velCmd;
                }
            } else {
                if (jc.hasDirectPosCmd) {
                    const double age = (now - jc.lastDirectPosTime).toSec();
                    if (age <= directCmdTimeoutSec_) {
                        useDirect = true;
                        cmdValue = jc.directPosCmd;
                    } else {
                        jc.hasDirectPosCmd = false;
                    }
                }
                if (!useDirect) {
                    cmdValue = jc.posCmd;
                }
            }

            cmdValue = clampWithJointLimits(jc, cmdValue);
            const double scale =
                (jc.controlMode == "velocity") ? jc.velocityScale : jc.positionScale;
            if (!std::isfinite(cmdValue) || !std::isfinite(scale) || scale <= 0.0) {
                ROS_ERROR_THROTTLE(1.0,
                                   "[CanDriverHW] Invalid command/scale for joint '%s' "
                                   "(cmd=%g, scale=%g).",
                                   jc.name.c_str(), cmdValue, scale);
                cmd.valid = false;
            } else {
                const double raw = cmdValue / scale;
                if (!std::isfinite(raw)) {
                    ROS_ERROR_THROTTLE(1.0,
                                       "[CanDriverHW] Non-finite raw command for joint '%s' "
                                       "(cmd=%g, scale=%g).",
                                       jc.name.c_str(), cmdValue, scale);
                    cmd.valid = false;
                } else {
                    const double clampedRaw = std::max(
                        static_cast<double>(std::numeric_limits<int32_t>::min()),
                        std::min(static_cast<double>(std::numeric_limits<int32_t>::max()), raw));
                    cmd.rawValue = static_cast<int32_t>(std::llround(clampedRaw));
                    cmd.valid = true;
                }
            }
            commands.push_back(cmd);
        }
    }

    std::map<std::pair<std::string, CanType>, std::vector<JointCommand>> commandGroups;
    for (const auto &cmd : commands) {
        if (cmd.valid) {
            commandGroups[{cmd.canDevice, cmd.protocol}].push_back(cmd);
        }
    }

    for (const auto &group : commandGroups) {
        const std::string &device = group.first.first;
        const CanType protocol = group.first.second;
        auto transport = getTransport(device);
        if (!transport || !transport->isReady()) {
            ROS_WARN_THROTTLE(1.0, "[CanDriverHW] Device '%s' not ready, skip command write.",
                              device.c_str());
            continue;
        }
        auto proto = getProtocol(device, protocol);
        auto devMutex = getDeviceMutex(device);
        if (!proto || !devMutex) {
            continue;
        }

        std::lock_guard<std::mutex> devLock(*devMutex);
        for (const auto &cmd : group.second) {
            try {
                if (cmd.controlMode == "velocity") {
                    if (!proto->setVelocity(cmd.motorId, cmd.rawValue)) {
                        ROS_WARN_THROTTLE(1.0,
                                          "[CanDriverHW] setVelocity rejected on '%s'.",
                                          device.c_str());
                    }
                } else {
                    if (!proto->setPosition(cmd.motorId, cmd.rawValue)) {
                        ROS_WARN_THROTTLE(1.0,
                                          "[CanDriverHW] setPosition rejected on '%s'.",
                                          device.c_str());
                    }
                }
            } catch (const std::exception &e) {
                ROS_ERROR_THROTTLE(1.0, "[CanDriverHW] write() command failed on '%s': %s",
                                   device.c_str(), e.what());
            } catch (...) {
                ROS_ERROR_THROTTLE(
                    1.0,
                    "[CanDriverHW] write() command failed on '%s' (unknown exception).",
                    device.c_str());
            }
        }
    }
#endif
}

// ---------------------------------------------------------------------------
// 内部辅助
// ---------------------------------------------------------------------------
bool CanDriverHW::initDevice(const std::string &device, bool loopback)
{
    std::unique_lock<std::shared_mutex> lock(protocolMutex_);

    std::shared_ptr<SocketCanController> transport;
    auto it = transports_.find(device);
    if (it != transports_.end()) {
        transport = it->second;
        // 已存在：重新初始化
        transport->shutdown();
        if (!transport->initialize(device, loopback)) {
            ROS_ERROR("[CanDriverHW] Re-init of '%s' failed.", device.c_str());
            return false;
        }
    } else {
        // 不存在：新建（通常不走这路，init() 时已全部创建）
        transport = std::make_shared<SocketCanController>();
        if (!transport->initialize(device, loopback)) {
            ROS_ERROR("[CanDriverHW] Failed to init '%s'.", device.c_str());
            return false;
        }
        transports_[device] = transport;
    }

    if (deviceCmdMutexes_.find(device) == deviceCmdMutexes_.end()) {
        deviceCmdMutexes_[device] = std::make_shared<std::mutex>();
    }

    bool hasMt = false;
    bool hasPp = false;
    std::vector<MotorID> mtIds;
    std::vector<MotorID> ppIds;
    for (const auto &jc : joints_) {
        if (jc.canDevice != device) {
            continue;
        }
        if (jc.protocol == CanType::MT) {
            hasMt = true;
            mtIds.push_back(jc.motorId);
        } else {
            hasPp = true;
            ppIds.push_back(jc.motorId);
        }
    }
    if (hasMt && mtProtocols_.find(device) == mtProtocols_.end()) {
        mtProtocols_[device] = std::make_shared<MtCan>(transport);
    }
    if (hasPp && eyouProtocols_.find(device) == eyouProtocols_.end()) {
        eyouProtocols_[device] = std::make_shared<EyouCan>(transport);
    }
    if (hasMt) {
        mtProtocols_[device]->initializeMotorRefresh(mtIds);
    }
    if (hasPp) {
        eyouProtocols_[device]->initializeMotorRefresh(ppIds);
    }

    ROS_INFO("[CanDriverHW] Initialized '%s'.", device.c_str());
    return true;
}

std::shared_ptr<CanProtocol> CanDriverHW::getProtocol(const std::string &device, CanType type)
{
    std::shared_lock<std::shared_mutex> lock(protocolMutex_);
    if (type == CanType::MT) {
        auto it = mtProtocols_.find(device);
        if (it != mtProtocols_.end()) {
            return std::static_pointer_cast<CanProtocol>(it->second);
        }
    } else {
        auto it = eyouProtocols_.find(device);
        if (it != eyouProtocols_.end()) {
            return std::static_pointer_cast<CanProtocol>(it->second);
        }
    }
    return nullptr;
}

std::shared_ptr<std::mutex> CanDriverHW::getDeviceMutex(const std::string &device)
{
    std::shared_lock<std::shared_mutex> lock(protocolMutex_);
    auto it = deviceCmdMutexes_.find(device);
    return (it != deviceCmdMutexes_.end()) ? it->second : nullptr;
}

std::shared_ptr<SocketCanController> CanDriverHW::getTransport(const std::string &device)
{
    std::shared_lock<std::shared_mutex> lock(protocolMutex_);
    auto it = transports_.find(device);
    return (it != transports_.end()) ? it->second : nullptr;
}

void CanDriverHW::publishMotorStates(const ros::TimerEvent & /*e*/)
{
    if (!active_.load(std::memory_order_acquire)) {
        return;
    }

    std::vector<can_driver::MotorState> msgs;
    msgs.reserve(joints_.size());
    {
        std::lock_guard<std::mutex> lock(jointStateMutex_);
        for (const auto &jc : joints_) {
            can_driver::MotorState msg;
            msg.motor_id = static_cast<uint16_t>(jc.motorId);
            msg.name     = jc.name;

            const auto clampToInt32 = [](double v) -> int32_t {
                if (!std::isfinite(v)) {
                    return 0;
                }
                const double c = std::max(
                    static_cast<double>(std::numeric_limits<int32_t>::min()),
                    std::min(static_cast<double>(std::numeric_limits<int32_t>::max()), v));
                return static_cast<int32_t>(std::llround(c));
            };
            const auto clampToInt16 = [](double v) -> int16_t {
                if (!std::isfinite(v)) {
                    return 0;
                }
                const double c = std::max(
                    static_cast<double>(std::numeric_limits<int16_t>::min()),
                    std::min(static_cast<double>(std::numeric_limits<int16_t>::max()), v));
                return static_cast<int16_t>(std::lround(c));
            };

            const double rawPos = jc.pos / jc.positionScale;
            const double rawVel = jc.vel / jc.velocityScale;
            msg.position = clampToInt32(rawPos);
            msg.velocity = clampToInt16(rawVel);
            msg.current  = clampToInt16(jc.eff);

            if (jc.controlMode == "velocity")
                msg.mode = can_driver::MotorState::MODE_VELOCITY;
            else
                msg.mode = can_driver::MotorState::MODE_POSITION;

            msgs.push_back(msg);
        }
    }
    if (!active_.load(std::memory_order_acquire)) {
        return;
    }
    for (const auto &msg : msgs) {
        motorStatesPub_.publish(msg);
    }
}

// ---------------------------------------------------------------------------
// Service 回调
// ---------------------------------------------------------------------------
bool CanDriverHW::onInit(can_driver::Init::Request &req,
                         can_driver::Init::Response &res)
{
    res.success = initDevice(req.device, req.loopback);
    if (res.success) {
        active_.store(true, std::memory_order_release);
        stateTimer_.start();
    }
    res.message = res.success ? "OK" : "Failed to initialize " + req.device;
    return true;
}

bool CanDriverHW::onShutdown(can_driver::Shutdown::Request & /*req*/,
                              can_driver::Shutdown::Response &res)
{
    active_.store(false, std::memory_order_release);
    stateTimer_.stop();
    std::unique_lock<std::shared_mutex> lock(protocolMutex_);

    mtProtocols_.clear();
    eyouProtocols_.clear();
    for (auto &kv : transports_) {
        kv.second->shutdown();
    }
    transports_.clear();
    deviceCmdMutexes_.clear();

    {
        std::lock_guard<std::mutex> stateLock(jointStateMutex_);
        for (auto &jc : joints_) {
            jc.hasDirectPosCmd = false;
            jc.hasDirectVelCmd = false;
        }
    }

    res.success = true;
    res.message = "All CAN devices shut down.";
    ROS_INFO("[CanDriverHW] All devices shut down.");
    return true;
}

bool CanDriverHW::onRecover(can_driver::Recover::Request &req,
                             can_driver::Recover::Response &res)
{
    if (!active_.load(std::memory_order_acquire)) {
        res.success = false;
        res.message = "Driver inactive.";
        return true;
    }

    // 优先精确匹配；若为通配值则对全部电机使能
    constexpr uint16_t kRecoverAllLegacy = 0;      // 兼容旧语义
    constexpr uint16_t kRecoverAllExplicit = 0xFFFF;
    const bool hasExactMatch = std::any_of(
        joints_.begin(), joints_.end(),
        [&req](const JointConfig &jc) {
            return static_cast<uint16_t>(jc.motorId) == req.motor_id;
        });
    const bool recoverAll =
        (req.motor_id == kRecoverAllExplicit) ||
        (req.motor_id == kRecoverAllLegacy && !hasExactMatch);

    bool found = false;
    for (const auto &jc : joints_) {
        if (recoverAll || static_cast<uint16_t>(jc.motorId) == req.motor_id) {
            auto transport = getTransport(jc.canDevice);
            if (!transport || !transport->isReady()) {
                continue;
            }
            auto proto = getProtocol(jc.canDevice, jc.protocol);
            auto devMutex = getDeviceMutex(jc.canDevice);
            if (proto && devMutex) {
                std::lock_guard<std::mutex> devLock(*devMutex);
                try {
                    if (!proto->Enable(jc.motorId)) {
                        continue;
                    }
                } catch (const std::exception &e) {
                    ROS_ERROR("[CanDriverHW] Recover failed on '%s' motor %u: %s",
                              jc.canDevice.c_str(),
                              static_cast<unsigned>(static_cast<uint16_t>(jc.motorId)),
                              e.what());
                    continue;
                } catch (...) {
                    ROS_ERROR("[CanDriverHW] Recover failed on '%s' motor %u (unknown exception).",
                              jc.canDevice.c_str(),
                              static_cast<unsigned>(static_cast<uint16_t>(jc.motorId)));
                    continue;
                }
                found = true;
            }
        }
    }
    res.success = found;
    res.message = found ? "Recovered." : "Motor not found.";
    return true;
}

bool CanDriverHW::onMotorCommand(can_driver::MotorCommand::Request &req,
                                  can_driver::MotorCommand::Response &res)
{
    if (!active_.load(std::memory_order_acquire)) {
        res.success = false;
        res.message = "Driver inactive.";
        return true;
    }

    for (const auto &jc : joints_) {
        if (static_cast<uint16_t>(jc.motorId) != req.motor_id) continue;

        auto transport = getTransport(jc.canDevice);
        if (!transport || !transport->isReady()) {
            res.success = false;
            res.message = "CAN device not ready.";
            return true;
        }

        auto proto = getProtocol(jc.canDevice, jc.protocol);
        auto devMutex = getDeviceMutex(jc.canDevice);
        if (!proto || !devMutex) {
            res.success = false;
            res.message = "Protocol not available.";
            return true;
        }

        std::lock_guard<std::mutex> devLock(*devMutex);
        try {
            switch (req.command) {
            case can_driver::MotorCommand::Request::CMD_ENABLE:
                if (!proto->Enable(jc.motorId)) {
                    res.success = false;
                    res.message = "Enable command rejected.";
                    return true;
                }
                break;
            case can_driver::MotorCommand::Request::CMD_DISABLE:
                if (!proto->Disable(jc.motorId)) {
                    res.success = false;
                    res.message = "Disable command rejected.";
                    return true;
                }
                {
                    std::lock_guard<std::mutex> stateLock(jointStateMutex_);
                    auto it = jointIndexByName_.find(jc.name);
                    if (it != jointIndexByName_.end()) {
                        joints_[it->second].hasDirectPosCmd = false;
                        joints_[it->second].hasDirectVelCmd = false;
                    }
                }
                break;
            case can_driver::MotorCommand::Request::CMD_STOP:
                if (!proto->Stop(jc.motorId)) {
                    res.success = false;
                    res.message = "Stop command rejected.";
                    return true;
                }
                {
                    std::lock_guard<std::mutex> stateLock(jointStateMutex_);
                    auto it = jointIndexByName_.find(jc.name);
                    if (it != jointIndexByName_.end()) {
                        joints_[it->second].hasDirectPosCmd = false;
                        joints_[it->second].hasDirectVelCmd = false;
                    }
                }
                break;
            case can_driver::MotorCommand::Request::CMD_SET_MODE: {
                if (req.value != 0.0 && req.value != 1.0) {
                    res.success = false;
                    res.message = "CMD_SET_MODE value must be 0 or 1.";
                    return true;
                }
                auto mode = (req.value == 0.0)
                                ? CanProtocol::MotorMode::Position
                                : CanProtocol::MotorMode::Velocity;
                if (!proto->setMode(jc.motorId, mode)) {
                    res.success = false;
                    res.message = "Set mode command rejected.";
                    return true;
                }
                break;
            }
            default:
                res.success = false;
                res.message = "Unknown command.";
                return true;
            }
        } catch (const std::exception &e) {
            res.success = false;
            res.message = std::string("Command execution failed: ") + e.what();
            return true;
        } catch (...) {
            res.success = false;
            res.message = "Command execution failed.";
            return true;
        }

        res.success = true;
        res.message = "OK";
        return true;
    }

    res.success = false;
    res.message = "Motor ID not found.";
    return true;
}
