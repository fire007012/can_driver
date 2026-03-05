#include "can_driver/CanDriverHW.h"
#include "can_driver/SafeCommand.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <set>
#include <string>
#include <xmlrpcpp/XmlRpcValue.h>

// ---------------------------------------------------------------------------
// 析构
// ---------------------------------------------------------------------------
CanDriverHW::~CanDriverHW()
{
    resetInternalState();

    initSrv_.shutdown();
    shutdownSrv_.shutdown();
    recoverSrv_.shutdown();
    motorCmdSrv_.shutdown();
}

// ---------------------------------------------------------------------------
// init
// ---------------------------------------------------------------------------
bool CanDriverHW::init(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
    (void)nh;
    resetInternalState();
    if (!loadRuntimeParams(pnh)) {
        return false;
    }
    if (!parseAndSetupJoints(pnh)) {
        resetInternalState();
        return false;
    }
    registerJointInterfaces();
    loadJointLimits(pnh);
    startMotorRefreshThreads();
    setupRosComm(pnh);

    ROS_INFO("[CanDriverHW] Initialized with %zu joints on %zu CAN device(s).",
             joints_.size(), deviceManager_.deviceCount());
    active_.store(true, std::memory_order_release);
    return true;
}

void CanDriverHW::resetInternalState()
{
    active_.store(false, std::memory_order_release);
    stateTimer_.stop();

    for (auto &kv : cmdVelSubs_) {
        kv.second.shutdown();
    }
    for (auto &kv : cmdPosSubs_) {
        kv.second.shutdown();
    }
    cmdVelSubs_.clear();
    cmdPosSubs_.clear();

    joints_.clear();
    jointIndexByName_.clear();
    jointGroups_.clear();
    rawCommandBuffer_.clear();
    commandValidBuffer_.clear();
    deviceManager_.shutdownAll();
}

bool CanDriverHW::loadRuntimeParams(const ros::NodeHandle &pnh)
{
    if (!pnh.getParam("direct_cmd_timeout_sec", directCmdTimeoutSec_)) {
        directCmdTimeoutSec_ = 0.5;
    }
    if (!std::isfinite(directCmdTimeoutSec_) || directCmdTimeoutSec_ < 0.0) {
        ROS_WARN("[CanDriverHW] Invalid direct_cmd_timeout_sec=%.9g, fallback to 0.5s.",
                 directCmdTimeoutSec_);
        directCmdTimeoutSec_ = 0.5;
    }

    if (!pnh.getParam("motor_state_period_sec", statePublishPeriodSec_)) {
        statePublishPeriodSec_ = 0.1;
    }
    if (!std::isfinite(statePublishPeriodSec_) || statePublishPeriodSec_ <= 0.0) {
        ROS_WARN("[CanDriverHW] Invalid motor_state_period_sec=%.9g, fallback to 0.1s.",
                 statePublishPeriodSec_);
        statePublishPeriodSec_ = 0.1;
    }

    if (!pnh.getParam("direct_cmd_queue_size", directCmdQueueSize_)) {
        directCmdQueueSize_ = 1;
    }
    if (directCmdQueueSize_ <= 0) {
        ROS_WARN("[CanDriverHW] Invalid direct_cmd_queue_size=%d, fallback to 1.",
                 directCmdQueueSize_);
        directCmdQueueSize_ = 1;
    }
    return true;
}

bool CanDriverHW::parseAndSetupJoints(const ros::NodeHandle &pnh)
{
    XmlRpc::XmlRpcValue jointList;
    if (!pnh.getParam("joints", jointList)) {
        ROS_ERROR("[CanDriverHW] Parameter 'joints' not found under %s",
                  pnh.getNamespace().c_str());
        return false;
    }
    std::vector<joint_config_parser::ParsedJointConfig> parsed;
    std::string errorMsg;
    if (!joint_config_parser::parse(jointList, parsed, errorMsg)) {
        ROS_ERROR("[CanDriverHW] %s", errorMsg.c_str());
        return false;
    }

    std::set<std::string> seenJointNames;
    std::set<uint16_t> seenMotorIds;
    for (const auto &p : parsed) {
        const std::string &jointName = p.name;
        const uint16_t motorId = static_cast<uint16_t>(p.motorId);

        if (!seenJointNames.insert(jointName).second) {
            ROS_ERROR("[CanDriverHW] Duplicate joint name '%s' in joints config.", jointName.c_str());
            return false;
        }
        if (!seenMotorIds.insert(motorId).second) {
            ROS_ERROR("[CanDriverHW] Duplicate motor_id=%u in joints config. "
                      "motor_id must be globally unique because service commands are addressed by motor_id only.",
                      static_cast<unsigned>(motorId));
            return false;
        }

        JointConfig jc;
        jc.name = p.name;
        jc.canDevice = p.canDevice;
        jc.controlMode = p.controlMode;
        jc.motorId = p.motorId;
        jc.protocol = p.protocol;
        jc.positionScale = p.positionScale;
        jc.velocityScale = p.velocityScale;

        joints_.push_back(jc);
        jointIndexByName_[jc.name] = joints_.size() - 1;

        if (!deviceManager_.ensureTransport(jc.canDevice)) {
            return false;
        }
        if (!deviceManager_.ensureProtocol(jc.canDevice, jc.protocol)) {
            ROS_ERROR("[CanDriverHW] Failed to ensure protocol on '%s'.", jc.canDevice.c_str());
            return false;
        }
    }

    rebuildJointGroups();
    rawCommandBuffer_.assign(joints_.size(), 0);
    commandValidBuffer_.assign(joints_.size(), 0);
    return true;
}

void CanDriverHW::rebuildJointGroups()
{
    std::map<std::pair<std::string, CanType>, std::vector<std::size_t>> groupedIndices;
    for (std::size_t i = 0; i < joints_.size(); ++i) {
        groupedIndices[{joints_[i].canDevice, joints_[i].protocol}].push_back(i);
    }

    jointGroups_.clear();
    jointGroups_.reserve(groupedIndices.size());
    for (const auto &entry : groupedIndices) {
        DeviceProtocolGroup group;
        group.canDevice = entry.first.first;
        group.protocol = entry.first.second;
        group.jointIndices = entry.second;
        jointGroups_.push_back(std::move(group));
    }
}

void CanDriverHW::registerJointInterfaces()
{
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
}

void CanDriverHW::loadJointLimits(const ros::NodeHandle &pnh)
{
    urdf::Model urdf;
    const bool urdfLoaded = urdf.initParam("robot_description");
    if (!urdfLoaded) {
        ROS_WARN("[CanDriverHW] Failed to load URDF from 'robot_description'. "
                 "Joint limits will not be enforced.");
    }

    for (auto &jc : joints_) {
        joint_limits_interface::JointLimits limits;
        joint_limits_interface::SoftJointLimits soft_limits;
        bool hasLimits = false;

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

        if (joint_limits_interface::getJointLimits(jc.name, pnh, limits)) {
            hasLimits = true;
            ROS_INFO("[CanDriverHW] Joint '%s': rosparam overrides limits.", jc.name.c_str());
        }
        joint_limits_interface::getSoftJointLimits(jc.name, pnh, soft_limits);

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
}

void CanDriverHW::startMotorRefreshThreads()
{
    std::map<std::string, std::vector<MotorID>> mtIds;
    std::map<std::string, std::vector<MotorID>> ppIds;
    for (const auto &jc : joints_) {
        if (jc.protocol == CanType::MT) {
            mtIds[jc.canDevice].push_back(jc.motorId);
        } else {
            ppIds[jc.canDevice].push_back(jc.motorId);
        }
    }
    for (auto &kv : mtIds) {
        deviceManager_.startRefresh(kv.first, CanType::MT, kv.second);
    }
    for (auto &kv : ppIds) {
        deviceManager_.startRefresh(kv.first, CanType::PP, kv.second);
    }
}

void CanDriverHW::setupRosComm(ros::NodeHandle &pnh)
{
    initSrv_     = pnh.advertiseService("init",          &CanDriverHW::onInit,         this);
    shutdownSrv_ = pnh.advertiseService("shutdown",      &CanDriverHW::onShutdown,     this);
    recoverSrv_  = pnh.advertiseService("recover",       &CanDriverHW::onRecover,      this);
    motorCmdSrv_ = pnh.advertiseService("motor_command", &CanDriverHW::onMotorCommand, this);

    const auto makeDirectCmdCallback =
        [this](std::size_t idx, bool isVelocity) {
            return [this, idx, isVelocity](const std_msgs::Float64::ConstPtr &msg) {
                if (!active_.load(std::memory_order_acquire)) {
                    return;
                }
                std::lock_guard<std::mutex> lock(jointStateMutex_);
                auto &jc = joints_[idx];
                if (isVelocity) {
                    jc.directVelCmd = msg->data;
                    jc.hasDirectVelCmd = true;
                    jc.lastDirectVelTime = ros::Time::now();
                } else {
                    jc.directPosCmd = msg->data;
                    jc.hasDirectPosCmd = true;
                    jc.lastDirectPosTime = ros::Time::now();
                }
            };
        };

    for (auto &jc : joints_) {
        const std::string velTopic = "motor/" + jc.name + "/cmd_velocity";
        const std::string posTopic = "motor/" + jc.name + "/cmd_position";
        const std::size_t idx = jointIndexByName_[jc.name];
        cmdVelSubs_[jc.name] = pnh.subscribe<std_msgs::Float64>(
            velTopic, static_cast<uint32_t>(directCmdQueueSize_),
            makeDirectCmdCallback(idx, true));

        cmdPosSubs_[jc.name] = pnh.subscribe<std_msgs::Float64>(
            posTopic, static_cast<uint32_t>(directCmdQueueSize_),
            makeDirectCmdCallback(idx, false));
    }

    motorStatesPub_ = pnh.advertise<can_driver::MotorState>("motor_states", 10);
    stateTimer_ = pnh.createTimer(ros::Duration(statePublishPeriodSec_),
                                  &CanDriverHW::publishMotorStates, this);
}

void CanDriverHW::clearDirectCmd(const std::string &jointName)
{
    std::lock_guard<std::mutex> stateLock(jointStateMutex_);
    const auto it = jointIndexByName_.find(jointName);
    if (it != jointIndexByName_.end()) {
        joints_[it->second].hasDirectPosCmd = false;
        joints_[it->second].hasDirectVelCmd = false;
    }
}

const CanDriverHW::JointConfig *CanDriverHW::findJointByMotorId(uint16_t motorId) const
{
    for (const auto &jc : joints_) {
        if (static_cast<uint16_t>(jc.motorId) == motorId) {
            return &jc;
        }
    }
    return nullptr;
}

CanDriverHW::MotorOpStatus CanDriverHW::executeOnMotor(
    const JointConfig &jc,
    const std::function<bool(const std::shared_ptr<CanProtocol> &, MotorID)> &op,
    const char *operationName)
{
    auto transport = getTransport(jc.canDevice);
    if (!transport || !transport->isReady()) {
        return MotorOpStatus::DeviceNotReady;
    }

    auto proto = getProtocol(jc.canDevice, jc.protocol);
    auto devMutex = getDeviceMutex(jc.canDevice);
    if (!proto || !devMutex) {
        return MotorOpStatus::ProtocolUnavailable;
    }

    std::lock_guard<std::mutex> devLock(*devMutex);
    try {
        return op(proto, jc.motorId) ? MotorOpStatus::Ok : MotorOpStatus::Rejected;
    } catch (const std::exception &e) {
        ROS_ERROR("[CanDriverHW] %s failed on '%s' motor %u: %s",
                  operationName,
                  jc.canDevice.c_str(),
                  static_cast<unsigned>(static_cast<uint16_t>(jc.motorId)),
                  e.what());
        return MotorOpStatus::Exception;
    } catch (...) {
        ROS_ERROR("[CanDriverHW] %s failed on '%s' motor %u (unknown exception).",
                  operationName,
                  jc.canDevice.c_str(),
                  static_cast<unsigned>(static_cast<uint16_t>(jc.motorId)));
        return MotorOpStatus::Exception;
    }
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

    for (const auto &group : jointGroups_) {
        const std::string &device = group.canDevice;
        const CanType protocol = group.protocol;
        auto proto = getProtocol(device, protocol);
        auto devMutex = getDeviceMutex(device);
        if (!proto || !devMutex) {
            continue;
        }

        std::lock_guard<std::mutex> devLock(*devMutex);
        for (const std::size_t i : group.jointIndices) {
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

        for (std::size_t i = 0; i < joints_.size(); ++i) {
            auto &jc = joints_[i];

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
            commandValidBuffer_[i] = static_cast<uint8_t>(
                can_driver::safe_command::scaleAndClampToInt32(
                    cmdValue, scale, jc.name, rawCommandBuffer_[i]));
        }
    }

    for (const auto &group : jointGroups_) {
        const std::string &device = group.canDevice;
        const CanType protocol = group.protocol;
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
        for (const std::size_t idx : group.jointIndices) {
            if (!commandValidBuffer_[idx]) {
                continue;
            }
            const auto &jc = joints_[idx];
            try {
                if (jc.controlMode == "velocity") {
                    if (!proto->setVelocity(jc.motorId, rawCommandBuffer_[idx])) {
                        ROS_WARN_THROTTLE(1.0,
                                          "[CanDriverHW] setVelocity rejected on '%s'.",
                                          device.c_str());
                    }
                } else {
                    if (!proto->setPosition(jc.motorId, rawCommandBuffer_[idx])) {
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
    std::vector<std::pair<CanType, MotorID>> motors;
    for (const auto &jc : joints_) {
        if (jc.canDevice != device) {
            continue;
        }
        motors.emplace_back(jc.protocol, jc.motorId);
    }
    return deviceManager_.initDevice(device, motors, loopback);
}

std::shared_ptr<CanProtocol> CanDriverHW::getProtocol(const std::string &device, CanType type) const
{
    return deviceManager_.getProtocol(device, type);
}

std::shared_ptr<std::mutex> CanDriverHW::getDeviceMutex(const std::string &device) const
{
    return deviceManager_.getDeviceMutex(device);
}

std::shared_ptr<SocketCanController> CanDriverHW::getTransport(const std::string &device) const
{
    return deviceManager_.getTransport(device);
}

void CanDriverHW::publishMotorStates(const ros::TimerEvent & /*e*/)
{
    if (!active_.load(std::memory_order_acquire)) {
        return;
    }

    struct StatusSnapshot {
        bool enabled{false};
        bool fault{false};
        bool valid{false};
    };
    std::vector<StatusSnapshot> statusSnapshots(joints_.size());
    for (const auto &group : jointGroups_) {
        auto proto = getProtocol(group.canDevice, group.protocol);
        auto devMutex = getDeviceMutex(group.canDevice);
        if (!proto || !devMutex) {
            continue;
        }

        std::lock_guard<std::mutex> devLock(*devMutex);
        for (const std::size_t idx : group.jointIndices) {
            const auto &jc = joints_[idx];
            statusSnapshots[idx].enabled = proto->isEnabled(jc.motorId);
            statusSnapshots[idx].fault = proto->hasFault(jc.motorId);
            statusSnapshots[idx].valid = true;
        }
    }

    std::vector<can_driver::MotorState> msgs;
    msgs.reserve(joints_.size());
    {
        std::lock_guard<std::mutex> lock(jointStateMutex_);
        for (std::size_t i = 0; i < joints_.size(); ++i) {
            const auto &jc = joints_[i];
            can_driver::MotorState msg;
            msg.motor_id = static_cast<uint16_t>(jc.motorId);
            msg.name     = jc.name;

            const double rawPos = jc.pos / jc.positionScale;
            const double rawVel = jc.vel / jc.velocityScale;
            msg.position = can_driver::safe_command::clampToInt32(rawPos);
            msg.velocity = can_driver::safe_command::clampToInt16(rawVel);
            msg.current  = can_driver::safe_command::clampToInt16(jc.eff);

            if (jc.controlMode == "velocity")
                msg.mode = can_driver::MotorState::MODE_VELOCITY;
            else
                msg.mode = can_driver::MotorState::MODE_POSITION;
            if (statusSnapshots[i].valid) {
                msg.enabled = statusSnapshots[i].enabled;
                msg.fault = statusSnapshots[i].fault;
            }

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
    deviceManager_.shutdownAll();

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
    bool hasFailure = false;
    MotorOpStatus firstFailure = MotorOpStatus::Ok;
    for (const auto &jc : joints_) {
        if (recoverAll || static_cast<uint16_t>(jc.motorId) == req.motor_id) {
            const auto status = executeOnMotor(
                jc,
                [](const std::shared_ptr<CanProtocol> &proto, MotorID id) {
                    return proto->Enable(id);
                },
                "Recover");
            if (status == MotorOpStatus::Ok) {
                found = true;
            } else if (!hasFailure) {
                hasFailure = true;
                firstFailure = status;
            }
        }
    }
    if (found) {
        res.success = true;
        res.message = "Recovered.";
    } else if (hasFailure) {
        res.success = false;
        if (firstFailure == MotorOpStatus::DeviceNotReady) {
            res.message = "CAN device not ready.";
        } else if (firstFailure == MotorOpStatus::ProtocolUnavailable) {
            res.message = "Protocol not available.";
        } else if (firstFailure == MotorOpStatus::Rejected) {
            res.message = "Recover command rejected.";
        } else {
            res.message = "Recover execution failed.";
        }
    } else {
        res.success = false;
        res.message = "Motor not found.";
    }
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

    const auto *target = findJointByMotorId(req.motor_id);
    if (!target) {
        res.success = false;
        res.message = "Motor ID not found.";
        return true;
    }

    auto handleFailure = [&res](MotorOpStatus status, const char *rejectedMsg) {
        if (status == MotorOpStatus::DeviceNotReady) {
            res.success = false;
            res.message = "CAN device not ready.";
        } else if (status == MotorOpStatus::ProtocolUnavailable) {
            res.success = false;
            res.message = "Protocol not available.";
        } else if (status == MotorOpStatus::Rejected) {
            res.success = false;
            res.message = rejectedMsg;
        } else {
            res.success = false;
            res.message = "Command execution failed.";
        }
    };

    if (req.command == can_driver::MotorCommand::Request::CMD_SET_MODE) {
        if (req.value != 0.0 && req.value != 1.0) {
            res.success = false;
            res.message = "CMD_SET_MODE value must be 0 or 1.";
            return true;
        }
        const auto mode = (req.value == 0.0)
                              ? CanProtocol::MotorMode::Position
                              : CanProtocol::MotorMode::Velocity;
        const auto status = executeOnMotor(
            *target,
            [&mode](const std::shared_ptr<CanProtocol> &proto, MotorID id) {
                return proto->setMode(id, mode);
            },
            "Set mode");
        if (status != MotorOpStatus::Ok) {
            handleFailure(status, "Set mode command rejected.");
            return true;
        }
        res.success = true;
        res.message = "OK";
        return true;
    }

    struct CmdEntry {
        uint8_t cmd;
        const char *name;
        bool clearDirect;
        std::function<bool(const std::shared_ptr<CanProtocol> &, MotorID)> action;
    };
    const std::vector<CmdEntry> table = {
        {can_driver::MotorCommand::Request::CMD_ENABLE,
         "Enable",
         false,
         [](const std::shared_ptr<CanProtocol> &proto, MotorID id) { return proto->Enable(id); }},
        {can_driver::MotorCommand::Request::CMD_DISABLE,
         "Disable",
         true,
         [](const std::shared_ptr<CanProtocol> &proto, MotorID id) { return proto->Disable(id); }},
        {can_driver::MotorCommand::Request::CMD_STOP,
         "Stop",
         true,
         [](const std::shared_ptr<CanProtocol> &proto, MotorID id) { return proto->Stop(id); }},
    };

    for (const auto &entry : table) {
        if (req.command != entry.cmd) {
            continue;
        }
        const auto status = executeOnMotor(*target, entry.action, entry.name);
        if (status != MotorOpStatus::Ok) {
            const std::string rejectedMsg = std::string(entry.name) + " command rejected.";
            handleFailure(status, rejectedMsg.c_str());
            return true;
        }
        if (entry.clearDirect) {
            clearDirectCmd(target->name);
        }
        res.success = true;
        res.message = "OK";
        return true;
    }

    res.success = false;
    res.message = "Unknown command.";
    return true;
}
