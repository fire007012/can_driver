#include "can_driver/CanDriverHW.h"
#include "can_driver/SafeCommand.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <set>
#include <string>
#include <thread>
#include <xmlrpcpp/XmlRpcValue.h>

CanDriverHW::CanDriverHW()
    : deviceManager_(std::make_shared<DeviceManager>())
{
    motorActionExecutor_.setDeviceManager(deviceManager_);
    lifecycleDriverOps_.configure(deviceManager_, &motorActionExecutor_);
    configureCommandGate();
    configureLifecycleCoordinator();
}

CanDriverHW::CanDriverHW(std::shared_ptr<IDeviceManager> deviceManager)
    : deviceManager_(std::move(deviceManager))
{
    if (!deviceManager_) {
        deviceManager_ = std::make_shared<DeviceManager>();
    }
    motorActionExecutor_.setDeviceManager(deviceManager_);
    lifecycleDriverOps_.configure(deviceManager_, &motorActionExecutor_);
    configureCommandGate();
    configureLifecycleCoordinator();
}

// ---------------------------------------------------------------------------
// 析构
// ---------------------------------------------------------------------------
CanDriverHW::~CanDriverHW()
{
    resetInternalState();

    motorCmdSrv_.shutdown();
    setZeroLimitSrv_.shutdown();
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
    if (!syncStartupPositionAndCommands()) {
        resetInternalState();
        return false;
    }
    setupMaintenanceRosComm(pnh);

    ROS_INFO("[CanDriverHW] Initialized with %zu joints on %zu CAN device(s).",
             joints_.size(), deviceManager_->deviceCount());
    active_.store(true, std::memory_order_release);
    lifecycleCoordinator_.SetConfigured();
    return true;
}

void CanDriverHW::resetInternalState()
{
    active_.store(false, std::memory_order_release);
    lifecycleCoordinator_.SetInactive();
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
    jointZeroOffsetRadByMotorId_.clear();
    commandGate_.reset();
    lifecycleDriverOps_.setTargets({});
    deviceManager_->shutdownAll();
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

    if (!pnh.getParam("motor_query_hz", motorQueryHz_)) {
        motorQueryHz_ = 0.0;
    }
    if (!std::isfinite(motorQueryHz_)) {
        ROS_WARN("[CanDriverHW] Invalid motor_query_hz=%.9g, fallback to auto strategy.",
                 motorQueryHz_);
        motorQueryHz_ = 0.0;
    }

    if (!pnh.getParam("direct_cmd_queue_size", directCmdQueueSize_)) {
        directCmdQueueSize_ = 1;
    }
    if (directCmdQueueSize_ <= 0) {
        ROS_WARN("[CanDriverHW] Invalid direct_cmd_queue_size=%d, fallback to 1.",
                 directCmdQueueSize_);
        directCmdQueueSize_ = 1;
    }

    if (!pnh.getParam("debug_bypass_ros_control", debugBypassRosControl_)) {
        debugBypassRosControl_ = false;
    }
    if (!pnh.getParam("startup_position_sync_timeout_sec", startupPositionSyncTimeoutSec_)) {
        startupPositionSyncTimeoutSec_ = 1.0;
    }
    if (!std::isfinite(startupPositionSyncTimeoutSec_) || startupPositionSyncTimeoutSec_ < 0.0) {
        ROS_WARN("[CanDriverHW] Invalid startup_position_sync_timeout_sec=%.9g, fallback to 1.0s.",
                 startupPositionSyncTimeoutSec_);
        startupPositionSyncTimeoutSec_ = 1.0;
    }
    if (!pnh.getParam("pp_fast_write_enabled", ppFastWriteEnabled_)) {
        ppFastWriteEnabled_ = false;
    }
    if (!pnh.getParam("safety_stop_on_fault", safetyStopOnFault_)) {
        safetyStopOnFault_ = true;
    }
    if (!pnh.getParam("safety_require_enabled_for_motion", safetyRequireEnabledForMotion_)) {
        safetyRequireEnabledForMotion_ = true;
    }
    if (!pnh.getParam("max_position_step_rad", maxPositionStepRad_)) {
        maxPositionStepRad_ = 0.0;
    }
    if (!std::isfinite(maxPositionStepRad_) || maxPositionStepRad_ < 0.0) {
        ROS_WARN("[CanDriverHW] Invalid max_position_step_rad=%.9g, fallback to 0(disabled).",
                 maxPositionStepRad_);
        maxPositionStepRad_ = 0.0;
    }
    if (!pnh.getParam("safety_hold_after_device_recover", safetyHoldAfterDeviceRecover_)) {
        safetyHoldAfterDeviceRecover_ = true;
    }

    deviceManager_->setPpFastWriteEnabled(ppFastWriteEnabled_);

    if (motorQueryHz_ > 0.0) {
        ROS_INFO("[CanDriverHW] motor_query_hz=%.3f Hz.", motorQueryHz_);
    }
    ROS_INFO("[CanDriverHW] pp_fast_write_enabled=%s.",
             ppFastWriteEnabled_ ? "true" : "false");
    ROS_INFO("[CanDriverHW] startup_position_sync_timeout_sec=%.3f s.",
             startupPositionSyncTimeoutSec_);
    ROS_INFO("[CanDriverHW] safety_stop_on_fault=%s, safety_require_enabled_for_motion=%s, max_position_step_rad=%.6f.",
             safetyStopOnFault_ ? "true" : "false",
             safetyRequireEnabledForMotion_ ? "true" : "false",
             maxPositionStepRad_);
    ROS_INFO("[CanDriverHW] safety_hold_after_device_recover=%s.",
             safetyHoldAfterDeviceRecover_ ? "true" : "false");
    ROS_WARN_STREAM_COND(debugBypassRosControl_,
                         "[CanDriverHW] debug_bypass_ros_control=true: "
                         "direct topic commands will bypass ros_control fallback.");
    return true;
}

bool CanDriverHW::syncStartupPositionAndCommands()
{
    struct JointSnapshot {
        double pos{0.0};
        double vel{0.0};
        double eff{0.0};
        bool valid{false};
    };

    std::vector<JointSnapshot> snapshots(joints_.size());

    // 给协议刷新线程一个短暂窗口拉取首轮反馈。
    const double timeout = startupPositionSyncTimeoutSec_;
    const auto sleepDur = std::chrono::milliseconds(20);
    const int maxPasses = std::max(1, static_cast<int>(std::ceil(timeout / 0.02)));

    for (int pass = 0; pass < maxPasses; ++pass) {
        for (const auto &group : jointGroups_) {
            auto proto = getProtocol(group.canDevice, group.protocol);
            auto devMutex = getDeviceMutex(group.canDevice);
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
        if (pass + 1 < maxPasses) {
            std::this_thread::sleep_for(sleepDur);
        }
    }

    bool startupOutOfRange = false;
    {
        std::lock_guard<std::mutex> lock(jointStateMutex_);
        for (std::size_t i = 0; i < joints_.size(); ++i) {
            auto &jc = joints_[i];
            if (snapshots[i].valid) {
                jc.pos = snapshots[i].pos;
                jc.vel = snapshots[i].vel;
                jc.eff = snapshots[i].eff;
            }

            if (jc.controlMode == "position") {
                // 上电后将位置命令对齐到当前反馈，避免控制循环首拍跳变。
                jc.posCmd = jc.pos;
                if (jc.hasLimits && jc.limits.has_position_limits) {
                    if (jc.pos < jc.limits.min_position || jc.pos > jc.limits.max_position) {
                        startupOutOfRange = true;
                        ROS_ERROR("[CanDriverHW] Joint '%s' startup position %.6f rad out of limits [%.6f, %.6f].",
                                  jc.name.c_str(),
                                  jc.pos,
                                  jc.limits.min_position,
                                  jc.limits.max_position);
                    }
                }
            } else {
                // 速度关节上电默认零速度命令。
                jc.velCmd = 0.0;
            }

            jc.hasDirectPosCmd = false;
            jc.hasDirectVelCmd = false;
            jc.stopIssuedOnFault = false;
        }
    }

    if (startupOutOfRange) {
        ROS_ERROR("[CanDriverHW] Startup position check failed. Refusing to activate to avoid limit violation.");
        return false;
    }

    ROS_INFO("[CanDriverHW] Startup position sync finished.");
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

        if (!deviceManager_->ensureTransport(jc.canDevice)) {
            return false;
        }
        if (!deviceManager_->ensureProtocol(jc.canDevice, jc.protocol)) {
            ROS_ERROR("[CanDriverHW] Failed to ensure protocol on '%s'.", jc.canDevice.c_str());
            return false;
        }
    }

    rebuildJointGroups();
    rawCommandBuffer_.assign(joints_.size(), 0);
    commandValidBuffer_.assign(joints_.size(), 0);
    syncLifecycleTargets();
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
    deviceManager_->setRefreshRateHz(motorQueryHz_);

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
        deviceManager_->startRefresh(kv.first, CanType::MT, kv.second);
    }
    for (auto &kv : ppIds) {
        deviceManager_->startRefresh(kv.first, CanType::PP, kv.second);
    }
}

void CanDriverHW::configureLifecycleCoordinator()
{
    can_driver::OperationalCoordinator::DriverOps ops;
    ops.init_device = [this](const std::string &device, bool loopback) {
        return initializeLifecycleDevice(device, loopback);
    };
    ops.enable_all = [this]() {
        return lifecycleDriverOps_.enableAll();
    };
    ops.disable_all = [this]() {
        return lifecycleDriverOps_.disableAll();
    };
    ops.halt_all = [this]() {
        return lifecycleDriverOps_.haltAll();
    };
    ops.recover_all = [this]() {
        return lifecycleDriverOps_.recoverAll();
    };
    ops.shutdown_all = [this](bool force) {
        return shutdownLifecycleDriver(force);
    };
    ops.motion_healthy = [this](std::string *detail) {
        return lifecycleDriverOps_.motionHealthy(detail);
    };
    ops.any_fault_active = [this]() {
        return lifecycleDriverOps_.anyFaultActive();
    };
    ops.hold_commands = [this]() {
        commandGate_.holdCommands();
    };
    ops.arm_fresh_command_latch = [this]() {
        commandGate_.armFreshCommandLatch();
    };
    lifecycleCoordinator_.SetDriverOps(std::move(ops));
}

void CanDriverHW::configureCommandGate()
{
    commandGate_.configure(
        [this]() {
            return captureCommandSnapshots();
        },
        [this]() {
            holdCommandsForLifecycleTransition();
        });
}

void CanDriverHW::setupMaintenanceRosComm(ros::NodeHandle &pnh)
{
    motorCmdSrv_ = pnh.advertiseService("motor_command", &CanDriverHW::onMotorCommand, this);
    setZeroLimitSrv_ = pnh.advertiseService("set_zero_limit",
                                            &CanDriverHW::onSetZeroLimit,
                                            this);

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

void CanDriverHW::holdCommandsForLifecycleTransition()
{
    std::lock_guard<std::mutex> stateLock(jointStateMutex_);
    for (std::size_t i = 0; i < joints_.size(); ++i) {
        auto &jc = joints_[i];
        jc.hasDirectPosCmd = false;
        jc.hasDirectVelCmd = false;
        if (jc.controlMode == "position") {
            jc.posCmd = jc.pos;
        } else {
            jc.velCmd = 0.0;
        }
        commandValidBuffer_[i] = 0;
    }
}

std::vector<CommandGate::Snapshot> CanDriverHW::captureCommandSnapshots() const
{
    std::lock_guard<std::mutex> stateLock(jointStateMutex_);
    std::vector<CommandGate::Snapshot> snapshots(joints_.size());
    for (std::size_t i = 0; i < joints_.size(); ++i) {
        const auto &jc = joints_[i];
        auto &snapshot = snapshots[i];
        snapshot.controlMode = jc.controlMode;
        snapshot.posCmd = jc.posCmd;
        snapshot.velCmd = jc.velCmd;
        snapshot.directPosCmd = jc.directPosCmd;
        snapshot.directVelCmd = jc.directVelCmd;
        snapshot.hasDirectPosCmd = jc.hasDirectPosCmd;
        snapshot.hasDirectVelCmd = jc.hasDirectVelCmd;
    }
    return snapshots;
}

void CanDriverHW::syncLifecycleTargets()
{
    std::vector<MotorActionExecutor::Target> targets;
    targets.reserve(joints_.size());
    for (const auto &jc : joints_) {
        targets.push_back(MotorActionExecutor::Target{jc.name, jc.canDevice, jc.protocol, jc.motorId});
    }
    lifecycleDriverOps_.setTargets(std::move(targets));
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

MotorActionExecutor::Target CanDriverHW::makeMotorTarget(const JointConfig &jc) const
{
    return MotorActionExecutor::Target{jc.name, jc.canDevice, jc.protocol, jc.motorId};
}

can_driver::OperationalCoordinator::Result CanDriverHW::initializeLifecycleDevice(
    const std::string &device,
    bool loopback)
{
    const auto result = lifecycleDriverOps_.initializeDevice(device, loopback);
    if (!result.ok) {
        return result;
    }
    active_.store(true, std::memory_order_release);
    stateTimer_.start();
    return result;
}

can_driver::OperationalCoordinator::Result CanDriverHW::shutdownLifecycleDriver(bool force)
{
    active_.store(false, std::memory_order_release);
    stateTimer_.stop();
    const auto result = lifecycleDriverOps_.shutdownAll(force);

    {
        std::lock_guard<std::mutex> stateLock(jointStateMutex_);
        for (auto &jc : joints_) {
            jc.hasDirectPosCmd = false;
            jc.hasDirectVelCmd = false;
            jc.stopIssuedOnFault = false;
        }
    }

    if (result.ok) {
        ROS_INFO("[CanDriverHW] All devices shut down.");
    }
    return result;
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

    if (lifecycleCoordinator_.mode() != can_driver::SystemOpMode::Running) {
        std::lock_guard<std::mutex> lock(jointStateMutex_);
        std::fill(commandValidBuffer_.begin(), commandValidBuffer_.end(), 0);
        return;
    }
    if (!commandGate_.consumeFreshCommandLatchIfSatisfied()) {
        std::lock_guard<std::mutex> lock(jointStateMutex_);
        std::fill(commandValidBuffer_.begin(), commandValidBuffer_.end(), 0);
        return;
    }

#if SOFTWARE_LOOPBACK_MODE
    // ========== 软件回环模式 ==========
    // 不发送 CAN 帧，命令值已经在 write() 被 ros_control 写入 posCmd/velCmd
    // read() 会直接读取这些值作为反馈
#else
    // ========== 真实 CAN 模式 ==========
    bool anyFaultObserved = false;

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
            bool hasCommand = true;
            if (jc.controlMode == "velocity") {
                if (jc.hasDirectVelCmd) {
                    if (debugBypassRosControl_) {
                        useDirect = true;
                        cmdValue = jc.directVelCmd;
                    } else {
                        const double age = (now - jc.lastDirectVelTime).toSec();
                        if (age <= directCmdTimeoutSec_) {
                            useDirect = true;
                            cmdValue = jc.directVelCmd;
                        } else {
                            jc.hasDirectVelCmd = false;
                        }
                    }
                }
                if (!useDirect) {
                    if (debugBypassRosControl_) {
                        hasCommand = false;
                    } else {
                        cmdValue = jc.velCmd;
                    }
                }
            } else {
                if (jc.hasDirectPosCmd) {
                    if (debugBypassRosControl_) {
                        useDirect = true;
                        cmdValue = jc.directPosCmd;
                    } else {
                        const double age = (now - jc.lastDirectPosTime).toSec();
                        if (age <= directCmdTimeoutSec_) {
                            useDirect = true;
                            cmdValue = jc.directPosCmd;
                        } else {
                            jc.hasDirectPosCmd = false;
                        }
                    }
                }
                if (!useDirect) {
                    if (debugBypassRosControl_) {
                        hasCommand = false;
                    } else {
                        cmdValue = jc.posCmd;
                    }
                }
            }

            if (!hasCommand) {
                commandValidBuffer_[i] = 0;
                continue;
            }

            cmdValue = clampWithJointLimits(jc, cmdValue);
            if (jc.controlMode == "position" && maxPositionStepRad_ > 0.0 &&
                std::isfinite(cmdValue) && std::isfinite(jc.pos)) {
                const double delta = cmdValue - jc.pos;
                if (std::fabs(delta) > maxPositionStepRad_) {
                    const double limitedCmd =
                        jc.pos + std::copysign(maxPositionStepRad_, delta);
                    ROS_WARN_THROTTLE(
                        1.0,
                        "[CanDriverHW] Joint '%s' position step limited: cur=%.6f, req=%.6f, limited=%.6f (max_step=%.6f).",
                        jc.name.c_str(),
                        jc.pos,
                        cmdValue,
                        limitedCmd,
                        maxPositionStepRad_);
                    cmdValue = limitedCmd;
                }
                cmdValue = clampWithJointLimits(jc, cmdValue);
            }

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
        const bool isReady = isDeviceReady(device);
        const auto deviceEvent = commandGate_.observeDeviceReady(device, isReady);

        if (!isReady) {
            if (deviceEvent == CommandGate::DeviceEvent::Lost) {
                ROS_ERROR_THROTTLE(
                    1.0,
                    "[CanDriverHW] Device '%s' not ready, clear direct commands and block motion.",
                    device.c_str());
            } else {
                ROS_WARN_THROTTLE(
                    1.0,
                    "[CanDriverHW] Device '%s' not ready, skip command write.",
                    device.c_str());
            }

            {
                std::lock_guard<std::mutex> lock(jointStateMutex_);
                for (const std::size_t idx : group.jointIndices) {
                    auto &jc = joints_[idx];
                    jc.hasDirectPosCmd = false;
                    jc.hasDirectVelCmd = false;
                    jc.stopIssuedOnFault = false;
                    if (jc.controlMode == "position") {
                        jc.posCmd = jc.pos;
                    } else {
                        jc.velCmd = 0.0;
                    }
                    commandValidBuffer_[idx] = 0;
                }
            }
            continue;
        }

        if (safetyHoldAfterDeviceRecover_ && deviceEvent == CommandGate::DeviceEvent::Recovered) {
            ROS_WARN_THROTTLE(
                1.0,
                "[CanDriverHW] Device '%s' recovered, hold one cycle and require fresh command.",
                device.c_str());
            commandGate_.holdCommands();
            commandGate_.armFreshCommandLatch();
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
                if (safetyStopOnFault_) {
                    const bool hasFault = proto->hasFault(jc.motorId);
                    bool needIssueStop = false;
                    {
                        std::lock_guard<std::mutex> lock(jointStateMutex_);
                        if (hasFault) {
                            if (!joints_[idx].stopIssuedOnFault) {
                                joints_[idx].stopIssuedOnFault = true;
                                needIssueStop = true;
                            }
                        } else {
                            joints_[idx].stopIssuedOnFault = false;
                        }
                    }

                    if (hasFault) {
                        anyFaultObserved = true;
                        if (needIssueStop) {
                            if (!proto->Stop(jc.motorId)) {
                                ROS_WARN_THROTTLE(
                                    1.0,
                                    "[CanDriverHW] Joint '%s' has fault, auto Stop rejected on '%s'.",
                                    jc.name.c_str(),
                                    device.c_str());
                            } else {
                                ROS_WARN_THROTTLE(
                                    1.0,
                                    "[CanDriverHW] Joint '%s' has fault, auto Stop sent and motion command blocked.",
                                    jc.name.c_str());
                            }
                        }
                        continue;
                    }
                }

                if (safetyRequireEnabledForMotion_ && !proto->isEnabled(jc.motorId)) {
                    ROS_WARN_THROTTLE(
                        1.0,
                        "[CanDriverHW] Joint '%s' is not enabled, motion command blocked.",
                        jc.name.c_str());
                    continue;
                }

                if (jc.controlMode == "velocity") {
                    if (!proto->setVelocity(jc.motorId, rawCommandBuffer_[idx])) {
                        ROS_WARN_THROTTLE(1.0,
                                          "[CanDriverHW] setVelocity rejected on '%s'.",
                                          device.c_str());
                    }
                } else if (jc.controlMode == "csp") {
                    if (!proto->quickSetPosition(jc.motorId, rawCommandBuffer_[idx])) {
                        ROS_WARN_THROTTLE(1.0,
                                          "[CanDriverHW] quickSetPosition rejected on '%s'.",
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
    lifecycleCoordinator_.UpdateFromFeedback(anyFaultObserved);
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
    return deviceManager_->initDevice(device, motors, loopback);
}

std::shared_ptr<CanProtocol> CanDriverHW::getProtocol(const std::string &device, CanType type) const
{
    return deviceManager_->getProtocol(device, type);
}

std::shared_ptr<std::mutex> CanDriverHW::getDeviceMutex(const std::string &device) const
{
    return deviceManager_->getDeviceMutex(device);
}

bool CanDriverHW::isDeviceReady(const std::string &device) const
{
    return deviceManager_->isDeviceReady(device);
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
    bool anyFault = false;
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
                anyFault = anyFault || statusSnapshots[i].fault;
            }

            msgs.push_back(msg);
        }
    }
    lifecycleCoordinator_.UpdateFromFeedback(anyFault);
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

    auto handleFailure = [&res](MotorActionExecutor::Status status, const char *rejectedMsg) {
        if (status == MotorActionExecutor::Status::DeviceNotReady) {
            res.success = false;
            res.message = "CAN device not ready.";
        } else if (status == MotorActionExecutor::Status::ProtocolUnavailable) {
            res.success = false;
            res.message = "Protocol not available.";
        } else if (status == MotorActionExecutor::Status::Rejected) {
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
        const auto status = motorActionExecutor_.execute(
            makeMotorTarget(*target),
            [&mode](const std::shared_ptr<CanProtocol> &proto, MotorID id) {
                return proto->setMode(id, mode);
            },
            "Set mode");
        if (status != MotorActionExecutor::Status::Ok) {
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
        const auto status = motorActionExecutor_.execute(makeMotorTarget(*target),
                                                         entry.action,
                                                         entry.name);
        if (status != MotorActionExecutor::Status::Ok) {
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

bool CanDriverHW::onSetZeroLimit(can_driver::SetZeroLimit::Request &req,
                                 can_driver::SetZeroLimit::Response &res)
{
    if (!active_.load(std::memory_order_acquire)) {
        res.success = false;
        res.message = "Driver inactive.";
        return true;
    }

    const JointConfig *target = findJointByMotorId(req.motor_id);
    if (!target) {
        res.success = false;
        res.message = "Motor ID not found.";
        return true;
    }
    if (target->controlMode != "position") {
        res.success = false;
        res.message = "Only position-control joints support zero/position limits.";
        return true;
    }

    double baseMin = req.min_position_rad;
    double baseMax = req.max_position_rad;
    if (req.use_urdf_limits) {
        urdf::Model urdf;
        if (!urdf.initParam("robot_description")) {
            res.success = false;
            res.message = "robot_description not found; cannot read URDF limits.";
            return true;
        }
        auto urdfJoint = urdf.getJoint(target->name);
        if (!urdfJoint) {
            res.success = false;
            res.message = "Joint not found in URDF: " + target->name;
            return true;
        }
        joint_limits_interface::JointLimits urdfLimits;
        if (!joint_limits_interface::getJointLimits(urdfJoint, urdfLimits) ||
            !urdfLimits.has_position_limits) {
            res.success = false;
            res.message = "URDF has no position limits for joint: " + target->name;
            return true;
        }
        baseMin = urdfLimits.min_position;
        baseMax = urdfLimits.max_position;
    }

    const double appliedMin = baseMin + req.zero_offset_rad;
    const double appliedMax = baseMax + req.zero_offset_rad;
    if (!std::isfinite(appliedMin) || !std::isfinite(appliedMax) || appliedMin >= appliedMax) {
        res.success = false;
        res.message = "Invalid limit range after zero offset.";
        return true;
    }

    auto proto = getProtocol(target->canDevice, target->protocol);
    auto devMutex = getDeviceMutex(target->canDevice);
    if (!proto || !devMutex) {
        res.success = false;
        res.message = "Protocol not available.";
        return true;
    }

    int64_t rawPos = 0;
    {
        std::lock_guard<std::mutex> devLock(*devMutex);
        // 连续读取几次，给协议层请求-返回留出时间。
        for (int i = 0; i < 5; ++i) {
            rawPos = proto->getPosition(target->motorId);
            std::this_thread::sleep_for(std::chrono::milliseconds(15));
        }
    }
    const double currentPosRad = static_cast<double>(rawPos) * target->positionScale;
    res.current_position_rad = currentPosRad;
    if (currentPosRad < appliedMin || currentPosRad > appliedMax) {
        res.success = false;
        res.applied_min_rad = appliedMin;
        res.applied_max_rad = appliedMax;
        res.message = "Current position is outside requested limit range.";
        return true;
    }

    if (req.apply_to_motor) {
        int32_t rawMin = 0;
        int32_t rawMax = 0;
        int32_t rawOffset = 0;
        if (!can_driver::safe_command::scaleAndClampToInt32(
                appliedMin, target->positionScale, target->name + ".min_limit", rawMin) ||
            !can_driver::safe_command::scaleAndClampToInt32(
                appliedMax, target->positionScale, target->name + ".max_limit", rawMax) ||
            !can_driver::safe_command::scaleAndClampToInt32(
                req.zero_offset_rad, target->positionScale, target->name + ".zero_offset", rawOffset)) {
            res.success = false;
            res.message = "Failed to convert limits/offset to protocol raw values.";
            return true;
        }

        const auto status = motorActionExecutor_.execute(
            makeMotorTarget(*target),
            [rawMin, rawMax, rawOffset](const std::shared_ptr<CanProtocol> &p, MotorID id) {
                const bool okOffset = p->setPositionOffset(id, rawOffset);
                const bool okLimits = p->configurePositionLimits(id, rawMin, rawMax, true);
                return okOffset && okLimits;
            },
            "Set zero/position limits");

        if (status != MotorActionExecutor::Status::Ok) {
            res.success = false;
            if (status == MotorActionExecutor::Status::DeviceNotReady) {
                res.message = "CAN device not ready.";
            } else if (status == MotorActionExecutor::Status::ProtocolUnavailable) {
                res.message = "Protocol not available.";
            } else if (status == MotorActionExecutor::Status::Rejected) {
                res.message = "Protocol rejected zero/limit settings (or unsupported by this protocol).";
            } else {
                res.message = "Set zero/limit execution failed.";
            }
            return true;
        }
    }

    {
        std::lock_guard<std::mutex> lock(jointStateMutex_);
        for (auto &jc : joints_) {
            if (static_cast<uint16_t>(jc.motorId) != req.motor_id) {
                continue;
            }
            jc.hasLimits = true;
            jc.limits.has_position_limits = true;
            jc.limits.min_position = appliedMin;
            jc.limits.max_position = appliedMax;
            jointZeroOffsetRadByMotorId_[req.motor_id] = req.zero_offset_rad;
            break;
        }
    }

    res.success = true;
    res.applied_min_rad = appliedMin;
    res.applied_max_rad = appliedMax;
    res.message = "OK";
    return true;
}
