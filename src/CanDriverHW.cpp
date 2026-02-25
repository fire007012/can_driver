#include "can_driver/CanDriverHW.h"

#include <ros/ros.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <stdexcept>
#include <string>

// ---------------------------------------------------------------------------
// 析构
// ---------------------------------------------------------------------------
CanDriverHW::~CanDriverHW()
{
    // 协议实例析构时会停止刷新线程，transport 析构时调用 shutdown()
    // 清空顺序：先协议层，再传输层
    mtProtocols_.clear();
    eyouProtocols_.clear();
    transports_.clear();
}

// ---------------------------------------------------------------------------
// init
// ---------------------------------------------------------------------------
bool CanDriverHW::init(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
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

        // motor_id: YAML 中用十六进制字符串（"0x141"）或整数均可
        int rawId = 0;
        if (jv["motor_id"].getType() == XmlRpc::XmlRpcValue::TypeInt) {
            rawId = static_cast<int>(jv["motor_id"]);
        } else {
            // 字符串形式 "0x141"
            rawId = static_cast<int>(
                std::stoul(static_cast<std::string>(jv["motor_id"]), nullptr, 0));
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

        // --- 按 can_device 按需创建传输和协议实例 ---
        if (transports_.find(jc.canDevice) == transports_.end()) {
            auto transport = std::make_shared<SocketCanController>();
            if (!transport->initialize(jc.canDevice)) {
                ROS_ERROR("[CanDriverHW] Failed to initialize CAN device '%s'.",
                          jc.canDevice.c_str());
                return false;
            }
            transports_[jc.canDevice] = transport;
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

    // --- ROS Services ---
    initSrv_     = nh.advertiseService("init",          &CanDriverHW::onInit,         this);
    shutdownSrv_ = nh.advertiseService("shutdown",      &CanDriverHW::onShutdown,     this);
    recoverSrv_  = nh.advertiseService("recover",       &CanDriverHW::onRecover,      this);
    motorCmdSrv_ = nh.advertiseService("motor_command", &CanDriverHW::onMotorCommand, this);

    // --- 直接命令 subscribers（per joint） ---
    for (auto &jc : joints_) {
        const std::string velTopic = "motor/" + jc.name + "/cmd_velocity";
        const std::string posTopic = "motor/" + jc.name + "/cmd_position";

        // 捕获 motorId/device/protocol 供 lambda 使用
        MotorID    mid   = jc.motorId;
        CanType    proto = jc.protocol;
        std::string dev  = jc.canDevice;

        cmdVelSubs_[jc.name] = nh.subscribe<std_msgs::Float64>(
            velTopic, 1,
            [this, mid, proto, dev](const std_msgs::Float64::ConstPtr &msg) {
                auto *p = getProtocol(dev, proto);
                if (p) p->setVelocity(mid, static_cast<int32_t>(msg->data));
            });

        cmdPosSubs_[jc.name] = nh.subscribe<std_msgs::Float64>(
            posTopic, 1,
            [this, mid, proto, dev](const std_msgs::Float64::ConstPtr &msg) {
                auto *p = getProtocol(dev, proto);
                if (p) p->setPosition(mid, static_cast<int32_t>(msg->data));
            });
    }

    // --- 电机状态发布定时器（10 Hz） ---
    motorStatesPub_ = nh.advertise<can_driver::MotorState>("motor_states", 10);
    stateTimer_ = nh.createTimer(ros::Duration(0.1),
                                 &CanDriverHW::publishMotorStates, this);

    ROS_INFO("[CanDriverHW] Initialized with %zu joints on %zu CAN device(s).",
             joints_.size(), transports_.size());
    return true;
}

// ---------------------------------------------------------------------------
// read
// ---------------------------------------------------------------------------
void CanDriverHW::read(const ros::Time & /*time*/, const ros::Duration & /*period*/)
{
    for (auto &jc : joints_) {
        auto *proto = getProtocol(jc.canDevice, jc.protocol);
        if (!proto) continue;

        jc.pos = static_cast<double>(proto->getPosition(jc.motorId));
        jc.vel = static_cast<double>(proto->getVelocity(jc.motorId));
        jc.eff = static_cast<double>(proto->getCurrent(jc.motorId));
    }
}

// ---------------------------------------------------------------------------
// write
// ---------------------------------------------------------------------------
void CanDriverHW::write(const ros::Time & /*time*/, const ros::Duration & /*period*/)
{
    for (auto &jc : joints_) {
        auto *proto = getProtocol(jc.canDevice, jc.protocol);
        if (!proto) continue;

        if (jc.controlMode == "velocity") {
            proto->setVelocity(jc.motorId, static_cast<int32_t>(jc.velCmd));
        } else {
            proto->setPosition(jc.motorId, static_cast<int32_t>(jc.posCmd));
        }
    }
}

// ---------------------------------------------------------------------------
// 内部辅助
// ---------------------------------------------------------------------------
bool CanDriverHW::initDevice(const std::string &device, bool loopback)
{
    auto it = transports_.find(device);
    if (it != transports_.end()) {
        // 已存在：重新初始化
        it->second->shutdown();
        if (!it->second->initialize(device, loopback)) {
            ROS_ERROR("[CanDriverHW] Re-init of '%s' failed.", device.c_str());
            return false;
        }
        ROS_INFO("[CanDriverHW] Re-initialized '%s'.", device.c_str());
        return true;
    }

    // 不存在：新建（通常不走这路，init() 时已全部创建）
    auto transport = std::make_shared<SocketCanController>();
    if (!transport->initialize(device, loopback)) {
        ROS_ERROR("[CanDriverHW] Failed to init '%s'.", device.c_str());
        return false;
    }
    transports_[device] = transport;
    ROS_INFO("[CanDriverHW] Opened '%s' (on-demand).", device.c_str());
    return true;
}

CanProtocol *CanDriverHW::getProtocol(const std::string &device, CanType type)
{
    if (type == CanType::MT) {
        auto it = mtProtocols_.find(device);
        return (it != mtProtocols_.end()) ? it->second.get() : nullptr;
    } else {
        auto it = eyouProtocols_.find(device);
        return (it != eyouProtocols_.end()) ? it->second.get() : nullptr;
    }
}

void CanDriverHW::publishMotorStates(const ros::TimerEvent & /*e*/)
{
    for (const auto &jc : joints_) {
        can_driver::MotorState msg;
        msg.motor_id = static_cast<uint16_t>(jc.motorId);
        msg.name     = jc.name;
        msg.position = static_cast<int32_t>(jc.pos);
        msg.velocity = static_cast<int16_t>(jc.vel);
        msg.current  = static_cast<int16_t>(jc.eff);

        if (jc.controlMode == "velocity")
            msg.mode = can_driver::MotorState::MODE_VELOCITY;
        else
            msg.mode = can_driver::MotorState::MODE_POSITION;

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
    res.message = res.success ? "OK" : "Failed to initialize " + req.device;
    return true;
}

bool CanDriverHW::onShutdown(can_driver::Shutdown::Request & /*req*/,
                              can_driver::Shutdown::Response &res)
{
    mtProtocols_.clear();
    eyouProtocols_.clear();
    for (auto &kv : transports_) {
        kv.second->shutdown();
    }
    transports_.clear();
    res.success = true;
    res.message = "All CAN devices shut down.";
    ROS_INFO("[CanDriverHW] All devices shut down.");
    return true;
}

bool CanDriverHW::onRecover(can_driver::Recover::Request &req,
                             can_driver::Recover::Response &res)
{
    // 简单实现：对匹配的电机重新使能
    bool found = false;
    for (const auto &jc : joints_) {
        if (req.motor_id == 0 ||
            static_cast<uint16_t>(jc.motorId) == req.motor_id) {
            auto *proto = getProtocol(jc.canDevice, jc.protocol);
            if (proto) {
                proto->Enable(jc.motorId);
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
    for (const auto &jc : joints_) {
        if (static_cast<uint16_t>(jc.motorId) != req.motor_id) continue;

        auto *proto = getProtocol(jc.canDevice, jc.protocol);
        if (!proto) {
            res.success = false;
            res.message = "Protocol not available.";
            return true;
        }

        switch (req.command) {
        case can_driver::MotorCommand::Request::CMD_ENABLE:
            proto->Enable(jc.motorId);
            break;
        case can_driver::MotorCommand::Request::CMD_DISABLE:
            proto->Disable(jc.motorId);
            break;
        case can_driver::MotorCommand::Request::CMD_STOP:
            proto->Stop(jc.motorId);
            break;
        case can_driver::MotorCommand::Request::CMD_SET_MODE: {
            auto mode = (req.value == 0.0)
                            ? CanProtocol::MotorMode::Position
                            : CanProtocol::MotorMode::Velocity;
            proto->setMode(jc.motorId, mode);
            break;
        }
        default:
            res.success = false;
            res.message = "Unknown command.";
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
