#include "can_driver/lifecycle_service_gateway.hpp"

#include "can_driver/operational_coordinator.hpp"

namespace {

constexpr uint16_t kGlobalRecoverMotorId = 0xFFFFu;

template <typename Response>
bool ensureGatewayReady(can_driver::OperationalCoordinator *coordinator, Response &res)
{
    if (coordinator != nullptr) {
        return true;
    }

    res.success = false;
    res.message = "lifecycle gateway not initialized";
    return false;
}

} // namespace

LifecycleServiceGateway::LifecycleServiceGateway(
    ros::NodeHandle &pnh,
    can_driver::OperationalCoordinator *coordinator)
{
    initialize(pnh, coordinator);
}

void LifecycleServiceGateway::initialize(
    ros::NodeHandle &pnh,
    can_driver::OperationalCoordinator *coordinator)
{
    coordinator_ = coordinator;
    initSrv_ = pnh.advertiseService("init", &LifecycleServiceGateway::onInit, this);
    shutdownSrv_ = pnh.advertiseService("shutdown", &LifecycleServiceGateway::onShutdown, this);
    recoverSrv_ = pnh.advertiseService("recover", &LifecycleServiceGateway::onRecover, this);
    enableSrv_ = pnh.advertiseService("enable", &LifecycleServiceGateway::onEnable, this);
    disableSrv_ = pnh.advertiseService("disable", &LifecycleServiceGateway::onDisable, this);
    haltSrv_ = pnh.advertiseService("halt", &LifecycleServiceGateway::onHalt, this);
    resumeSrv_ = pnh.advertiseService("resume", &LifecycleServiceGateway::onResume, this);
}

bool LifecycleServiceGateway::onInit(can_driver::Init::Request &req,
                                     can_driver::Init::Response &res)
{
    if (!ensureGatewayReady(coordinator_, res)) {
        return true;
    }
    const auto result = coordinator_->RequestInit(req.device, req.loopback);
    res.success = result.ok;
    res.message = result.ok
                      ? (result.message.empty() ? "initialized (armed)" : result.message)
                      : (result.message.empty() ? "init failed" : result.message);
    return true;
}

bool LifecycleServiceGateway::onShutdown(can_driver::Shutdown::Request &req,
                                         can_driver::Shutdown::Response &res)
{
    if (!ensureGatewayReady(coordinator_, res)) {
        return true;
    }
    const auto result = coordinator_->RequestShutdown(req.force);
    res.success = result.ok;
    res.message = result.ok
                      ? (result.message.empty() ? "communication stopped; call ~/init first"
                                                : result.message)
                      : (result.message.empty() ? "shutdown failed" : result.message);
    return true;
}

bool LifecycleServiceGateway::onRecover(can_driver::Recover::Request &req,
                                        can_driver::Recover::Response &res)
{
    if (!ensureGatewayReady(coordinator_, res)) {
        return true;
    }
    if (req.motor_id != kGlobalRecoverMotorId) {
        res.success = false;
        res.message = "per-motor recover has been removed; use motor_id=65535 for global recover";
        return true;
    }
    const auto result = coordinator_->RequestRecover();
    res.success = result.ok;
    res.message = result.ok
                      ? (result.message.empty() ? "recovered (standby)" : result.message)
                      : (result.message.empty() ? "recover failed" : result.message);
    return true;
}

bool LifecycleServiceGateway::onEnable(std_srvs::Trigger::Request &req,
                                       std_srvs::Trigger::Response &res)
{
    if (!ensureGatewayReady(coordinator_, res)) {
        return true;
    }
    const auto result = coordinator_->RequestEnable();
    res.success = result.ok;
    res.message = result.ok
                      ? (result.message.empty() ? "enabled (armed)" : result.message)
                      : (result.message.empty() ? "enable failed" : result.message);
    return true;
}

bool LifecycleServiceGateway::onDisable(std_srvs::Trigger::Request &req,
                                        std_srvs::Trigger::Response &res)
{
    if (!ensureGatewayReady(coordinator_, res)) {
        return true;
    }
    const auto result = coordinator_->RequestDisable();
    res.success = result.ok;
    res.message = result.ok
                      ? (result.message.empty() ? "disabled (standby)" : result.message)
                      : (result.message.empty() ? "disable failed" : result.message);
    return true;
}

bool LifecycleServiceGateway::onHalt(std_srvs::Trigger::Request &req,
                                     std_srvs::Trigger::Response &res)
{
    if (!ensureGatewayReady(coordinator_, res)) {
        return true;
    }
    const auto result = coordinator_->RequestHalt();
    res.success = result.ok;
    res.message = result.ok
                      ? (result.message.empty() ? "halted" : result.message)
                      : (result.message.empty() ? "halt failed" : result.message);
    return true;
}

bool LifecycleServiceGateway::onResume(std_srvs::Trigger::Request &req,
                                       std_srvs::Trigger::Response &res)
{
    if (!ensureGatewayReady(coordinator_, res)) {
        return true;
    }
    const auto result = coordinator_->RequestRelease();
    res.success = result.ok;
    res.message = result.ok
                      ? (result.message.empty() ? "resumed" : result.message)
                      : (result.message.empty() ? "resume failed" : result.message);
    return true;
}
