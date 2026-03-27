#include "can_driver/lifecycle_service_gateway.hpp"

#include "can_driver/CanDriverHW.h"

namespace {

template <typename Response>
bool ensureGatewayReady(CanDriverHW *hw, Response &res)
{
    if (hw != nullptr) {
        return true;
    }

    res.success = false;
    res.message = "lifecycle gateway not initialized";
    return false;
}

} // namespace

LifecycleServiceGateway::LifecycleServiceGateway(ros::NodeHandle &pnh, CanDriverHW *hw)
{
    initialize(pnh, hw);
}

void LifecycleServiceGateway::initialize(ros::NodeHandle &pnh, CanDriverHW *hw)
{
    hw_ = hw;
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
    if (!ensureGatewayReady(hw_, res)) {
        return true;
    }
    return hw_->handleInit(req, res);
}

bool LifecycleServiceGateway::onShutdown(can_driver::Shutdown::Request &req,
                                         can_driver::Shutdown::Response &res)
{
    if (!ensureGatewayReady(hw_, res)) {
        return true;
    }
    return hw_->handleShutdown(req, res);
}

bool LifecycleServiceGateway::onRecover(can_driver::Recover::Request &req,
                                        can_driver::Recover::Response &res)
{
    if (!ensureGatewayReady(hw_, res)) {
        return true;
    }
    return hw_->handleRecover(req, res);
}

bool LifecycleServiceGateway::onEnable(std_srvs::Trigger::Request &req,
                                       std_srvs::Trigger::Response &res)
{
    if (!ensureGatewayReady(hw_, res)) {
        return true;
    }
    return hw_->handleEnable(req, res);
}

bool LifecycleServiceGateway::onDisable(std_srvs::Trigger::Request &req,
                                        std_srvs::Trigger::Response &res)
{
    if (!ensureGatewayReady(hw_, res)) {
        return true;
    }
    return hw_->handleDisable(req, res);
}

bool LifecycleServiceGateway::onHalt(std_srvs::Trigger::Request &req,
                                     std_srvs::Trigger::Response &res)
{
    if (!ensureGatewayReady(hw_, res)) {
        return true;
    }
    return hw_->handleHalt(req, res);
}

bool LifecycleServiceGateway::onResume(std_srvs::Trigger::Request &req,
                                       std_srvs::Trigger::Response &res)
{
    if (!ensureGatewayReady(hw_, res)) {
        return true;
    }
    return hw_->handleResume(req, res);
}
