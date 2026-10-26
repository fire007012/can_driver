#ifndef CAN_DRIVER_CAN_DRIVER_SYSTEM_HPP
#define CAN_DRIVER_CAN_DRIVER_SYSTEM_HPP

#include "can_driver/CanDriverHwTypes.h"
#include "can_driver/DeviceManager.h"

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

#include <memory>
#include <limits>
#include "robot_safety/domain.hpp"
#include "robot_safety/runtime.hpp"
#include "robot_safety/axis_session.hpp"
#include "robot_safety/freshness.hpp"
#include "robot_safety/parameters.hpp"
#include <functional>
#include <string>
#include <vector>

namespace can_driver {

class CanDriverSystem final : public hardware_interface::SystemInterface {
public:
    CanDriverSystem() = default;
    explicit CanDriverSystem(std::shared_ptr<IDeviceManager> manager) : deviceManager_(std::move(manager)) {}
    ~CanDriverSystem() override { backendServices_.reset(); shutdownDevices(); }
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo &info) override;
    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State &previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type read(const rclcpp::Time &time,
                                          const rclcpp::Duration &period) override;
    hardware_interface::return_type write(const rclcpp::Time &time,
                                           const rclcpp::Duration &period) override;

    hardware_interface::return_type perform_command_mode_switch(
        const std::vector<std::string> &start, const std::vector<std::string> &stop) override;
    hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &) override
    { shutdownDevices(); return hardware_interface::CallbackReturn::SUCCESS; }
private:
    void stopAll();
    void stopGroup(const std::string & group);
    bool prepareAxis(std::size_t index);
    void prepareGroup(const std::string & group);
    bool grouped_ = false;
    std::shared_ptr<robot_safety::Runtime> guard_, backendServices_;
    bool fail(const std::string &reason);
    std::shared_ptr<robot_safety::Domain> safety_;
    uint64_t safetyEpoch_ = 0;
    std::atomic<bool> active_{false};
    bool permissionLost_ = false;
    struct Axis {
        CanDriverJointConfig config;
        robot_safety::AxisSession session;
        robot_safety::Freshness::Thresholds feedbackLimits{250000000,500000000,100000000};
        std::array<robot_safety::Freshness::Thresholds,5> fieldLimits{};
        std::map<std::size_t,robot_safety::Freshness> fieldHealth;
        uint64_t observedPreparation = UINT64_MAX;
        double position{0.0};
        double velocity{0.0};
        double effort{0.0};
        double positionCommand{std::numeric_limits<double>::quiet_NaN()};
        double velocityCommand{std::numeric_limits<double>::quiet_NaN()};
        bool configured{false};
        bool claimed{false};
        bool commandReceived{false};
        double lower = -std::numeric_limits<double>::infinity(), upper = std::numeric_limits<double>::infinity();
        double velocityLimit = std::numeric_limits<double>::infinity();
    };

    static bool parseMotorId(const std::string &value, MotorID *out);
    static CanType parseProtocol(const std::string &value);
    static std::string parameter(const hardware_interface::ComponentInfo &joint,
                                 const std::string &name,
                                 const std::string &fallback = {});
    bool initDevices();
    void shutdownDevices();

    std::shared_ptr<IDeviceManager> deviceManager_;
    std::vector<Axis> axes_;
    std::vector<std::string> stateInterfaceNames_;
    std::vector<std::string> commandInterfaceNames_;
    bool devicesInitialized_{false};
    std::map<std::string,uint64_t> transportErrors_;
};

} // namespace can_driver

#endif // CAN_DRIVER_CAN_DRIVER_SYSTEM_HPP
