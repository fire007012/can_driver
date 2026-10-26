cmake_minimum_required(VERSION 3.16)
project(can_driver LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(robot_safety REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(urdf REQUIRED)
find_package(hardware_interface REQUIRED)

set(INNFOS_SDK_DIR "")
set(INNFOS_SDK_CANDIDATES
  "${CMAKE_CURRENT_SOURCE_DIR}/lib/innfos-cpp-sdk"
  "${CMAKE_CURRENT_SOURCE_DIR}/1MINTASCA/innfos-cpp-sdk"
)
foreach(candidate ${INNFOS_SDK_CANDIDATES})
  if(EXISTS "${candidate}")
    set(INNFOS_SDK_DIR "${candidate}")
    break()
  endif()
endforeach()
if(INNFOS_SDK_DIR STREQUAL "")
  message(FATAL_ERROR "Failed to locate innfos-cpp-sdk under lib/ or 1MINTASCA/.")
endif()

set(INNFOS_SDK_INCLUDE_DIR "${INNFOS_SDK_DIR}/sdk/include")
set(INNFOS_SDK_LIB_DIR "${INNFOS_SDK_DIR}/sdk/lib/linux_x86_64")
find_library(INNFOS_ACTUATOR_LIB
  NAMES ActuatorController
  PATHS ${INNFOS_SDK_LIB_DIR}
  NO_DEFAULT_PATH
  REQUIRED
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorState.msg"
  "srv/ApplyLimits.srv"
  "srv/Init.srv"
  "srv/Shutdown.srv"
  "srv/Recover.srv"
  "srv/MotorCommand.srv"
  "srv/SetZero.srv"
  "srv/SetZeroLimit.srv"
  "action/MoveMotor.action"
  "action/HomeMotor.action"
  DEPENDENCIES action_msgs std_msgs
)

add_library(can_driver_core SHARED
  src/CanDriverRuntime.cpp
  src/DeviceManager.cpp
  src/DeviceRuntime.cpp
  src/MtCan.cpp
  src/EyouCan.cpp
  src/DamiaoCan.cpp
  src/InnfosEcbProtocol.cpp
  src/command_gate.cpp
  src/lifecycle_driver_ops.cpp
  src/motor_action_executor.cpp
  src/operational_coordinator.cpp
  src/SocketCanController.cpp
  src/UdpCanTransport.cpp
  src/SafeCommand.cpp
)
target_include_directories(can_driver_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(can_driver_core PRIVATE ${INNFOS_SDK_INCLUDE_DIR})
target_link_libraries(can_driver_core ${INNFOS_ACTUATOR_LIB} pthread)
ament_target_dependencies(can_driver_core rclcpp)
set_target_properties(can_driver_core PROPERTIES
  BUILD_RPATH "${INNFOS_SDK_LIB_DIR}"
  INSTALL_RPATH "$ORIGIN"
)

add_library(can_driver_system SHARED src/can_driver_system.cpp)
target_include_directories(can_driver_system PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(can_driver_system can_driver_core)
ament_target_dependencies(can_driver_system robot_safety hardware_interface pluginlib rclcpp rclcpp_lifecycle)
pluginlib_export_plugin_description_file(hardware_interface can_driver_plugins.xml)

add_executable(can_driver_node src/can_driver_node.cpp)
target_include_directories(can_driver_node PRIVATE include)
target_link_libraries(can_driver_node can_driver_core yaml-cpp)
ament_target_dependencies(can_driver_node
  ament_index_cpp rclcpp rclcpp_action rclcpp_lifecycle std_msgs std_srvs yaml_cpp_vendor)
add_dependencies(can_driver_node ${PROJECT_NAME}__rosidl_generator_cpp)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(can_driver_node "${cpp_typesupport_target}")

install(FILES ${INNFOS_ACTUATOR_LIB} DESTINATION lib)

install(TARGETS can_driver_core can_driver_system can_driver_node
  EXPORT can_driver_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(EXPORT can_driver_targets
  FILE can_driver_targets.cmake
  NAMESPACE can_driver::
  DESTINATION share/${PROJECT_NAME}/cmake)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES can_driver_plugins.xml DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/realtime_motor_ui.py scripts/mt_motor_interface.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  foreach(protocol_test test_eyou_can_protocol test_eyou_can_csp test_damiao_can_protocol test_mt_can_protocol test_refresh_scheduler test_device_runtime test_limits_enforcement test_operational_coordinator)
    ament_add_gtest(${protocol_test} tests/${protocol_test}.cpp)
    target_link_libraries(${protocol_test} can_driver_core)
    target_include_directories(${protocol_test} PRIVATE include)
  endforeach()
  ament_add_gtest(test_grouped_system tests/test_grouped_system.cpp)
  target_link_libraries(test_grouped_system can_driver_system can_driver_core)
  target_include_directories(test_grouped_system PRIVATE include)
  ament_target_dependencies(test_grouped_system hardware_interface rclcpp robot_safety)
  ament_add_gtest(test_safe_command tests/test_safe_command.cpp)
  if(TARGET test_safe_command)
    target_link_libraries(test_safe_command can_driver_core)
    target_include_directories(test_safe_command PRIVATE include)
  endif()
endif()

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime hardware_interface pluginlib robot_safety)
ament_package()
