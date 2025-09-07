cmake_minimum_required(VERSION 3.16)
project(ros2_i2cpwmboard)

# Core
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Interfaces
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# ---- Interface-Generierung ----
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Position.msg"
  "msg/PositionArray.msg"
  "msg/Servo.msg"
  "msg/ServoArray.msg"
  "msg/ServoConfig.msg"
  "msg/ServoConfigArray.msg"
  "srv/DriveMode.srv"
  "srv/IntValue.srv"
  "srv/ServosConfig.srv"
  "srv/StopServos.srv"
  DEPENDENCIES builtin_interfaces std_msgs
)

# ---- Node ----
add_executable(i2cpwm_controller src/i2cpwm_controller.cpp)
target_include_directories(i2cpwm_controller PUBLIC include)
ament_target_dependencies(i2cpwm_controller rclcpp std_msgs)

# Modern: Typesupport verlinken (statt deprecated rosidl_target_interfaces)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(i2cpwm_controller "${cpp_typesupport_target}")

# ---- Install ----
install(TARGETS i2cpwm_controller
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
