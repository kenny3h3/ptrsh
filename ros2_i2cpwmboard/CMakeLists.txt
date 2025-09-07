cmake_minimum_required(VERSION 3.5)
project(ros2_i2cpwmboard)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate interfaces
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

# Add executable
add_executable(i2cpwm_controller src/i2cpwm_controller.cpp)
ament_target_dependencies(i2cpwm_controller rclcpp std_msgs)
target_link_libraries(i2cpwm_controller i2c) # Link libi2c (ensure libi2c-dev is installed)

# Link ROS 2 interfaces
rosidl_target_interfaces(i2cpwm_controller
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# Install targets
install(TARGETS
  i2cpwm_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
