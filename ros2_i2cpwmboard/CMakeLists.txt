find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

find_package(rosidl_default_generators REQUIRED)

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

add_executable(i2cpwm_controller src/i2cpwm_controller.cpp)
ament_target_dependencies(i2cpwm_controller rclcpp std_msgs)
target_link_libraries(i2cpwm_controller i2c) # Link libi2c
rosidl_target_interfaces(i2cpwm_controller
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  i2cpwm_controller
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

ament_package()
