cmake_minimum_required(VERSION 3.5)
project(ros2_i2cpwmboard)

# Finde Abhängigkeiten
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_i2cpwmboard REQUIRED)

# Generiere Interfaces für Nachrichten und Services
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

# Finde alle Quelldateien
file(GLOB_RECURSE SOURCE_FILES
  src/*.cpp
)

# Definiere das ausführbare Programm
add_executable(i2cpwm_controller
  ${SOURCE_FILES}
)

# Setze die Include-Verzeichnisse
target_include_directories(i2cpwm_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Linke die Abhängigkeiten
ament_target_dependencies(i2cpwm_controller rclcpp ros2_i2cpwmboard)

# Installiere das ausführbare Programm
install(TARGETS i2cpwm_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Installiere Header-Dateien
install(DIRECTORY include/
  DESTINATION include
)

ament_package()