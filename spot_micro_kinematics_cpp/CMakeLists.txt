cmake_minimum_required(VERSION 3.16)
project(spot_micro_kinematics)

# C++ Standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ROS 2 / Ament
find_package(ament_cmake REQUIRED)

# Abhängigkeiten
find_package(Eigen3 REQUIRED NO_MODULE)

# ---- Bibliothek ----
add_library(${PROJECT_NAME}
  src/spot_micro_kinematics.cpp
  src/spot_micro_leg.cpp
  src/utils.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
)

# Export für Downstream-Pakete
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)

# ---- Installation ----
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

# ---- Tests (optional, nur wenn vorhanden) ----
# ament_cmake_gtest ist die ROS2-Integration für GoogleTest
find_package(ament_cmake_gtest QUIET)

if(ament_cmake_gtest_FOUND)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_utils.cpp")
    ament_add_gtest(test_utils tests/test_utils.cpp)
    target_link_libraries(test_utils ${PROJECT_NAME})
  endif()

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_spot_micro_leg.cpp")
    ament_add_gtest(test_spot_micro_leg tests/test_spot_micro_leg.cpp)
    target_link_libraries(test_spot_micro_leg ${PROJECT_NAME})
  endif()

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_spot_micro_kinematics.cpp")
    ament_add_gtest(test_spot_micro_kinematics tests/test_spot_micro_kinematics.cpp)
    target_link_libraries(test_spot_micro_kinematics ${PROJECT_NAME})
  endif()
endif()

ament_package()
