cmake_minimum_required(VERSION 3.14)
project(robstride_control VERSION 0.1.0 LANGUAGES CXX)

# Default to C++17 (matches upstream RobStride_Control project).
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# --- Driver library ---
add_library(robstride_driver
  src/can_interface.cpp
  src/motor_driver.cpp
)
target_include_directories(robstride_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(robstride_driver PUBLIC cxx_std_17)

# --- ROS 2 node ---
add_executable(robstride_node src/robstride_node.cpp)
target_link_libraries(robstride_node robstride_driver)
ament_target_dependencies(robstride_node rclcpp sensor_msgs)

# --- Install ---
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS robstride_driver
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  TARGETS robstride_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(robstride_driver)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp sensor_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
