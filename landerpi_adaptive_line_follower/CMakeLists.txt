cmake_minimum_required(VERSION 3.8)
project(landerpi_adaptive_line_follower)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(servo_controller_msgs REQUIRED)

# Pure logic, no rclcpp dependency: kept as their own targets so they can be
# gtest-ed without bringing up rclcpp.
add_library(line_tracker src/line_tracker.cpp)
target_include_directories(line_tracker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS})
target_link_libraries(line_tracker PUBLIC ${OpenCV_LIBS})

add_library(pid src/pid.cpp)
target_include_directories(pid PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(zone_monitor src/zone_monitor.cpp)
target_include_directories(zone_monitor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(line_follower_node_lib src/line_follower_node.cpp)
target_include_directories(line_follower_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(line_follower_node_lib line_tracker pid zone_monitor)
ament_target_dependencies(line_follower_node_lib
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_srvs
  interfaces
  cv_bridge
  servo_controller_msgs)

add_executable(line_follower_node src/line_follower_node_main.cpp)
target_link_libraries(line_follower_node line_follower_node_lib)
ament_target_dependencies(line_follower_node rclcpp lifecycle_msgs)

install(TARGETS line_tracker pid zone_monitor line_follower_node_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS line_follower_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/line_color_calibration.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_line_tracker test/test_line_tracker.cpp)
  target_link_libraries(test_line_tracker line_tracker)

  ament_add_gtest(test_pid test/test_pid.cpp)
  target_link_libraries(test_pid pid)

  ament_add_gtest(test_zone_monitor test/test_zone_monitor.cpp)
  target_link_libraries(test_zone_monitor zone_monitor)
endif()

ament_package()
