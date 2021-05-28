cmake_minimum_required(VERSION 3.5)
project(raspimouse_ros2_examples)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(raspimouse_msgs REQUIRED)
find_package(raspimouse REQUIRED)
find_package(rt_usb_9axisimu_driver REQUIRED)


include_directories(include)

add_library(object_tracking_component SHARED
  src/object_tracking_component.cpp)
target_compile_definitions(object_tracking_component
  PRIVATE "RASPIMOUSE_ROS2_EXAMPLES_BUILDING_DLL")
ament_target_dependencies(object_tracking_component
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  OpenCV)
rclcpp_components_register_nodes(object_tracking_component "object_tracking::Tracker")

add_library(line_follower_component SHARED
  src/line_follower_component.cpp)
target_compile_definitions(line_follower_component
  PRIVATE "RASPIMOUSE_ROS2_EXAMPLES_BUILDING_DLL")
ament_target_dependencies(line_follower_component
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  std_msgs
  std_srvs
  geometry_msgs
  raspimouse_msgs)
rclcpp_components_register_nodes(line_follower_component "line_follower::Follower")

add_library(direction_controller_component SHARED
  src/direction_controller_component.cpp)
target_compile_definitions(direction_controller_component
  PRIVATE "RASPIMOUSE_ROS2_EXAMPLES_BUILDING_DLL")
ament_target_dependencies(direction_controller_component
  rclcpp
  rclcpp_components
  std_msgs
  std_srvs
  geometry_msgs
  raspimouse_msgs)
rclcpp_components_register_nodes(direction_controller_component "direction_controller::Controller")

add_executable(lifecycle_node_manager src/lifecycle_node_manager.cpp)
ament_target_dependencies(lifecycle_node_manager
  rclcpp
  lifecycle_msgs)

add_executable(line_follower src/line_follower.cpp)
target_link_libraries(line_follower line_follower_component)
ament_target_dependencies(line_follower
  rclcpp
  raspimouse)

add_executable(direction_controller src/direction_controller.cpp)
target_link_libraries(direction_controller direction_controller_component)
ament_target_dependencies(direction_controller
  rclcpp
  raspimouse
  rt_usb_9axisimu_driver)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(rclcpp)
ament_export_dependencies(rclcpp_components)
ament_export_dependencies(rclcpp_lifecycle)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(std_msgs)
ament_export_dependencies(std_srvs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(raspimouse_msgs)
ament_export_dependencies(OpenCV)

ament_export_include_directories(include)
ament_export_libraries(
  object_tracking_component
  line_follower_component
  direction_controller_component)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  object_tracking_component
  line_follower_component
  direction_controller_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

install(TARGETS
  lifecycle_node_manager
  line_follower
  direction_controller
  DESTINATION lib/${PROJECT_NAME})

# python scripts
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
    ${PROJECT_NAME}/joystick_control.py
    DESTINATION lib/${PROJECT_NAME})

# launch files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
