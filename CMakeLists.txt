cmake_minimum_required(VERSION 3.20)
project(raspimouse_ros2_examples)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# ROS 2 Lyrical では C++20 が標準
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

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
find_package(cv_bridge REQUIRED)

# --- ターゲット定義 ---

# 1. object_tracking_component
add_library(object_tracking_component SHARED
  src/object_tracking_component.cpp)
target_compile_definitions(object_tracking_component
  PRIVATE "RASPIMOUSE_ROS2_EXAMPLES_BUILDING_DLL")
target_include_directories(object_tracking_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(object_tracking_component PUBLIC
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_lifecycle::rclcpp_lifecycle
  std_msgs::std_msgs__rosidl_typesupport_cpp
  std_srvs::std_srvs__rosidl_typesupport_cpp
  sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
  geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
  cv_bridge::cv_bridge
  ${OpenCV_LIBS})
rclcpp_components_register_nodes(object_tracking_component "object_tracking::Tracker")

# 2. camera_line_follower_component
add_library(camera_line_follower_component SHARED
  src/camera_line_follower_component.cpp)
target_compile_definitions(camera_line_follower_component
  PRIVATE "RASPIMOUSE_ROS2_EXAMPLES_BUILDING_DLL")
target_include_directories(camera_line_follower_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(camera_line_follower_component PUBLIC
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_lifecycle::rclcpp_lifecycle
  std_msgs::std_msgs__rosidl_typesupport_cpp
  std_srvs::std_srvs__rosidl_typesupport_cpp
  sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
  geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
  cv_bridge::cv_bridge
  raspimouse_msgs::raspimouse_msgs__rosidl_typesupport_cpp
  ${OpenCV_LIBS})
rclcpp_components_register_nodes(camera_line_follower_component "camera_line_follower::CameraFollower")

# 3. line_follower_component
add_library(line_follower_component SHARED
  src/line_follower_component.cpp)
target_compile_definitions(line_follower_component
  PRIVATE "RASPIMOUSE_ROS2_EXAMPLES_BUILDING_DLL")
target_include_directories(line_follower_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(line_follower_component PUBLIC
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_lifecycle::rclcpp_lifecycle
  std_msgs::std_msgs__rosidl_typesupport_cpp
  std_srvs::std_srvs__rosidl_typesupport_cpp
  geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
  raspimouse_msgs::raspimouse_msgs__rosidl_typesupport_cpp)
rclcpp_components_register_nodes(line_follower_component "line_follower::Follower")

# 4. direction_controller_component
add_library(direction_controller_component SHARED
  src/direction_controller_component.cpp)
target_compile_definitions(direction_controller_component
  PRIVATE "RASPIMOUSE_ROS2_EXAMPLES_BUILDING_DLL")
target_include_directories(direction_controller_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(direction_controller_component PUBLIC
  rclcpp::rclcpp
  rclcpp_components::component
  std_msgs::std_msgs__rosidl_typesupport_cpp
  sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
  std_srvs::std_srvs__rosidl_typesupport_cpp
  geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
  raspimouse_msgs::raspimouse_msgs__rosidl_typesupport_cpp)
rclcpp_components_register_nodes(direction_controller_component "direction_controller::Controller")

# Executables
add_executable(lifecycle_node_manager src/lifecycle_node_manager.cpp)
target_include_directories(lifecycle_node_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(lifecycle_node_manager PUBLIC
  rclcpp::rclcpp
  lifecycle_msgs::lifecycle_msgs__rosidl_typesupport_cpp)

add_executable(line_follower src/line_follower.cpp)
target_include_directories(line_follower PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(line_follower PUBLIC
  line_follower_component
  rclcpp::rclcpp
  raspimouse::raspimouse_component)

add_executable(direction_controller src/direction_controller.cpp)
target_include_directories(direction_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(direction_controller PUBLIC
  direction_controller_component
  rclcpp::rclcpp)

# --- エクスポートとインストール設定 ---

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
  camera_line_follower_component
  line_follower_component
  direction_controller_component)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  object_tracking_component
  camera_line_follower_component
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