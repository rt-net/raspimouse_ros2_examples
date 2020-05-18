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
find_package(sensor_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(OpenCV REQUIRED)


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
  sensor_msgs
  OpenCV)
rclcpp_components_register_nodes(object_tracking_component "object_tracking::Tracker")

add_executable(object_tracking src/object_tracking.cpp)
target_link_libraries(object_tracking object_tracking_component)
ament_target_dependencies(object_tracking
  rclcpp
  std_msgs
  lifecycle_msgs)

install(TARGETS
  object_tracking_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS
  object_tracking
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
