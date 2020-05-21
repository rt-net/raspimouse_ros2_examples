// Copyright 2020 RT Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RASPIMOUSE_ROS2_EXAMPLES__OBJECT_TRACKING_COMPONENT_HPP_
#define RASPIMOUSE_ROS2_EXAMPLES__OBJECT_TRACKING_COMPONENT_HPP_

#include <memory>
#include <string>

#include "raspimouse_ros2_examples/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "opencv2/highgui/highgui.hpp"

namespace object_tracking
{

class Tracker : public rclcpp_lifecycle::LifecycleNode
{
public:
  RASPIMOUSE_ROS2_EXAMPLES_PUBLIC
  explicit Tracker(const rclcpp::NodeOptions & options);

protected:
  void on_image_timer();
  void on_cmd_vel_timer();

private:
  cv::VideoCapture cap_;
  int device_index_;
  double image_width_;
  double image_height_;
  bool object_is_detected_;
  cv::Point2d object_normalized_point_;
  double object_normalized_area_;
  geometry_msgs::msg::Twist cmd_vel_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>> image_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>> result_image_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>> cmd_vel_pub_;
  std::shared_ptr<rclcpp::Client<std_srvs::srv::SetBool>> motor_power_client_;
  rclcpp::TimerBase::SharedPtr image_timer_;
  rclcpp::TimerBase::SharedPtr cmd_vel_timer_;

  std::string mat_type2encoding(int mat_type);
  void convert_frame_to_message(
    const cv::Mat & frame,
    sensor_msgs::msg::Image & msg);

  void tracking(const cv::Mat & input_frame, cv::Mat & result_frame);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &);
};

}  // namespace object_tracking

#endif  // RASPIMOUSE_ROS2_EXAMPLES__OBJECT_TRACKING_COMPONENT_HPP_
