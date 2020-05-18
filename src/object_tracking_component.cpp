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

#include "raspimouse_ros2_examples/object_tracking_component.hpp"

#include <memory>
#include <chrono>
#include <iostream>
#include <utility>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"

using namespace std::chrono_literals;

namespace object_tracking
{

Tracker::Tracker(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("tracker", options),
  frame_id_(0)
{
}

void Tracker::on_image_timer()
{
  auto msg = std::make_unique<sensor_msgs::msg::Image>();
  msg->is_bigendian = false;

  cv::Mat frame;
  cap_ >> frame;

  if (!frame.empty()) {
    // Publish the image message and increment the frame_id.
    convert_frame_to_message(frame, frame_id_, *msg);
    image_pub_->publish(std::move(msg));
    ++frame_id_;
  }
}

// Ref: https://github.com/ros2/demos/blob/dashing/image_tools/src/cam2image.cpp
std::string Tracker::mat_type2encoding(int mat_type)
{
  switch (mat_type) {
    case CV_8UC1:
      return "mono8";
    case CV_8UC3:
      return "bgr8";
    case CV_16SC1:
      return "mono16";
    case CV_8UC4:
      return "rgba8";
    default:
      throw std::runtime_error("Unsupported encoding type");
  }
}

// Ref: https://github.com/ros2/demos/blob/dashing/image_tools/src/cam2image.cpp
void Tracker::convert_frame_to_message(
  const cv::Mat & frame, size_t frame_id, sensor_msgs::msg::Image & msg)
{
  // copy cv information into ros message
  msg.height = frame.rows;
  msg.width = frame.cols;
  msg.encoding = mat_type2encoding(frame.type());
  msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
  size_t size = frame.step * frame.rows;
  msg.data.resize(size);
  memcpy(&msg.data[0], frame.data, size);
  msg.header.frame_id = std::to_string(frame_id);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Tracker::on_configure(const rclcpp_lifecycle::State &)
{
  image_timer_ = create_wall_timer(100ms, std::bind(&Tracker::on_image_timer, this));

  size_t depth = rmw_qos_profile_default.depth;
  rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
  rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
  std::string topic("image");
  size_t width = 320;
  size_t height = 240;

  RCLCPP_INFO(this->get_logger(), "on_configure() is called.");

  auto qos = rclcpp::QoS(
    rclcpp::QoSInitialization(
      history_policy,
      depth));
  qos.reliability(reliability_policy);
  image_pub_ = create_publisher<sensor_msgs::msg::Image>(topic, qos);

  // Initialize OpenCV video capture stream.
  // Always open device 0.
  cap_.open(0);
  cap_.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(width));
  cap_.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(height));
  if (!cap_.isOpened()) {
    RCLCPP_ERROR(this->get_logger(), "Could not open video stream");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Tracker::on_activate(const rclcpp_lifecycle::State &)
{
  image_pub_->on_activate();
  RCLCPP_INFO(this->get_logger(), "on_activate() is called.");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Tracker::on_deactivate(const rclcpp_lifecycle::State &)
{
  image_pub_->on_deactivate();
  RCLCPP_INFO(this->get_logger(), "on_deactivate() is called.");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Tracker::on_cleanup(const rclcpp_lifecycle::State &)
{
  image_pub_.reset();
  image_timer_.reset();

  RCLCPP_INFO(this->get_logger(), "on_cleanup() is called.");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Tracker::on_shutdown(const rclcpp_lifecycle::State &)
{
  image_pub_.reset();
  image_timer_.reset();

  RCLCPP_INFO(this->get_logger(), "on_shutdown() is called.");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

}  // namespace object_tracking

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(object_tracking::Tracker)
