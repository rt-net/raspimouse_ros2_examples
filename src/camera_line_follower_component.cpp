// Copyright 2023 RT Corporation
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

#include "raspimouse_ros2_examples/camera_line_follower_component.hpp"

#include <memory>
#include <chrono>
#include <iostream>
#include <utility>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "cv_bridge/cv_bridge.h"

constexpr auto MIN_BRIGHTNESS_PARAM = "min_brightness";
constexpr auto MAX_BRIGHTNESS_PARAM = "max_brightness";
constexpr auto LINEAR_VEL_PARAM = "max_linear_vel";
constexpr auto ANGULAR_VEL_PARAM = "max_angular_vel";


namespace camera_line_follower
{
using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

Camera_Follower::Camera_Follower(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("camera_follower", options),
  object_is_detected_(false)
{
}

void Camera_Follower::image_callback(const sensor_msgs::msg::Image::SharedPtr msg_image)
{
  const auto cv_img = cv_bridge::toCvShare(msg_image, msg_image->encoding);
  auto result_msg = std::make_unique<sensor_msgs::msg::Image>();

  cv::Mat frame, result_frame;
  cv::cvtColor(cv_img->image, frame, CV_RGB2BGR);

  if (!frame.empty()) {
    object_is_detected_ = detect_line(frame, result_frame);
    convert_frame_to_message(result_frame, *result_msg);
    result_image_pub_->publish(std::move(result_msg));
  }
}

void Camera_Follower::callback_switches(const raspimouse_msgs::msg::Switches::SharedPtr msg)
{
  if (msg->switch0) {
    RCLCPP_INFO(this->get_logger(), "Stop following.");
    enable_following_ = false;
  } else if (msg->switch2) {
    RCLCPP_INFO(this->get_logger(), "Start following.");
    enable_following_ = true;
  }
}

void Camera_Follower::on_cmd_vel_timer()
{
  constexpr double OBJECT_AREA_THRESHOLD = 0.01;  // 0.0 ~ 1.0
  geometry_msgs::msg::Twist cmd_vel;

  // Follow the line
  // when the number of pixels of the object is greater than the threshold.
  if (object_is_detected_ && object_normalized_area_ > OBJECT_AREA_THRESHOLD) {
    cmd_vel.linear.x = get_parameter(LINEAR_VEL_PARAM).as_double();
    cmd_vel.angular.z = -get_parameter(ANGULAR_VEL_PARAM).as_double() * object_normalized_point_.x;
  } else {
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
  }

  if (!enable_following_) {
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
  }

  auto msg = std::make_unique<geometry_msgs::msg::Twist>(cmd_vel);
  cmd_vel_pub_->publish(std::move(msg));
}

// Ref: https://github.com/ros2/demos/blob/dashing/image_tools/src/cam2image.cpp
std::string Camera_Follower::mat_type2encoding(const int mat_type) const
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
void Camera_Follower::convert_frame_to_message(
  const cv::Mat & frame, sensor_msgs::msg::Image & msg) const
{
  // copy cv information into ros message
  msg.height = frame.rows;
  msg.width = frame.cols;
  msg.encoding = mat_type2encoding(frame.type());
  msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
  size_t size = frame.step * frame.rows;
  msg.data.resize(size);
  memcpy(&msg.data[0], frame.data, size);
  msg.header.frame_id = "camera_frame";
}

bool Camera_Follower::detect_line(const cv::Mat & input_frame, cv::Mat & result_frame)
{
  // Specific colors are extracted from the input image and converted to binary values.
  cv::Mat gray;
  cv::cvtColor(input_frame, gray, cv::COLOR_BGR2GRAY);
  cv::Mat extracted_bin;
  cv::inRange(
    gray,
    get_parameter(MIN_BRIGHTNESS_PARAM).as_int(),
    get_parameter(MAX_BRIGHTNESS_PARAM).as_int(),
    extracted_bin);
  input_frame.copyTo(result_frame, extracted_bin);

  // Remove noise with morphology transformation
  cv::Mat morph_bin;
  cv::morphologyEx(extracted_bin, morph_bin, cv::MORPH_CLOSE, cv::Mat());

  // Extracting contours
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(morph_bin, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  // Extracting the largest contours
  double max_area = 0;
  int max_area_index = -1;
  for (unsigned int index = 0; index < contours.size(); index++) {
    double area = cv::contourArea(contours.at(index));
    if (area > max_area) {
      max_area = area;
      max_area_index = index;
    }
  }

  // If the contour exists (if the object exists), find the centroid of the contour
  if (max_area_index >= 0) {
    cv::Moments mt = cv::moments(contours.at(max_area_index));
    cv::Point mt_point = cv::Point(mt.m10 / mt.m00, mt.m01 / mt.m00);

    // Normalize the centroid coordinates to [-1.0, 1.0].
    object_normalized_point_ = cv::Point2d(
      2.0 * mt_point.x / input_frame.cols - 1.0,
      2.0 * mt_point.y / input_frame.rows - 1.0
    );
    // Normalize the the contour area to [0.0, 1.0].
    object_normalized_area_ = max_area / (input_frame.rows * input_frame.cols);

    std::string text = "Area:" + std::to_string(object_normalized_area_ * 100) + "%";
    cv::drawContours(
      result_frame, contours, max_area_index,
      cv::Scalar(0, 255, 0), 2, cv::LINE_4, hierarchy);
    cv::circle(result_frame, mt_point, 30, cv::Scalar(0, 0, 255), 2, cv::LINE_4);
    cv::putText(
      result_frame, text, cv::Point(0, 30),
      cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
    return true;
  } else {
    return false;
  }
}

CallbackReturn Camera_Follower::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "on_configure() is called.");

  cmd_vel_timer_ = create_wall_timer(50ms, std::bind(&Camera_Follower::on_cmd_vel_timer, this));
  // Don't actually start publishing data until activated
  cmd_vel_timer_->cancel();

  result_image_pub_ = create_publisher<sensor_msgs::msg::Image>("result_image", 1);
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    "camera/color/image_raw", rclcpp::SensorDataQoS(),
    std::bind(&Camera_Follower::image_callback, this, std::placeholders::_1));
  switches_sub_ = create_subscription<raspimouse_msgs::msg::Switches>(
    "switches", 1, std::bind(&Camera_Follower::callback_switches, this, std::placeholders::_1));

  // Set parameter defaults
  declare_parameter(MIN_BRIGHTNESS_PARAM, 0);
  declare_parameter(MAX_BRIGHTNESS_PARAM, 90);
  declare_parameter(LINEAR_VEL_PARAM, 0.05);
  declare_parameter(ANGULAR_VEL_PARAM, 0.8);

  return CallbackReturn::SUCCESS;
}

CallbackReturn Camera_Follower::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "on_activate() is called.");

  motor_power_client_ = create_client<std_srvs::srv::SetBool>("motor_power");
  if (!motor_power_client_->wait_for_service(5s)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Service motor_power is not avaliable.");
    return CallbackReturn::FAILURE;
  }
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;
  auto future_result = motor_power_client_->async_send_request(request);

  result_image_pub_->on_activate();
  cmd_vel_pub_->on_activate();
  cmd_vel_timer_->reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn Camera_Follower::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "on_deactivate() is called.");
  result_image_pub_->on_deactivate();
  cmd_vel_pub_->on_deactivate();
  cmd_vel_timer_->cancel();

  object_is_detected_ = false;

  return CallbackReturn::SUCCESS;
}

CallbackReturn Camera_Follower::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "on_cleanup() is called.");

  result_image_pub_.reset();
  cmd_vel_pub_.reset();
  cmd_vel_timer_.reset();
  image_sub_.reset();
  switches_sub_.reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn Camera_Follower::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "on_shutdown() is called.");

  result_image_pub_.reset();
  cmd_vel_pub_.reset();
  cmd_vel_timer_.reset();
  image_sub_.reset();
  switches_sub_.reset();

  return CallbackReturn::SUCCESS;
}

}  // namespace camera_line_follower

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(camera_line_follower::Camera_Follower)
