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

#include <opencv2/opencv.hpp>
#include <memory>
#include <chrono>
#include <iostream>
#include <utility>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace object_tracking
{


Tracker::Tracker(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("tracker", options),
  device_index_(0), image_width_(320), image_height_(240),
  object_is_detected_(false)
{
}

void Tracker::on_image_timer()
{
  auto msg = std::make_unique<sensor_msgs::msg::Image>();
  auto result_msg = std::make_unique<sensor_msgs::msg::Image>();
  msg->is_bigendian = false;
  result_msg->is_bigendian = false;

  cv::Mat frame, result_frame;
  cap_ >> frame;

  if (!frame.empty()) {
    tracking(frame, result_frame);
    convert_frame_to_message(result_frame, *result_msg);
    result_image_pub_->publish(std::move(result_msg));

    convert_frame_to_message(frame, *msg);
    image_pub_->publish(std::move(msg));
  }
}

void Tracker::on_cmd_vel_timer()
{
  const double LINEAR_VEL = -0.5;  // unit: m/s
  const double ANGULAR_VEL = -1.0; // unit: rad/s
  const double TARGET_AREA = 0.3; // %
  const double OBJECT_AREA_THRESHOLD = 0.05;  // %
  const double ATTENUATION_RATE = 0.8;

  // Detects an object and tracks it 
  // when the number of pixels of the object is greater than the threshold.
  if (object_is_detected_ && object_normalized_area_ > OBJECT_AREA_THRESHOLD) {
    cmd_vel_.linear.x = LINEAR_VEL * (object_normalized_area_ - TARGET_AREA);
    cmd_vel_.angular.z = ANGULAR_VEL * object_normalized_point_.x;
  } else {
    cmd_vel_.linear.x *= ATTENUATION_RATE;
    cmd_vel_.angular.z *= ATTENUATION_RATE;
  }
  auto msg = std::make_unique<geometry_msgs::msg::Twist>(cmd_vel_);
  cmd_vel_pub_->publish(std::move(msg));
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
  const cv::Mat & frame, sensor_msgs::msg::Image & msg)
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

void Tracker::tracking(const cv::Mat & input_frame, cv::Mat & result_frame)
{
  // Specific colors are extracted from the input image and converted to binary values.
  cv::Mat hsv;
  cv::cvtColor(input_frame, hsv, cv::COLOR_BGR2HSV);
  cv::Mat extracted_bin;
  cv::inRange(hsv, cv::Scalar(9, 100, 150), cv::Scalar(29, 255, 255), extracted_bin);
  input_frame.copyTo(result_frame, extracted_bin);

  // Remove noise with morphology transformation
  cv::Mat morph_bin;
  cv::morphologyEx(extracted_bin, morph_bin, cv::MORPH_CLOSE, cv::Mat());

  // Extracting contours
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(morph_bin, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

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
    object_is_detected_ = true;

    cv::drawContours(result_frame, contours, max_area_index,
      cv::Scalar(0, 255, 0), 2, cv::LINE_4, hierarchy);
    cv::circle(result_frame, mt_point, 30, cv::Scalar(0, 0, 255), 2, cv::LINE_4);
    cv::putText(result_frame, std::to_string(object_normalized_area_), cv::Point(0, 30),
      cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0), 2);
  } else {
    object_is_detected_ = false;
  }
}

CallbackReturn Tracker::on_configure(const rclcpp_lifecycle::State &)
{
  image_timer_ = create_wall_timer(50ms, std::bind(&Tracker::on_image_timer, this));
  cmd_vel_timer_ = create_wall_timer(10ms, std::bind(&Tracker::on_cmd_vel_timer, this));
  // Don't actually start publishing data until activated
  image_timer_->cancel();
  cmd_vel_timer_->cancel();

  RCLCPP_INFO(this->get_logger(), "on_configure() is called.");

  // auto qos = rclcpp::QoS(1);
  // qos.best_effort();
  image_pub_ = create_publisher<sensor_msgs::msg::Image>("raw_image", 1);
  result_image_pub_ = create_publisher<sensor_msgs::msg::Image>("result_image", 1);
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

  // Initialize OpenCV video capture stream.
  cap_.open(device_index_);
  cap_.set(cv::CAP_PROP_FRAME_WIDTH, image_width_);
  cap_.set(cv::CAP_PROP_FRAME_HEIGHT, image_height_);
  if (!cap_.isOpened()) {
    RCLCPP_ERROR(this->get_logger(), "Could not open video stream");
    return CallbackReturn::FAILURE;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn Tracker::on_activate(const rclcpp_lifecycle::State &)
{
  motor_power_client_ = create_client<std_srvs::srv::SetBool>("motor_power");
  if (!motor_power_client_->wait_for_service(5s)) {
    RCLCPP_ERROR(this->get_logger(),
      "Service motor_power is not avaliable.");
    return CallbackReturn::FAILURE;
  }
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;
  auto future_result = motor_power_client_->async_send_request(request);

  image_pub_->on_activate();
  result_image_pub_->on_activate();
  cmd_vel_pub_->on_activate();
  image_timer_->reset();
  cmd_vel_timer_->reset();
  RCLCPP_INFO(this->get_logger(), "on_activate() is called.");

  return CallbackReturn::SUCCESS;
}

CallbackReturn Tracker::on_deactivate(const rclcpp_lifecycle::State &)
{
  image_pub_->on_deactivate();
  result_image_pub_->on_deactivate();
  cmd_vel_pub_->on_deactivate();
  image_timer_->cancel();
  cmd_vel_timer_->cancel();
  RCLCPP_INFO(this->get_logger(), "on_deactivate() is called.");

  return CallbackReturn::SUCCESS;
}

CallbackReturn Tracker::on_cleanup(const rclcpp_lifecycle::State &)
{
  image_pub_.reset();
  result_image_pub_.reset();
  cmd_vel_pub_.reset();
  image_timer_.reset();
  cmd_vel_timer_.reset();

  RCLCPP_INFO(this->get_logger(), "on_cleanup() is called.");

  return CallbackReturn::SUCCESS;
}

CallbackReturn Tracker::on_shutdown(const rclcpp_lifecycle::State &)
{
  image_pub_.reset();
  result_image_pub_.reset();
  cmd_vel_pub_.reset();
  image_timer_.reset();
  cmd_vel_timer_.reset();

  RCLCPP_INFO(this->get_logger(), "on_shutdown() is called.");

  return CallbackReturn::SUCCESS;
}

}  // namespace object_tracking

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(object_tracking::Tracker)
