// Copyright 2023-2024 RT Corporation
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
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "cv_bridge/cv_bridge.hpp"

namespace camera_line_follower
{
using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// パラメータ名の定数
constexpr auto MIN_BRIGHTNESS_PARAM = "min_brightness";
constexpr auto MAX_BRIGHTNESS_PARAM = "max_brightness";
constexpr auto LINEAR_VEL_PARAM = "max_linear_vel";
constexpr auto ANGULAR_VEL_PARAM = "max_angular_vel";
constexpr auto AREA_THRESHOLD_PARAM = "area_threshold";

CameraFollower::CameraFollower(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("camera_follower", options),
  object_is_detected_(false),
  enable_following_(false)
{
}

void CameraFollower::image_callback(const sensor_msgs::msg::Image::SharedPtr msg_image)
{
  // 受信した画像メッセージをOpenCV形式に変換する
  const auto cv_img = cv_bridge::toCvShare(msg_image, msg_image->encoding);
  auto result_msg = std::make_unique<sensor_msgs::msg::Image>();

  cv::Mat frame, result_frame;
  cv::cvtColor(cv_img->image, frame, CV_RGB2BGR);

  if (!frame.empty()) {
    // ライン検出を実行し、結果画像をパブリッシュする
    object_is_detected_ = detect_line(frame, result_frame);
    convert_frame_to_message(result_frame, *result_msg);
    result_image_pub_->publish(std::move(result_msg));
  }
}

void CameraFollower::callback_switches(const raspimouse_msgs::msg::Switches::SharedPtr msg)
{
  // SW0で追従停止、SW2で追従開始
  if (msg->switch0) {
    RCLCPP_INFO(this->get_logger(), "Stop following.");
    set_motor_power(false);
    enable_following_ = false;
  } else if (msg->switch2) {
    RCLCPP_INFO(this->get_logger(), "Start following.");
    set_motor_power(true);
    enable_following_ = true;
  }
}

void CameraFollower::on_cmd_vel_timer()
{
  geometry_msgs::msg::TwistStamped cmd_vel;

  // 物体が検出され、かつ面積が閾値以上のときにライン追従走行する
  if (
    object_is_detected_ &&
    object_normalized_area_ > get_parameter(AREA_THRESHOLD_PARAM).as_double())
  {
    cmd_vel.twist.linear.x = get_parameter(LINEAR_VEL_PARAM).as_double();
    // 物体の重心位置に応じて旋回方向を決める（正規化座標 -1.0〜1.0）
    cmd_vel.twist.angular.z =
      -get_parameter(ANGULAR_VEL_PARAM).as_double() * object_normalized_point_.x;
  } else {
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = 0.0;
  }

  // 追従が無効のときは速度をゼロにする
  if (!enable_following_) {
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = 0.0;
  }

  auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>(cmd_vel);
  cmd_vel_pub_->publish(std::move(msg));
}

void CameraFollower::set_motor_power(const bool motor_on)
{
  if (motor_power_client_ == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "Service motor_power is not available.");
    return;
  }
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = motor_on;
  auto future_result = motor_power_client_->async_send_request(request);
}

// Ref: https://github.com/ros2/demos/blob/dashing/image_tools/src/cam2image.cpp
std::string CameraFollower::mat_type2encoding(const int mat_type) const
{
  // OpenCVの画像形式をROSメッセージのエンコーディング文字列に変換する
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
void CameraFollower::convert_frame_to_message(
  const cv::Mat & frame, sensor_msgs::msg::Image & msg) const
{
  // OpenCVの画像データをROSのImageメッセージにコピーする
  msg.height = frame.rows;
  msg.width = frame.cols;
  msg.encoding = mat_type2encoding(frame.type());
  msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
  size_t size = frame.step * frame.rows;
  msg.data.resize(size);
  memcpy(&msg.data[0], frame.data, size);
  msg.header.frame_id = "camera_frame";
}

bool CameraFollower::detect_line(const cv::Mat & input_frame, cv::Mat & result_frame)
{
  // 入力画像をグレースケールに変換し、輝度範囲でライン色を二値化する
  cv::Mat gray;
  cv::cvtColor(input_frame, gray, cv::COLOR_BGR2GRAY);
  cv::Mat extracted_bin;
  cv::inRange(
    gray, get_parameter(MIN_BRIGHTNESS_PARAM).as_int(),
    get_parameter(MAX_BRIGHTNESS_PARAM).as_int(), extracted_bin);
  input_frame.copyTo(result_frame, extracted_bin);

  // モルフォロジー変換でノイズを除去する
  cv::Mat morph_bin;
  cv::morphologyEx(extracted_bin, morph_bin, cv::MORPH_CLOSE, cv::Mat());

  // 輪郭を抽出する
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(morph_bin, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  // 最大面積の輪郭を選択する
  double max_area = 0;
  int max_area_index = -1;
  for (unsigned int index = 0; index < contours.size(); index++) {
    double area = cv::contourArea(contours.at(index));
    if (area > max_area) {
      max_area = area;
      max_area_index = index;
    }
  }

  // 輪郭が存在する場合は重心を計算して正規化する
  if (max_area_index >= 0) {
    cv::Moments mt = cv::moments(contours.at(max_area_index));
    cv::Point mt_point = cv::Point(mt.m10 / mt.m00, mt.m01 / mt.m00);

    // 重心座標を [-1.0, 1.0] に正規化する
    object_normalized_point_ = cv::Point2d(
      2.0 * mt_point.x / input_frame.cols - 1.0, 2.0 * mt_point.y / input_frame.rows - 1.0);
    // 輪郭面積を [0.0, 1.0] に正規化する
    object_normalized_area_ = max_area / (input_frame.rows * input_frame.cols);

    std::string text = "Area:" + std::to_string(object_normalized_area_ * 100) + "%";
    cv::drawContours(
      result_frame, contours, max_area_index, cv::Scalar(0, 255, 0), 2, cv::LINE_4, hierarchy);
    cv::circle(result_frame, mt_point, 30, cv::Scalar(0, 0, 255), 2, cv::LINE_4);
    cv::putText(
      result_frame, text, cv::Point(0, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255),
      2);
    return true;
  } else {
    return false;
  }
}

CallbackReturn CameraFollower::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "on_configure() is called.");

  // ライフサイクルノードのconfigure時にパブリッシャ・サブスクライバ・タイマーを作成する
  cmd_vel_timer_ = create_wall_timer(50ms, std::bind(&CameraFollower::on_cmd_vel_timer, this));
  // activate状態に遷移するまでタイマーを停止しておく
  cmd_vel_timer_->cancel();

  motor_power_client_ = create_client<std_srvs::srv::SetBool>("motor_power");
  if (!motor_power_client_->wait_for_service(5s)) {
    RCLCPP_ERROR(this->get_logger(), "Service motor_power is not available.");
    return CallbackReturn::FAILURE;
  }

  result_image_pub_ = create_publisher<sensor_msgs::msg::Image>("result_image", 1);
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 1);
  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    "camera/color/image_raw", rclcpp::SensorDataQoS(),
    std::bind(&CameraFollower::image_callback, this, std::placeholders::_1));
  switches_sub_ = create_subscription<raspimouse_msgs::msg::Switches>(
    "switches", 1, std::bind(&CameraFollower::callback_switches, this, std::placeholders::_1));

  // パラメータのデフォルト値を宣言する
  declare_parameter(MIN_BRIGHTNESS_PARAM, 0);
  declare_parameter(MAX_BRIGHTNESS_PARAM, 90);
  declare_parameter(LINEAR_VEL_PARAM, 0.05);
  declare_parameter(ANGULAR_VEL_PARAM, 0.8);
  declare_parameter(AREA_THRESHOLD_PARAM, 0.2);

  return CallbackReturn::SUCCESS;
}

CallbackReturn CameraFollower::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "on_activate() is called.");

  // ライフサイクルパブリッシャをアクティブにしてからタイマーを開始する
  result_image_pub_->on_activate();
  cmd_vel_pub_->on_activate();
  cmd_vel_timer_->reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn CameraFollower::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "on_deactivate() is called.");

  set_motor_power(false);

  result_image_pub_->on_deactivate();
  cmd_vel_pub_->on_deactivate();
  cmd_vel_timer_->cancel();

  object_is_detected_ = false;

  return CallbackReturn::SUCCESS;
}

CallbackReturn CameraFollower::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "on_cleanup() is called.");

  set_motor_power(false);

  result_image_pub_.reset();
  cmd_vel_pub_.reset();
  cmd_vel_timer_.reset();
  image_sub_.reset();
  switches_sub_.reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn CameraFollower::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "on_shutdown() is called.");

  set_motor_power(false);

  result_image_pub_.reset();
  cmd_vel_pub_.reset();
  cmd_vel_timer_.reset();
  image_sub_.reset();
  switches_sub_.reset();

  return CallbackReturn::SUCCESS;
}

}  // namespace camera_line_follower

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(camera_line_follower::CameraFollower)
