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

#include "raspimouse_ros2_examples/direction_controller_component.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <numeric>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace direction_controller
{

enum CONTROL_MODE
{
  MODE_NONE = 0,
  MODE_CALIBRATION = 1,
  MODE_KEEP_ZERO_RADIAN = 2,
  MODE_ROTATION = 3
};

Controller::Controller(const rclcpp::NodeOptions & options)
: Node("direction_controller", options)
{

  using namespace std::placeholders;  // for _1, _2, _3...

  cmd_vel_timer_ = create_wall_timer(16ms, std::bind(&Controller::on_cmd_vel_timer, this));

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
  buzzer_pub_ = create_publisher<std_msgs::msg::Int16>("buzzer", 1);
  switches_sub_ = create_subscription<raspimouse_msgs::msg::Switches>(
    "switches", 1, std::bind(&Controller::callback_switches, this, _1));
  imu_data_raw_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "imu/data_raw", 1, std::bind(&Controller::callback_imu_data_raw, this, _1));

  motor_power_client_ = create_client<std_srvs::srv::SetBool>("motor_power");

  change_mouse_state_client_ = 
    create_client<lifecycle_msgs::srv::ChangeState>("raspimouse/change_state");
  change_imu_state_client_ =
    create_client<lifecycle_msgs::srv::ChangeState>("rt_usb_9axisimu_driver/change_state");

  change_mouse_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  change_mouse_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  control_mode_ = MODE_NONE;
  omega_pid_controller_.set_gain(10, 0, 20);
  omega_bias_ = 0.0;
  heading_angle_ = 0.0;
  prev_heading_calculation_time_ = this->now().seconds();
  target_angle_ = 0.0;
  increase_target_angle_ = true;
}

void Controller::on_cmd_vel_timer()
{
  if(switches_.switch0 || switches_.switch1 || switches_.switch2
    || filtered_acc_.z > 0.0){
    if(control_mode_ == MODE_KEEP_ZERO_RADIAN || control_mode_ == MODE_ROTATION){
      control_mode_ = MODE_NONE;
      set_motor_power(false);
      beep_failure();
      rclcpp::sleep_for(100ms);
      switches_ = raspimouse_msgs::msg::Switches();  // Reset switch values
    }
  }

  if(control_mode_ == MODE_NONE){
    if (switches_.switch0) {
      RCLCPP_INFO(this->get_logger(), "SW0 pressed.");
      change_imu_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
      change_imu_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
      beep_success();
      control_mode_ = MODE_CALIBRATION;
    } else if (switches_.switch1) {
      RCLCPP_INFO(this->get_logger(), "SW1 pressed.");
      beep_success();
      set_motor_power(true);
      control_mode_ = MODE_KEEP_ZERO_RADIAN;
    } else if (switches_.switch2) {
      RCLCPP_INFO(this->get_logger(), "SW2 pressed.");
      beep_success();
      set_motor_power(true);
      control_mode_ = MODE_ROTATION;
    }
    switches_ = raspimouse_msgs::msg::Switches();  // Reset switch values

  }else if(control_mode_ == MODE_KEEP_ZERO_RADIAN){
    angle_control(0.0);
  }else if(control_mode_ == MODE_ROTATION){
    rotation();
  }
}

void Controller::callback_switches(const raspimouse_msgs::msg::Switches::SharedPtr msg)
{
  switches_ = *msg;
}

void Controller::callback_imu_data_raw(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  imu_data_raw_ = *msg;

  calculate_heading_angle(imu_data_raw_.angular_velocity.z, this->now().seconds());

  if(control_mode_ == MODE_CALIBRATION){
    if(omega_calibration(imu_data_raw_.angular_velocity.z)){
      control_mode_ = MODE_NONE;
      heading_angle_ = 0.0;
      prev_heading_calculation_time_ = this->now().seconds();

      beep_success();
    }
  }

  filter_acceleration(imu_data_raw_.linear_acceleration);
}

bool Controller::set_motor_power(const bool motor_on)
{
  if (!motor_power_client_->wait_for_service(5s)) {
    RCLCPP_ERROR(this->get_logger(),
      "Service motor_power is not avaliable.");
    return false;
  }
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = motor_on;
  auto future_result = motor_power_client_->async_send_request(request);
  return true;
}

bool Controller::change_mouse_state(const std::uint8_t transition)
{
  if (!change_mouse_state_client_->wait_for_service(5s)) {
    RCLCPP_ERROR(this->get_logger(),
      "Service %s is not avaliable.", change_mouse_state_client_->get_service_name());
    return false;
  }
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = transition;
  auto future_result = change_mouse_state_client_->async_send_request(request);

  return true;
}

bool Controller::change_imu_state(const std::uint8_t transition)
{
  if (!change_imu_state_client_->wait_for_service(5s)) {
    RCLCPP_ERROR(this->get_logger(),
      "Service %s is not avaliable.", change_imu_state_client_->get_service_name());
    return false;
  }
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = transition;
  auto future_result = change_imu_state_client_->async_send_request(request);

  return true;
}

bool Controller::omega_calibration(const double omega)
{
  const int SAMPLE_NUM = 100;
  bool complete = false;

  omega_samples_.push_back(omega);

  if(omega_samples_.size() >= SAMPLE_NUM){
    omega_bias_ = std::accumulate(std::begin(omega_samples_), std::end(omega_samples_), 0.0) / omega_samples_.size();
    omega_samples_.clear();
    complete = true;
  }

  return complete;
}

void Controller::calculate_heading_angle(const double omega, const double current_time)
{
  const double ALPHA = 1.0;

  double diff_time = current_time - prev_heading_calculation_time_;
  double biased_omega = ALPHA * (omega - omega_bias_);

  heading_angle_ += biased_omega * diff_time;
  prev_heading_calculation_time_ = current_time;
}

void Controller::filter_acceleration(const geometry_msgs::msg::Vector3 acc)
{
  const double ALPHA = 0.1;

  // Simple low pass filter
  filtered_acc_.x = ALPHA * acc.x + (1.0 - ALPHA) * prev_acc_.x;
  filtered_acc_.y = ALPHA * acc.y + (1.0 - ALPHA) * prev_acc_.y;
  filtered_acc_.z = ALPHA * acc.z + (1.0 - ALPHA) * prev_acc_.z;
  prev_acc_ = filtered_acc_;
}

void Controller::angle_control(const double target_angle)
{
  const double SIGN = 1.0;

  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
  cmd_vel->angular.z = SIGN * omega_pid_controller_.update(heading_angle_, target_angle);

  cmd_vel_pub_->publish(std::move(cmd_vel));
}

void Controller::rotation(void)
{
  const double ADD_ANGLE = 2.0 * M_PI / 180.0;
  const double START_ANGLE = -M_PI * 0.5;
  const double END_ANGLE = M_PI * 0.5;

  if(increase_target_angle_){
    target_angle_ += ADD_ANGLE;
  }else{
    target_angle_ -= ADD_ANGLE;
  }

  if(target_angle_ >= END_ANGLE){
    target_angle_ = END_ANGLE;
    increase_target_angle_ = false;
  }else if(target_angle_ <= START_ANGLE){
    target_angle_ = START_ANGLE;
    increase_target_angle_ = true;
  }

  angle_control(target_angle_);
}

void Controller::beep_buzzer(const int freq, const std::chrono::nanoseconds & beep_time)
{
  auto msg = std::make_unique<std_msgs::msg::Int16>();
  msg->data = freq;
  buzzer_pub_->publish(std::move(msg));

  rclcpp::sleep_for(beep_time);

  msg = std::make_unique<std_msgs::msg::Int16>();
  msg->data = 0;
  buzzer_pub_->publish(std::move(msg));
}

void Controller::beep_start(void)
{
  beep_buzzer(1000, 500ms);
}

void Controller::beep_success(void)
{
  beep_buzzer(1000, 100ms);
  rclcpp::sleep_for(100ms);
  beep_buzzer(1000, 100ms);
}

void Controller::beep_failure(void)
{
  for (int i = 0; i < 4; i++) {
    beep_buzzer(500, 100ms);
    rclcpp::sleep_for(100ms);
  }
}


}  // namespace direction_controller

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(direction_controller::Controller)
