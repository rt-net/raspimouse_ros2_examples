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

#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace direction_controller
{

Controller::Controller(const rclcpp::NodeOptions & options)
: Node("direction_controller", options)
{

  using namespace std::placeholders;  // for _1, _2, _3...

  cmd_vel_timer_ = create_wall_timer(50ms, std::bind(&Controller::on_cmd_vel_timer, this));
  // Don't actually start publishing data until activated
  cmd_vel_timer_->cancel();

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
  buzzer_pub_ = create_publisher<std_msgs::msg::Int16>("buzzer", 1);
  switches_sub_ = create_subscription<raspimouse_msgs::msg::Switches>(
    "switches", 1, std::bind(&Controller::callback_switches, this, _1));

  motor_power_client_ = create_client<std_srvs::srv::SetBool>("motor_power");
}

void Controller::on_cmd_vel_timer()
{
  if (switches_.switch0) {
    RCLCPP_INFO(this->get_logger(), "SW0 pressed.");
  } else if (switches_.switch1) {
    RCLCPP_INFO(this->get_logger(), "SW1 pressed.");
  } else if (switches_.switch2) {
    RCLCPP_INFO(this->get_logger(), "SW2 pressed.");
  }
  switches_ = raspimouse_msgs::msg::Switches();  // Reset switch values

  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
  cmd_vel_pub_->publish(std::move(cmd_vel));
}

void Controller::callback_switches(const raspimouse_msgs::msg::Switches::SharedPtr msg)
{
  switches_ = *msg;
}

void Controller::set_motor_power(const bool motor_on)
{
  if (!motor_power_client_->wait_for_service(5s)) {
    RCLCPP_ERROR(this->get_logger(),
      "Service motor_power is not avaliable.");
    return;
  }
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = motor_on;
  auto future_result = motor_power_client_->async_send_request(request);
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
