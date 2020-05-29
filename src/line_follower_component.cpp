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

#include "raspimouse_ros2_examples/line_follower_component.hpp"

#include <memory>
#include <chrono>
#include <functional>
#include <iostream>
#include <utility>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace line_follower
{

Follower::Follower(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("follower", options)
{
}

void Follower::on_cmd_vel_timer()
{
  RCLCPP_INFO(this->get_logger(), "Switches: %d, %d, %d",
    switches_.switch0, switches_.switch1, switches_.switch2);
  RCLCPP_INFO(this->get_logger(), "Sensors: %d %d, %d, %d",
    light_sensors_.left, light_sensors_.forward_l, light_sensors_.forward_r, light_sensors_.right);
  cmd_vel_.linear.x = 0.0;
  cmd_vel_.angular.z = 0.0;
  auto msg = std::make_unique<geometry_msgs::msg::Twist>(cmd_vel_);
  cmd_vel_pub_->publish(std::move(msg));
}

void Follower::callback_light_sensors(const raspimouse_msgs::msg::LightSensors::SharedPtr msg)
{
  light_sensors_ = *msg;
}

void Follower::callback_switches(const raspimouse_msgs::msg::Switches::SharedPtr msg)
{
  switches_ = *msg;
}


CallbackReturn Follower::on_configure(const rclcpp_lifecycle::State &)
{
  using namespace std::placeholders;  // for _1, _2, _3...

  RCLCPP_INFO(this->get_logger(), "on_configure() is called.");

  cmd_vel_timer_ = create_wall_timer(50ms, std::bind(&Follower::on_cmd_vel_timer, this));
  // Don't actually start publishing data until activated
  cmd_vel_timer_->cancel();

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
  light_sensors_sub_ = create_subscription<raspimouse_msgs::msg::LightSensors>(
    "light_sensors", 1, std::bind(&Follower::callback_light_sensors, this, _1));
  switches_sub_ = create_subscription<raspimouse_msgs::msg::Switches>(
    "switches", 1, std::bind(&Follower::callback_switches, this, _1));

  return CallbackReturn::SUCCESS;
}

CallbackReturn Follower::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "on_activate() is called.");

  motor_power_client_ = create_client<std_srvs::srv::SetBool>("motor_power");
  if (!motor_power_client_->wait_for_service(5s)) {
    RCLCPP_ERROR(this->get_logger(),
      "Service motor_power is not avaliable.");
    return CallbackReturn::FAILURE;
  }
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;
  auto future_result = motor_power_client_->async_send_request(request);

  cmd_vel_pub_->on_activate();
  cmd_vel_timer_->reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn Follower::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "on_deactivate() is called.");
  cmd_vel_pub_->on_deactivate();
  cmd_vel_timer_->cancel();

  cmd_vel_ = geometry_msgs::msg::Twist();

  return CallbackReturn::SUCCESS;
}

CallbackReturn Follower::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "on_cleanup() is called.");

  cmd_vel_pub_.reset();
  cmd_vel_timer_.reset();
  light_sensors_sub_.reset();
  switches_sub_.reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn Follower::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "on_shutdown() is called.");

  cmd_vel_pub_.reset();
  cmd_vel_timer_.reset();
  light_sensors_sub_.reset();
  switches_sub_.reset();

  return CallbackReturn::SUCCESS;
}

}  // namespace line_follower

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(line_follower::Follower)
