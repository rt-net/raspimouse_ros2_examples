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

#ifndef RASPIMOUSE_ROS2_EXAMPLES__LINE_FOLLOWER_COMPONENT_HPP_
#define RASPIMOUSE_ROS2_EXAMPLES__LINE_FOLLOWER_COMPONENT_HPP_

#include <memory>
#include <string>

#include "raspimouse_ros2_examples/visibility_control.h"
#include "raspimouse_msgs/msg/switches.hpp"
#include "raspimouse_msgs/msg/light_sensors.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace line_follower
{

class Follower : public rclcpp_lifecycle::LifecycleNode
{
public:
  RASPIMOUSE_ROS2_EXAMPLES_PUBLIC
  explicit Follower(const rclcpp::NodeOptions & options);

protected:
  void on_cmd_vel_timer();

private:
  geometry_msgs::msg::Twist cmd_vel_;
  raspimouse_msgs::msg::LightSensors light_sensors_;
  raspimouse_msgs::msg::Switches switches_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int16>> buzzer_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>> cmd_vel_pub_;
  rclcpp::Subscription<raspimouse_msgs::msg::LightSensors>::SharedPtr light_sensors_sub_;
  rclcpp::Subscription<raspimouse_msgs::msg::Switches>::SharedPtr switches_sub_;
  std::shared_ptr<rclcpp::Client<std_srvs::srv::SetBool>> motor_power_client_;

  rclcpp::TimerBase::SharedPtr cmd_vel_timer_;
  rclcpp::TimerBase::SharedPtr one_off_timer;

  void callback_light_sensors(const raspimouse_msgs::msg::LightSensors::SharedPtr msg);
  void callback_switches(const raspimouse_msgs::msg::Switches::SharedPtr msg);

  void set_motor_power(const bool motor_on);
  void beep_buzzer(const int freq, const std::chrono::nanoseconds & beep_time);
  void beep_start(void);
  void beep_success(void);
  void beep_failure(void);

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

}  // namespace line_follower

#endif  // RASPIMOUSE_ROS2_EXAMPLES__LINE_FOLLOWER_COMPONENT_HPP_
