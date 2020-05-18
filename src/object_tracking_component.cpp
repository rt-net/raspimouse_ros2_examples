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

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"

using namespace std::chrono_literals;

namespace object_tracking
{

Tracker::Tracker(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("tracker", options), count_(0)
{
}

void Tracker::on_timer()
{
  if (!pub_->is_activated()) {
    RCLCPP_INFO(
      this->get_logger(),
      "Lifecycle publisher is currently inactive. Mesages are not published.");
  }

  auto msg = std::make_unique<std_msgs::msg::String>();
  msg->data = "Hello World: " + std::to_string(++count_);
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg->data.c_str());
  std::flush(std::cout);

  pub_->publish(std::move(msg));
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Tracker::on_configure(const rclcpp_lifecycle::State &)
{
  // Create a publisher of "std_mgs/String" messages on the "chatter" topic.
  pub_ = create_publisher<std_msgs::msg::String>("chatter", 10);

  // Use a timer to schedule periodic message publishing.
  timer_ = create_wall_timer(1s, std::bind(&Tracker::on_timer, this));

  RCLCPP_INFO(this->get_logger(), "on_configure() is called.");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Tracker::on_activate(const rclcpp_lifecycle::State &)
{
  pub_->on_activate();
  RCLCPP_INFO(this->get_logger(), "on_activate() is called.");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Tracker::on_deactivate(const rclcpp_lifecycle::State &)
{
  pub_->on_deactivate();
  RCLCPP_INFO(this->get_logger(), "on_deactivate() is called.");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Tracker::on_cleanup(const rclcpp_lifecycle::State &)
{
  timer_.reset();
  pub_.reset();

  RCLCPP_INFO(this->get_logger(), "on_cleanup() is called.");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Tracker::on_shutdown(const rclcpp_lifecycle::State &)
{
  timer_.reset();
  pub_.reset();

  RCLCPP_INFO(this->get_logger(), "on_shutdown() is called.");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

}  // namespace object_tracking

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(object_tracking::Tracker)
