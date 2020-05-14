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

#include "raspimouse_ros2_examples/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace object_tracking
{

class Talker : public rclcpp::Node
{
public:
  RASPIMOUSE_ROS2_EXAMPLES_PUBLIC
  explicit Talker(const rclcpp::NodeOptions & options);

protected:
  void on_timer();

private:
  size_t count_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace object_tracking

#endif  // RASPIMOUSE_ROS2_EXAMPLES__OBJECT_TRACKING_COMPONENT_HPP_
