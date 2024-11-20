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

#include <memory>
#include "rclcpp/rclcpp.hpp"

#include "raspimouse/raspimouse_component.hpp"
#include "raspimouse_ros2_examples/direction_controller_component.hpp"
#include "rt_usb_9axisimu_driver/rt_usb_9axisimu_driver_component.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;
  auto direction_controller = std::make_shared<direction_controller::Controller>(options);
  exec.add_node(direction_controller->get_node_base_interface());
  exec.spin();
  rclcpp::shutdown();
}
