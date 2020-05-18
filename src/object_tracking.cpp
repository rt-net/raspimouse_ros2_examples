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

#include <chrono>
#include <memory>
#include <string>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

#include "rclcpp/rclcpp.hpp"


using namespace std::chrono_literals;
using MsgState = lifecycle_msgs::msg::State;
using MsgTransition = lifecycle_msgs::msg::Transition;
using SrvGetState = lifecycle_msgs::srv::GetState;
using SrvChangeState = lifecycle_msgs::srv::ChangeState;

bool all_nodes_state_are(
  rclcpp::Node::SharedPtr node, std::string service_name,
  std::uint8_t desired_id, std::chrono::seconds time_out = 10s)
{
  auto request = std::make_shared<SrvGetState::Request>();
  auto client = node->create_client<SrvGetState>(service_name);

  if (!client->wait_for_service(time_out)) {
    RCLCPP_ERROR(node->get_logger(), "Service is not avaliable.");
    return false;
  }

  auto future_result = client->async_send_request(request);
  auto future_status = rclcpp::spin_until_future_complete(node, future_result, time_out);

  if (future_status != rclcpp::executor::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Service time out while getting current state.");
    return false;
  }

  if (future_result.get()->current_state.id == desired_id) {
    return true;
  } else {
    return false;
  }
}

bool all_nodes_are_unconfigured(rclcpp::Node::SharedPtr node, std::string service_name)
{
  return all_nodes_state_are(node, service_name, MsgState::PRIMARY_STATE_UNCONFIGURED, 10s);
}

bool all_nodes_are_inactive(rclcpp::Node::SharedPtr node, std::string service_name)
{
  return all_nodes_state_are(node, service_name, MsgState::PRIMARY_STATE_INACTIVE, 10s);
}

bool all_nodes_are_active(rclcpp::Node::SharedPtr node, std::string service_name)
{
  return all_nodes_state_are(node, service_name, MsgState::PRIMARY_STATE_ACTIVE, 10s);
}

bool change_all_nodes_state(
  rclcpp::Node::SharedPtr node, std::string service_name,
  std::uint8_t transition, std::chrono::seconds time_out = 10s)
{
  auto request = std::make_shared<SrvChangeState::Request>();
  request->transition.id = transition;

  auto client = node->create_client<SrvChangeState>(service_name);

  if (!client->wait_for_service(time_out)) {
    RCLCPP_ERROR(node->get_logger(), "Service is not avaliable.");
    return false;
  }

  auto future_result = client->async_send_request(request);
  auto future_status = rclcpp::spin_until_future_complete(node, future_result, time_out);

  if (future_status != rclcpp::executor::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Service time out while changing current state.");
    return false;
  }

  return future_result.get()->success;
}

bool configure_all_nodes(rclcpp::Node::SharedPtr node, std::string service_name)
{
  return change_all_nodes_state(node, service_name, MsgTransition::TRANSITION_CONFIGURE, 10s);
}

int main(int argc, char * argv[])
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  std::string get_state_service_name = "raspimouse/get_state";
  std::string change_state_service_name = "raspimouse/change_state";

  auto node = rclcpp::Node::make_shared("object_tracking_observer");

  if (!all_nodes_are_unconfigured(node, get_state_service_name)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to launch nodes.");
    rclcpp::shutdown();
  }
  RCLCPP_INFO(node->get_logger(), "All nodes launched.");

  if (!configure_all_nodes(node, change_state_service_name)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to configure nodes.");
    rclcpp::shutdown();
  }
  RCLCPP_INFO(node->get_logger(), "All nodes configured.");

  while (rclcpp::ok()) {
    if (all_nodes_are_inactive(node, get_state_service_name)) {
      RCLCPP_INFO(node->get_logger(), "All nodes are inactive.");
    } else {
      // all node shutdown
      RCLCPP_ERROR(node->get_logger(), "Any node is not inactive.");
      break;
    }
    rclcpp::sleep_for(5s);
  }

  rclcpp::shutdown();

  return 0;
}
