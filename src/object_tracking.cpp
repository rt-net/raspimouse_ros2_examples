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

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

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

std::uint8_t state_of(
  std::string target_node_name,
  rclcpp::Node::SharedPtr node, std::chrono::seconds time_out = 10s)
{
  auto request = std::make_shared<SrvGetState::Request>();
  auto service_name = target_node_name + "/get_state";
  auto client = node->create_client<SrvGetState>(service_name);

  if (!client->wait_for_service(time_out)) {
    RCLCPP_ERROR(node->get_logger(), "Service is not avaliable.");
    return MsgState::PRIMARY_STATE_UNKNOWN;
  }

  auto future_result = client->async_send_request(request);
  auto future_status = rclcpp::spin_until_future_complete(node, future_result, time_out);

  if (future_status != rclcpp::executor::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Service time out while getting current state.");
    return MsgState::PRIMARY_STATE_UNKNOWN;
  }

  return future_result.get()->current_state.id;
}

bool all_nodes_are_unconfigured(
  rclcpp::Node::SharedPtr node,
  const std::vector<std::string> & target_node_names)
{
  return std::all_of(target_node_names.begin(), target_node_names.end(),
           [&](std::string s) {
             return state_of(s, node, 10s) == MsgState::PRIMARY_STATE_UNCONFIGURED;
           });
}

bool all_nodes_are_inactive(
  rclcpp::Node::SharedPtr node,
  const std::vector<std::string> & target_node_names)
{
  return std::all_of(target_node_names.begin(), target_node_names.end(),
           [&](std::string s) {
             return state_of(s, node, 10s) == MsgState::PRIMARY_STATE_INACTIVE;
           });
}

bool all_nodes_are_active(
  rclcpp::Node::SharedPtr node,
  const std::vector<std::string> & target_node_names)
{
  return std::all_of(target_node_names.begin(), target_node_names.end(),
           [&](std::string s) {
             return state_of(s, node, 10s) == MsgState::PRIMARY_STATE_ACTIVE;
           });
}

bool change_state(
  std::string target_node_name, rclcpp::Node::SharedPtr node,
  std::uint8_t transition, std::chrono::seconds time_out = 10s)
{
  auto request = std::make_shared<SrvChangeState::Request>();
  request->transition.id = transition;

  auto service_name = target_node_name + "/change_state";
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

bool configure_all_nodes(
  rclcpp::Node::SharedPtr node,
  const std::vector<std::string> & target_node_names)
{
  return std::all_of(target_node_names.begin(), target_node_names.end(),
           [&](std::string s) {
             return change_state(s, node, MsgTransition::TRANSITION_CONFIGURE, 10s);
           });
}

bool activate_all_nodes(
  rclcpp::Node::SharedPtr node,
  const std::vector<std::string> & target_node_names)
{
  return std::all_of(target_node_names.begin(), target_node_names.end(),
           [&](std::string s) {
             return change_state(s, node, MsgTransition::TRANSITION_ACTIVATE, 10s);
           });
}


int main(int argc, char * argv[])
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  std::vector<std::string> target_node_names{"raspimouse", "tracker"};

  auto node = rclcpp::Node::make_shared("object_tracking_observer");

  if (!all_nodes_are_unconfigured(node, target_node_names)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to launch nodes.");
    rclcpp::shutdown();
  } else {
    RCLCPP_INFO(node->get_logger(), "All nodes launched.");
  }

  if (!configure_all_nodes(node, target_node_names)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to configure nodes.");
    rclcpp::shutdown();
  } else {
    RCLCPP_INFO(node->get_logger(), "All nodes configured.");
  }

  if (!activate_all_nodes(node, target_node_names)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to activate nodes.");
    rclcpp::shutdown();
  } else {
    RCLCPP_INFO(node->get_logger(), "All nodes configured.");
  }

  while (rclcpp::ok()) {
    if (all_nodes_are_active(node, target_node_names)) {
      RCLCPP_INFO(node->get_logger(), "All nodes are active.");
    } else {
      // all node shutdown
      RCLCPP_ERROR(node->get_logger(), "Any node is not active.");
      break;
    }
    rclcpp::sleep_for(5s);
  }

  rclcpp::shutdown();

  return 0;
}
