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

const int Follower::NUM_OF_SAMPLES = 10;

Follower::Follower(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("follower", options),
  present_sensor_values_(SensorsType(SENSOR_NUM, 0)),
  sensor_line_values_(SensorsType(SENSOR_NUM, 0)),
  sensor_field_values_(SensorsType(SENSOR_NUM, 0)),
  line_thresholds_(SensorsType(SENSOR_NUM, 0)),
  sampling_values_(SensorsType(SENSOR_NUM, 0)),
  line_is_detected_by_sensor_(std::vector<bool>(SENSOR_NUM, false)),
  sampling_count_(0),
  line_values_are_sampled_(false), field_values_are_sampled_(false),
  line_sampling_(false), field_sampling_(false),
  can_publish_cmdvel_(false)
{
}

void Follower::on_cmd_vel_timer()
{
  if (line_sampling_ || field_sampling_) {
    return;
  }

  if (switches_.switch0) {
    set_motor_power(true);
    beep_start();

  } else if (switches_.switch1) {
    set_motor_power(false);
    beep_start();
    line_sampling_ = true;

  } else if (switches_.switch2) {
    set_motor_power(true);
    beep_start();
    field_sampling_ = true;
  }
  // Reset
  switches_ = raspimouse_msgs::msg::Switches();
  indicate_line_detections();

  // RCLCPP_INFO(this->get_logger(), "Line values: L:%d, ML:%d, MR:%d, R:%d",
  //   sensor_line_values_[LEFT],
  //   sensor_line_values_[MID_LEFT],
  //   sensor_line_values_[MID_RIGHT],
  //   sensor_line_values_[RIGHT]
  //   );

  // RCLCPP_INFO(this->get_logger(), "Field values: L:%d, ML:%d, MR:%d, R:%d",
  //   sensor_field_values_[LEFT],
  //   sensor_field_values_[MID_LEFT],
  //   sensor_field_values_[MID_RIGHT],
  //   sensor_field_values_[RIGHT]
  //   );

  RCLCPP_INFO(this->get_logger(), "Thresholds: L:%d, ML:%d, MR:%d, R:%d",
    line_thresholds_[LEFT],
    line_thresholds_[MID_LEFT],
    line_thresholds_[MID_RIGHT],
    line_thresholds_[RIGHT]
    );

  // RCLCPP_INFO(this->get_logger(), "Switches: %d, %d, %d",
  //   switches_.switch0, switches_.switch1, switches_.switch2);
  cmd_vel_.linear.x = 0.0;
  cmd_vel_.angular.z = 0.0;
  auto msg = std::make_unique<geometry_msgs::msg::Twist>(cmd_vel_);

  // RCLCPP_INFO(this->get_logger(), "Sensor values: L:%d, ML:%d, MR:%d, R:%d",
  //   present_sensor_values[LEFT],
  //   present_sensor_values[MID_LEFT],
  //   present_sensor_values[MID_RIGHT],
  //   present_sensor_values[RIGHT]
  //   );
  // cmd_vel_pub_->publish(std::move(msg));
}

void Follower::callback_light_sensors(const raspimouse_msgs::msg::LightSensors::SharedPtr msg)
{
  present_sensor_values_[LEFT] = msg->forward_r;
  present_sensor_values_[MID_LEFT] = msg->right;
  present_sensor_values_[MID_RIGHT] = msg->left;
  present_sensor_values_[RIGHT] = msg->forward_l;

  if (line_sampling_ || field_sampling_) {
    multisampling();
  }

  if (sampling_is_done()) {
    update_line_detection();
  }
}

void Follower::callback_switches(const raspimouse_msgs::msg::Switches::SharedPtr msg)
{
  switches_ = *msg;
}

void Follower::set_motor_power(const bool motor_on)
{
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = motor_on;
  auto future_result = motor_power_client_->async_send_request(request);
}

void Follower::update_line_detection(void)
{
  for (int sensor_i = 0; sensor_i < SENSOR_NUM; sensor_i++) {
    bool is_positive = present_sensor_values_[sensor_i] > line_thresholds_[sensor_i];

    if (line_is_bright() == is_positive) {
      line_is_detected_by_sensor_[sensor_i] = true;
    } else {
      line_is_detected_by_sensor_[sensor_i] = false;
    }
  }

}

bool Follower::line_is_bright(void)
{
  const SensorIndex SAMPLE = RIGHT;
  if (sensor_line_values_[SAMPLE] > sensor_field_values_[SAMPLE]) {
    return true;
  } else {
    return false;
  }
}

void Follower::indicate_line_detections(void)
{
  auto msg = std::make_unique<raspimouse_msgs::msg::Leds>();
  msg->led0 = line_is_detected_by_sensor_[RIGHT];
  msg->led1 = line_is_detected_by_sensor_[MID_RIGHT];
  msg->led2 = line_is_detected_by_sensor_[MID_LEFT];
  msg->led3 = line_is_detected_by_sensor_[LEFT];
  leds_pub_->publish(std::move(msg));
}

void Follower::beep_buzzer(const int freq, const std::chrono::nanoseconds & beep_time)
{
  auto msg = std::make_unique<std_msgs::msg::Int16>();
  msg->data = freq;
  buzzer_pub_->publish(std::move(msg));

  rclcpp::sleep_for(beep_time);

  msg = std::make_unique<std_msgs::msg::Int16>();
  msg->data = 0;
  buzzer_pub_->publish(std::move(msg));
}

void Follower::beep_start(void)
{
  beep_buzzer(1000, 500ms);
}

void Follower::beep_success(void)
{
  beep_buzzer(1000, 100ms);
  rclcpp::sleep_for(100ms);
  beep_buzzer(1000, 100ms);
}

void Follower::beep_failure(void)
{
  for (int i=0; i<4; i++) {
    beep_buzzer(500, 100ms);
    rclcpp::sleep_for(100ms);
  }
}

bool Follower::sampling_is_done(void)
{
  if (line_values_are_sampled_ && field_values_are_sampled_) {
    return true;
  } else {
    return false;
  }
}

void Follower::multisampling(void)
{
  if (sampling_count_ < NUM_OF_SAMPLES) {
    for (int sensor_i = 0; sensor_i < SENSOR_NUM; sensor_i++) {
      sampling_values_[sensor_i] += present_sensor_values_[sensor_i];
    }
    sampling_count_++;

  } else {
    for (int sensor_i = 0; sensor_i < SENSOR_NUM; sensor_i++) {
      sampling_values_[sensor_i] = sampling_values_[sensor_i] / NUM_OF_SAMPLES;
    }

    if (line_sampling_) {
      sensor_line_values_ = sampling_values_;
      line_values_are_sampled_ = true;
    } else {
      sensor_field_values_ = sampling_values_;
      field_values_are_sampled_ = true;
    }
    sampling_count_ = 0;
    sampling_values_ = SensorsType(SENSOR_NUM, 0);
    line_sampling_ = field_sampling_ = false;

    set_line_thresholds();
    beep_success();
  }
}

int Follower::median(const int value1, const int value2)
{
  int diff = std::abs(value1 - value2);

  if (value1 < value2) {
    return value1 + diff * 0.5;
  } else {
    return value2 + diff * 0.5;
  }
}

void Follower::set_line_thresholds(void)
{
  if (sampling_is_done() == false){
    return;
  }

  for (int sensor_i = 0; sensor_i < SENSOR_NUM; sensor_i++) {
    line_thresholds_[sensor_i] = median(
      sensor_line_values_[sensor_i], sensor_field_values_[sensor_i]);
  }
}

CallbackReturn Follower::on_configure(const rclcpp_lifecycle::State &)
{
  using namespace std::placeholders;  // for _1, _2, _3...

  RCLCPP_INFO(this->get_logger(), "on_configure() is called.");

  cmd_vel_timer_ = create_wall_timer(50ms, std::bind(&Follower::on_cmd_vel_timer, this));
  // Don't actually start publishing data until activated
  cmd_vel_timer_->cancel();

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
  buzzer_pub_ = create_publisher<std_msgs::msg::Int16>("buzzer", 1);
  leds_pub_ = create_publisher<raspimouse_msgs::msg::Leds>("leds", 1);
  light_sensors_sub_ = create_subscription<raspimouse_msgs::msg::LightSensors>(
    "light_sensors", 1, std::bind(&Follower::callback_light_sensors, this, _1));
  switches_sub_ = create_subscription<raspimouse_msgs::msg::Switches>(
    "switches", 1, std::bind(&Follower::callback_switches, this, _1));

  motor_power_client_ = create_client<std_srvs::srv::SetBool>("motor_power");
  if (!motor_power_client_->wait_for_service(5s)) {
    RCLCPP_ERROR(this->get_logger(),
      "Service motor_power is not avaliable.");
    return CallbackReturn::FAILURE;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn Follower::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "on_activate() is called.");

  buzzer_pub_->on_activate();
  cmd_vel_pub_->on_activate();
  leds_pub_->on_activate();
  cmd_vel_timer_->reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn Follower::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "on_deactivate() is called.");

  buzzer_pub_->on_deactivate();
  cmd_vel_pub_->on_deactivate();
  leds_pub_->on_deactivate();
  cmd_vel_timer_->cancel();

  cmd_vel_ = geometry_msgs::msg::Twist();

  return CallbackReturn::SUCCESS;
}

CallbackReturn Follower::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "on_cleanup() is called.");

  buzzer_pub_.reset();
  cmd_vel_pub_.reset();
  leds_pub_.reset();
  cmd_vel_timer_.reset();
  light_sensors_sub_.reset();
  switches_sub_.reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn Follower::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "on_shutdown() is called.");

  buzzer_pub_.reset();
  cmd_vel_pub_.reset();
  leds_pub_.reset();
  cmd_vel_timer_.reset();
  light_sensors_sub_.reset();
  switches_sub_.reset();

  return CallbackReturn::SUCCESS;
}

}  // namespace line_follower

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(line_follower::Follower)
