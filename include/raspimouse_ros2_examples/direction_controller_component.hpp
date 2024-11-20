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

#ifndef RASPIMOUSE_ROS2_EXAMPLES__DIRECTION_CONTROLLER_COMPONENT_HPP_
#define RASPIMOUSE_ROS2_EXAMPLES__DIRECTION_CONTROLLER_COMPONENT_HPP_

#include <memory>
#include <string>
#include <vector>

#include "raspimouse_ros2_examples/visibility_control.h"
#include "raspimouse_msgs/msg/switches.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"

namespace direction_controller
{

class PIDController
{
public:
  PIDController()
  : p_gain_(0.0), i_gain_(0.0), d_gain_(0.0), error1_(0.0), error2_(0.0), output_(0.0)
  {
  }

  double update(const double current, const double target)
  {
    double error = target - current;

    double delta_output = p_gain_ * (error - error1_);
    delta_output += i_gain_ * (error);
    delta_output += d_gain_ * (error - 2.0 * error1_ + error2_);

    output_ += delta_output;

    error2_ = error1_;
    error1_ = error;

    return output_;
  }

  void set_gain(const double p_gain, const double i_gain, const double d_gain)
  {
    p_gain_ = p_gain;
    i_gain_ = i_gain;
    d_gain_ = d_gain;
  }

  void reset_output_and_errors()
  {
    error1_ = 0.0;
    error2_ = 0.0;
    output_ = 0.0;
  }

private:
  double p_gain_;
  double i_gain_;
  double d_gain_;
  double error1_;
  double error2_;
  double output_;
};

class Controller : public rclcpp::Node
{
public:
  RASPIMOUSE_ROS2_EXAMPLES_PUBLIC
  explicit Controller(const rclcpp::NodeOptions & options);

protected:
  void on_cmd_vel_timer();

private:
  raspimouse_msgs::msg::Switches switches_;
  sensor_msgs::msg::Imu imu_data_raw_;

  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr buzzer_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heading_angle_pub_;
  rclcpp::Subscription<raspimouse_msgs::msg::Switches>::SharedPtr switches_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_data_raw_sub_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr motor_power_client_;

  rclcpp::TimerBase::SharedPtr cmd_vel_timer_;

  int pressed_switch_number_;
  int control_mode_;
  PIDController omega_pid_controller_;
  std::vector<double> omega_samples_;
  double omega_bias_;
  double heading_angle_;
  double prev_heading_calculation_time_;
  double target_angle_;
  bool increase_target_angle_;
  geometry_msgs::msg::Vector3 filtered_acc_;
  geometry_msgs::msg::Vector3 prev_acc_;

  void callback_switches(const raspimouse_msgs::msg::Switches::SharedPtr msg);
  void callback_imu_data_raw(const sensor_msgs::msg::Imu::SharedPtr msg);

  bool set_motor_power(const bool motor_on);
  void filter_acceleration(const geometry_msgs::msg::Vector3 acc);
  bool omega_calibration(const double omega);
  void calculate_heading_angle(const double omega, const double current_time);
  void angle_control(const double target_angle);
  void rotation(void);
  void beep_buzzer(const int freq, const std::chrono::nanoseconds & beep_time);
  void beep_start(void);
  void beep_success(void);
  void beep_failure(void);
};

}  // namespace direction_controller

#endif  // RASPIMOUSE_ROS2_EXAMPLES__DIRECTION_CONTROLLER_COMPONENT_HPP_
