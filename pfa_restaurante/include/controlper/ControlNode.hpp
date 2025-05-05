// Copyright 2024 Intelligent Robotics Lab
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

#ifndef CONTROL__CONTROL_NODE_HPP_
#define CONTROL__CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/transform_listener.h"
#include "controlper/PIDControler.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace controlper
{

class ControlNode : public rclcpp::Node
{
public:
  
  ControlNode();

private:
  void seguir();
  rclcpp::TimerBase::SharedPtr timer_;
  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr mover_pub_;
  geometry_msgs::msg::Twist message_;
  PIDController lin_pid_, rot_pid_;
  bool detected_ = true;
  double atrx;
  double atry;
  double lastz;
};
} // namespace controlper

#endif  // CONTROL__CONTROL_NODE_HPP_
