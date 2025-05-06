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

#ifndef CONTROL__TF_PUBLISHER_HPP_
#define CONTROL__TF_PUBLISHER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"


namespace controlper
{

class TfPublisher : public rclcpp::Node
{
public:
  
  TfPublisher();

private:
  void cameracallback(const vision_msgs::msg::Detection3DArray::SharedPtr msg);
  tf2::BufferCore tf_buffer_;
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr camera_sub_;
  tf2_ros::TransformListener tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  geometry_msgs::msg::TransformStamped transform_;
};
} // namespace controlper

#endif  // CONTROL__TF_PUBLISHER_HPP_