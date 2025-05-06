#include <string>
#include <iostream>

#include "controlper/CountPeople.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace controlper
{

using namespace std::chrono_literals;


}  // namespace controlper

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<controlper::CountPeople>("CountPeople");
}