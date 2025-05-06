#ifndef CONTROLPER__GOTODOOR_HPP_
#define CONTROLPER__GOTODOOR_HPP_

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace   controlper
{

class GoToDoor : public BT::ActionNodeBase
{
public:
  explicit GoToDoor(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();

private:
};

}  // controlper

#endif  // CONTROLPER__GOTODOOR_HPP_
