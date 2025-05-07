#include <string>
#include <iostream>
#include <limits>

#include "controlper/CountPeople.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace controlper
{

using namespace std::chrono_literals;

CountPeople::CountPeople(const std::string& name, const BT::NodeConfiguration& config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList CountPeople::providedPorts()
{
  return { BT::OutputPort<int>("out") };
}

BT::NodeStatus CountPeople::tick()
{
  int count = 0;
  std::cout << "How many persons? ";
  std::cin >> count;

  if (std::cin.fail() || count <= 0) {
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::cerr << "Invalid number of persons." << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  if (!setOutput("out", count)) {
    std::cerr << "Failed to set output [out] on blackboard." << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  std::cout << "Registered " << count << " person(s)." << std::endl;
  return BT::NodeStatus::SUCCESS;
}

}  // namespace controlper

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<controlper::CountPeople>("CountPeople");
}
