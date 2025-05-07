/*
#include <string>
#include <iostream>

#include "controlper/GoToDoor.hpp"

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
  factory.registerNodeType<controlper::GoToDoor>("GoToDoor");
}
*/
// GoToDoor.cpp

#include "controlper/GoToDoor.hpp"

namespace controlper
{

GoToDoor::GoToDoor(const std::string& name, const BT::NodeConfiguration& config)
: BT::AsyncActionNode(name, config)
{
  node_ = rclcpp::Node::make_shared("go_to_door_bt_node");
  action_client_ = rclcpp_action::create_client<NavigateToPose>(node_, "navigate_to_pose");

  while (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_INFO(node_->get_logger(), "Waiting for nav2 action server...");
  }
}

BT::PortsList GoToDoor::providedPorts()
{
  return {};  // No input ports
}

BT::NodeStatus GoToDoor::tick()
{
  // Objetivo fijo
  double x = 13.0;
  double y = 13.0;
  double theta = 0.0;

  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.header.stamp = node_->now();
  goal.pose.position.x = x;
  goal.pose.position.y = y;
  goal.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, theta);
  goal.pose.orientation = tf2::toMsg(q);

  NavigateToPose::Goal nav_goal;
  nav_goal.pose = goal;

  auto send_goal_future = action_client_->async_send_goal(nav_goal);
  if (rclcpp::spin_until_future_complete(node_, send_goal_future) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to send goal");
    return BT::NodeStatus::FAILURE;
  }

  auto goal_handle = send_goal_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
    return BT::NodeStatus::FAILURE;
  }

  auto result_future = action_client_->async_get_result(goal_handle);
  if (rclcpp::spin_until_future_complete(node_, result_future) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "Failed while waiting for result");
    return BT::NodeStatus::FAILURE;
  }

  auto result = result_future.get();
  if (result.result->navigation_time.sec > 0) {
    RCLCPP_INFO(node_->get_logger(), "Navigation succeeded");
    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Navigation failed");
    return BT::NodeStatus::FAILURE;
  }
}

void GoToDoor::halt()
{
  RCLCPP_WARN(node_->get_logger(), "GoToDoor was halted");
  // Opcional: podrías cancelar la navegación aquí
}

}  // namespace controlper
