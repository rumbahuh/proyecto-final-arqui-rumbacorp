/*
#include <string>
#include <iostream>

#include "controlper/GoToTable.hpp"


#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace controlper
{

using namespace std::chrono_literals;


}  // namespace controlper

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<controlper::GoToTable>("GoToTable");
}
*/
// GoToTable.cpp
#include "behaviortree_cpp_v3/behavior_tree.h"

#include "controlper/GoToTable.hpp"

namespace controlper
{

GoToTable::GoToTable(const std::string& name, const BT::NodeConfiguration& config)
: BT::AsyncActionNode(name, config)
{
  node_ = rclcpp::Node::make_shared("go_to_Table_bt_node");
  action_client_ = rclcpp_action::create_client<NavigateToPose>(node_, "navigate_to_pose");

  while (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_INFO(node_->get_logger(), "Waiting for nav2 action server...\n");
  }
}

BT::PortsList GoToTable::providedPorts()
{
  return {};  // No input ports
}

BT::NodeStatus GoToTable::tick()
{
  // Dos objetivos, mesa de seis persona o mesa de cuatro
  /* coordenadas mesa de 6>n>4 :
  	x: -2.6915394668397883
		y: -2.809132875211364
		z: 0.0
  
  coordenadas mesa pequeña (la mesa pequeña 3) 4>n :
  x: -0.49837595256644057
	y: -3.049161757190125 
	z: 0.0
	*/
  
  auto bb = config().blackboard;
  std::string mesa_destino;

  if (!bb->get("mesa_destino", mesa_destino)) {
    RCLCPP_ERROR(node_->get_logger(), "Could not find 'mesa_destino' on the blackboard.\n");
    return BT::NodeStatus::FAILURE;
  }
  
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;

	if (mesa_destino == "BIG") {
    x = -2.6915;
    y = -2.8091;
    theta = 0.0;
    RCLCPP_INFO(node_->get_logger(), "On path to BIG desk.\n");
  } else if (mesa_destino == "SMALL") {
    x = -0.4983;
    y = -3.0491;
    theta = 0.0;
    RCLCPP_INFO(node_->get_logger(), "On path to SMALL desk.\n");
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Unwanted value on variable mesa_destino: '%s'\n", mesa_destino.c_str());
    return BT::NodeStatus::FAILURE;
  }

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

  auto start_time = node_->now();  // ← Marca de tiempo inicial

  auto send_goal_future = action_client_->async_send_goal(nav_goal);
  if (rclcpp::spin_until_future_complete(node_, send_goal_future) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to send goal\n");
    return BT::NodeStatus::FAILURE;
  }

  auto goal_handle = send_goal_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server\n");
    return BT::NodeStatus::FAILURE;
  }

  auto result_future = action_client_->async_get_result(goal_handle);
  if (rclcpp::spin_until_future_complete(node_, result_future) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "Failed while waiting for result\n");
    return BT::NodeStatus::FAILURE;
  }

  auto result = result_future.get();
  auto end_time = node_->now();  // ← Marca de tiempo final

  rclcpp::Duration duration = end_time - start_time;

  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(node_->get_logger(), "Navigation succeeded in %.2f seconds\n", duration.seconds());
    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Navigation failed\n");
    return BT::NodeStatus::FAILURE;
  }
}


void GoToTable::halt()
{
  RCLCPP_WARN(node_->get_logger(), "GoToTable was halted\n");
  // Opcional: podrías cancelar la navegación aquí
}

}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<controlper::GoToTable>("GoToTable");
}
