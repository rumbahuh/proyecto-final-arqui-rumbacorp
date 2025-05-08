#include "controlper/Speak.hpp"

namespace controlper
{

Speak::Speak(const std::string &name, const BT::NodeConfiguration &config)
: BT::SyncActionNode(name, config)
{
  node_ = rclcpp::Node::make_shared("speak_bt_node");
  tts_pub_ = node_->create_publisher<std_msgs::msg::String>("/tts", 10);
}

BT::NodeStatus Speak::tick()
{
  std::string text;
  if (!getInput("text", text)) {
    RCLCPP_ERROR(node_->get_logger(), "Speak: missing 'text' input");
    return BT::NodeStatus::FAILURE;
  }

  std_msgs::msg::String msg;
  msg.data = text;
  tts_pub_->publish(msg);
  RCLCPP_INFO(node_->get_logger(), "Speaking: '%s'", text.c_str());

  // Opcional: espera corta
  rclcpp::sleep_for(std::chrono::seconds(2));

  return BT::NodeStatus::SUCCESS;
}

}  // namespace controlper

