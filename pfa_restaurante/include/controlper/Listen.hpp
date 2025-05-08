#pragma once

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "whisper_msgs/msg/transcription.hpp"

namespace controlper
{

class Listen : public BT::SyncActionNode
{
public:
  Listen(const std::string &name, const BT::NodeConfiguration &config);

  static BT::PortsList providedPorts()
  {
    return { BT::OutputPort<std::string>("text") };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::string last_transcription_;
  bool received_;
  rclcpp::Subscription<whisper_msgs::msg::Transcription>::SharedPtr sub_;

  void callback(const whisper_msgs::msg::Transcription::SharedPtr msg);
};

}  // namespace controlper

