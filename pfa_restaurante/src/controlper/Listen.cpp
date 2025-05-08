#include "controlper/Listen.hpp"

namespace controlper
{

Listen::Listen(const std::string &name, const BT::NodeConfiguration &config)
: BT::SyncActionNode(name, config), received_(false)
{
  node_ = rclcpp::Node::make_shared("listen_bt_node");

  sub_ = node_->create_subscription<whisper_msgs::msg::Transcription>(
    "/whisper/transcription", 10,
    std::bind(&Listen::callback, this, std::placeholders::_1));
}

void Listen::callback(const whisper_msgs::msg::Transcription::SharedPtr msg)
{
  if (!msg->text.empty()) {
    last_transcription_ = msg->text;
    received_ = true;
    RCLCPP_INFO(node_->get_logger(), "Heard: '%s'", msg->text.c_str());
  }
}

BT::NodeStatus Listen::tick()
{
  received_ = false;

  // Esperar a recibir transcripción (máx. 5s)
  rclcpp::Time start = node_->now();
  while (!received_ && (node_->now() - start).seconds() < 5.0) {
    rclcpp::spin_some(node_);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  if (!received_) {
    RCLCPP_WARN(node_->get_logger(), "No speech detected");
    return BT::NodeStatus::FAILURE;
  }

  setOutput("text", last_transcription_);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace controlper

