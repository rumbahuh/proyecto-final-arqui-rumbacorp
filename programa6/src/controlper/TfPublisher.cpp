#include "rclcpp/rclcpp.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "controlper/TfPublisher.hpp"

namespace controlper
{

using std::placeholders::_1;

TfPublisher::TfPublisher()
: Node("tf_publisher"), tf_buffer_(), tf_listener_(tf_buffer_)
{
  camera_sub_= create_subscription<vision_msgs::msg::Detection3DArray>(
    "detection_3d", 10,
    std::bind(&TfPublisher::cameracallback, this, _1));
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
}

void
TfPublisher::cameracallback(const vision_msgs::msg::Detection3DArray::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "Camera detect");

  transform_.header.stamp = now();
  transform_.header.frame_id = "camera_link";
  transform_.child_frame_id = "target";

  transform_.transform.translation.x = msg->detections[0].bbox.center.position.z;
  transform_.transform.translation.y = -(msg->detections[0].bbox.center.position.x);
  //transform_.transform.translation.z = msg->detections[0].bbox.center.position.-y;

  tf_broadcaster_->sendTransform(transform_);
}

}
