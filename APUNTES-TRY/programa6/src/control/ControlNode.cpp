#include "rclcpp/rclcpp.hpp"
#include "control/ControlNode.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "control/PIDControler.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath>

namespace control
{

using std::placeholders::_1;
using namespace std::chrono_literals;

ControlNode::ControlNode()
: Node("control_node"), tf_buffer_(), tf_listener_(tf_buffer_), lin_pid_(0.0, 1.0, 0.0, 0.7), rot_pid_(0.0, 1.0, 0.3, 1.0), last_(this->now())
{
  mover_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  timer_ = create_wall_timer(50ms, std::bind(&ControlNode::seguir, this));
  message_.linear.x = 0;
  message_.angular.z = 0;
  atrx = 0;
  atry = 0;
}

void
ControlNode::seguir()
{
  RCLCPP_INFO(get_logger(), "Camera detect");
  last_ = this->now();

  tf2::Stamped<tf2::Transform> bf2camera;
  std::string error;

  if (tf_buffer_.canTransform("base_footprint", "camera_link", tf2::TimePointZero, &error)) {
    auto bf2camera_msg = tf_buffer_.lookupTransform(
      "base_footprint", "camera_link", tf2::TimePointZero);
    
    tf2::fromMsg(bf2camera_msg, bf2camera);
    }

  tf2::Stamped<tf2::Transform> camera2target;

  RCLCPP_INFO(get_logger(), "2");
  
  if (tf_buffer_.canTransform("camera_link", "target", tf2::TimePointZero, &error)) {
    auto camera2target_msg = tf_buffer_.lookupTransform(
      "camera_link", "target", tf2::TimePointZero);
    
    tf2::fromMsg(camera2target_msg, camera2target);
    }

  auto bf2target = bf2camera * camera2target;

  atrx = bf2target.getOrigin().x();
  atry = bf2target.getOrigin().y();

  RCLCPP_INFO(
    get_logger(), "Pelota en = (%lf, %lf)", atrx, atry);
  

  if ((this->now() - ).seconds() < 0.1){
    detected_=true;
  } else {
    detected_=false;
  }

  if (detected_){
    double x = atrx;
    double y = atry;

    double angle = atan2(y, x);
    double dist = sqrt(x * x + y * y);

    message_.angular.z = std::clamp(rot_pid_.get_output(angle), -3.0, 3.0);
    message_.linear.x = std::clamp(lin_pid_.get_output(dist-1.0), -0.5, 0.5);

    lastz = atan2(atrx, atry)/abs(atan2(atrx, atry));

    mover_pub_->publish(message_);
  } else {
    message_.angular.z = 1 * lastz;
    message_.linear.x = 0;

    mover_pub_->publish(message_);
  }
}
} // namespace control
