#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace velocity_subscriber
{

class VelocitySubscriber : public rclcpp::Node
{
public:
  VelocitySubscriber(const rclcpp::NodeOptions & options) : Node("velocity_subscriber", options)
  {
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&VelocitySubscriber::topic_callback, this, std::placeholders::_1));
  }

private:
  void topic_callback(const geometry_msgs::msg::Twist & msg) const
  {
    RCLCPP_INFO(
      this->get_logger(), "Listening: linear.x: '%f', angular.z: '%f'", msg.linear.x,
      msg.angular.z);
  }
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

}  // namespace velocity_subscriber

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(velocity_subscriber::VelocitySubscriber)