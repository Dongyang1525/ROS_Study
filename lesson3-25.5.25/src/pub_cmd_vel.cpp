#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace velocity_publisher
{

class VelocityPublisher : public rclcpp::Node
{
public:
  VelocityPublisher(const rclcpp::NodeOptions & options) : Node("velocity_publisher", options)
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    timer_ = this->create_wall_timer(1000ms, std::bind(&VelocityPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = velocity_.linear.x;
    message.angular.z = velocity_.angular.z;
    RCLCPP_INFO(
      this->get_logger(), "Publishing: linear.x: '%f', angular.z: '%f'", message.linear.x,
      message.angular.z);
    publisher_->publish(message);
    velocity_update();
  }

  void velocity_update()
  {
    velocity_.linear.x += 0.2;
    velocity_.angular.z += 0.2;

    if (velocity_.linear.x > 1.0) {
      velocity_.linear.x = -1.0;
    }
    if (velocity_.angular.z > 1.0) {
      velocity_.angular.z = -1.0;
    }
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  geometry_msgs::msg::Twist velocity_;
};

}  // namespace velocity_publisher

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(velocity_publisher::VelocityPublisher)