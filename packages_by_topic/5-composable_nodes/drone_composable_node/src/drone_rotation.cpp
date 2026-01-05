#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <cmath>


namespace drone_rotation{

class DroneRotation : public rclcpp::Node
{
public:
  explicit DroneRotation(const rclcpp::NodeOptions & options)
  : Node("drone_rotation", options), angle_(0.0)
  {
    rotation_pub_ = this->create_publisher<std_msgs::msg::Float64>("rotation", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(&DroneRotation::rotate_clockwise, this));

    RCLCPP_INFO(this->get_logger(), "DroneRotation started — rotating clockwise...");
  }

  ~DroneRotation() override
  {
    RCLCPP_INFO(this->get_logger(), "DroneRotation stopped — node unloaded.");
  }

private:
  void rotate_clockwise()
  {
    angle_ = std::fmod(angle_ + 5.0, 360.0);  // Rotate 5° each cycle
    std_msgs::msg::Float64 msg;
    msg.data = angle_;
    rotation_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Current rotation: %.1f°", angle_);
  }

  double angle_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rotation_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(drone_rotation::DroneRotation)
