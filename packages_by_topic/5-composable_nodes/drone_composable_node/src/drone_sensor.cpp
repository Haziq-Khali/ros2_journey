#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <chrono>
#include <cmath>

namespace drone_sensor
{

class DroneSensor : public rclcpp::Node
{
public:
  explicit DroneSensor(const rclcpp::NodeOptions & options)
  : Node("drone_sensor", options),
    target_altitude_(0.0),
    current_altitude_(0.0),
    target_rotation_(0.0),
    current_rotation_(0.0)
  {
    // --- Declare parameters ---
    this->declare_parameter<std::string>("rotation_direction", "cw");  // cw or ccw
    this->declare_parameter<double>("rotation_speed", 2.0);            // degrees per tick

    // --- Publishers ---
    altitude_pub_ = this->create_publisher<std_msgs::msg::Float64>("altitude", 10);
    rotation_pub_ = this->create_publisher<std_msgs::msg::Float64>("rotation", 10);

    // --- Subscribers ---
    command_altitude_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "command_altitude", 10,
      std::bind(&DroneSensor::command_altitude_callback, this, std::placeholders::_1));

    command_rotation_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "command_rotation", 10,
      std::bind(&DroneSensor::command_rotation_callback, this, std::placeholders::_1));

    // --- Timer ---
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&DroneSensor::update_state, this));

    RCLCPP_INFO(this->get_logger(),
      "✅ DroneSensor started (namespace: '%s')",
      this->get_namespace());
  }

private:
  // --- Callbacks for commands ---
  void command_altitude_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    target_altitude_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "Received new target altitude: %.2f", target_altitude_);
  }

  void command_rotation_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    target_rotation_ = fmod(msg->data, 360.0);  // keep within 0–360°
    RCLCPP_INFO(this->get_logger(), "Received new target rotation: %.2f°", target_rotation_);
  }

  // --- Periodic update ---
  void update_state()
  {
    // Get parameters each cycle (so you can change at runtime)
    std::string rotation_dir = this->get_parameter("rotation_direction").as_string();
    double rotation_speed = this->get_parameter("rotation_speed").as_double();

    // --- Update altitude ---
    if (current_altitude_ < target_altitude_ - 0.1)
      current_altitude_ += 0.2;
    else if (current_altitude_ > target_altitude_ + 0.1)
      current_altitude_ -= 0.2;

    // --- Update rotation ---
    double diff = target_rotation_ - current_rotation_;

    // Normalize to [-180, 180] range
    while (diff > 180.0) diff -= 360.0;
    while (diff < -180.0) diff += 360.0;

    if (fabs(diff) > 1.0)
    {
      if (rotation_dir == "cw")
        current_rotation_ += (diff > 0 ? rotation_speed : -rotation_speed);
      else if (rotation_dir == "ccw")
        current_rotation_ -= (diff > 0 ? rotation_speed : -rotation_speed);
    }

    // Wrap around 0–360
    if (current_rotation_ < 0) current_rotation_ += 360.0;
    if (current_rotation_ >= 360.0) current_rotation_ -= 360.0;

    // --- Publish updated states ---
    std_msgs::msg::Float64 alt_msg;
    alt_msg.data = current_altitude_;
    altitude_pub_->publish(alt_msg);

    std_msgs::msg::Float64 rot_msg;
    rot_msg.data = current_rotation_;
    rotation_pub_->publish(rot_msg);

    // --- Log info ---
    RCLCPP_INFO(this->get_logger(),
      "Altitude: %.2f | Rotation: %.1f° | Direction: %s",
      current_altitude_, current_rotation_, rotation_dir.c_str());
  }

  // --- Member variables ---
  double target_altitude_;
  double current_altitude_;
  double target_rotation_;
  double current_rotation_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr altitude_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rotation_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr command_altitude_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr command_rotation_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace drone_sensor

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(drone_sensor::DroneSensor)
