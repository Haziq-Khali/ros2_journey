#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>


namespace drone_altitude{

class DroneAltitude : public rclcpp::Node
{
public:
  explicit DroneAltitude(const rclcpp::NodeOptions & options)
  : Node("drone_altitude", options)
  {
    // Declare parameters
    this->declare_parameter<double>("target_altitude", 10.0);
    this->declare_parameter<double>("rate", 0.5);

    publisher_ = this->create_publisher<std_msgs::msg::Float64>("command_altitude", 10);
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&DroneAltitude::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "DroneAltitude started in ns: %s", this->get_namespace());
  }

private:
  void timer_callback()
  {
    double target_altitude = this->get_parameter("target_altitude").as_double();
    std_msgs::msg::Float64 msg;
    msg.data = target_altitude;
    RCLCPP_INFO(this->get_logger(), "Publishing target altitude: %.2f", msg.data);
    publisher_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(drone_altitude::DroneAltitude)
//RCLCPP_COMPONENTS_REGISTER_NODE(<namespace>::<class name>)
