#include <rclcpp/rclcpp.hpp>
#include <string>

class HaziqParam : public rclcpp::Node
{
public:
  HaziqParam()
  : Node("haziq_param_node")
  {
    // Declare parameters with default values
    this->declare_parameter<std::string>("my_parameter", "world");
    this->declare_parameter<int>("my_quantity", 9);
    this->declare_parameter<std::string>("my_item", "Nvidia Graphic");

    // Create timer (1 second)
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&HaziqParam::timer_callback, this)
    );
  }

private:
  void timer_callback()
  {
    // Get parameters
    auto my_param = this->get_parameter("my_parameter").as_string();
    auto my_item = this->get_parameter("my_item").as_string();
    auto my_quantity = this->get_parameter("my_quantity").as_int();

    // Log the message
    RCLCPP_INFO(
      this->get_logger(),
      "Hello %s! We only sell %ld %s",
      my_param.c_str(),
      my_quantity,
      my_item.c_str()
    );

    // Example of setting a new parameter (optional)
    // rclcpp::Parameter new_param("my_parameter", "world");
    // this->set_parameters({new_param});
  }

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HaziqParam>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
