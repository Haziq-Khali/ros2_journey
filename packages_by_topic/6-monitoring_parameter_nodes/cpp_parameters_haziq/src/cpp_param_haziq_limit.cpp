#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/integer_range.hpp"
#include <string>

class HaziqParam : public rclcpp::Node
{
public:
  HaziqParam()
  : Node("haziq_param_limit_node")
  {
    // ----------------------------
    // Integer range for my_quantity (1 to 100)
    // ----------------------------
    rcl_interfaces::msg::IntegerRange int_range;
    int_range.from_value = 1;
    int_range.to_value = 100;
    int_range.step = 1;

    // Parameter descriptor for my_quantity
    rcl_interfaces::msg::ParameterDescriptor quantity_descriptor;
    quantity_descriptor.description = "Allowed range for my_quantity (1–100)";
    quantity_descriptor.integer_range.push_back(int_range);

    // ----------------------------
    // Declare parameters
    // ----------------------------
    this->declare_parameter<std::string>("my_parameter_limit", "world");
    this->declare_parameter<int>("my_quantity_limit", 9, quantity_descriptor);
    this->declare_parameter<std::string>("my_item_limit", "Nvidia Graphic");

    // ----------------------------
    // Timer (1 second)
    // ----------------------------
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&HaziqParam::timer_callback, this)
    );
  }

private:
  void timer_callback()
  {
    // Get parameter values
    auto my_param = this->get_parameter("my_parameter_limit").as_string();
    auto my_item = this->get_parameter("my_item_limit").as_string();
    auto my_quantity = this->get_parameter("my_quantity_limit").as_int();

    // Check stock limit
    if (my_quantity > 100)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "We don't have that much in stock! (Requested: %ld %s)",
        my_quantity,
        my_item.c_str()
      );
    }
    else
    {
      RCLCPP_INFO(
        this->get_logger(),
        "Hello %s! We only sell %ld %s",
        my_param.c_str(),
        my_quantity,
        my_item.c_str()
      );
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HaziqParam>());
  rclcpp::shutdown();
  return 0;
}
