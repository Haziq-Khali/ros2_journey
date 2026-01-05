#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "order_haziq_base/order_interfaces.hpp"

class OrderNode : public rclcpp::Node
{
public:
  OrderNode()
  : Node("order_node"),
    set_loader_("order_haziq_base", "order_haziq_base::OrderSet"),
    calc_loader_("order_haziq_base", "order_haziq_base::OrderCalculator")
  {
    // Declare parameters with defaults
    this->declare_parameter<std::string>("order_set", "SetA");
    this->declare_parameter<std::string>("calculator", "BasicCalculator");
    this->declare_parameter<int>("quantity", 1);

    // Register callback for live parameter updates
    callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&OrderNode::on_parameter_change, this, std::placeholders::_1)
    );

    // Timer to print result every 2 seconds
    timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&OrderNode::process_order, this));
  }

private:
  // Handle parameter changes dynamically
  rcl_interfaces::msg::SetParametersResult on_parameter_change(
    const std::vector<rclcpp::Parameter> &params)
  {
    RCLCPP_INFO(this->get_logger(), "Parameters updated.");
    return rcl_interfaces::msg::SetParametersResult().set__successful(true);
  }

  void process_order()
  {
    try {
      auto set_name = this->get_parameter("order_set").as_string();
      auto calc_name = this->get_parameter("calculator").as_string();
      int quantity = this->get_parameter("quantity").as_int();

      // Load plugins
      auto set = set_loader_.createSharedInstance(set_name);
      auto calculator = calc_loader_.createSharedInstance(calc_name);

      // Scale items by quantity
      auto items = set->get_items();
      for (auto &item : items) {
        item.quantity *= quantity;
      }

      // Calculate total
      double total = calculator->calculate_total(items);

      // Print result
      RCLCPP_INFO(this->get_logger(),
                  "Order: %d x %s | Calculator: %s | Total = %.2f",
                  quantity,
                  set->get_name().c_str(),
                  calculator->get_name().c_str(),
                  total);

    } catch (pluginlib::PluginlibException &ex) {
      RCLCPP_ERROR(this->get_logger(), "Plugin failed to load: %s", ex.what());
    }
  }

  // Plugin loaders
  pluginlib::ClassLoader<order_haziq_base::OrderSet> set_loader_;
  pluginlib::ClassLoader<order_haziq_base::OrderCalculator> calc_loader_;

  // ROS handles
  rclcpp::TimerBase::SharedPtr timer_;
  OnSetParametersCallbackHandle::SharedPtr callback_handle_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OrderNode>());
  rclcpp::shutdown();
  return 0;
}
