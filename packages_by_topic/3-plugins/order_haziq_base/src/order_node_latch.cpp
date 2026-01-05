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

    // Compute once at startup
    process_order("SetA", "BasicCalculator", 1);
  }

private:
  // Handle parameter changes dynamically
  rcl_interfaces::msg::SetParametersResult on_parameter_change(
    const std::vector<rclcpp::Parameter> &params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    // Use local variables with current values
    std::string set_name = this->get_parameter("order_set").as_string();
    std::string calc_name = this->get_parameter("calculator").as_string();
    int quantity = this->get_parameter("quantity").as_int();

    // Override them with any just-updated params
    for (const auto &param : params) {
      if (param.get_name() == "order_set") {
        set_name = param.as_string();
      } else if (param.get_name() == "calculator") {
        calc_name = param.as_string();
      } else if (param.get_name() == "quantity") {
        quantity = param.as_int();
      }
    }

    // Now recompute using the new values immediately
    process_order(set_name, calc_name, quantity);
    return result;
  }

  void process_order(const std::string &set_name,
                     const std::string &calc_name,
                     int quantity)
  {
    try {
      // Load plugins dynamically
      auto set = set_loader_.createSharedInstance(set_name);
      auto calculator = calc_loader_.createSharedInstance(calc_name);

      // Scale items by quantity
      auto items = set->get_items();
      for (auto &item : items) {
        item.quantity *= quantity;
      }

      // Calculate total
      double total = calculator->calculate_total(items);

      // Just print result to terminal
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
  OnSetParametersCallbackHandle::SharedPtr callback_handle_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OrderNode>());
  rclcpp::shutdown();
  return 0;
}
