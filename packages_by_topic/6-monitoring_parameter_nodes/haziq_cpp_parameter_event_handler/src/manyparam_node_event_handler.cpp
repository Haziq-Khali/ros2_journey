#include <memory>
#include "rclcpp/rclcpp.hpp"

class MonitorParameters : public rclcpp::Node
{
public:
  MonitorParameters()
  : Node("node_with_parameters")
  {
    // Parameter event handler for this node
    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

    auto cb = [this](const rclcpp::Parameter & p) {
      std::string value;

      // Handle both string and integer parameter types
      if (p.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
        value = p.as_string();
      } else if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        value = std::to_string(p.as_int());
      } else {
        value = "<unsupported type>";
      }

      RCLCPP_INFO(
        this->get_logger(),
        "Received update from [%s]: \"%s\" = %s (%s)",
        p.get_name().c_str(),
        p.get_name().c_str(),
        value.c_str(),
        p.get_type_name().c_str()
      );
    };

    auto remote_node_name = std::string("haziq_param_node");

    // Register callbacks for all parameters you want to monitor
    cb_handle_param_   = param_subscriber_->add_parameter_callback("my_parameter", cb, remote_node_name);
    cb_handle_item_    = param_subscriber_->add_parameter_callback("my_item", cb, remote_node_name);
    cb_handle_quantity_= param_subscriber_->add_parameter_callback("my_quantity", cb, remote_node_name);
  }

private:
  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_param_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_item_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_quantity_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MonitorParameters>());
  rclcpp::shutdown();
  return 0;
}
