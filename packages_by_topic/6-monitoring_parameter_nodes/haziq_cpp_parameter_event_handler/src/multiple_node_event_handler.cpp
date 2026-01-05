#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"

class SampleNodeWithParameters : public rclcpp::Node
{
public:
  SampleNodeWithParameters()
  : Node("ultimate_node_parameter_monitoring")
  {
    // Create a parameter event handler for monitoring other nodes
    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

    // Description for parameters
    rcl_interfaces::msg::ParameterDescriptor descriptor_a;  
    descriptor_a.description = "Set a node to monitor.";
    this->declare_parameter<std::string>("remote_node_name", "haziq_param_limit_node", descriptor_a);      //CHANGE

    rcl_interfaces::msg::ParameterDescriptor descriptor_b;
    descriptor_b.description = "Set a parameter to monitor.";
    this->declare_parameter<std::string>("remote_param_name", "my_quantity_limit", descriptor_b);   //CHANGE

    // Get initial parameters
    remote_node_name_ = this->get_parameter("remote_node_name").as_string();
    remote_param_name_ = this->get_parameter("remote_param_name").as_string();

    // Start monitoring
    setup_remote_monitoring();

    // Watch for changes to *this node’s* parameters (so user can swap target remotely)
    parameter_change_callback_handle_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params)
      {
        bool updated = false;
        for (const auto & param : params) {
          if (param.get_name() == "remote_node_name") {
            remote_node_name_ = param.as_string();
            updated = true;
          } else if (param.get_name() == "remote_param_name") {
            remote_param_name_ = param.as_string();
            updated = true;
          }
        }

        if (updated) {
          RCLCPP_INFO(
            this->get_logger(),
            "Swapping to monitor '%s' from node '%s'...",
            remote_param_name_.c_str(), remote_node_name_.c_str());

          // Recreate the subscription to the new node/param
          setup_remote_monitoring();
        }

        return rcl_interfaces::msg::SetParametersResult().set__successful(true);
      });
  }

private:
  void setup_remote_monitoring()
  {
    // Remove old callback handle if it exists
    cb_handle_a_.reset();

    auto cb_generic = [this](const rclcpp::Parameter & p)
    {
      RCLCPP_INFO(
        this->get_logger(),
        "Update from [%s]: \"%s\" = %s (type: %s)",
        remote_node_name_.c_str(),
        p.get_name().c_str(),
        p.value_to_string().c_str(),
        p.get_type_name().c_str());
    };

    cb_handle_a_ = param_subscriber_->add_parameter_callback(remote_param_name_, cb_generic, remote_node_name_);

    RCLCPP_INFO(
      this->get_logger(),
      "Now monitoring \"%s\" from node \"%s\"",
      remote_param_name_.c_str(),
      remote_node_name_.c_str());
  }

  // Members
  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_a_;
  OnSetParametersCallbackHandle::SharedPtr parameter_change_callback_handle_;
  std::string remote_node_name_;
  std::string remote_param_name_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SampleNodeWithParameters>());
  rclcpp::shutdown();
  return 0;
}