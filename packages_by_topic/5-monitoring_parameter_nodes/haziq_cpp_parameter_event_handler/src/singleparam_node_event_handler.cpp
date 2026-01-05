#include <memory>
#include "rclcpp/rclcpp.hpp"

class MonitorParameters : public rclcpp::Node
{
    public : MonitorParameters()
    : Node("node_with_parameters")
    {
        // Parameter subscriber (necessary)
        param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

        // Add callback to monitor any changes in remote node's parameters 
        // because the node is in another package named cpp_parameters_haziq.
        auto cb = [this](const rclcpp::Parameter & p){
            std::string value = p.as_string();
            RCLCPP_INFO(
                this->get_logger(), "Received an update to parameter \"%s\" of type: %s: \"%s\"",
                p.get_name().c_str(),
                p.get_type_name().c_str(),
                value.c_str()
            );
        };

        auto remote_node_name = std::string("haziq_param_node");
        auto remote_param_name = std::string("my_item");

        // Register callback
        cb_handle2_ = param_subscriber_->add_parameter_callback(remote_param_name, cb, remote_node_name);
    }

    private:
    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle2_;
};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MonitorParameters>());
    rclcpp::shutdown();
    return 0;
}
