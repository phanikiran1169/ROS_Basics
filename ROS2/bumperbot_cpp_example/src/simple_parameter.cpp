#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <string>
#include <memory>
#include <vector>

using std::placeholders::_1;

class SimpleParameter : public rclcpp::Node
{

public:
    SimpleParameter() : Node("simple_parameter")
    {
        declare_parameter<int>("simple_int_param", 1);
        declare_parameter<std::string>("simple_string_param", "ROS2 Parameter");
        param_callback_handle_ = add_on_set_parameters_callback(std::bind(&SimpleParameter::paramChangeCallback, this, _1));
    }

private:
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    rcl_interfaces::msg::SetParametersResult paramChangeCallback(const std::vector<rclcpp::Parameter>& parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;

        for(const auto& parameter : parameters)
        {
            if(parameter.get_name() == "simple_int_param" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                RCLCPP_INFO_STREAM(get_logger(), "Parameter simple_int_param has been updated. The new value is " << parameter.as_int());
                result.successful = true;
            }

            if(parameter.get_name() == "simple_string_param" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
            {
                RCLCPP_INFO_STREAM(get_logger(), "Parameter simple_string_param has been updated. The new value is " << parameter.as_string());
                result.successful = true;
            }
        }

        return result;
    }


};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleParameter>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}