#ifndef WOLF_CONTROLLER_UTILS_PARAM_SETTER_ROS2_H
#define WOLF_CONTROLLER_UTILS_PARAM_SETTER_ROS2_H

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include <memory>
#include <string>
#include <chrono>
#include <type_traits>

namespace wolf_controller_utils {

#define WAIT_SEC 5  // Default timeout in seconds

// Generalized function to set a parameter on a remote node
template <typename T>
bool set_parameter_on_remote_node(
    const std::shared_ptr<rclcpp::Node>& node,
    const std::string& param_path,
    T value,
    std::chrono::seconds timeout = std::chrono::seconds(WAIT_SEC))
{
    size_t pos = param_path.find('/');
    if (pos == std::string::npos) {
        RCLCPP_ERROR(node->get_logger(), "Invalid parameter path format: %s. Expected format: '/node_name/param_name'", param_path.c_str());
        return false;
    }

    std::string remote_node_name = param_path.substr(0, pos);
    std::string param_name = param_path.substr(pos + 1);

    // Create a client for the SetParameters service
    auto client = node->create_client<rcl_interfaces::srv::SetParameters>(remote_node_name + "/set_parameters");
    if (!client->wait_for_service(timeout)) {
        RCLCPP_ERROR(node->get_logger(), "Service %s/set_parameters not available!", remote_node_name.c_str());
        return false;
    }

    // Create the request and set up the parameter
    auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    rcl_interfaces::msg::Parameter parameter;
    parameter.name = param_name;

    // Handle different parameter types using template specialization
    if constexpr (std::is_same<T, std::string>::value) {
        rcl_interfaces::msg::ParameterValue param_value;
        param_value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
        param_value.string_value = value;
        parameter.value = param_value;

    } else if constexpr (std::is_same<T, int64_t>::value) {
        rcl_interfaces::msg::ParameterValue param_value;
        param_value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
        param_value.integer_value = value;
        parameter.value = param_value;

    } else if constexpr (std::is_same<T, double>::value) {
        rcl_interfaces::msg::ParameterValue param_value;
        param_value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        param_value.double_value = value;
        parameter.value = param_value;

    } else if constexpr (std::is_same<T, bool>::value) {
        rcl_interfaces::msg::ParameterValue param_value;
        param_value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
        param_value.bool_value = value;
        parameter.value = param_value;

    } else {
        RCLCPP_ERROR(node->get_logger(), "Unsupported parameter type!");
        return false;
    }

    request->parameters.push_back(parameter);

    // Send the request and wait for the response
    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS) {
        auto response = future.get();
        if (response) {
            RCLCPP_INFO(node->get_logger(), "Successfully set parameter '%s' on remote node '%s'", param_name.c_str(), remote_node_name.c_str());
            return true;
        } else {
            RCLCPP_ERROR(node->get_logger(), "Failed to set parameter '%s' on remote node '%s'", param_name.c_str(), remote_node_name.c_str());
        }
    } else {
        RCLCPP_ERROR(node->get_logger(), "Service call to set parameter %s on node %s failed", param_name.c_str(), remote_node_name.c_str());
    }

    return false;
}

}

#endif
