#ifndef WOLF_CONTROLLER_UTILS_PARAM_GETTER_ROS2_H
#define WOLF_CONTROLLER_UTILS_PARAM_GETTER_ROS2_H

#include <rcl_interfaces/srv/get_parameters.hpp>

namespace wolf_controller_utils
{

#define WAIT_SEC 10

using GetParameters = rcl_interfaces::srv::GetParameters;

// Utility to get a singleton node instance
std::shared_ptr<rclcpp::Node> get_global_node() {
    static auto global_node = rclcpp::Node::make_shared("ros2_param_getter");
    return global_node;
}

// Generalized function to retrieve a parameter of any type
template <typename T>
T get_parameter_from_remote_node(
    const std::shared_ptr<rclcpp::Node>& node,
    const std::string& param_path,
    T default_value,
    unsigned int expected_type,
    std::chrono::seconds timeout = std::chrono::seconds(WAIT_SEC))
{
    size_t pos = param_path.find('/');
    if (pos == std::string::npos) {
        RCLCPP_ERROR(node->get_logger(), "Invalid parameter path format: %s. Expected format: '/node_name/param_name'", param_path.c_str());
        return default_value;
    }

    std::string remote_node_name = param_path.substr(0, pos);
    std::string param_name = param_path.substr(pos + 1);

    auto client = node->create_client<GetParameters>(remote_node_name + "/get_parameters");
    if (!client->wait_for_service(timeout)) {
        RCLCPP_ERROR(node->get_logger(), "Service %s/get_parameters not available!", remote_node_name.c_str());
        return default_value;
    }

    auto request = std::make_shared<GetParameters::Request>();
    request->names.push_back(param_name);

    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS) {
        auto result = future.get();
        if (!result->values.empty()) {
            auto param_value = result->values[0];
            if (param_value.type == expected_type) {
                if constexpr (std::is_same<T, std::string>::value) {return param_value.string_value;}
                if constexpr (std::is_same<T, int64_t>::value) {return param_value.integer_value;}
                if constexpr (std::is_same<T, double>::value) {return param_value.double_value;}
                if constexpr (std::is_same<T, bool>::value) {return param_value.bool_value;}
            } else {
                RCLCPP_ERROR(node->get_logger(), "Parameter %s is not of expected type", param_name.c_str());
            }
        } else {
            RCLCPP_ERROR(node->get_logger(), "Parameter %s not found on remote node %s", param_name.c_str(), remote_node_name.c_str());
        }
    } else {
        RCLCPP_ERROR(node->get_logger(), "Service call to get parameter %s from node %s failed", param_name.c_str(), remote_node_name.c_str());
    }

    return default_value;
}

// Specialized functions for specific parameter types, using the generalized function

inline std::string get_string_parameter_from_remote_node(
    const std::shared_ptr<rclcpp::Node>& node, const std::string& param_path, const std::string& default_value = "", std::chrono::seconds timeout = std::chrono::seconds(WAIT_SEC))
{
    return get_parameter_from_remote_node<std::string>(node, param_path, default_value, rcl_interfaces::msg::ParameterType::PARAMETER_STRING, timeout);
}

inline int64_t get_int_parameter_from_remote_node(
    const std::shared_ptr<rclcpp::Node>& node, const std::string& param_path, int64_t default_value = 0, std::chrono::seconds timeout = std::chrono::seconds(WAIT_SEC))
{
    return get_parameter_from_remote_node<int64_t>(node, param_path, default_value, rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER, timeout);
}

inline double get_double_parameter_from_remote_node(
    const std::shared_ptr<rclcpp::Node>& node, const std::string& param_path, double default_value = 0.0, std::chrono::seconds timeout = std::chrono::seconds(WAIT_SEC))
{
    return get_parameter_from_remote_node<double>(node, param_path, default_value, rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE, timeout);
}

inline bool get_bool_parameter_from_remote_node(
    const std::shared_ptr<rclcpp::Node>& node, const std::string& param_path, bool default_value = false, std::chrono::seconds timeout = std::chrono::seconds(WAIT_SEC))
{
    return get_parameter_from_remote_node<bool>(node, param_path, default_value, rcl_interfaces::msg::ParameterType::PARAMETER_BOOL, timeout);
}

// Overloads that use the global node
inline std::string get_string_parameter_from_remote_node(
    const std::string& param_path, const std::string& default_value = "", std::chrono::seconds timeout = std::chrono::seconds(WAIT_SEC))
{
    return get_string_parameter_from_remote_node(get_global_node(), param_path, default_value, timeout);
}

inline int64_t get_int_parameter_from_remote_node(
    const std::string& param_path, int64_t default_value = 0, std::chrono::seconds timeout = std::chrono::seconds(WAIT_SEC))
{
    return get_int_parameter_from_remote_node(get_global_node(), param_path, default_value, timeout);
}

inline double get_double_parameter_from_remote_node(
    const std::string& param_path, double default_value = 0.0, std::chrono::seconds timeout = std::chrono::seconds(WAIT_SEC))
{
    return get_double_parameter_from_remote_node(get_global_node(), param_path, default_value, timeout);
}

inline bool get_bool_parameter_from_remote_node(
    const std::string& param_path, bool default_value = false, std::chrono::seconds timeout = std::chrono::seconds(WAIT_SEC))
{
    return get_bool_parameter_from_remote_node(get_global_node(), param_path, default_value, timeout);
}

}  // namespace wolf_controller_utils

#endif  // WOLF_CONTROLLER_UTILS_PARAM_GETTER_ROS2_H
