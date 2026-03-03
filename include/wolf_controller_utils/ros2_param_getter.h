#ifndef WOLF_CONTROLLER_UTILS_PARAM_GETTER_ROS2_H
#define WOLF_CONTROLLER_UTILS_PARAM_GETTER_ROS2_H

#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <chrono>
#include <iostream>
#include <cmath>
#include <limits>
#include <vector>
#include <wolf_controller_utils/namespace_utils.h>

namespace wolf_controller_utils
{

#define WAIT_SEC 3
#define RETRY_ATTEMPTS 3

using GetParameters = rcl_interfaces::srv::GetParameters;

// Utility to get a singleton node instance
inline std::shared_ptr<rclcpp::Node> get_global_node()
{
    static auto global_node = []() {
        rclcpp::NodeOptions options;
        options.start_parameter_services(false);
        options.start_parameter_event_publisher(false);
        options.enable_rosout(false);
        return rclcpp::Node::make_shared("ros2_param_getter", options);
    }();
    return global_node;
}

// Generalized function to retrieve a parameter of any type
template <typename T>
T get_parameter_from_remote_node(
    const std::shared_ptr<rclcpp::Node> &node,
    const std::string &param_path,
    T default_value,
    unsigned int expected_type,
    std::chrono::seconds timeout = std::chrono::seconds(WAIT_SEC))
{
    RCLCPP_INFO(node->get_logger(), "Fetching parameter: %s", param_path.c_str());

    const size_t pos = param_path.find_last_of('/');
    if (pos == std::string::npos || pos == 0 || pos + 1 >= param_path.size())
    {
        RCLCPP_ERROR(
            node->get_logger(),
            "Invalid parameter path format: %s. Expected format: '/node_name/param_name'",
            param_path.c_str());
        return default_value;
    }

    std::string remote_node_name = param_path.substr(0, pos);
    std::string param_name = param_path.substr(pos + 1);
    if (!remote_node_name.empty() && remote_node_name.front() != '/')
      remote_node_name = "/" + remote_node_name;

    auto callback_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto client = node->create_client<GetParameters>(remote_node_name + "/get_parameters", rmw_qos_profile_services_default, callback_group);

    // Wait for service with extended timeout
    for (int attempt = 1; attempt <= RETRY_ATTEMPTS; ++attempt)
    {
        if (client->wait_for_service(timeout))
        {
            break;
        }
        if (attempt == RETRY_ATTEMPTS)
        {
            RCLCPP_ERROR(node->get_logger(), "Service %s/get_parameters not available after %d attempts!", remote_node_name.c_str(), RETRY_ATTEMPTS);
            return default_value;
        }
        RCLCPP_WARN(node->get_logger(), "Service not available, retrying (%d/%d)...", attempt, RETRY_ATTEMPTS);
    }

    auto request = std::make_shared<GetParameters::Request>();
    request->names.push_back(param_name);

    auto future = client->async_send_request(request);

    RCLCPP_INFO(node->get_logger(), "Waiting for parameter response...");
    if (rclcpp::spin_until_future_complete(node, future, timeout) == rclcpp::FutureReturnCode::SUCCESS)
    {
        auto result = future.get();
        if (!result->values.empty())
        {
            auto param_value = result->values[0];
            if (param_value.type == expected_type)
            {
                if constexpr (std::is_same<T, std::string>::value)
                    return param_value.string_value;
                if constexpr (std::is_same<T, int64_t>::value)
                    return param_value.integer_value;
                if constexpr (std::is_same<T, double>::value)
                    return param_value.double_value;
                if constexpr (std::is_same<T, bool>::value)
                    return param_value.bool_value;
            }
            else
            {
                RCLCPP_ERROR(node->get_logger(), "Parameter %s is not of expected type", param_name.c_str());
            }
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "Parameter %s not found on remote node %s", param_name.c_str(), remote_node_name.c_str());
        }
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Service call to get parameter %s from node %s failed", param_name.c_str(), remote_node_name.c_str());
    }

    return default_value;
}

// Specialized functions for specific parameter types
inline std::string get_string_parameter_from_remote_node(
    const std::shared_ptr<rclcpp::Node> &node, const std::string &param_path, const std::string &default_value = "", std::chrono::seconds timeout = std::chrono::seconds(WAIT_SEC))
{
    return get_parameter_from_remote_node<std::string>(node, param_path, default_value, rcl_interfaces::msg::ParameterType::PARAMETER_STRING, timeout);
}

inline int64_t get_int_parameter_from_remote_node(
    const std::shared_ptr<rclcpp::Node> &node, const std::string &param_path, int64_t default_value = 0, std::chrono::seconds timeout = std::chrono::seconds(WAIT_SEC))
{
    return get_parameter_from_remote_node<int64_t>(node, param_path, default_value, rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER, timeout);
}

inline double get_double_parameter_from_remote_node(
    const std::shared_ptr<rclcpp::Node> &node, const std::string &param_path, double default_value = 0.0, std::chrono::seconds timeout = std::chrono::seconds(WAIT_SEC))
{
    return get_parameter_from_remote_node<double>(node, param_path, default_value, rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE, timeout);
}

inline bool get_bool_parameter_from_remote_node(
    const std::shared_ptr<rclcpp::Node> &node, const std::string &param_path, bool default_value = false, std::chrono::seconds timeout = std::chrono::seconds(WAIT_SEC))
{
    return get_parameter_from_remote_node<bool>(node, param_path, default_value, rcl_interfaces::msg::ParameterType::PARAMETER_BOOL, timeout);
}

// Overloads that use the global node
inline std::string get_string_parameter_from_remote_node(
    const std::string &param_path, const std::string &default_value = "", std::chrono::seconds timeout = std::chrono::seconds(WAIT_SEC))
{
    return get_string_parameter_from_remote_node(get_global_node(), param_path, default_value, timeout);
}

inline int64_t get_int_parameter_from_remote_node(
    const std::string &param_path, int64_t default_value = 0, std::chrono::seconds timeout = std::chrono::seconds(WAIT_SEC))
{
    return get_int_parameter_from_remote_node(get_global_node(), param_path, default_value, timeout);
}

inline double get_double_parameter_from_remote_node(
    const std::string &param_path, double default_value = 0.0, std::chrono::seconds timeout = std::chrono::seconds(WAIT_SEC))
{
    return get_double_parameter_from_remote_node(get_global_node(), param_path, default_value, timeout);
}

inline bool get_bool_parameter_from_remote_node(
    const std::string &param_path, bool default_value = false, std::chrono::seconds timeout = std::chrono::seconds(WAIT_SEC))
{
    return get_bool_parameter_from_remote_node(get_global_node(), param_path, default_value, timeout);
}

inline std::vector<std::string> build_remote_node_candidates(
    const std::string &robot_name,
    const std::string &node_name)
{
    const std::string ns = normalize_namespace(robot_name);
    std::vector<std::string> nodes;
    if (ns.empty())
    {
        nodes.push_back("/" + normalize_namespace(node_name));
    }
    else
    {
        // In multi-robot setups, avoid fallback to a global node to prevent repeated
        // slow service timeouts when only the namespaced controller exists.
        nodes.push_back("/" + ns + "/" + normalize_namespace(node_name));
    }

    return nodes;
}

inline double get_double_parameter_from_remote_nodes(
    const std::vector<std::string> &node_candidates,
    const std::string &parameter_name,
    double default_value = 0.0,
    std::chrono::seconds timeout = std::chrono::seconds(WAIT_SEC))
{
    for (const auto &node : node_candidates)
    {
        const double value = get_double_parameter_from_remote_node(
            node + "/" + parameter_name,
            std::numeric_limits<double>::quiet_NaN(),
            timeout);
        if (std::isfinite(value))
            return value;
    }
    return default_value;
}

inline std::string get_string_parameter_from_remote_nodes(
    const std::vector<std::string> &node_candidates,
    const std::string &parameter_name,
    const std::string &default_value = "",
    std::chrono::seconds timeout = std::chrono::seconds(WAIT_SEC))
{
    static const std::string kMissing = "__wolf_param_missing__";
    for (const auto &node : node_candidates)
    {
        const std::string value = get_string_parameter_from_remote_node(
            node + "/" + parameter_name,
            kMissing,
            timeout);
        if (value != kMissing)
            return value;
    }
    return default_value;
}

inline double get_double_parameter_from_remote_controller_node(
    const std::string &robot_name,
    const std::string &parameter_name,
    double default_value = 0.0,
    std::chrono::seconds timeout = std::chrono::seconds(WAIT_SEC))
{
    return get_double_parameter_from_remote_nodes(
        build_remote_node_candidates(robot_name, "wolf_controller"),
        parameter_name,
        default_value,
        timeout);
}

inline std::string get_string_parameter_from_remote_controller_node(
    const std::string &robot_name,
    const std::string &parameter_name,
    const std::string &default_value = "",
    std::chrono::seconds timeout = std::chrono::seconds(WAIT_SEC))
{
    return get_string_parameter_from_remote_nodes(
        build_remote_node_candidates(robot_name, "wolf_controller"),
        parameter_name,
        default_value,
        timeout);
}

} // namespace wolf_controller_utils

#endif // WOLF_CONTROLLER_UTILS_PARAM_GETTER_ROS2_H
