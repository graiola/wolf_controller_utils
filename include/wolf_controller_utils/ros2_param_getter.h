#ifndef WOLF_CONTROLLER_UTILS_PARAM_GETTER_ROS2_H
#define WOLF_CONTROLLER_UTILS_PARAM_GETTER_ROS2_H

#include <rcl_interfaces/srv/get_parameters.hpp>
#include <unordered_map>
#include <chrono>
#include <vector>
#include <future>
#include <mutex>

namespace wolf_controller_utils {

#define WAIT_SEC 30

using GetParameters = rcl_interfaces::srv::GetParameters;

// Utility to get a singleton node instance
inline std::shared_ptr<rclcpp::Node> get_global_node() {
    static auto global_node = rclcpp::Node::make_shared("ros2_param_getter");
    return global_node;
}

// Cache for service availability (thread-safe with a mutex)
inline std::unordered_map<std::string, bool>& service_availability_cache() {
    static std::unordered_map<std::string, bool> cache;
    return cache;
}
inline std::mutex& cache_mutex() {
    static std::mutex mutex;
    return mutex;
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
        RCLCPP_WARN(node->get_logger(), "Invalid parameter path format: %s. Expected format: '/node_name/param_name'", param_path.c_str());
        return default_value;
    }

    std::string remote_node_name = param_path.substr(0, pos);
    std::string param_name = param_path.substr(pos + 1);

    auto client = node->create_client<GetParameters>(remote_node_name + "/get_parameters");
    // Check and cache service availability
    std::string service_name = remote_node_name + "/get_parameters";
    {
        std::lock_guard<std::mutex> lock(cache_mutex());
        auto& cache = service_availability_cache();
        if (cache.find(service_name) == cache.end()) {
            cache[service_name] = client->wait_for_service(timeout);
        }
        if (!cache[service_name]) {
            RCLCPP_WARN(node->get_logger(), "Service %s not available!", service_name.c_str());
            return default_value;
        }
    }

    auto request = std::make_shared<GetParameters::Request>();
    request->names.push_back(param_name);

        std::cout << "HEREEEE" << std::endl;

    auto future = client->async_send_request(request);
    while (rclcpp::ok() && future.wait_for(std::chrono::milliseconds(10)) != std::future_status::ready) {
        rclcpp::spin_some(node);
    }

    if (future.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready) {
        auto result = future.get();
        if (!result->values.empty() && result->values[0].type == expected_type) {
            auto param_value = result->values[0];
            if constexpr (std::is_same<T, std::string>::value) return param_value.string_value;
            if constexpr (std::is_same<T, int64_t>::value) return param_value.integer_value;
            if constexpr (std::is_same<T, double>::value) return param_value.double_value;
            if constexpr (std::is_same<T, bool>::value) return param_value.bool_value;
        }
        RCLCPP_WARN(node->get_logger(), "Parameter %s type mismatch or not found", param_name.c_str());
    } else {
        RCLCPP_WARN(node->get_logger(), "Service call to get parameter %s from node %s timed out", param_name.c_str(), remote_node_name.c_str());
    }

    return default_value;
}

template <typename T>
T get_parameter_from_remote_node(
    const std::string& node_name,
    const std::string& param_path,
    T default_value,
    unsigned int expected_type,
    std::chrono::seconds timeout = std::chrono::seconds(WAIT_SEC))
{
    return get_parameter_from_remote_node(rclcpp::Node::make_shared(node_name),param_path,default_value,expected_type,timeout);
}

// Generalized function to handle multiple concurrent requests
template <typename T>
std::vector<T> get_parameters_from_remote_nodes(
    const std::shared_ptr<rclcpp::Node>& node,
    const std::vector<std::string>& param_paths,
    const T& default_value,
    unsigned int expected_type,
    std::chrono::seconds timeout = std::chrono::seconds(WAIT_SEC))
{
    std::vector<std::future<T>> futures;
    std::vector<T> results;

    for (const auto& param_path : param_paths) {
        // Launch each request asynchronously
        futures.push_back(std::async(std::launch::async, [&]() {
            return get_parameter_from_remote_node(node, param_path, default_value, expected_type, timeout);
        }));
    }

    for (auto& future : futures) {
        try {
            // Collect results as they become available
            results.push_back(future.get());
        } catch (const std::exception& e) {
            RCLCPP_WARN(node->get_logger(), "Failed to get parameter: %s", e.what());
            results.push_back(default_value);
        }
    }

    return results;
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

// Overloads that use a new node with node_name
inline std::string get_string_parameter_from_remote_node(
    const std::string& node_name, const std::string& param_path, const std::string& default_value = "", std::chrono::seconds timeout = std::chrono::seconds(WAIT_SEC))
{
    return get_parameter_from_remote_node<std::string>(node_name, param_path, default_value, rcl_interfaces::msg::ParameterType::PARAMETER_STRING, timeout);
}

inline int64_t get_int_parameter_from_remote_node(
    const std::string& node_name, const std::string& param_path, int64_t default_value = 0, std::chrono::seconds timeout = std::chrono::seconds(WAIT_SEC))
{
    return get_parameter_from_remote_node<int64_t>(node_name, param_path, default_value, rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER, timeout);
}

inline double get_double_parameter_from_remote_node(
    const std::string& node_name, const std::string& param_path, double default_value = 0.0, std::chrono::seconds timeout = std::chrono::seconds(WAIT_SEC))
{
    return get_parameter_from_remote_node<double>(node_name, param_path, default_value, rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE, timeout);
}

inline bool get_bool_parameter_from_remote_node(
    const std::string& node_name, const std::string& param_path, bool default_value = false, std::chrono::seconds timeout = std::chrono::seconds(WAIT_SEC))
{
    return get_parameter_from_remote_node<bool>(node_name, param_path, default_value, rcl_interfaces::msg::ParameterType::PARAMETER_BOOL, timeout);
}

}  // namespace wolf_controller_utils

#endif  // WOLF_CONTROLLER_UTILS_PARAM_GETTER_ROS2_H
