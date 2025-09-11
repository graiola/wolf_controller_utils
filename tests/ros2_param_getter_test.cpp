#include <iostream>
#include <string>
#include <memory>
#include <chrono>
#include <cassert>
#include "rclcpp/rclcpp.hpp"
#include "wolf_controller_utils/ros2_param_getter.h"  // Include the header with the function to be tested.

using namespace wolf_controller_utils;

// A helper function to simulate a "mock" remote node for testing purposes.
// In real-world scenarios, you'd interact with a remote node or mock it.
std::shared_ptr<rclcpp::Node> create_mock_node() {
  return std::make_shared<rclcpp::Node>("getter");
}

// Test for getting parameters of different types
void test_get_parameter_from_remote_node() {
  auto node = create_mock_node();

  // Assuming we have a mock parameter server that responds with test values

  // Test for a string parameter
  std::string param_path = "server/string_param";
  std::string expected_string_value = "test_string";
  std::string string_result = get_parameter_from_remote_node(node, param_path, std::string("default_value"), rcl_interfaces::msg::ParameterType::PARAMETER_STRING);
  if(string_result != expected_string_value)
  {
    std::cout << "Failed to get the correct string parameter!" << std::endl;
    std::cout << "Got: " << string_result << " Expected: " << expected_string_value << std::endl;
  }

  // Test for an integer parameter
  param_path = "server/int_param";
  int64_t expected_int_value = 42;
  int64_t int_result = get_parameter_from_remote_node(node, param_path, int64_t(0), rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER);
  if(int_result != expected_int_value)
  {
    std::cout << "Failed to get the correct integer parameter!" << std::endl;
    std::cout << "Got: " << int_result << " Expected: " << expected_int_value << std::endl;
  }

  // Test for a boolean parameter
  param_path = "server/bool_param";
  bool expected_bool_value = true;
  bool bool_result = get_parameter_from_remote_node(node, param_path, false, rcl_interfaces::msg::ParameterType::PARAMETER_BOOL);
  if(bool_result != expected_bool_value)
  {
    std::cout << "Failed to get the correct boolean parameter!" << std::endl;
    std::cout << "Got: " << bool_result << " Expected: " << expected_bool_value << std::endl;
  }

  // Test for a double parameter
  param_path = "server/double_param";
  double expected_double_value = 42.42;
  double double_result = get_parameter_from_remote_node(node, param_path, 0.0, rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE);
  if(double_result != expected_double_value)
  {
    std::cout << "Failed to get the correct double parameter!" << std::endl;
    std::cout << "Got: " << double_result << " Expected: " << expected_double_value << std::endl;
  }

  std::cout << "All tests finished!" << std::endl;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  try {
    // Run the tests
    test_get_parameter_from_remote_node();
  } catch (const std::exception &e) {
    std::cerr << "Error during tests: " << e.what() << std::endl;
    return 1;  // Return non-zero on failure
  }

  rclcpp::shutdown();
  return 0;  // Return zero on success
}
