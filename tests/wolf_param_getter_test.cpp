#include <iostream>
#include <string>
#include <memory>
#include <chrono>
#include <cassert>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include "wolf_controller_utils/ros2_param_getter.h"  // Include the header with the function to be tested.

using namespace wolf_controller_utils;

static std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> spinner_;
static std::shared_ptr<std::thread> spinner_thread_;

// A helper function to simulate a "mock" remote node for testing purposes.
// In real-world scenarios, you'd interact with a remote node or mock it.
std::shared_ptr<rclcpp::Node> create_mock_node() {
  return std::make_shared<rclcpp::Node>("getter");
}

// Test for getting parameters of different types
void test_get_parameter_from_remote_node() {
  auto node = create_mock_node();

  // Start the async spinner in a separate thread
  spinner_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  spinner_->add_node(node->get_node_base_interface());

  // Run the spinner in a new thread
  spinner_thread_ = std::make_shared<std::thread>([]() {
    spinner_->spin();
  });


  // Assuming we have a mock parameter server that responds with test values

  std::string param_path = "wolf_controller/gains.lf_foot.weight";
  double expected_double_value = 100.0;
  double double_result = get_double_parameter_from_remote_node(param_path, 0.0);
  if(double_result != expected_double_value)
  {
    std::cout << "Failed to get the correct double parameter!" << std::endl;
    std::cout << "Got: " << double_result << " Expected: " << expected_double_value << std::endl;
  }
  else {
    std::cout << param_path << ": " << double_result << std::endl;
  }


  param_path = "wolf_controller/gains.lf_foot.lambda1";
  expected_double_value = 100.0;
  double_result = get_double_parameter_from_remote_node(param_path, 0.0);
  if(double_result != expected_double_value)
  {
    std::cout << "Failed to get the correct double parameter!" << std::endl;
    std::cout << "Got: " << double_result << " Expected: " << expected_double_value << std::endl;
  }
  else {
    std::cout << param_path << ": " << double_result << std::endl;
  }

  param_path = "wolf_controller/gains.lf_foot.lambda2";
  expected_double_value = 100.0;
  double_result = get_double_parameter_from_remote_node(param_path, 0.0);
  if(double_result != expected_double_value)
  {
    std::cout << "Failed to get the correct double parameter!" << std::endl;
    std::cout << "Got: " << double_result << " Expected: " << expected_double_value << std::endl;
  }
  else {
    std::cout << param_path << ": " << double_result << std::endl;
  }

  param_path = "wolf_controller/gains.rf_foot.weight";
  expected_double_value = 100.0;
  double_result = get_double_parameter_from_remote_node(param_path, 0.0);
  if(double_result != expected_double_value)
  {
    std::cout << "Failed to get the correct double parameter!" << std::endl;
    std::cout << "Got: " << double_result << " Expected: " << expected_double_value << std::endl;
  }
  else {
    std::cout << param_path << ": " << double_result << std::endl;
  }

  param_path = "wolf_controller/gains.rf_foot.lambda1";
  expected_double_value = 100.0;
  double_result = get_double_parameter_from_remote_node(param_path, 0.0);
  if(double_result != expected_double_value)
  {
    std::cout << "Failed to get the correct double parameter!" << std::endl;
    std::cout << "Got: " << double_result << " Expected: " << expected_double_value << std::endl;
  }
  else {
    std::cout << param_path << ": " << double_result << std::endl;
  }

  param_path = "wolf_controller/gains.rf_foot.lambda2";
  expected_double_value = 100.0;
  double_result = get_double_parameter_from_remote_node(param_path, 0.0);
  if(double_result != expected_double_value)
  {
    std::cout << "Failed to get the correct double parameter!" << std::endl;
    std::cout << "Got: " << double_result << " Expected: " << expected_double_value << std::endl;
  }
  else {
    std::cout << param_path << ": " << double_result << std::endl;
  }

  param_path = "wolf_controller/gains.waist.weight";
  double expected_first_value = 100.0;
  double first_result = get_double_parameter_from_remote_node(param_path, 1.0);
  if(first_result != expected_first_value)
  {
    std::cout << "Failed to get the correct double parameter!" << std::endl;
    std::cout << "Got: " << first_result << " Expected: " << expected_first_value << std::endl;
  }
  else {
    std::cout << param_path << ": " << first_result << std::endl;
  }

  param_path = "wolf_controller/gains.lf_foot.weight";
  double expected_second_value = 100.0;
  double second_result = get_double_parameter_from_remote_node(param_path, 1.0, std::chrono::seconds(30));
  if(second_result != expected_second_value)
  {
    std::cout << "Failed to get the correct double parameter!" << std::endl;
    std::cout << "Got: " << second_result << " Expected: " << expected_second_value << std::endl;
  }
  else {
    std::cout << param_path << ": " << second_result << std::endl;
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
