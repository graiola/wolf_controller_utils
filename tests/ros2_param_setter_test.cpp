#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>
#include "wolf_controller_utils/ros2_param_setter.h"  // Include the setter header

using namespace wolf_controller_utils;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("setter");

  auto server = std::make_shared<rclcpp::Node>("server");

  // Declare parameters on the server node
  server->declare_parameter<std::string>("string_param", "test_string");
  server->declare_parameter<int64_t>("int_param", 42);
  server->declare_parameter<double>("double_param", 42.42);
  server->declare_parameter<bool>("bool_param", true);

  // Start the async spinner in a separate thread
  auto spinner = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  spinner->add_node(server->get_node_base_interface());

  // Run the spinner in a new thread
  auto spinner_thread = std::make_shared<std::thread>([spinner]() {
    spinner->spin();
  });

  // Set a string parameter
  bool result_string = set_parameter_on_remote_node(node, "server/string_param", std::string("test_string"));

  // Set an integer parameter
  bool result_int = set_parameter_on_remote_node(node, "server/int_param", int64_t(42));

  // Set a double parameter
  bool result_double = set_parameter_on_remote_node(node, "server/double_param", double(42.42));

  // Set a boolean parameter
  bool result_bool = set_parameter_on_remote_node(node, "server/bool_param", bool(true));

  rclcpp::spin(node);

  if (spinner_thread->joinable()) {
    spinner_thread->join();
  }

  return 0;
}
