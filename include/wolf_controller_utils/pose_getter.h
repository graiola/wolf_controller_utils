#ifndef WOLF_CONTROLLER_UTILS_POSE_GETTER_H
#define WOLF_CONTROLLER_UTILS_POSE_GETTER_H

#ifdef ROS
  #include <ros/ros.h>
  #include <geometry_msgs/Pose.h>
#endif
#ifdef ROS2
  #include <rclcpp/rclcpp.hpp>
  #include <geometry_msgs/msg/pose.hpp>
#endif

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Core>
#include <Eigen/Dense>

namespace wolf_controller_utils
{

class PoseGetter
{
public:
  using Ptr = std::shared_ptr<PoseGetter>;

  PoseGetter();

#ifdef ROS
  geometry_msgs::Pose getPose(const std::string& base_link, const std::string& distal_link);
#endif
#ifdef ROS2
  geometry_msgs::msg::Pose getPose(const std::string& base_link, const std::string& distal_link);
#endif

  Eigen::Affine3d getAffine3d(const std::string& base_link, const std::string& distal_link);

private:
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};

} // namespace wolf_controller_utils

#endif // WOLF_CONTROLLER_UTILS_POSE_GETTER_H
