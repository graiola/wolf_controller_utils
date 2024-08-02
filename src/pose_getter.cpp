#include "wolf_controller_utils/pose_getter.h"

namespace wolf_controller_utils
{

PoseGetter::PoseGetter()
{
#ifdef ROS
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>();
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
#elif defined(ROS2)
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(rclcpp::Clock::make_shared());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
#endif
}

#ifdef ROS
geometry_msgs::Pose PoseGetter::getPose(const std::string &base_link, const std::string &distal_link)
{
  geometry_msgs::TransformStamped transform;
  geometry_msgs::Pose pose;

  try
  {
    transform = tf_buffer_->lookupTransform(base_link, distal_link, ros::Time(0), ros::Duration(5.0));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_ERROR("%s", ex.what());
    return pose;
  }

  pose.position.x = transform.transform.translation.x;
  pose.position.y = transform.transform.translation.y;
  pose.position.z = transform.transform.translation.z;

  pose.orientation.x = transform.transform.rotation.x;
  pose.orientation.y = transform.transform.rotation.y;
  pose.orientation.z = transform.transform.rotation.z;
  pose.orientation.w = transform.transform.rotation.w;

  return pose;
}
#endif

#ifdef ROS2
geometry_msgs::msg::Pose PoseGetter::getPose(const std::string &base_link, const std::string &distal_link)
{
  geometry_msgs::msg::TransformStamped transform;
  geometry_msgs::msg::Pose pose;

  try
  {
    transform = tf_buffer_->lookupTransform(base_link, distal_link, tf2::TimePointZero, tf2::durationFromSec(5.0));
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_ERROR(rclcpp::get_logger("PoseGetter"), "%s", ex.what());
    return pose;
  }

  pose.position.x = transform.transform.translation.x;
  pose.position.y = transform.transform.translation.y;
  pose.position.z = transform.transform.translation.z;

  pose.orientation.x = transform.transform.rotation.x;
  pose.orientation.y = transform.transform.rotation.y;
  pose.orientation.z = transform.transform.rotation.z;
  pose.orientation.w = transform.transform.rotation.w;

  return pose;
}
#endif

Eigen::Affine3d PoseGetter::getAffine3d(const std::string &base_link, const std::string &distal_link)
{
  Eigen::Affine3d out;

  tf2::fromMsg(getPose(base_link, distal_link), out);

  return out;
}

} // namespace wolf_controller_utils
