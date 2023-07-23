#include "wolf_controller_utils/pose_getter.h"

using namespace wolf_controller_utils;

PoseGetter::PoseGetter()
{
  tf_buffer_   = std::make_unique<tf2_ros::Buffer>();
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
}

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
    ROS_ERROR("[PoseGetter] %s",ex.what());
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

Eigen::Affine3d PoseGetter::getAffine3d(const std::string &base_link, const std::string &distal_link)
{
  Eigen::Affine3d out;
  tf2::convert(getPose(base_link,distal_link), out);
  return out;
}
