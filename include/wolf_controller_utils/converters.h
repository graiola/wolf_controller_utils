/*
 * Copyright (C) 2022 Gennaro Raiola
 * Author: Gennaro Raiola
 * email:  gennaro.raiola@gmail.com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
 *
 * original code and license notice here: http://wiki.ros.org/explore_lite
*/

#ifndef WOLF_CONTROLLER_UTILS_CONVERTERS_H
#define WOLF_CONTROLLER_UTILS_CONVERTERS_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

#include <wolf_controller_utils/common.h>

namespace wolf_controller_utils {

inline Eigen::Affine3d odometryToAffine3d(const nav_msgs::Odometry &odom)
{
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();

    // Extract the position
    const auto &position = odom.pose.pose.position;
    transform.translation() << position.x, position.y, position.z;

    // Extract the orientation
    const auto &orientation = odom.pose.pose.orientation;
    Eigen::Quaterniond quat(orientation.w, orientation.x, orientation.y, orientation.z);
    transform.rotate(quat);

    return transform;
}

inline nav_msgs::Odometry affine3dToOdometry(const Eigen::Affine3d& affine, const std::string& frame_id, const std::string& child_frame_id)
{
    nav_msgs::Odometry odometry_msg;
    odometry_msg.header.stamp = ros::Time::now();
    odometry_msg.header.frame_id = frame_id;
    odometry_msg.child_frame_id = child_frame_id;

    // Extract translation components
    odometry_msg.pose.pose.position.x = affine.translation().x();
    odometry_msg.pose.pose.position.y = affine.translation().y();
    odometry_msg.pose.pose.position.z = affine.translation().z();

    // Extract rotation components
    Eigen::Quaterniond quaternion(affine.rotation());
    odometry_msg.pose.pose.orientation.w = quaternion.w();
    odometry_msg.pose.pose.orientation.x = quaternion.x();
    odometry_msg.pose.pose.orientation.y = quaternion.y();
    odometry_msg.pose.pose.orientation.z = quaternion.z();

    return odometry_msg;
}

inline Eigen::Affine3d transformStampedToAffine3d(const geometry_msgs::TransformStamped& transformStamped) {
    // Convert translation and rotation to Eigen data types
    Eigen::Translation3d translation(transformStamped.transform.translation.x,
                                     transformStamped.transform.translation.y,
                                     transformStamped.transform.translation.z);
    Eigen::Quaterniond rotation(transformStamped.transform.rotation.w,
                                transformStamped.transform.rotation.x,
                                transformStamped.transform.rotation.y,
                                transformStamped.transform.rotation.z);

    // Create an Affine3d transformation matrix
    Eigen::Affine3d affine;
    affine = translation * rotation;

    return affine;
}

inline geometry_msgs::TransformStamped affine3dToTransformStamped(const Eigen::Affine3d& affine, const std::string& parent_frame, const std::string& child_frame, const ros::Time& stamp)
{
    geometry_msgs::TransformStamped transform_stamped;

    transform_stamped.header.stamp = stamp;
    transform_stamped.header.frame_id = parent_frame;
    transform_stamped.child_frame_id = child_frame;

    // Extract translation
    transform_stamped.transform.translation.x = affine.translation().x();
    transform_stamped.transform.translation.y = affine.translation().y();
    transform_stamped.transform.translation.z = affine.translation().z();

    // Extract rotation
    Eigen::Quaterniond q(affine.rotation());
    transform_stamped.transform.rotation.x = q.x();
    transform_stamped.transform.rotation.y = q.y();
    transform_stamped.transform.rotation.z = q.z();
    transform_stamped.transform.rotation.w = q.w();

    return transform_stamped;
}

inline Eigen::Affine3d poseToAffine3d(const geometry_msgs::Pose &in)
{
  Eigen::Affine3d out;

  // Set translation components
  out.translation().x() = in.position.x;
  out.translation().y() = in.position.y;
  out.translation().z() = in.position.z;

  // Set rotation components
  Eigen::Quaterniond q(in.orientation.w, in.orientation.x, in.orientation.y, in.orientation.z);
  out.linear() = q.toRotationMatrix();

  return out;
}

inline geometry_msgs::TransformStamped poseStampedToTransformStamped(
  const geometry_msgs::PoseStamped & in,
  const std::string& parent_frame,
  const std::string& child_frame)
{
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = in.header.stamp;
  transformStamped.header.frame_id = parent_frame;
  transformStamped.child_frame_id = child_frame;
  transformStamped.transform.translation.x = in.pose.position.x;
  transformStamped.transform.translation.y = in.pose.position.y;
  transformStamped.transform.translation.z = in.pose.position.z;
  transformStamped.transform.rotation.x = in.pose.orientation.x;
  transformStamped.transform.rotation.y = in.pose.orientation.y;
  transformStamped.transform.rotation.z = in.pose.orientation.z;
  transformStamped.transform.rotation.w = in.pose.orientation.w;
  return transformStamped;
}

template <class MarkerType>
inline void affine3dToVisualizationPose(const Eigen::Affine3d& Frame, MarkerType& Marker)
{
  Marker.pose.position.x = Frame.translation().x();
  Marker.pose.position.y = Frame.translation().y();
  Marker.pose.position.z = Frame.translation().z();
  Eigen::Quaterniond q(Frame.linear());
  Marker.pose.orientation.x = q.x();
  Marker.pose.orientation.y = q.y();
  Marker.pose.orientation.z = q.z();
  Marker.pose.orientation.w = q.w();
}

template <class MarkerType>
inline void poseToVisualizationPose(const geometry_msgs::Pose& Frame, MarkerType& Marker)
{
  Marker.pose.position.x = Frame.position.x;
  Marker.pose.position.y = Frame.position.y;
  Marker.pose.position.z = Frame.position.z;
  Marker.pose.orientation.x = Frame.orientation.x;
  Marker.pose.orientation.y = Frame.orientation.y;
  Marker.pose.orientation.z = Frame.orientation.z;
  Marker.pose.orientation.w = Frame.orientation.w;
}

inline void affine3dToPose(const Eigen::Affine3d& affine3d, geometry_msgs::Pose& pose)
{
  // Translation
  pose.position.x = affine3d.translation().x();
  pose.position.y = affine3d.translation().y();
  pose.position.z = affine3d.translation().z();
  // Note unfortunately affine3d can not be converted in quaternion directly...
  // Rotation
  pose.orientation.x = static_cast<Eigen::Quaterniond>(affine3d.linear()).x();
  pose.orientation.y = static_cast<Eigen::Quaterniond>(affine3d.linear()).y();
  pose.orientation.z = static_cast<Eigen::Quaterniond>(affine3d.linear()).z();
  pose.orientation.w = static_cast<Eigen::Quaterniond>(affine3d.linear()).w();
}

inline void vector3dToPosePosition(const Eigen::Vector3d& vector3d, geometry_msgs::Pose& pose)
{
  // Translation
  pose.position.x = vector3d.x();
  pose.position.y = vector3d.y();
  pose.position.z = vector3d.z();
}

inline void quaterniondToPoseOrientation(const Eigen::Quaterniond& quaterniond, geometry_msgs::Pose& pose)
{
  // Rotation
  pose.orientation.x = quaterniond.x();
  pose.orientation.y = quaterniond.y();
  pose.orientation.z = quaterniond.z();
  pose.orientation.w = quaterniond.w();
}

inline void vector6dToTwist(const Eigen::Vector6d& vector6d, geometry_msgs::Twist& twist)
{
  twist.linear.x  = vector6d(0);
  twist.linear.y  = vector6d(1);
  twist.linear.z  = vector6d(2);
  twist.angular.x = vector6d(3);
  twist.angular.y = vector6d(4);
  twist.angular.z = vector6d(5);
}

inline void vector3dToVector3(const Eigen::Vector3d& vector3d, geometry_msgs::Vector3& vector3)
{
  vector3.x = vector3d.x();
  vector3.y = vector3d.y();
  vector3.z = vector3d.z();
}

inline void covarianceToEigen(const boost::array<double, 36>& in, Eigen::Matrix6d& out)
{
  //out = Eigen::Map<Eigen::Matrix<double,6,6> >(in.data());
  for(unsigned int i=0;i<6;i++)
    for(unsigned int j=0;j<6;j++)
      out(i,j) = in[i*6 + j];
}
inline void eigenToCovariance(const Eigen::Matrix6d& in, boost::array<double, 36>& out)
{
  //out = Eigen::Map<Eigen::Matrix<double,6,6> >(in.data());
  for(unsigned int i=0;i<6;i++)
    for(unsigned int j=0;j<6;j++)
      out[i*6 + j] = in(i,j);
}

}; // namespace

#endif
