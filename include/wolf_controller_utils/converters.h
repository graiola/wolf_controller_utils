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
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

namespace wolf_controller_utils {

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
