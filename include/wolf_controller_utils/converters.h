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

#ifdef ROS
    #include <geometry_msgs/Pose.h>
    #include <geometry_msgs/Twist.h>
    #include <geometry_msgs/PoseStamped.h>
    #include <geometry_msgs/TransformStamped.h>
    #include <nav_msgs/Odometry.h>
    #include <ros/time.h>
    using namespace geometry_msgs;
    using namespace nav_msgs;
    using ros::Time;
#elif defined(ROS2)
    #include <geometry_msgs/msg/pose.hpp>
    #include <geometry_msgs/msg/twist.hpp>
    #include <geometry_msgs/msg/pose_stamped.hpp>
    #include <geometry_msgs/msg/transform_stamped.hpp>
    #include <nav_msgs/msg/odometry.hpp>
    #include <rclcpp/rclcpp.hpp>
    using namespace geometry_msgs::msg;
    using namespace nav_msgs::msg;
    using rclcpp::Time;
    using rclcpp::Clock;
#endif

#include <wolf_controller_utils/common.h>

namespace wolf_controller_utils {

inline Eigen::Affine3d odometryToAffine3d(const Odometry &odom) {
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    transform.translation() << odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z;
    Eigen::Quaterniond quat(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z);
    transform.rotate(quat);
    return transform;
}

inline Odometry affine3dToOdometry(const Eigen::Affine3d& affine, const std::string& frame_id, const std::string& child_frame_id) {
    Odometry odometry_msg;
#ifdef ROS
    odometry_msg.header.stamp = Time::now();
#elif defined(ROS2)
    odometry_msg.header.stamp = Clock().now();
#endif
    odometry_msg.header.frame_id = frame_id;
    odometry_msg.child_frame_id = child_frame_id;
    odometry_msg.pose.pose.position.x = affine.translation().x();
    odometry_msg.pose.pose.position.y = affine.translation().y();
    odometry_msg.pose.pose.position.z = affine.translation().z();
    Eigen::Quaterniond quaternion(affine.rotation());
    odometry_msg.pose.pose.orientation.w = quaternion.w();
    odometry_msg.pose.pose.orientation.x = quaternion.x();
    odometry_msg.pose.pose.orientation.y = quaternion.y();
    odometry_msg.pose.pose.orientation.z = quaternion.z();
    return odometry_msg;
}

inline TransformStamped affine3dToTransformStamped(const Eigen::Affine3d& affine, const std::string& parent_frame, const std::string& child_frame, const Time& stamp) {
    TransformStamped transform_stamped;
    transform_stamped.header.stamp = stamp;
    transform_stamped.header.frame_id = parent_frame;
    transform_stamped.child_frame_id = child_frame;
    transform_stamped.transform.translation.x = affine.translation().x();
    transform_stamped.transform.translation.y = affine.translation().y();
    transform_stamped.transform.translation.z = affine.translation().z();
    Eigen::Quaterniond q(affine.rotation());
    transform_stamped.transform.rotation.x = q.x();
    transform_stamped.transform.rotation.y = q.y();
    transform_stamped.transform.rotation.z = q.z();
    transform_stamped.transform.rotation.w = q.w();
    return transform_stamped;
}

inline Eigen::Affine3d transformStampedToAffine3d(const TransformStamped& transformStamped) {
    Eigen::Translation3d translation(transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z);
    Eigen::Quaterniond rotation(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);
    return translation * rotation;
}

inline Eigen::Affine3d poseToAffine3d(const Pose &in) {
    Eigen::Affine3d out;
    out.translation() << in.position.x, in.position.y, in.position.z;
    Eigen::Quaterniond q(in.orientation.w, in.orientation.x, in.orientation.y, in.orientation.z);
    out.linear() = q.toRotationMatrix();
    return out;
}

inline Eigen::Matrix<double, 6, 1> twistToVector6d(const Twist &in) {
    Eigen::Matrix<double, 6, 1> out;
    out << in.linear.x, in.linear.y, in.linear.z, in.angular.x, in.angular.y, in.angular.z;
    return out;
}

inline TransformStamped poseStampedToTransformStamped(const PoseStamped &in, const std::string& parent_frame, const std::string& child_frame) {
    TransformStamped transformStamped;
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
inline void affine3dToVisualizationPose(const Eigen::Affine3d& Frame, MarkerType& Marker) {
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
inline void poseToVisualizationPose(const Pose& Frame, MarkerType& Marker) {
    Marker.pose.position.x = Frame.position.x;
    Marker.pose.position.y = Frame.position.y;
    Marker.pose.position.z = Frame.position.z;
    Marker.pose.orientation.x = Frame.orientation.x;
    Marker.pose.orientation.y = Frame.orientation.y;
    Marker.pose.orientation.z = Frame.orientation.z;
    Marker.pose.orientation.w = Frame.orientation.w;
}

inline void affine3dToPose(const Eigen::Affine3d& affine3d, Pose& pose) {
    pose.position.x = affine3d.translation().x();
    pose.position.y = affine3d.translation().y();
    pose.position.z = affine3d.translation().z();
    Eigen::Quaterniond quat(affine3d.linear());
    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();
    pose.orientation.w = quat.w();
}

inline void vector3dToPosePosition(const Eigen::Vector3d& vector3d, Pose& pose) {
    pose.position.x = vector3d.x();
    pose.position.y = vector3d.y();
    pose.position.z = vector3d.z();
}

inline void quaterniondToPoseOrientation(const Eigen::Quaterniond& quaterniond, Pose& pose) {
    pose.orientation.x = quaterniond.x();
    pose.orientation.y = quaterniond.y();
    pose.orientation.z = quaterniond.z();
    pose.orientation.w = quaterniond.w();
}

inline void vector6dToTwist(const Eigen::Matrix<double, 6, 1>& vector6d, Twist& twist) {
    twist.linear.x  = vector6d(0);
    twist.linear.y  = vector6d(1);
    twist.linear.z  = vector6d(2);
    twist.angular.x = vector6d(3);
    twist.angular.y = vector6d(4);
    twist.angular.z = vector6d(5);
}

inline void vector3dToVector3(const Eigen::Vector3d& vector3d, Vector3& vector3) {
    vector3.x = vector3d.x();
    vector3.y = vector3d.y();
    vector3.z = vector3d.z();
}

inline void covarianceToEigen(const std::array<double, 36>& in, Eigen::Matrix<double, 6, 6>& out) {
    for(unsigned int i = 0; i < 6; i++)
        for(unsigned int j = 0; j < 6; j++)
            out(i, j) = in[i * 6 + j];
}

inline void eigenToCovariance(const Eigen::Matrix<double, 6, 6>& in, std::array<double, 36>& out) {
    for(unsigned int i = 0; i < 6; i++)
        for(unsigned int j = 0; j < 6; j++)
            out[i * 6 + j] = in(i, j);
}

inline void covarianceToEigen(const boost::array<double, 36>& in, Eigen::Matrix<double, 6, 6>& out) {
    for(unsigned int i = 0; i < 6; i++)
        for(unsigned int j = 0; j < 6; j++)
            out(i, j) = in[i * 6 + j];
}

inline void eigenToCovariance(const Eigen::Matrix<double, 6, 6>& in, boost::array<double, 36>& out) {
    for(unsigned int i = 0; i < 6; i++)
        for(unsigned int j = 0; j < 6; j++)
            out[i * 6 + j] = in(i, j);
}

}; // namespace wolf_controller_utils

#endif // WOLF_CONTROLLER_UTILS_CONVERTERS_H
