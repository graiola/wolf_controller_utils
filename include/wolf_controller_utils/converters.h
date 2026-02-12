/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) Gennaro Raiola
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
    typedef geometry_msgs::Pose PoseRosMsg;
    typedef geometry_msgs::Twist TwistRosMsg;
    typedef geometry_msgs::PoseStamped PoseStampedRosMsg;
    typedef geometry_msgs::TransformStamped TransformStampedRosMsg;
    typedef geometry_msgs::Vector3 Vector3RosMsg;
    typedef nav_msgs::Odometry OdometryRosMsg;
    typedef ros::Time TimeRos;
#elif defined(ROS2)
    #include <geometry_msgs/msg/pose.hpp>
    #include <geometry_msgs/msg/twist.hpp>
    #include <geometry_msgs/msg/pose_stamped.hpp>
    #include <geometry_msgs/msg/transform_stamped.hpp>
    #include <nav_msgs/msg/odometry.hpp>
    #include <rclcpp/rclcpp.hpp>
    typedef geometry_msgs::msg::Pose PoseRosMsg;
    typedef geometry_msgs::msg::Twist TwistRosMsg;
    typedef geometry_msgs::msg::PoseStamped PoseStampedRosMsg;
    typedef geometry_msgs::msg::TransformStamped TransformStampedRosMsg;
    typedef geometry_msgs::msg::Vector3 Vector3RosMsg;
    typedef nav_msgs::msg::Odometry OdometryRosMsg;
    typedef rclcpp::Time TimeRos;
#endif

#include <wolf_controller_utils/common.h>

namespace wolf_controller_utils {

inline Eigen::Affine3d odometryToAffine3d(const OdometryRosMsg &odom) {
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    transform.translation() << odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z;
    Eigen::Quaterniond quat(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z);
    transform.rotate(quat);
    return transform;
}

inline OdometryRosMsg affine3dToOdometry(const Eigen::Affine3d& affine,
                                         const std::string& frame_id,
                                         const std::string& child_frame_id
#ifdef ROS2
                                         , const rclcpp::Clock::SharedPtr& clock  // Pass clock for ROS2
#endif
) {
    OdometryRosMsg odometry_msg;
#ifdef ROS
    odometry_msg.header.stamp = TimeRos::now();  // Use TimeRos::now() in ROS1
#elif defined(ROS2)
    odometry_msg.header.stamp = clock->now();  // Use clock->now() in ROS2
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


inline TransformStampedRosMsg affine3dToTransformStamped(const Eigen::Affine3d& affine,
                                                         const std::string& parent_frame,
                                                         const std::string& child_frame,
                                                         const TimeRos& stamp
#ifdef ROS2
                                                         , const rclcpp::Clock::SharedPtr& clock  // For ROS2, pass the clock if needed
#endif
) {
    TransformStampedRosMsg transform_stamped;
#ifdef ROS
    transform_stamped.header.stamp = stamp;  // Use stamp directly in ROS1
#elif defined(ROS2)
    transform_stamped.header.stamp = clock->now();  // Use clock->now() for ROS2 if needed
#endif
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


inline Eigen::Affine3d transformStampedToAffine3d(const TransformStampedRosMsg& transformStamped) {
    Eigen::Translation3d translation(transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z);
    Eigen::Quaterniond rotation(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);
    return translation * rotation;
}

inline Eigen::Affine3d poseToAffine3d(const PoseRosMsg &in) {
    Eigen::Affine3d out;
    out.translation() << in.position.x, in.position.y, in.position.z;
    Eigen::Quaterniond q(in.orientation.w, in.orientation.x, in.orientation.y, in.orientation.z);
    out.linear() = q.toRotationMatrix();
    return out;
}

inline Eigen::Matrix<double, 6, 1> twistToVector6d(const TwistRosMsg &in) {
    Eigen::Matrix<double, 6, 1> out;
    out << in.linear.x, in.linear.y, in.linear.z, in.angular.x, in.angular.y, in.angular.z;
    return out;
}

inline TransformStampedRosMsg poseStampedToTransformStamped(const PoseStampedRosMsg &in, const std::string& parent_frame, const std::string& child_frame) {
    TransformStampedRosMsg transformStamped;
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
inline void poseToVisualizationPose(const PoseRosMsg& Frame, MarkerType& Marker) {
    Marker.pose.position.x = Frame.position.x;
    Marker.pose.position.y = Frame.position.y;
    Marker.pose.position.z = Frame.position.z;
    Marker.pose.orientation.x = Frame.orientation.x;
    Marker.pose.orientation.y = Frame.orientation.y;
    Marker.pose.orientation.z = Frame.orientation.z;
    Marker.pose.orientation.w = Frame.orientation.w;
}

inline void affine3dToPose(const Eigen::Affine3d& affine3d, PoseRosMsg& pose) {
    pose.position.x = affine3d.translation().x();
    pose.position.y = affine3d.translation().y();
    pose.position.z = affine3d.translation().z();
    Eigen::Quaterniond quat(affine3d.linear());
    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();
    pose.orientation.w = quat.w();
}

inline void vector3dToPosePosition(const Eigen::Vector3d& vector3d, PoseRosMsg& pose) {
    pose.position.x = vector3d.x();
    pose.position.y = vector3d.y();
    pose.position.z = vector3d.z();
}

inline void quaterniondToPoseOrientation(const Eigen::Quaterniond& quaterniond, PoseRosMsg& pose) {
    pose.orientation.x = quaterniond.x();
    pose.orientation.y = quaterniond.y();
    pose.orientation.z = quaterniond.z();
    pose.orientation.w = quaterniond.w();
}

inline void vector6dToTwist(const Eigen::Matrix<double, 6, 1>& vector6d, TwistRosMsg& twist) {
    twist.linear.x  = vector6d(0);
    twist.linear.y  = vector6d(1);
    twist.linear.z  = vector6d(2);
    twist.angular.x = vector6d(3);
    twist.angular.y = vector6d(4);
    twist.angular.z = vector6d(5);
}

inline void vector3dToVector3(const Eigen::Vector3d& vector3d, Vector3RosMsg& vector3) {
    vector3.x = vector3d.x();
    vector3.y = vector3d.y();
    vector3.z = vector3d.z();
}

#ifdef ROS
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
#elif defined(ROS2)
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
#endif

}; // namespace wolf_controller_utils

#endif // WOLF_CONTROLLER_UTILS_CONVERTERS_H
