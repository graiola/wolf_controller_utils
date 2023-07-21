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
*/

#ifndef WOLF_CONTROLLER_UTILS_POSE_GETTER_H
#define WOLF_CONTROLLER_UTILS_POSE_GETTER_H

// ROS
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Pose.h>

namespace wolf_controller_utils
{

class PoseGetter
{

public:

  typedef std::shared_ptr<PoseGetter> Ptr;

  PoseGetter();

  geometry_msgs::Pose getPose(const std::string& base_link, const std::string& distal_link);

private:

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

};

} // namespace

#endif
