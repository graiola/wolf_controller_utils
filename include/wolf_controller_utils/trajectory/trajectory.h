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

#ifndef WOLF_CONTROLLER_TRAJECTORY_H
#define WOLF_CONTROLLER_TRAJECTORY_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <memory>
#include <vector>

#include <wolf_controller_utils/geometry.h>

namespace wolf_controller_utils
{
namespace trajectory
{

class Trajectory {

public:

  typedef std::shared_ptr<Trajectory> Ptr;
  typedef std::shared_ptr<const Trajectory> ConstPtr;

  struct WayPoint {

    WayPoint();
    WayPoint(Eigen::Affine3d frame, double time);

    Eigen::Affine3d frame_;
    Eigen::Vector6d vel_;
    Eigen::Vector6d acc_;
    double time_;
  };

  typedef std::vector<WayPoint> WayPointVector;

  Trajectory();

  void addWayPoint(double time, const Eigen::Affine3d& frame);

  void addWayPoint(const WayPoint& waypoint, double time_offset = 0.0);

  void clear();

  const std::vector<WayPoint>& getWayPoints() const;

  void compute();

  Eigen::Affine3d evaluate(double time, Eigen::Vector6d * const vel = nullptr, Eigen::Vector6d * const acc = nullptr);

  int getCurrentSegmentId(double time) const;

  bool isTrajectoryEnded(double time);

private:

  void sortFrames();

  std::vector<WayPoint> frames_;

};

} // namespace

} // namespace

#endif
