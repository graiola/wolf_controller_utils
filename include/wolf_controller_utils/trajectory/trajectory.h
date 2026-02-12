/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) Gennaro Raiola
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

  // Add a waypoint defined by absolute time and pose.
  void addWayPoint(double time, const Eigen::Affine3d& frame);

  // Add a waypoint with an optional time offset (useful when appending trajectories).
  void addWayPoint(const WayPoint& waypoint, double time_offset = 0.0);

  void clear();

  const std::vector<WayPoint>& getWayPoints() const;

  // Reserved hook for future precomputation.
  void compute();

  // Evaluate pose and optional linear velocity/acceleration at `time`.
  // If no waypoint exists, the identity pose and zero derivatives are returned.
  Eigen::Affine3d evaluate(double time, Eigen::Vector6d * const vel = nullptr, Eigen::Vector6d * const acc = nullptr);

  // Return active segment index, or -1 before the first segment / when empty.
  int getCurrentSegmentId(double time) const;

  bool isTrajectoryEnded(double time) const;

private:

  void sortFrames();

  std::vector<WayPoint> frames_;

};

} // namespace

} // namespace

#endif
