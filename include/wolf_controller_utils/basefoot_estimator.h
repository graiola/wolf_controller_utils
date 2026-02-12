/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) Gennaro Raiola
 */

#ifndef WOLF_CONTROLLER_BASEFOOT_ESTIMATOR_H
#define WOLF_CONTROLLER_BASEFOOT_ESTIMATOR_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <memory>
#include <vector>

namespace wolf_controller_utils {

class BasefootEstimator
{

public:
    BasefootEstimator();

    // Update per-foot contact state and measured base-to-foot heights.
    // Inputs must have same non-zero size; invalid inputs are ignored.
    void setContacts(const std::vector<bool>& foot_contacts, const std::vector<double>& foot_height);

    const Eigen::Isometry3d& getBasefootPoseInBase() const;

    const Eigen::Isometry3d& getBasefootPoseInOdom() const;

    void setBasePoseInOdom(const Eigen::Isometry3d& pose);

    void update();

private:
    double estimateHeight();

    std::vector<bool> foot_contacts_;
    std::vector<double> foot_height_;
    Eigen::Isometry3d base_T_basefoot_;
    Eigen::Isometry3d odom_T_basefoot_;
    Eigen::Isometry3d odom_T_base_;
    Eigen::Isometry3d basefoot_T_odom_;
    Eigen::Matrix3d odom_R_hf_;
    Eigen::Matrix3d hf_R_base_;


    Eigen::Vector3d tmp_v_;
    Eigen::Matrix3d tmp_R_;

};

} // namespace

#endif
