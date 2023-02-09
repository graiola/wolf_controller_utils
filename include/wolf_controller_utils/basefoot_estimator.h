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

    void setContacts(const std::vector<bool> &foot_contacts, std::vector<double> &foot_height);

    const Eigen::Isometry3d& getBasefootPoseInBase();

    const Eigen::Isometry3d& getBasefootPoseInOdom();

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

    Eigen::Vector3d tmp_v_;
    Eigen::Matrix3d tmp_R_;

};

} // namespace

#endif
