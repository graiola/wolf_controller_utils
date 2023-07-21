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

#ifndef WOLF_CONTROLLER_UTILS_COMMON_H
#define WOLF_CONTROLLER_UTILS_COMMON_H

#include <Eigen/Core>

namespace Eigen
{
    typedef Matrix<double,6,1> Vector6d;
    typedef Matrix<double,7,1> Vector7d;
    typedef Matrix<double,6,6> Matrix6d;
    typedef Matrix<double,4,3> Matrix4x3d;
    typedef Matrix<double,3,6> Matrix3x6d;
}

#endif
