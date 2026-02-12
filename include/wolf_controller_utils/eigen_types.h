/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) Gennaro Raiola
 */

#ifndef WOLF_CONTROLLER_UTILS_EIGEN_TYPES_H
#define WOLF_CONTROLLER_UTILS_EIGEN_TYPES_H

#include <Eigen/Core>

namespace Eigen
{
typedef Matrix<double,6,1> Vector6d;
typedef Matrix<double,7,1> Vector7d;
typedef Matrix<double,6,6> Matrix6d;
typedef Matrix<double,4,3> Matrix4x3d;
typedef Matrix<double,3,6> Matrix3x6d;
}

namespace wolf_controller_utils {
namespace eigen {

template <typename T>
using RotMat = Eigen::Matrix<T, 3, 3>;

template <typename T>
using Vec2 = Eigen::Matrix<T, 2, 1>;

template <typename T>
using Vec3 = Eigen::Matrix<T, 3, 1>;

template <typename T>
using Vec4 = Eigen::Matrix<T, 4, 1>;

template <typename T>
using Vec6 = Eigen::Matrix<T, 6, 1>;

template <typename T>
using Vec10 = Eigen::Matrix<T, 10, 1>;

template <typename T>
using Vec12 = Eigen::Matrix<T, 12, 1>;

template <typename T>
using Vec18 = Eigen::Matrix<T, 18, 1>;

template <typename T>
using Vec28 = Eigen::Matrix<T, 28, 1>;

template <typename T>
using Mat3 = Eigen::Matrix<T, 3, 3>;

template <typename T>
using Quat = Eigen::Matrix<T, 4, 1>;

template <typename T>
using SVec = Eigen::Matrix<T, 6, 1>;

template <typename T>
using SXform = Eigen::Matrix<T, 6, 6>;

template <typename T>
using Mat6 = Eigen::Matrix<T, 6, 6>;

template <typename T>
using Mat12 = Eigen::Matrix<T, 12, 12>;

template <typename T>
using Mat18 = Eigen::Matrix<T, 18, 18>;

template <typename T>
using Mat28 = Eigen::Matrix<T, 28, 28>;

template <typename T>
using Mat34 = Eigen::Matrix<T, 3, 4>;

template <typename T>
using Mat4 = Eigen::Matrix<T, 4, 4>;

} // namespace eigen
} // namespace wolf_controller_utils

#endif // WOLF_CONTROLLER_UTILS_EIGEN_TYPES_H
