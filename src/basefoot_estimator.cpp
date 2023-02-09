#include <wolf_controller_utils/basefoot_estimator.h>
#include <wolf_controller_utils/geometry.h>

using namespace wolf_controller_utils;

BasefootEstimator::BasefootEstimator()
{
  base_T_basefoot_.setIdentity();
  odom_T_basefoot_.setIdentity();
  odom_T_base_.setIdentity();
  basefoot_T_odom_.setIdentity();
}

void BasefootEstimator::setContacts(const std::vector<bool> &foot_contacts, std::vector<double> &foot_height)
{
  if(foot_height_.size() == foot_contacts_.size())
  {
    foot_contacts_ = foot_contacts;
    foot_height_ = foot_height;
  }
}

void BasefootEstimator::setBasePoseInOdom(const Eigen::Isometry3d& pose)
{
  odom_T_base_ = pose;
}

const Eigen::Isometry3d &BasefootEstimator::getBasefootPoseInBase()
{
  return base_T_basefoot_;
}

const Eigen::Isometry3d &BasefootEstimator::getBasefootPoseInOdom()
{
  return odom_T_basefoot_;
}

void BasefootEstimator::update()
{
  // Reset
  base_T_basefoot_.setIdentity();
  odom_T_basefoot_.setIdentity();
  basefoot_T_odom_.setIdentity();
  tmp_v_.setZero();
  tmp_R_.setIdentity();

  // Create base_T_basefoot
  base_T_basefoot_.translation().x() = 0.0;
  base_T_basefoot_.translation().y() = 0.0;
  base_T_basefoot_.translation().z() = -estimateHeight();

  // Create odom_T_basefoot
  tmp_v_ =  odom_T_base_.translation();
  tmp_v_(2) = tmp_v_(2) + base_T_basefoot_.translation().z();
  double base_yaw   = std::atan2(odom_T_base_.linear()(1,0),odom_T_base_.linear()(0,0));
  rpyToRotTranspose(0.0,0.0,base_yaw,tmp_R_);
  tmp_v_ = - tmp_R_ * tmp_v_;
  basefoot_T_odom_.translation().x() = tmp_v_(0);
  basefoot_T_odom_.translation().y() = tmp_v_(1);
  basefoot_T_odom_.translation().z() = tmp_v_(2);
  basefoot_T_odom_.linear()          = tmp_R_;
  odom_T_basefoot_ = basefoot_T_odom_.inverse();
}

double BasefootEstimator::estimateHeight()
{
  // Estimate z using the foot heights
  double estimated_z = 0.0;
  int feet_in_stance = 0;
  for(unsigned int i = 0; i<foot_contacts_.size(); i++)
  {
    if(foot_contacts_[i])
    {
      feet_in_stance++;
      estimated_z += foot_height_[i];
    }
  }
  estimated_z /= feet_in_stance;
  return estimated_z;
}
