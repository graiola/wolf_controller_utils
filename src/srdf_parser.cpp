#include "wolf_controller_utils/srdf_parser.h"
#include "wolf_controller_utils/tools.h"
#include "wolf_controller_utils/common.h"

#include <urdf_parser/urdf_parser.h>
#include <srdfdom/srdf_writer.h>

using namespace wolf_controller_utils;

SRDFParser::SRDFParser(const std::string& description_param_name)
  :description_param_name_(description_param_name)
{

}

#ifdef ROS
#include <ros/ros.h>
void SRDFParser::parseSRDF(const std::string& robot_namespace)
{
  ros::NodeHandle nh(robot_namespace);

  std::string urdf, srdf;

  if(!nh.getParam(description_param_name_,urdf))
  {
    PRINT_WARN_NAMED(CLASS_NAME,"robot_description not available in the ROS parameter server");
  }

  if(!nh.getParam(description_param_name_+"_semantic",srdf))
  {
    PRINT_WARN_NAMED(CLASS_NAME,"robot_description_semantic not available in the ROS parameter server");
  }

  parseSRDF(urdf,srdf);
}

#elif defined(ROS2)
#include <rclcpp/rclcpp.hpp>
void SRDFParser::parseSRDF(const std::string& robot_namespace)
{
  auto nh = rclcpp::Node::make_shared(robot_namespace);

  std::string urdf, srdf;

  if (!nh->get_parameter(description_param_name_, urdf))
  {
    PRINT_WARN_NAMED(CLASS_NAME, "robot_description not available in the ROS parameter server");
  }

  if (!nh->get_parameter(description_param_name_+"_semantic", srdf))
  {
    PRINT_WARN_NAMED(CLASS_NAME, "robot_description_semantic not available in the ROS parameter server");
  }

  parseSRDF(urdf, srdf);
}
#endif

void SRDFParser::parseSRDF(const std::string& urdf, const std::string& srdf)
{
  if(urdf.empty())
  {
    PRINT_WARN_NAMED(CLASS_NAME,"urdf string is empty!");
    return;
  }
  else
    urdf_ = urdf;

  if(srdf.empty())
  {
    PRINT_WARN_NAMED(CLASS_NAME,"srdf string is empty!");
    return;
  }
  else
    srdf_ = srdf;

  srdf::Model srdf_model;
  if(createSRDFModel(srdf_model))
  {
    robot_model_name_ = srdf_model.getName();

    for(unsigned int i=0;i < srdf_model.getGroups().size(); i++)
    {
      const auto& chains = srdf_model.getGroups()[i].chains_;
      const auto& joints = srdf_model.getGroups()[i].joints_;
      const auto& links  = srdf_model.getGroups()[i].links_;

      // Parse the foot tip_link from the SRDF file
      if(srdf_model.getGroups()[i].name_.find("leg") != std::string::npos)
      {
        leg_names_.push_back(srdf_model.getGroups()[i].name_);
        for(unsigned int j=0;j<chains.size();j++)
          foot_names_.push_back(chains[j].second);
        for(unsigned int j=0;j<joints.size();j++)
          joint_leg_names_[srdf_model.getGroups()[i].name_].push_back(joints[j]);
      }
      // Parse the arm tip_link from the SRDF file
      if(srdf_model.getGroups()[i].name_.find("arm") != std::string::npos)
      {
        arm_names_.push_back(srdf_model.getGroups()[i].name_);
        for(unsigned int j=0;j<chains.size();j++)
          ee_names_.push_back(chains[j].second);
        for(unsigned int j=0;j<joints.size();j++)
          joint_arm_names_[srdf_model.getGroups()[i].name_].push_back(joints[j]);
      }
      // Parse the hip tip_link from the SRDF file
      if(srdf_model.getGroups()[i].name_.find("hip") != std::string::npos)
      {
        for(unsigned int j=0;j<chains.size();j++)
          hip_names_.push_back(chains[j].second);
      }
      // Parse the base link name from the SRDF file
      if(srdf_model.getGroups()[i].name_.find("base") != std::string::npos)
      {
        if(links.size()==1)
          base_name_ = links[0];
        else
          throw std::runtime_error("There can be only one base defined in the SRDF file!");
      }
      // Parse the imu link name from the SRDF file
      if(srdf_model.getGroups()[i].name_.find("imu") != std::string::npos)
      {
        if(links.size()==1)
          imu_name_ = links[0];
        else
          throw std::runtime_error("There can be only one imu defined in the SRDF file!");
      }
    }
    // Parse the joint names from the standup groupstate
    auto group_states = srdf_model.getGroupStates();
    for(unsigned int i=0;i<group_states.size();i++)
      if(group_states[i].name_ == "standup") // Look for the standup group state and get the names of the joints in there
        for(auto & tmp : group_states[i].joint_values_)
          joint_names_.push_back(tmp.first);

    hip_names_ = sortByLegPrefix(hip_names_);
    foot_names_ = sortByLegPrefix(foot_names_);
    leg_names_ = sortByLegPrefix(leg_names_);

  }
  else
    throw std::runtime_error("Can not parse the SRDF file!");
}

const std::string& SRDFParser::getSRDFString()
{
  return srdf_;
}

const std::string& SRDFParser::getURDFString()
{
  return urdf_;
}

const std::vector<std::string>& SRDFParser::getJointNames()
{
  return joint_names_;
}

const std::string& SRDFParser::getImuLinkName()
{
  return imu_name_;
}

const std::string& SRDFParser::getBaseLinkName()
{
  return base_name_;
}

const std::vector<std::string>& SRDFParser::getContactNames()
{
  return contact_names_;
}

const std::vector<std::string>& SRDFParser::getHipNames()
{
  return hip_names_;
}

const std::vector<std::string>& SRDFParser::getArmNames()
{
  return arm_names_;
}

const std::vector<std::string>& SRDFParser::getLegNames()
{
  return leg_names_;
}

const std::vector<std::string>& SRDFParser::getEndEffectorNames()
{
  return ee_names_;
}

const std::vector<std::string>& SRDFParser::getFootNames()
{
  return foot_names_;
}

const SRDFParser::joints_map_t& SRDFParser::getJointLegNames()
{
  return joint_leg_names_;
}

const SRDFParser::joints_map_t& SRDFParser::getJointArmNames()
{
  return joint_arm_names_;
}

const std::string& SRDFParser::getRobotModelName()
{
  return robot_model_name_;
}

bool SRDFParser::createSRDFModel(srdf::Model& srdf_model)
{
  urdf::ModelInterfaceSharedPtr u = urdf::parseURDF(urdf_);
  if(!srdf_model.initString(*u,srdf_))
  {
    PRINT_ERROR_NAMED(CLASS_NAME,"Can not initialize SRDF model from XML string!");
    return false;
  }
  return true;
}
