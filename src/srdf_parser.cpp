#include "wolf_controller_utils/srdf_parser.h"

#include <urdf_parser/urdf_parser.h>
#include <srdfdom/srdf_writer.h>

using namespace wolf_controller_utils;

SRDFParser::SRDFParser()
{

}

SRDFParser::SRDFParser(const std::string& robot_namespace)
  :nh_(robot_namespace)
{

}

void SRDFParser::setNodeHandle(ros::NodeHandle& nh)
{
  nh_ = nh;
}

std::vector<std::string> SRDFParser::parseJointNames()
{
  std::vector<std::string> joint_names;
  srdf::Model srdf_model;
  if(parseSRDF(srdf_model))
  {
    auto group_states = srdf_model.getGroupStates();
    for(unsigned int i=0;i<group_states.size();i++)
      if(group_states[i].name_ == "standup") // Look for the standup group state and get the names of the joints in there
        for(auto & tmp : group_states[i].joint_values_)
          joint_names.push_back(tmp.first);
  }
  return joint_names;
}

std::string SRDFParser::parseImuLinkName()
{
  std::string imu_name;
  srdf::Model srdf_model;

  if(parseSRDF(srdf_model))
  {
    auto groups = srdf_model.getGroups();
    for(unsigned int i=0;i < groups.size(); i++)
    {
      const auto& links  = groups[i].links_;
      if(groups[i].name_.find("imu") != std::string::npos)
      {
        if(links.size()==1)
          imu_name = links[0];
        else
          throw std::runtime_error("There can be only one imu_sensor defined in the SRDF file!");
      }
    }
  }
  return imu_name;
}

std::string SRDFParser::parseBaseLinkName()
{
  std::string base_name;
  srdf::Model srdf_model;
  if(parseSRDF(srdf_model))
  {
    auto groups = srdf_model.getGroups();
    for(unsigned int i=0;i < groups.size(); i++)
    {
      const auto& links  = groups[i].links_;
      if(groups[i].name_.find("base") != std::string::npos)
      {
        if(links.size()==1)
          base_name = links[0];
        else
          throw std::runtime_error("There can be only one base defined in the SRDF file!");
      }
    }
  }
  return base_name;
}

std::vector<std::string> SRDFParser::parseContactNames()
{
  std::vector<std::string> contact_names;
  srdf::Model srdf_model;
  if(parseSRDF(srdf_model))
  {
    auto groups = srdf_model.getGroups();
    for(unsigned int i=0;i < groups.size(); i++)
    {
      const auto& links  = groups[i].links_;
      if(groups[i].name_.find("contacts") != std::string::npos)
        for(unsigned int j=0;j<links.size();j++)
          contact_names.push_back(links[j]);
    }
  }
  return contact_names;
}

std::vector<std::string> SRDFParser::parseHipNames()
{
  std::vector<std::string> hip_names;
  srdf::Model srdf_model;
  if(parseSRDF(srdf_model))
  {
    auto groups = srdf_model.getGroups();
    for(unsigned int i=0;i < groups.size(); i++)
    {
      const auto& chains = groups[i].chains_;
      if(groups[i].name_.find("hip") != std::string::npos)
        for(unsigned int j=0;j<chains.size();j++)
          hip_names.push_back(chains[j].second);
    }
  }
  return hip_names;
}

std::vector<std::string> SRDFParser::parseArmNames()
{
  return parseGroupNames("arm");
}

std::vector<std::string> SRDFParser::parseLegNames()
{
  return parseGroupNames("leg");
}

bool SRDFParser::parseSRDF(srdf::Model& srdf_model)
{
  std::string srdf, urdf;
  if(!nh_.getParam("robot_description",urdf))
  {
    ROS_ERROR_NAMED(CLASS_NAME,"robot_description not available in the ros param server");
    return false;
  }
  if(!nh_.getParam("robot_description_semantic",srdf))
  {
    ROS_ERROR_NAMED(CLASS_NAME,"robot_description_semantic not available in the ros param server");
    return false;
  }

  urdf::ModelInterfaceSharedPtr u = urdf::parseURDF(urdf);
  if(!srdf_model.initString(*u,srdf))
  {
    ROS_ERROR_NAMED(CLASS_NAME,"Can not initialize SRDF model from XML string!");
    return false;
  }

  return true;

}

std::vector<std::string> SRDFParser::parseGroupNames(const std::string& group_name)
{
  std::vector<std::string> names;
  srdf::Model srdf_model;
  if(parseSRDF(srdf_model))
  {
    auto groups = srdf_model.getGroups();
    for(unsigned int i=0;i < groups.size(); i++)
    {
      if(groups[i].name_.find(group_name) != std::string::npos)
          names.push_back(groups[i].name_);
    }
  }
  return names;
}
