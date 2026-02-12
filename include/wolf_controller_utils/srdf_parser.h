/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) Gennaro Raiola
 */

#ifndef WOLF_CONTROLLER_UTILS_SRDF_PARSER_H
#define WOLF_CONTROLLER_UTILS_SRDF_PARSER_H

#include <urdf/model.h>
#include <srdfdom/model.h>

namespace wolf_controller_utils
{

class SRDFParser
{
public:

    typedef std::map<std::string,std::vector<std::string>> joints_map_t;

    typedef std::shared_ptr<SRDFParser> Ptr;

    const std::string CLASS_NAME = "SRDFParser";

    SRDFParser(const std::string& description_param_name = "robot_description");

    void parseSRDF(const std::string& urdf, const std::string& srdf);

    void parseSRDF(const std::string& robot_namespace);

    const std::vector<std::string>& getJointNames();
    const std::string& getImuLinkName();
    const std::string& getBaseLinkName();
    const std::vector<std::string>& getContactNames();
    const std::vector<std::string>& getHipNames();
    const std::vector<std::string>& getArmNames();
    const std::vector<std::string>& getLegNames();
    const std::vector<std::string>& getEndEffectorNames();
    const std::vector<std::string>& getFootNames();
    const joints_map_t& getJointLegNames();
    const joints_map_t& getJointArmNames();
    const std::string& getRobotModelName();

    const std::string& getSRDFString();
    const std::string& getURDFString();

protected:

    std::string description_param_name_;

    std::vector<std::string> foot_names_; // foot tip names
    std::vector<std::string> hip_names_;
    std::vector<std::string> leg_names_;
    std::vector<std::string> arm_names_;
    std::vector<std::string> joint_names_;
    std::vector<std::string> ee_names_; // end-effector names
    std::vector<std::string> contact_names_; // foot + arm names
    std::vector<std::string> limb_names_; // chain names
    std::string base_name_;
    std::string imu_name_;
    std::string robot_model_name_;

    joints_map_t joint_leg_names_;
    joints_map_t joint_arm_names_;

    std::string urdf_;
    std::string srdf_;

    bool createSRDFModel(srdf::Model& srdf_model);
    std::vector<std::string> parseGroupNames(const std::string& group_name);
};

}

#endif
