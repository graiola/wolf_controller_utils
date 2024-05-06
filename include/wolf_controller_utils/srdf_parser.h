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
*/

#ifndef WOLF_CONTROLLER_UTILS_SRDF_PARSER_H
#define WOLF_CONTROLLER_UTILS_SRDF_PARSER_H

#include <ros/ros.h>
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

    SRDFParser();

    void parseSRDF(const std::string& robot_namespace);

    std::vector<std::string> getJointNames();
    std::string getImuLinkName();
    std::string getBaseLinkName();
    std::vector<std::string> getContactNames();
    std::vector<std::string> getHipNames();
    std::vector<std::string> getArmNames();
    std::vector<std::string> getLegNames();
    std::vector<std::string> getEndEffectorNames();
    std::vector<std::string> getFootNames();
    joints_map_t getJointLegNames();
    joints_map_t getJointArmNames();
    std::string getRobotModelName();

protected:

    ros::NodeHandle nh_;
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

    bool parseSRDF(srdf::Model& srdf_model);
    std::vector<std::string> parseGroupNames(const std::string& group_name);
};

}

#endif
