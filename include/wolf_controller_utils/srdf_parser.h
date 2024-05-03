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

    typedef std::shared_ptr<SRDFParser> Ptr;

    const std::string CLASS_NAME = "SRDFParser";

    SRDFParser();

    SRDFParser(const std::string& robot_namespace);

    void setNodeHandle(ros::NodeHandle& nh);

    std::vector<std::string> parseJointNames();
    std::string parseImuLinkName();
    std::string parseBaseLinkName();
    std::vector<std::string> parseContactNames();
    std::vector<std::string> parseHipNames();
    std::vector<std::string> parseArmNames();
    std::vector<std::string> parseLegNames();

protected:

    ros::NodeHandle nh_;
    bool parseSRDF(srdf::Model& srdf_model);
    std::vector<std::string> parseGroupNames(const std::string& group_name);
};

}

#endif
