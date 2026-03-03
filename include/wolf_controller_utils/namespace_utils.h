/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) Gennaro Raiola
 */

#ifndef WOLF_CONTROLLER_UTILS_NAMESPACE_UTILS_H
#define WOLF_CONTROLLER_UTILS_NAMESPACE_UTILS_H

#include <string>

namespace wolf_controller_utils
{

inline std::string normalize_namespace(std::string ns)
{
  while(!ns.empty() && ns.front() == '/')
    ns.erase(ns.begin());
  while(!ns.empty() && ns.back() == '/')
    ns.pop_back();
  return ns;
}

inline std::string controller_namespace(const std::string& robot_name)
{
  const std::string ns = normalize_namespace(robot_name);
  if(ns.empty())
    return "/wolf_controller";
  return "/" + ns + "/wolf_controller";
}

inline std::string controller_topic(const std::string& robot_name, const std::string& topic_name)
{
  const std::string topic = normalize_namespace(topic_name);
  if(topic.empty())
    return controller_namespace(robot_name);
  return controller_namespace(robot_name) + "/" + topic;
}

} // namespace wolf_controller_utils

#endif // WOLF_CONTROLLER_UTILS_NAMESPACE_UTILS_H
