/*
 * ros_params_helper.h
 *
 *  Created on: Feb 22, 2013
 *      Author: cforster
 *
 * from libpointmatcher_ros
 */

#ifndef ROS_PARAMS_HELPER_H_
#define ROS_PARAMS_HELPER_H_

#include <string>
#include <ros/ros.h>

namespace vk {

inline
bool hasParam(const std::string& name)
{
  return ros::param::has(name);
}

template<typename T>
T getParam(const std::string& name, const T& defaultValue)
{
  T v;
  if(ros::param::get(name, v))
  {
    ROS_INFO_STREAM("Found parameter: " << name << ", value: " << v);
    return v;
  }
  else
    ROS_WARN_STREAM("Cannot find value for parameter: " << name << ", assigning default: " << defaultValue);
  return defaultValue;
}

template<typename T>
T getParam(const std::string& name)
{
  T v;
  int i = 0;
  while(ros::param::get(name, v) == false)
  {
    ROS_ERROR_STREAM("Cannot find value for parameter: " << name << ", will try again.");
    if ((i ++) >= 5) return T();
  }
  
  ROS_INFO_STREAM("Found parameter: " << name << ", value: " << v);
  return v;
}

} // namespace vk

#endif // ROS_PARAMS_HELPER_H_
