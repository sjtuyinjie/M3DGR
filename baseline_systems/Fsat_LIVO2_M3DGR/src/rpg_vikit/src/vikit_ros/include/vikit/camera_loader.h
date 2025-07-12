/*
 * camera_loader.h
 *
 *  Created on: Feb 11, 2014
 *      Author: cforster
 */

#ifndef VIKIT_CAMERA_LOADER_H_
#define VIKIT_CAMERA_LOADER_H_

#include <string>
#include <vikit/abstract_camera.h>
#include <vikit/pinhole_camera.h>
#include <vikit/atan_camera.h>
#include <vikit/omni_camera.h>
#include <vikit/equidistant_camera.h>
#include <vikit/polynomial_camera.h>
#include <vikit/params_helper.h>

namespace vk {
namespace camera_loader {

/// Load from ROS Namespace
bool loadFromRosNs(const std::string& ns, vk::AbstractCamera*& cam)
{
  bool res = true;
  std::string cam_model(getParam<std::string>(ns+"/cam_model"));
  if(cam_model == "Ocam")
  {
    cam = new vk::OmniCamera(getParam<std::string>(ns+"/cam_calib_file", ""));
  }
  else if(cam_model == "Pinhole")
  {
    cam = new vk::PinholeCamera(
        getParam<int>(ns+"/cam_width"),
        getParam<int>(ns+"/cam_height"),
        getParam<double>(ns+"/scale", 1.0),
        getParam<double>(ns+"/cam_fx"),
        getParam<double>(ns+"/cam_fy"),
        getParam<double>(ns+"/cam_cx"),
        getParam<double>(ns+"/cam_cy"),
        getParam<double>(ns+"/cam_d0", 0.0),
        getParam<double>(ns+"/cam_d1", 0.0),
        getParam<double>(ns+"/cam_d2", 0.0),
        getParam<double>(ns+"/cam_d3", 0.0));
  }
  else if(cam_model == "EquidistantCamera")
  {
    cam = new vk::EquidistantCamera(
        getParam<int>(ns+"/cam_width"),
        getParam<int>(ns+"/cam_height"),
        getParam<double>(ns+"/scale", 1.0),
        getParam<double>(ns+"/cam_fx"),
        getParam<double>(ns+"/cam_fy"),
        getParam<double>(ns+"/cam_cx"),
        getParam<double>(ns+"/cam_cy"),
        getParam<double>(ns+"/k1", 0.0),
        getParam<double>(ns+"/k2", 0.0),
        getParam<double>(ns+"/k3", 0.0),
        getParam<double>(ns+"/k4", 0.0));
  }
  else if(cam_model == "PolynomialCamera")
  {
    cam = new vk::PolynomialCamera(
        getParam<int>(ns+"/cam_width"),
        getParam<int>(ns+"/cam_height"),
        // getParam<double>(ns+"/scale", 1.0),
        getParam<double>(ns+"/cam_fx"),
        getParam<double>(ns+"/cam_fy"),
        getParam<double>(ns+"/cam_cx"),
        getParam<double>(ns+"/cam_cy"),
        getParam<double>(ns+"/cam_skew"),
        getParam<double>(ns+"/k2", 0.0),
        getParam<double>(ns+"/k3", 0.0),
        getParam<double>(ns+"/k4", 0.0),
        getParam<double>(ns+"/k5", 0.0),
        getParam<double>(ns+"/k6", 0.0),
        getParam<double>(ns+"/k7", 0.0));
  }
  else if(cam_model == "ATAN")
  {
    cam = new vk::ATANCamera(
        getParam<int>(ns+"/cam_width"),
        getParam<int>(ns+"/cam_height"),
        getParam<double>(ns+"/cam_fx"),
        getParam<double>(ns+"/cam_fy"),
        getParam<double>(ns+"/cam_cx"),
        getParam<double>(ns+"/cam_cy"),
        getParam<double>(ns+"/cam_d0"));
  }
  else
  {
    cam = NULL;
    res = false;
  }
  return res;
}

bool loadFromRosNs(const std::string& ns, std::vector<vk::AbstractCamera*>& cam_list)
{
  bool res = true;
  std::string cam_model(getParam<std::string>(ns+"/cam_model"));
  int cam_num = getParam<int>(ns+"/cam_num");
  for (int i = 0; i < cam_num; i ++)
  {
    std::string cam_ns = ns + "/cam_" + std::to_string(i);
    std::string cam_model(getParam<std::string>(cam_ns+"/cam_model"));
    if(cam_model == "FishPoly")
    {
      cam_list.push_back(new vk::PolynomialCamera(
        getParam<int>(cam_ns+"/image_width"),
        getParam<int>(cam_ns+"/image_height"),
        // getParam<double>(cam_ns+"/scale", 1.0),
        getParam<double>(cam_ns+"/A11"),  // cam_fx
        getParam<double>(cam_ns+"/A22"),  // cam_fy
        getParam<double>(cam_ns+"/u0"),  // cam_cx
        getParam<double>(cam_ns+"/v0"),  // cam_cy
        getParam<double>(cam_ns+"/A12"), // cam_skew
        getParam<double>(cam_ns+"/k2", 0.0),
        getParam<double>(cam_ns+"/k3", 0.0),
        getParam<double>(cam_ns+"/k4", 0.0),
        getParam<double>(cam_ns+"/k5", 0.0),
        getParam<double>(cam_ns+"/k6", 0.0),
        getParam<double>(cam_ns+"/k7", 0.0)));
    }
    else if(cam_model == "Pinhole")
    {
      cam_list.push_back(new vk::PinholeCamera(
          getParam<int>(ns+"/cam_width"),
          getParam<int>(ns+"/cam_height"),
          getParam<double>(ns+"/scale", 1.0),
          getParam<double>(ns+"/cam_fx"),
          getParam<double>(ns+"/cam_fy"),
          getParam<double>(ns+"/cam_cx"),
          getParam<double>(ns+"/cam_cy"),
          getParam<double>(ns+"/cam_d0", 0.0),
          getParam<double>(ns+"/cam_d1", 0.0),
          getParam<double>(ns+"/cam_d2", 0.0),
          getParam<double>(ns+"/cam_d3", 0.0)));
    }
    else 
    {
      // cam_list.clear();
      res = false;
    }
  }
  
  return res;
}

} // namespace camera_loader
} // namespace vk

#endif // VIKIT_CAMERA_LOADER_H_
