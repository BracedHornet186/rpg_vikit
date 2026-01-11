/*
 * camera_loader.h
 *
 * Created on: Feb 11, 2014
 * Author: cforster
 * Converted to ROS 2
 */

#ifndef VIKIT_CAMERA_LOADER_H_
#define VIKIT_CAMERA_LOADER_H_

#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <vikit/abstract_camera.h>
#include <vikit/pinhole_camera.h>
#include <vikit/atan_camera.h>
#include <vikit/omni_camera.h>
#include <vikit/equidistant_camera.h>
#include <vikit/polynomial_camera.h>
#include <vikit/params_helper.h> // Assumes this is the ROS 2 converted version

namespace vk {
namespace camera_loader {

/// Load from ROS Namespace
/// @param node Pointer to the ROS 2 node that handles parameters
/// @param ns   The parameter namespace/prefix (e.g., "camera" or "laserMapping")
inline bool loadFromRosNs(rclcpp::Node* node, const std::string& ns, std::shared_ptr<vk::AbstractCamera>& cam)
{
  bool res = true;
  std::string prefix = ns.empty() ? "" : ns + ".";
  std::string cam_model = getParam<std::string>(node, prefix + "cam_model");
  
  if(cam_model == "Ocam")
  {
    cam = std::make_shared<vk::OmniCamera>(getParam<std::string>(node, prefix + "cam_calib_file", ""));
  }
  else if(cam_model == "Pinhole")
  {
    cam = std::make_shared<vk::PinholeCamera>(
        getParam<int>(node, prefix + "cam_width"),
        getParam<int>(node, prefix + "cam_height"),
        getParam<double>(node, prefix + "scale", 1.0),
        getParam<double>(node, prefix + "cam_fx"),
        getParam<double>(node, prefix + "cam_fy"),
        getParam<double>(node, prefix + "cam_cx"),
        getParam<double>(node, prefix + "cam_cy"),
        getParam<double>(node, prefix + "cam_d0", 0.0),
        getParam<double>(node, prefix + "cam_d1", 0.0),
        getParam<double>(node, prefix + "cam_d2", 0.0),
        getParam<double>(node, prefix + "cam_d3", 0.0));
  }
  else if(cam_model == "EquidistantCamera")
  {
    cam = std::make_shared<vk::EquidistantCamera>(
        getParam<int>(node, prefix + "cam_width"),
        getParam<int>(node, prefix + "cam_height"),
        getParam<double>(node, prefix + "scale", 1.0),
        getParam<double>(node, prefix + "cam_fx"),
        getParam<double>(node, prefix + "cam_fy"),
        getParam<double>(node, prefix + "cam_cx"),
        getParam<double>(node, prefix + "cam_cy"),
        getParam<double>(node, prefix + "k1", 0.0),
        getParam<double>(node, prefix + "k2", 0.0),
        getParam<double>(node, prefix + "k3", 0.0),
        getParam<double>(node, prefix + "k4", 0.0));
  }
  else if(cam_model == "PolynomialCamera")
  {
    cam = std::make_shared<vk::PolynomialCamera>(
        getParam<int>(node, prefix + "cam_width"),
        getParam<int>(node, prefix + "cam_height"),
        // getParam<double>(node, ns+".scale", 1.0),
        getParam<double>(node, prefix + "cam_fx"),
        getParam<double>(node, prefix + "cam_fy"),
        getParam<double>(node, prefix + "cam_cx"),
        getParam<double>(node, prefix + "cam_cy"),
        getParam<double>(node, prefix + "cam_skew"),
        getParam<double>(node, prefix + "k2", 0.0),
        getParam<double>(node, prefix + "k3", 0.0),
        getParam<double>(node, prefix + "k4", 0.0),
        getParam<double>(node, prefix + "k5", 0.0),
        getParam<double>(node, prefix + "k6", 0.0),
        getParam<double>(node, prefix + "k7", 0.0));
  }
  else if(cam_model == "ATAN")
  {
    cam = std::make_shared<vk::ATANCamera>(
        getParam<int>(node, prefix + "cam_width"),
        getParam<int>(node, prefix + "cam_height"),
        getParam<double>(node, prefix + "cam_fx"),
        getParam<double>(node, prefix + "cam_fy"),
        getParam<double>(node, prefix + "cam_cx"),
        getParam<double>(node, prefix + "cam_cy"),
        getParam<double>(node, prefix + "cam_d0"));
  }
  else
  {
    cam = NULL;
    res = false;
  }
  return res;
}

/// Load list of cameras from ROS Namespace
inline bool loadFromRosNs(rclcpp::Node* node, const std::string& ns, std::vector<std::shared_ptr<vk::AbstractCamera>>& cam_list)
{
  bool res = true;
  std::string prefix = ns.empty() ? "" : ns + ".";
  std::string cam_model = getParam<std::string>(node, ns+".cam_model");
  int cam_num = getParam<int>(node, ns+".cam_num");
  
  for (int i = 0; i < cam_num; i ++)
  {
    std::string cam_ns = ns + ".cam_" + std::to_string(i);
    std::string cam_model_i = getParam<std::string>(node, cam_ns+".cam_model");
    
    if(cam_model_i == "FishPoly")
    {
      cam_list.push_back(std::make_shared<vk::PolynomialCamera>(
        getParam<int>(node, cam_ns+".image_width"),
        getParam<int>(node, cam_ns+".image_height"),
        // getParam<double>(node, cam_ns+".scale", 1.0),
        getParam<double>(node, cam_ns+".A11"),  // cam_fx
        getParam<double>(node, cam_ns+".A22"),  // cam_fy
        getParam<double>(node, cam_ns+".u0"),  // cam_cx
        getParam<double>(node, cam_ns+".v0"),  // cam_cy
        getParam<double>(node, cam_ns+".A12"), // cam_skew
        getParam<double>(node, cam_ns+".k2", 0.0),
        getParam<double>(node, cam_ns+".k3", 0.0),
        getParam<double>(node, cam_ns+".k4", 0.0),
        getParam<double>(node, cam_ns+".k5", 0.0),
        getParam<double>(node, cam_ns+".k6", 0.0),
        getParam<double>(node, cam_ns+".k7", 0.0)));
    }
    else if(cam_model_i == "Pinhole")
    {
      cam_list.push_back(std::make_shared<vk::PinholeCamera>(
          getParam<int>(node, ns+".cam_width"),
          getParam<int>(node, ns+".cam_height"),
          getParam<double>(node, ns+".scale", 1.0),
          getParam<double>(node, ns+".cam_fx"),
          getParam<double>(node, ns+".cam_fy"),
          getParam<double>(node, ns+".cam_cx"),
          getParam<double>(node, ns+".cam_cy"),
          getParam<double>(node, ns+".cam_d0", 0.0),
          getParam<double>(node, ns+".cam_d1", 0.0),
          getParam<double>(node, ns+".cam_d2", 0.0),
          getParam<double>(node, ns+".cam_d3", 0.0)));
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
