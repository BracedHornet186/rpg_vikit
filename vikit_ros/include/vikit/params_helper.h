/*
 * ros_params_helper.h
 *
 * Robust ROS 2 Parameter Helper
 */

#ifndef ROS_PARAMS_HELPER_H_
#define ROS_PARAMS_HELPER_H_

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <stdexcept>

namespace vk {

/**
 * @brief Check if a parameter exists/is declared on the given node.
 */
inline bool hasParam(rclcpp::Node* node, const std::string& name)
{
  return node->has_parameter(name);
}

/**
 * @brief Get a parameter with a fallback default value.
 * * Logic:
 * 1. If parameter exists, get it.
 * 2. If not, declare it using the provided default value.
 * 3. Log warnings only if the default value had to be used.
 */
template<typename T>
T getParam(rclcpp::Node* node, const std::string& name, const T& defaultValue)
{
  T v;
  
  // Case 1: Parameter already declared (e.g. by another part of the code)
  if (node->has_parameter(name)) 
  {
    if (node->get_parameter(name, v)) 
    {
      return v;
    }
  }

  // Case 2: Not declared, try to declare with default
  try {
    // declare_parameter returns the value set. 
    // If it was in the YAML, we get the YAML value. If not, we get defaultValue.
    v = node->declare_parameter<T>(name, defaultValue);
    
    // Check if the returned value matches default to log appropriately
    if (v == defaultValue) {
        RCLCPP_WARN_STREAM(node->get_logger(), "Parameter '" << name << "' not found/set. Using default: " << defaultValue);
    } else {
        RCLCPP_INFO_STREAM(node->get_logger(), "Loaded parameter '" << name << "': " << v);
    }
    return v;

  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException& e) {
    // Race condition safety: if it got declared between our check and call
    if (node->get_parameter(name, v)) {
      return v;
    }
  } catch (const rclcpp::exceptions::InvalidParameterTypeException& e) {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Parameter '" << name << "' type mismatch: " << e.what());
  }

  return defaultValue;
}

/**
 * @brief Get a REQUIRED parameter (No default value).
 * * Logic:
 * 1. Try to declare the parameter WITHOUT a default.
 * 2. If it is in the YAML config, this succeeds and we get the value.
 * 3. If it is NOT in the YAML config, ROS 2 throws ParameterUninitializedException.
 * We catch this and throw a runtime error to stop the node safely.
 */
template<typename T>
T getParam(rclcpp::Node* node, const std::string& name)
{
  // 1. Ensure parameter is declared
  if (!node->has_parameter(name)) {
    try {
      // This will THROW if the parameter is not in the YAML file
      node->declare_parameter<T>(name);
    } catch (const rclcpp::exceptions::ParameterUninitializedException&) {
      RCLCPP_ERROR_STREAM(node->get_logger(), "CRITICAL ERROR: Required parameter '" << name << "' is missing from configuration!");
      throw std::runtime_error("Missing required parameter: " + name);
    } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&) {
      // Already declared, proceed to get it
    } catch (const rclcpp::exceptions::InvalidParameterTypeException& e) {
      RCLCPP_ERROR_STREAM(node->get_logger(), "Parameter '" << name << "' has invalid type: " << e.what());
      throw;
    }
  }

  // 2. Retrieve the value
  T v;
  if (node->get_parameter(name, v)) {
    // Optional: Verbose logging for debugging
    RCLCPP_INFO_STREAM(node->get_logger(), "Loaded required parameter '" << name << "': " << v);
    return v;
  }
  
  // Should rarely reach here if declare_parameter succeeded
  RCLCPP_ERROR_STREAM(node->get_logger(), "Failed to retrieve value for '" << name << "'");
  throw std::runtime_error("Failed to retrieve parameter: " + name);
}

} // namespace vk

#endif // ROS_PARAMS_HELPER_H_