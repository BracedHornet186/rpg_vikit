/*
 * output_helper.h
 *
 * Created on: Jan 20, 2013
 * Author: chrigi
 * Converted to ROS 2
 */

#ifndef VIKIT_OUTPUT_HELPER_H_
#define VIKIT_OUTPUT_HELPER_H_

#include <string>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.h>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace vk {
namespace output_helper {

using std::string;
using Eigen::Vector3d;
using Eigen::Matrix3d;

void publishTfTransform(const Sophus::SE3& T, const rclcpp::Time& stamp,
                        const string& frame_id, const string& child_frame_id,
                        tf2_ros::TransformBroadcaster& br);

void publishPointMarker(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
                        const Vector3d& pos,
                        const string& ns,
                        const rclcpp::Time& timestamp,
                        int id,
                        int action,
                        double marker_scale,
                        const Vector3d& color,
                        rclcpp::Duration lifetime = rclcpp::Duration::from_seconds(0.0));

void publishLineMarker(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
                       const Vector3d& start,
                       const Vector3d& end,
                       const string& ns,
                       const rclcpp::Time& timestamp,
                       int id,
                       int action,
                       double marker_scale,
                       const Vector3d& color,
                       rclcpp::Duration lifetime = rclcpp::Duration::from_seconds(0.0));

void publishArrowMarker(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
                        const Vector3d& pos,
                        const Vector3d& dir,
                        double scale,
                        const string& ns,
                        const rclcpp::Time& timestamp,
                        int id,
                        int action,
                        double marker_scale,
                        const Vector3d& color);

void publishHexacopterMarker(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
                             const string& frame_id,
                             const string& ns,
                             const rclcpp::Time& timestamp,
                             int id,
                             int action,
                             double marker_scale,
                             const Vector3d& color);

void publishCameraMarker(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
                         const string& frame_id,
                         const string& ns,
                         const rclcpp::Time& timestamp,
                         int id,
                         double marker_scale,
                         const Vector3d& color);

void publishFrameMarker(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
                        const Matrix3d& rot,
                        const Vector3d& pos,
                        const string& ns,
                        const rclcpp::Time& timestamp,
                        int id,
                        int action,
                        double marker_scale,
                        rclcpp::Duration lifetime = rclcpp::Duration::from_seconds(0));

} // namespace output_helper
} // namespace vk

#endif // VIKIT_OUTPUT_HELPER_H_
