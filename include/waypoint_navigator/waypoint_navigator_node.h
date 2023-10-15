/*
 * Copyright (c) 2017, Marija Popovic, ASL, ETH Zurich, Switzerland
 * Copyright (c) 2017, Inkyu Sa, ASL, ETH Zurich, Switzerland
 * Copyright (c) 2016, Raghav Khanna, ASL, ETH Zurich, Switzerland
 * Copyright (c) 2015, Enric Galceran, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef WAYPOINT_NAVIGATOR_NODE_H
#define WAYPOINT_NAVIGATOR_NODE_H

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iostream>

#include <Eigen/Dense>
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

#include <rclcpp/rclcpp.hpp>

#include <mav_msgs/conversions.hpp>
#include <mav_msgs/default_topics.hpp>
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mav_planning_msgs/msg/polynomial_trajectory4_d.hpp>

#include <std_srvs/srv/empty.hpp>

#include "tf2/LinearMath/Quaternion.h"
// #include <geodetic_utils/geodetic_conv.hpp>

#include <waypoint_navigator/srv/execute_path_from_file.hpp>
#include <waypoint_navigator/srv/go_to_height.hpp>
#include <waypoint_navigator/srv/go_to_waypoint.hpp>
#include <waypoint_navigator/srv/go_to_waypoints.hpp>
#include <waypoint_navigator/srv/go_to_pose_waypoints.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <sensor_msgs/msg/nav_sat_fix.h>

namespace waypoint_navigator {
class WaypointNavigatorNode :public rclcpp::Node {
 public:
  WaypointNavigatorNode(const rclcpp::NodeOptions& options,
    const std::string& path_file, const std::string& robot_config_file);
  WaypointNavigatorNode(const WaypointNavigatorNode&) = delete;
  WaypointNavigatorNode& operator=(const WaypointNavigatorNode&) = delete;
  ~WaypointNavigatorNode() = default;

 private:
  //
  YAML::Node path;
  YAML::Node robot_config;
  // Fetches config
  void loadPath(const std::string&);
  // Read a series of waypoints from file
  // NB: Only call after odometry has been received, as
  // MAV position is set as first point.
  bool loadPathFromFile();

  // Interpolates intermediate points between waypoints in a sequence.
  void addIntermediateWaypoints();

  // Adds current MAV position as a waypoint to the path.
  void addCurrentOdometryWaypoint();

  // Creates and optimizes a smooth polynomial trajectory from a waypoint list.
  void createTrajectory();

  // Starts sending execution commands to the controller.
  void publishCommands();

  // Deletes old polynomial trajectory markers.
  void deletePolynomialMarkers();

  // Service callbacks.
  // Starts the execution of a loaded path.
  bool executePathCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                           std::shared_ptr<std_srvs::srv::Empty::Response> response);
  // Executes a new mission from .yaml file
  bool executePathFromFileCallback(
      const std::shared_ptr<waypoint_navigator::srv::ExecutePathFromFile::Request> request,
      std::shared_ptr<waypoint_navigator::srv::ExecutePathFromFile::Response> response);
  // Goes to a custom (x,y,z) waypoint.
  bool goToWaypointCallback(
      const std::shared_ptr<waypoint_navigator::srv::GoToWaypoint::Request> request,
      std::shared_ptr<waypoint_navigator::srv::GoToWaypoint::Response> response);
  // Goes to a custom (x,y,z) waypoint.
  void goToWaypointSubCallback(
      const std::shared_ptr<geometry_msgs::msg::PoseStamped> request);
  // Goes to a custom sequence of(x,y,z) waypoints.
  // Note: Does not add intermediate poses.
  bool goToWaypointsCallback(
      const std::shared_ptr<waypoint_navigator::srv::GoToWaypoints::Request> request,
      std::shared_ptr<waypoint_navigator::srv::GoToWaypoints::Response> response);

  // Goes to a custom sequence of Pose waypoints, but only yaw is used.
  // Note: Does not add intermediate poses.
  bool goToPoseWaypointsCallback(
      const std::shared_ptr<waypoint_navigator::srv::GoToPoseWaypoints::Request> request,
      std::shared_ptr<waypoint_navigator::srv::GoToPoseWaypoints::Response> response);


  // Goes to a specific height with current x-y position.
  bool goToHeightCallback(const std::shared_ptr<waypoint_navigator::srv::GoToHeight::Request> request,
                          std::shared_ptr<waypoint_navigator::srv::GoToHeight::Response> response);
  // Send a landing command.
  bool landCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                    std::shared_ptr<std_srvs::srv::Empty::Response> response);
  // Send a take-off command.
  bool takeoffCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                       std::shared_ptr<std_srvs::srv::Empty::Response> response);
  // Cancel mission and keep helicopter in current position
  bool abortPathCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                         std::shared_ptr<std_srvs::srv::Empty::Response> response);
  // Publish path rviz markers given most recent odometry measurement.
  bool visualizePathCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                             std::shared_ptr<std_srvs::srv::Empty::Response> response);
  // Publishes a single waypoint to go to if the path mode is 'poses' [5Hz].
  void poseTimerCallback();
  // Show path commands published in rviz
  void visualizationTimerCallback();

  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr odometry_message);

  static const double kCommandTimerFrequency;
  // Distance before a waypoint is considered reached [m].
  static const double kWaypointAchievementDistance;
  // Minimum distance between intermediate waypoints [m].
  static const double kIntermediatePoseTolerance;
  // Number of dimensions in the problem.
  static const int kDimensions;
  // Order of derivative to optimize polynomial trajectory for.
  static const int kDerivativeToOptimize;
  // Number of coefficients of polynomial trajectory.
  static const int kPolynomialCoefficients;

  // ROS comms.

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<mav_planning_msgs::msg::PolynomialTrajectory4D>::SharedPtr path_segments_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_points_marker_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_marker_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr polynomial_publisher_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_subscriber_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr visualize_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr takeoff_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr land_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr abort_path_service_;
  rclcpp::Service<waypoint_navigator::srv::ExecutePathFromFile>::SharedPtr new_path_service_;
  rclcpp::Service<waypoint_navigator::srv::GoToWaypoint>::SharedPtr waypoint_service_;
  rclcpp::Service<waypoint_navigator::srv::GoToWaypoints>::SharedPtr waypoints_service_;
  rclcpp::Service<waypoint_navigator::srv::GoToPoseWaypoints>::SharedPtr pose_waypoints_service_;
  rclcpp::Service<waypoint_navigator::srv::GoToHeight>::SharedPtr height_service_;

  rclcpp::TimerBase::SharedPtr command_timer_;
  rclcpp::TimerBase::SharedPtr visualization_timer_;

  // Parameters.
  // GPS/ENU coordinates.
  std::string coordinate_type_;
  // Trajectory/poses command publisher.
  std::string path_mode_;
  // Heading alignment method.
  std::string heading_mode_;
  std::string mav_name_;
  std::string frame_id_;
  // Addition of intermediate command poses.
  bool intermediate_poses_;
  // Maximum distance between poses [m].
  double intermediate_pose_separation_;
  // Maximum speed (m/s).
  double reference_speed_;
  // Maximum acceleration (m/s^2).
  double reference_acceleration_;
  // Height for takeoff command [m].
  double takeoff_height_;
  // Height for landing command [m].
  double landing_height_;

  // Geodetic coordinate conversion (from lat/lon to Cartesian ENU).
//   geodetic_converter::GeodeticConverter geodetic_converter_;

  bool got_odometry_;
  mav_msgs::EigenOdometry odometry_;

  // A list of waypoints to visit.
  // [x,y,z,heading]
  std::vector<mav_msgs::EigenTrajectoryPoint> coarse_waypoints_;

  // Polynomial trajectory markers.
  visualization_msgs::msg::MarkerArray markers_;

  // Path execution state (for pose publishing).
  size_t current_leg_;

  // Path vertices and segments.
  mav_trajectory_generation::Trajectory polynomial_trajectory_;
  mav_trajectory_generation::Vertex::Vector polynomial_vertices_;
  mav_trajectory_generation::Trajectory yaw_trajectory_;
  mav_trajectory_generation::Vertex::Vector yaw_vertices_;

  // Callback number for command_timer_.
  unsigned int timer_counter_;
};
}  // namespace: waypoint_navigator

#endif  // WAYPOINT_NAVIGATOR_NODE_H
