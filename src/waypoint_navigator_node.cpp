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

#include <waypoint_navigator/waypoint_navigator_node.h>

namespace waypoint_navigator {
//
using namespace std::placeholders;
using namespace std::chrono_literals;
//
const double WaypointNavigatorNode::kCommandTimerFrequency = 5.0;
const double WaypointNavigatorNode::kWaypointAchievementDistance = 0.5;
const double WaypointNavigatorNode::kIntermediatePoseTolerance = 0.1;
const int WaypointNavigatorNode::kDimensions = 3;
// const int WaypointNavigatorNode::kDerivativeToOptimize =
//     mav_trajectory_generation::derivative_order::ACCELERATION;
const int WaypointNavigatorNode::kPolynomialCoefficients = 10;

WaypointNavigatorNode::WaypointNavigatorNode(const rclcpp::NodeOptions& options,
  const std::string& path_file, const std::string& robot_config_file)
    : Node("waypoint_navigator_node", options),
      got_odometry_(false)
{
  //
  robot_config = YAML::LoadFile(robot_config_file);
  //
  loadPath(path_file);

  odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odometry", rclcpp::SensorDataQoS(),
      std::bind(&WaypointNavigatorNode::odometryCallback, this, _1));
  pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      mav_msgs::default_topics::COMMAND_POSE, rclcpp::SystemDefaultsQoS());
  path_segments_publisher_ =
      this->create_publisher<mav_planning_msgs::msg::PolynomialTrajectory4D>
      ("path_segments", rclcpp::SystemDefaultsQoS());

  // Visualization.
  path_points_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "waypoint_navigator_path_points_marker", rclcpp::SystemDefaultsQoS());
  path_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "waypoint_navigator_path_marker", rclcpp::SystemDefaultsQoS());
  polynomial_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "waypoint_navigator_polynomial_markers", rclcpp::SystemDefaultsQoS());

  visualize_service_ = this->create_service<std_srvs::srv::Empty>( "visualize_path",
    std::bind(&WaypointNavigatorNode::visualizePathCallback, this, _1, _2));
  start_service_ = this->create_service<std_srvs::srv::Empty>( "execute_path",
    std::bind(&WaypointNavigatorNode::executePathCallback, this, _1, _2));
  takeoff_service_ = this->create_service<std_srvs::srv::Empty>( "takeoff",
    std::bind(&WaypointNavigatorNode::takeoffCallback, this, _1, _2));
  land_service_ = this->create_service<std_srvs::srv::Empty>( "land",
    std::bind(&WaypointNavigatorNode::landCallback, this, _1, _2));
  abort_path_service_ = this->create_service<std_srvs::srv::Empty>( "abort_path",
    std::bind(&WaypointNavigatorNode::abortPathCallback, this, _1, _2));
  new_path_service_ = this->create_service<waypoint_navigator::srv::ExecutePathFromFile>(
      "execute_path_from_file",
      std::bind(&WaypointNavigatorNode::executePathFromFileCallback, this, _1, _2));
  waypoint_service_ = this->create_service<waypoint_navigator::srv::GoToWaypoint>(
      "go_to_waypoint", std::bind(&WaypointNavigatorNode::goToWaypointCallback, this, _1, _2));
  waypoints_service_ = this->create_service<waypoint_navigator::srv::GoToWaypoints>(
      "go_to_waypoints", std::bind(&WaypointNavigatorNode::goToWaypointsCallback, this, _1, _2));
  pose_waypoints_service_ = this->create_service<waypoint_navigator::srv::GoToPoseWaypoints>(
      "go_to_pose_waypoints", std::bind(&WaypointNavigatorNode::goToPoseWaypointsCallback, this, _1, _2));
  height_service_ = this->create_service<waypoint_navigator::srv::GoToHeight>(
      "go_to_height", std::bind(&WaypointNavigatorNode::goToHeightCallback, this, _1, _2));

    command_timer_ = this->create_wall_timer
      (std::chrono::duration<float>(1.0 / kCommandTimerFrequency),
      std::bind(&WaypointNavigatorNode::poseTimerCallback, this));
    command_timer_->cancel();

  // Wait until GPS reference parameters are initialized.
  // while (!geodetic_converter_.isInitialised() && coordinate_type_ == "gps") {
  //   LOG_FIRST_N(INFO, 1) << "Waiting for GPS reference parameters...";

  //   double latitude = path["gps_ref_latitude"].as<double>();
  //   double longitude = path["gps_ref_longitude"].as<double>();
  //   double altitude = path["gps_ref_altitude"].as<double>();

  //  geodetic_converter_.initialiseReference(latitude, longitude, altitude);
  //  std::this_thread::sleep_for(500ms);
  // }

  LOG(INFO)
      << "Waypoint navigator ready. Call 'execute_path' service to get going.";
}

void WaypointNavigatorNode::loadPath(const std::string& path_file) {
  //
  RCLCPP_INFO(this->get_logger(), ": Loading path:[%s]", path_file.c_str());
  path = YAML::LoadFile(path_file);
  //
  coordinate_type_ = path["coordinate_type"].as<std::string>();
  path_mode_ = path["path_mode"].as<std::string>();
  heading_mode_ = path["heading_mode"].as<std::string>();
  reference_speed_ = path["reference_speed"].as<double>();
  reference_acceleration_ = path["reference_acceleration"].as<double>();
  takeoff_height_ = path["takeoff_height"].as<double>();
  landing_height_ = path["landing_height"].as<double>();
  // mav_name_ = robot_config["mav_name"].as<std::string>();
  frame_id_ = robot_config["world_frame"].as<std::string>();
  intermediate_poses_ = path["intermediate_poses"].as<bool>();
  
  if (coordinate_type_ == "gps" || coordinate_type_ == "enu") {
  } else {
    LOG(FATAL) << ("Unknown coordinate type - please enter 'gps' or 'enu'.");
  }

  assert(path_mode_ != "polynomial" && " Polynomial mode not supported.");
  if (path_mode_ == "poses" || path_mode_ == "polynomial") {
  } else {
    LOG(FATAL) << "Unknown path type - please enter 'poses', or 'trajectory'.";
  }

  if (heading_mode_ == "auto" || heading_mode_ == "manual" ||
      heading_mode_ == "zero") {
  } else {
    LOG(FATAL) << "Unknown heading alignment mode - please enter 'auto', "
                  "'manual', or 'zero'.";
  }

  if (intermediate_poses_) {
    intermediate_pose_separation_ = path["intermediate_pose_separation"].as<double>();
    LOG(INFO) << "intermediate_pose_separation:" << intermediate_pose_separation_;
  }
}

bool WaypointNavigatorNode::loadPathFromFile() {
  // Fetch the trajectory from the parameter server.
  std::vector<double> easting;
  std::vector<double> northing;
  std::vector<double> height;
  std::vector<double> heading;
  //
  easting = path["easting"].as<std::vector<double>>();
  northing = path["northing"].as<std::vector<double>>();
  height = path["height"].as<std::vector<double>>();

  if (heading_mode_ == "manual"){
    heading = path["heading"].as<std::vector<double>>();
  }

  // Check for valid trajectory inputs.
  if (!(easting.size() == northing.size() &&
        northing.size() == height.size())) {
    LOG(FATAL) << "Error: path parameter arrays are not the same size";
  }
  if (heading_mode_ == "manual" && !(height.size() == heading.size())) {
    LOG(FATAL) << "Error: path parameter arrays are not the same size";
  }

  coarse_waypoints_.clear();
  addCurrentOdometryWaypoint();

  // Add (x,y,z) co-ordinates from file to path.
  for (size_t i = 0; i < easting.size(); i++) {
    mav_msgs::EigenTrajectoryPoint cwp;
    // GPS path co-ordinates.
    // if (coordinate_type_ == "gps") {
    //   double initial_latitude;
    //   double initial_longitude;
    //   double initial_altitude;

    //   // Convert GPS point to ENU co-ordinates.
    //   // NB: waypoint altitude = desired height above reference + registered
    //   // reference altitude.
    //   geodetic_converter_.getReference(&initial_latitude, &initial_longitude,
    //                                    &initial_altitude);
    //   geodetic_converter_.geodetic2Enu(
    //       northing[i], easting[i], (initial_altitude + height[i]),
    //       &cwp.position_W.x(), &cwp.position_W.y(), &cwp.position_W.z());
    // }
    // ENU path co-ordinates.
    if (coordinate_type_ == "enu") {
      cwp.position_W.x() = easting[i];
      cwp.position_W.y() = northing[i];
      cwp.position_W.z() = height[i];
    }
    coarse_waypoints_.push_back(cwp);
  }

  // Add heading from file to path.
  for (size_t i = 1; i < coarse_waypoints_.size(); i++) {
    if (heading_mode_ == "manual") {
      coarse_waypoints_[i].setFromYaw(heading[i] * (M_PI / 180.0));
    } else if (heading_mode_ == "auto") {
      // Compute heading in direction towards next point.
      coarse_waypoints_[i].setFromYaw(
          atan2(coarse_waypoints_[i].position_W.y() -
                    coarse_waypoints_[i - 1].position_W.y(),
                coarse_waypoints_[i].position_W.x() -
                    coarse_waypoints_[i - 1].position_W.x()));
    } else if (heading_mode_ == "zero") {
      coarse_waypoints_[i].setFromYaw(0.0);
    }
  }

  // As first target point, add current (x,y) position, but with height at
  // that of the first requested waypoint, so that the MAV first adjusts height
  // moving only vertically.
  if (coarse_waypoints_.size() >= 2) {
    mav_msgs::EigenTrajectoryPoint vwp;
    vwp.position_W.x() = odometry_.position_W.x();
    vwp.position_W.y() = odometry_.position_W.y();
    vwp.position_W.z() = coarse_waypoints_[1].position_W.z();
    if (heading_mode_ == "zero") {
      vwp.setFromYaw(0.0);
    } else if (heading_mode_ == "manual") {
      // Do not change heading.
      vwp.orientation_W_B = coarse_waypoints_[0].orientation_W_B;
    }
    coarse_waypoints_.insert(coarse_waypoints_.begin() + 1, vwp);
  }

  // Limit maximum distance between waypoints.
  if (intermediate_poses_) {
    addIntermediateWaypoints();
  }

  LOG(INFO) << "Path loaded from file. Number of points in path: "
            << coarse_waypoints_.size();

  current_leg_ = 0;
  return true;
}

void WaypointNavigatorNode::addIntermediateWaypoints() {
  for (size_t i = 1; i < coarse_waypoints_.size(); ++i) {
    mav_msgs::EigenTrajectoryPoint wpa = coarse_waypoints_[i - 1];
    mav_msgs::EigenTrajectoryPoint wpb = coarse_waypoints_[i];
    double dist = (wpa.position_W - wpb.position_W).norm();

    // Minimum tolerance between points set to avoid subsequent numerical errors
    // in trajectory optimization.
    while (dist > intermediate_pose_separation_ &&
           dist > kIntermediatePoseTolerance) {
      mav_msgs::EigenTrajectoryPoint iwp;
      iwp.position_W.x() = wpa.position_W.x() +
                           (intermediate_pose_separation_ / dist) *
                               (wpb.position_W.x() - wpa.position_W.x());
      iwp.position_W.y() = wpa.position_W.y() +
                           (intermediate_pose_separation_ / dist) *
                               (wpb.position_W.y() - wpa.position_W.y());
      iwp.position_W.z() = wpa.position_W.z() +
                           (intermediate_pose_separation_ / dist) *
                               (wpb.position_W.z() - wpa.position_W.z());
      iwp.orientation_W_B = wpb.orientation_W_B;
      coarse_waypoints_.insert(coarse_waypoints_.begin() + i, iwp);
      wpa = iwp;
      dist = (wpa.position_W - wpb.position_W).norm();
      i++;
    }
  }
}

void WaypointNavigatorNode::addCurrentOdometryWaypoint() {
  mav_msgs::EigenTrajectoryPoint vwp;
  vwp.position_W = odometry_.position_W;
  vwp.orientation_W_B = odometry_.orientation_W_B;
  coarse_waypoints_.push_back(vwp);
}

void WaypointNavigatorNode::createTrajectory() {
  // polynomial_vertices_.clear();
  // polynomial_trajectory_.clear();
  // yaw_vertices_.clear();
  // yaw_trajectory_.clear();
  // deletePolynomialMarkers();

  // // Create a list of vertices.
  // for (size_t i = 0; i < coarse_waypoints_.size(); i++) {
  //   mav_trajectory_generation::Vertex vertex(kDimensions);
  //   mav_trajectory_generation::Vertex yaw(1);

  //   // Position.
  //   if (i == 0 || i == coarse_waypoints_.size() - 1) {
  //     vertex.makeStartOrEnd(coarse_waypoints_[i].position_W,
  //                           mav_trajectory_generation::derivative_order::SNAP);
  //   } else {
  //     vertex.addConstraint(
  //         mav_trajectory_generation::derivative_order::POSITION,
  //         coarse_waypoints_[i].position_W);
  //   }
  //   // Yaw.
  //   if (i != 0) {
  //     // Check whether to rotate clockwise or counter-clockwise in yaw.
  //     double yaw_mod = fmod(
  //         coarse_waypoints_[i].getYaw() - coarse_waypoints_[i - 1].getYaw(),
  //         2 * M_PI);
  //     if (yaw_mod < -M_PI) {
  //       yaw_mod += 2 * M_PI;
  //     } else if (yaw_mod > M_PI) {
  //       yaw_mod -= 2 * M_PI;
  //     }
  //     coarse_waypoints_[i].setFromYaw(coarse_waypoints_[i - 1].getYaw() +
  //                                     yaw_mod);
  //   }
  //   yaw.addConstraint(mav_trajectory_generation::derivative_order::ORIENTATION,
  //                     coarse_waypoints_[i].getYaw());

  //   polynomial_vertices_.push_back(vertex);
  //   yaw_vertices_.push_back(yaw);
  // }

  // // Optimize the polynomial trajectory.
  // // Position.
  // std::vector<double> segment_times;
  // segment_times =
  //     estimateSegmentTimes(polynomial_vertices_, reference_speed_,
  //                          reference_acceleration_);

  // mav_trajectory_generation::PolynomialOptimization<kPolynomialCoefficients>
  //     opt(kDimensions);
  // opt.setupFromVertices(polynomial_vertices_, segment_times,
  //                       kDerivativeToOptimize);
  // opt.solveLinear();
  // opt.getTrajectory(&polynomial_trajectory_);
  // // Yaw.
  // mav_trajectory_generation::PolynomialOptimization<kPolynomialCoefficients>
  //     yaw_opt(1);
  // yaw_opt.setupFromVertices(yaw_vertices_, segment_times,
  //                           kDerivativeToOptimize);
  // yaw_opt.solveLinear();
  // yaw_opt.getTrajectory(&yaw_trajectory_);
}

void WaypointNavigatorNode::publishCommands() {
  if (path_mode_ == "poses") {
    command_timer_->reset();
  } else if (path_mode_ == "polynomial") {
    // createTrajectory();
    // // Publish the trajectory directly to the trajectory sampler.
    // mav_planning_msgs::msg::PolynomialTrajectory4D msg;
    // mav_trajectory_generation::Trajectory traj_with_yaw;
    // polynomial_trajectory_.getTrajectoryWithAppendedDimension(yaw_trajectory_,
    //                                                           &traj_with_yaw);
    // mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(
    //     traj_with_yaw, &msg);
    // path_segments_publisher_->publish(msg);
  }
}

void WaypointNavigatorNode::deletePolynomialMarkers() {
  for (size_t i = 0; i < markers_.markers.size(); ++i) {
    markers_.markers[i].action = visualization_msgs::msg::Marker::DELETE;
  }
  polynomial_publisher_->publish(markers_);
}

bool WaypointNavigatorNode::executePathCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response) {
  CHECK(got_odometry_)
      << "No odometry received yet, can't start path following.";
  RCLCPP_INFO(this->get_logger(), ": Received request for execute path.");
  command_timer_->cancel();
  current_leg_ = 0;
  timer_counter_ = 0;

  CHECK(loadPathFromFile()) << "Path could not be loaded!";

  // Display the path markers in rviz.
  auto empty_request = std::make_shared<std_srvs::srv::Empty::Request>();
  auto empty_response = std::make_shared<std_srvs::srv::Empty::Response>();

  visualizePathCallback(empty_request, empty_response);

  publishCommands();
  LOG(INFO) << "Starting path execution...";
  return true;
}

bool WaypointNavigatorNode::executePathFromFileCallback(
    const std::shared_ptr<waypoint_navigator::srv::ExecutePathFromFile::Request> request,
    std::shared_ptr<waypoint_navigator::srv::ExecutePathFromFile::Response> response) {
  // Stop executing the current path.
  auto empty_request = std::make_shared<std_srvs::srv::Empty::Request>();
  auto empty_response = std::make_shared<std_srvs::srv::Empty::Response>();
  //
  abortPathCallback(empty_request, empty_response);
  // Change: Absolute path of file needs to be given  
  std::ifstream filename(request->filename.data.c_str());
  if(!filename.good());
  executePathCallback(empty_request, empty_response);
  return true;
}

bool WaypointNavigatorNode::goToWaypointCallback(
    const std::shared_ptr<waypoint_navigator::srv::GoToWaypoint::Request> request,
    std::shared_ptr<waypoint_navigator::srv::GoToWaypoint::Response> response) {
  coarse_waypoints_.clear();
  current_leg_ = 0;
  timer_counter_ = 0;
  command_timer_->cancel();

  addCurrentOdometryWaypoint();

  // Add the new waypoint.
  mav_msgs::EigenTrajectoryPoint vwp;
  vwp.position_W.x() = request->point.x;
  vwp.position_W.y() = request->point.y;
  vwp.position_W.z() = request->point.z;
  if (heading_mode_ == "zero") {
    vwp.setFromYaw(0.0);
  } else if (sqrt(pow(request->point.y - odometry_.position_W.y(), 2) +
                  pow(request->point.x - odometry_.position_W.x(), 2)) < 0.05) {
    vwp.orientation_W_B = odometry_.orientation_W_B;
  } else {
    vwp.setFromYaw(atan2(request->point.y - odometry_.position_W.y(),
                        request->point.x - odometry_.position_W.x()));
  }
  coarse_waypoints_.push_back(vwp);

  // Limit the maximum distance between waypoints.
  if (intermediate_poses_) {
    addIntermediateWaypoints();
  }

  publishCommands();
  LOG(INFO) << "Going to a new waypoint...";
  return true;
}

bool WaypointNavigatorNode::goToWaypointsCallback(
    const std::shared_ptr<waypoint_navigator::srv::GoToWaypoints::Request> request,
    std::shared_ptr<waypoint_navigator::srv::GoToWaypoints::Response> response) {
  coarse_waypoints_.clear();
  current_leg_ = 0;
  timer_counter_ = 0;
  command_timer_->cancel();

  addCurrentOdometryWaypoint();

  // Add points to a new path.
  std::vector<geometry_msgs::msg::Point> points = request->points;
  mav_msgs::EigenTrajectoryPoint vwp;
  for (size_t i = 0; i < points.size(); ++i) {
    vwp.position_W.x() = points[i].x;
    vwp.position_W.y() = points[i].y;
    vwp.position_W.z() = points[i].z;
    coarse_waypoints_.push_back(vwp);
  }

  // Add heading to path.
  for (size_t i = 0; i < coarse_waypoints_.size(); i++) {
    if (heading_mode_ == "auto") {
      if (i == 0) {
        continue;
      }
      // Compute heading in direction towards next point.
      coarse_waypoints_[i].setFromYaw(
          atan2(coarse_waypoints_[i].position_W.y() -
                    coarse_waypoints_[i - 1].position_W.y(),
                coarse_waypoints_[i].position_W.x() -
                    coarse_waypoints_[i - 1].position_W.x()));
    }
    // For both 'manual' and 'zero' heading modes, set zero heading.
    else {
      coarse_waypoints_[i].setFromYaw(0.0);
    }
  }
  // Display the path markers in rviz.
  visualization_timer_ = this->create_wall_timer(100ms,
    std::bind(&WaypointNavigatorNode::visualizationTimerCallback, this));
  publishCommands();
  return true;
}

bool WaypointNavigatorNode::goToPoseWaypointsCallback(
    const std::shared_ptr<waypoint_navigator::srv::GoToPoseWaypoints::Request> request,
    std::shared_ptr<waypoint_navigator::srv::GoToPoseWaypoints::Response> response) {
  coarse_waypoints_.clear();
  current_leg_ = 0;
  timer_counter_ = 0;
  command_timer_->cancel();

  // Add points to a new path.
  std::vector<geometry_msgs::msg::Pose> waypoints = request->waypoints;
  mav_msgs::EigenTrajectoryPoint vwp;
  for (size_t i = 0; i < waypoints.size(); ++i) {
    vwp.position_W.x() = waypoints[i].position.x;
    vwp.position_W.y() = waypoints[i].position.y;
    vwp.position_W.z() = waypoints[i].position.z;

    vwp.orientation_W_B.x() = waypoints[i].orientation.x;
    vwp.orientation_W_B.y() = waypoints[i].orientation.y;
    vwp.orientation_W_B.z() = waypoints[i].orientation.z;
    vwp.orientation_W_B.w() = waypoints[i].orientation.w;

    if (i==0)
    {
      const double dist_to_end =
        (vwp.position_W - odometry_.position_W).norm();
          
      if (dist_to_end > kWaypointAchievementDistance) {
        LOG(INFO) << "Extra waypoint added because current pose is too far (" << dist_to_end << "m) from the first waypoint.";
        addCurrentOdometryWaypoint();
      }
    }

    coarse_waypoints_.push_back(vwp);
  }

  if(coarse_waypoints_.size() > 1)
  {
    LOG(INFO) << coarse_waypoints_.size()<<" waypoints received.";
    // Display the path markers in rviz.
    visualization_timer_ = this->create_wall_timer(100ms,
      std::bind(&WaypointNavigatorNode::visualizationTimerCallback, this));
    publishCommands();
  }else{
    LOG(INFO) << " Nothing to do because the destination is too close.";
    return false;
  }

  return true;
}

bool WaypointNavigatorNode::goToHeightCallback(
  const std::shared_ptr<waypoint_navigator::srv::GoToHeight::Request> request,
  std::shared_ptr<waypoint_navigator::srv::GoToHeight::Response> response) {
  auto vwp = std::make_shared<waypoint_navigator::srv::GoToWaypoint::Request>();
  auto empty_response = std::make_shared<waypoint_navigator::srv::GoToWaypoint::Response>();
  vwp->point.x = odometry_.position_W.x();
  vwp->point.y = odometry_.position_W.y();
  vwp->point.z = request->height.data;
  return goToWaypointCallback(vwp, empty_response);
}

bool WaypointNavigatorNode::landCallback(
  const std::shared_ptr<std_srvs::srv::Empty::Request> request,
  std::shared_ptr<std_srvs::srv::Empty::Response> response) {
  auto vwp = std::make_shared<waypoint_navigator::srv::GoToWaypoint::Request>();
  auto  empty_response = std::make_shared<waypoint_navigator::srv::GoToWaypoint::Response>();

  vwp->point.x = odometry_.position_W.x();
  vwp->point.y = odometry_.position_W.y();
  vwp->point.z = landing_height_;
  return goToWaypointCallback(vwp, empty_response);
}

bool WaypointNavigatorNode::takeoffCallback(
  const std::shared_ptr<std_srvs::srv::Empty::Request> request,
  std::shared_ptr<std_srvs::srv::Empty::Response> response) {
  auto vwp = std::make_shared<waypoint_navigator::srv::GoToWaypoint::Request>();
  auto  empty_response = std::make_shared<waypoint_navigator::srv::GoToWaypoint::Response>();

  vwp->point.x = odometry_.position_W.x();
  vwp->point.y = odometry_.position_W.y();
  vwp->point.z = odometry_.position_W.z() + takeoff_height_;
  return goToWaypointCallback(vwp, empty_response);
}

bool WaypointNavigatorNode::abortPathCallback(
  const std::shared_ptr<std_srvs::srv::Empty::Request> request,
  std::shared_ptr<std_srvs::srv::Empty::Response> response) {
  coarse_waypoints_.clear();
  // polynomial_trajectory_.clear();
  // polynomial_vertices_.clear();
  // yaw_trajectory_.clear();
  // yaw_vertices_.clear();
  visualization_timer_->cancel();

  // Stop sending commands to the controller.
  if (path_mode_ == "polynomial") {
    auto empty_request = std::make_shared<std_srvs::srv::Empty::Request>();
    auto client = this->create_client<std_srvs::srv::Empty>(
        "/" + mav_name_ + "/stop_trajectory_sampling");
    client->async_send_request(empty_request);
  } else {
    command_timer_->cancel();
  }
  LOG(INFO) << "Aborting path execution...";
  return true;
}

bool WaypointNavigatorNode::visualizePathCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response) {
  CHECK(got_odometry_) << "No odometry received yet, can't visualize the path.";
  CHECK(loadPathFromFile()) << "Path could not be loaded!";

  // if (path_mode_ == "polynomial") {
  //   createTrajectory();
  // }

  visualization_timer_ = this->create_wall_timer(100ms,
    std::bind(&WaypointNavigatorNode::visualizationTimerCallback, this));
  return true;
}

void WaypointNavigatorNode::poseTimerCallback() {
  // Check for leg completion based on distance.
  // If current leg has been completed, go to the next one.
  const double dist_to_end =
      (coarse_waypoints_[current_leg_].position_W - odometry_.position_W)
          .norm();

  if (current_leg_ != coarse_waypoints_.size() - 1 &&
      dist_to_end < kWaypointAchievementDistance) {
    if (current_leg_ == 0) {
      LOG(INFO) << "Going to first waypoint... ";
    } else {
      LOG(INFO) << "Leg " << current_leg_ << " of "
                << coarse_waypoints_.size() - 1 << " completed!";
    }
    current_leg_++;
  }

  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = this->get_clock()->now();
  pose.pose.position.x = coarse_waypoints_[current_leg_].position_W.x();
  pose.pose.position.y = coarse_waypoints_[current_leg_].position_W.y();
  pose.pose.position.z = coarse_waypoints_[current_leg_].position_W.z();
  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, coarse_waypoints_[current_leg_].getYaw());
  pose.pose.orientation.x = orientation.x();
  pose.pose.orientation.y = orientation.y();
  pose.pose.orientation.z = orientation.z();
  pose.pose.orientation.w = orientation.w();

  pose_publisher_->publish(pose);
  timer_counter_++;
}

void WaypointNavigatorNode::visualizationTimerCallback() {
  // Fill out markers.
  visualization_msgs::msg::Marker path_points_marker;
  path_points_marker.header.frame_id = frame_id_;
  path_points_marker.header.stamp = this->get_clock()->now();
  path_points_marker.ns = "waypoints";
  path_points_marker.id = 0;
  path_points_marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
  path_points_marker.action = visualization_msgs::msg::Marker::ADD;
  path_points_marker.pose.position.x = 0.0;
  path_points_marker.pose.position.y = 0.0;
  path_points_marker.pose.position.z = 0.0;
  path_points_marker.pose.orientation.x = 0.0;
  path_points_marker.pose.orientation.y = 0.0;
  path_points_marker.pose.orientation.z = 0.0;
  path_points_marker.pose.orientation.w = 1.0;
  path_points_marker.scale.x = 0.1;
  path_points_marker.scale.y = 0.1;
  path_points_marker.scale.z = 0.1;
  path_points_marker.color.a = 1.0;
  path_points_marker.color.r = 1.0;
  path_points_marker.color.g = 0.0;
  path_points_marker.color.b = 0.0;

  visualization_msgs::msg::Marker path_marker;
  path_marker.header.frame_id = frame_id_;
  path_marker.header.stamp = this->get_clock()->now();
  path_marker.ns = "path";
  path_marker.id = 0;
  path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  path_marker.action = visualization_msgs::msg::Marker::ADD;
  path_marker.pose.position.x = 0.0;
  path_marker.pose.position.y = 0.0;
  path_marker.pose.position.z = 0.0;
  path_marker.pose.orientation.x = 0.0;
  path_marker.pose.orientation.y = 0.0;
  path_marker.pose.orientation.z = 0.0;
  path_marker.pose.orientation.w = 1.0;
  path_marker.scale.x = 0.02;
  path_marker.color.a = 1.0;
  path_marker.color.r = 1.0;
  path_marker.color.g = 1.0;
  path_marker.color.b = 1.0;

  path_points_marker.points.clear();
  path_marker.points.clear();
  for (size_t i = 0; i < coarse_waypoints_.size(); ++i) {
    if (coarse_waypoints_.empty()) {
      continue;
    }
    geometry_msgs::msg::Point pt;
    pt.x = coarse_waypoints_[i].position_W.x();
    pt.y = coarse_waypoints_[i].position_W.y();
    pt.z = coarse_waypoints_[i].position_W.z();
    path_points_marker.points.push_back(pt);
    path_marker.points.push_back(pt);
  }

  path_points_marker_publisher_->publish(path_points_marker);
  path_marker_publisher_->publish(path_marker);

  // if (path_mode_ == "polynomial") {
  //   mav_trajectory_generation::Trajectory traj_with_yaw;
  //   polynomial_trajectory_.getTrajectoryWithAppendedDimension(yaw_trajectory_,
  //                                                             &traj_with_yaw);
  //   mav_trajectory_generation::drawMavTrajectory(traj_with_yaw, 1.0, frame_id_,
  //                                                &markers_);
  //   polynomial_publisher_->publish(markers_);
  // }
}

void WaypointNavigatorNode::odometryCallback(
    const nav_msgs::msg::Odometry::SharedPtr odometry_message) {
  if (!got_odometry_) {
    got_odometry_ = true;
  }
  mav_msgs::eigenOdometryFromMsg(*odometry_message, &odometry_);
}
}

int main(int argc, char** argv) {
  // Start the logging.
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = 1;
  //
  if(argc < 2)
  {
      LOG(INFO) << "No config file provided.";
  }
  else
  {

      char* path_config = argv[1];
      char* robot_config = argv[2];
      // Check if config file exists  
      if(path_config != NULL && robot_config != NULL)
      {
        LOG(INFO) << "Path config file provided:" << path_config;
        LOG(INFO) << "Robot config file provided:" << robot_config;
        rclcpp::init(argc, argv);
        rclcpp::NodeOptions options;
        auto wp_nav_node = std::make_shared<waypoint_navigator::WaypointNavigatorNode>
          (options, path_config, robot_config);
        rclcpp::spin(wp_nav_node);
      } 
      else
      {
          if(path_config == NULL)
              LOG(INFO) << "File: not found:" << path_config;
          if(robot_config == NULL)
              LOG(INFO) << "File: not found:" << robot_config;
      }
  }
  rclcpp::shutdown();
  return 0;
}
