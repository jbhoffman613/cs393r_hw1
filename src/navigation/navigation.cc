//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    navigation.cc
\brief   Starter code for navigation.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"
#include <assert.h>

using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;

using namespace math_util;
using namespace ros_helpers;
using namespace std;

namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;
const float acceleration = 4.0;
const float deceleration = -4.0;
const int hertz = 20;
const float max_velocity = 1.0;

// constants in meters
const float MARGIN = 0.2;
const float WIDTH = 0.2 + MARGIN;
const float LENGTH = 0.5 + MARGIN;
const float WHEELBASE = 0.5;
const float TRACK = 0.2;
const float LATENCYVALUE = 0.1; // in seconds

} //namespace

namespace navigation {

Navigation::Navigation(const string& map_file, ros::NodeHandle* n) :
    odom_initialized_(false),
    localization_initialized_(false),
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    nav_complete_(true),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0),
    current_velocity_(0),
    current_curvature_(0.25),
    counter(0) {
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
  localization_initialized_ = true;
  robot_loc_ = loc;
  robot_angle_ = angle;
}

void Navigation::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel) {
  robot_omega_ = ang_vel;
  robot_vel_ = vel;
  if (!odom_initialized_) {
    odom_start_angle_ = angle;
    odom_start_loc_ = loc;
    odom_initialized_ = true;
    odom_loc_ = loc;
    odom_angle_ = angle;
    return;
  }
  odom_loc_ = loc;
  odom_angle_ = angle;
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
  point_cloud_ = cloud;                                     
}

void Navigation::Run() {
  // This function gets called 20 times a second to form the control loop.
  
  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) return;

  // The control iteration goes here. 
  // Feel free to make helper functions to structure the control appropriately.
  
  // The latest observed point cloud is accessible via "point_cloud_"

  // Eventually, you will have to set the control values to issue drive commands:
  // drive_msg_.curvature = ...;
  // drive_msg_.velocity = ...;

  LatencyCompensation();

  current_curvature_ = 0.25;
  float distance_to_goal = FreePathLength(current_curvature_);
  current_velocity_ = ComputeVelocity(current_velocity_, distance_to_goal);

  drive_msg_.velocity = current_velocity_;
  drive_msg_.curvature = current_curvature_;
  counter++;


  // Add timestamps to all messages.
  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();
  drive_msg_.header.stamp = ros::Time::now();
  // Publish messages.
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
  drive_pub_.publish(drive_msg_);
}

void Navigation::LatencyCompensation() {
  float x, y;

  // forward predict position
  float turning_radius = std::abs(1.0 / current_curvature_);
  float distance_traveled = current_velocity_ * LATENCYVALUE;
  float arc_length_radians = distance_traveled / turning_radius;
  float new_x = turning_radius * std::cos(arc_length_radians);
  float new_y = turning_radius * std::sin(arc_length_radians);

  // adjust each point in the point cloud based on forward predicted position
  for (unsigned int i = 0; i < point_cloud_.size(); i++) {
    x = point_cloud_[i](0);
    y = point_cloud_[i](1);
    point_cloud_[i] = Vector2f(x - new_x, y - new_y);
  }
}

double Navigation::PointFreePath(const Vector2f& point, const Vector2f& turning_center, float small_radius, float mid_radius, float large_radius, float turning_radius) {
  // Consumes a point and the three radii of turning. It computes the free path length that the car can turn 
  // without colliding with the point. 
  // NOTE: This method should only be used after having already checked that a collision will eventually occur. 
  float beta, theta, abs_y;

  float point_radius = std::sqrt(
    std::pow(turning_center(0) - point(0), 2) + 
    std::pow(turning_center(1) - point(1), 2));
  
  if (point_radius >= small_radius && point_radius <= mid_radius) {
    // Collides with the side 
    beta = std::acos(small_radius / point_radius);
  } else if (point_radius > mid_radius && point_radius <= large_radius) {
    // Collides with the front
    float dist_to_front = (LENGTH + WHEELBASE) / 2;
    beta = std::asin(dist_to_front / point_radius);
  } else {
    // won't collide
    assert(false && "You passed in a point that never collides with the car.");
    return 0; // keeps compiler from complaining that beta is uninitialized later
  }

  abs_y = std::abs(point(1)); // absolute value of y
  if (point(0) > 0) {
    if (abs_y > turning_radius) {
      theta = M_PI / 2.0 + std::acos(point(0) / point_radius);
    } else {
      theta = M_PI / 2.0 - std::acos(point(0) / point_radius);
    }
  } else {
    if (abs_y > turning_radius) {
      theta = 3 * M_PI / 2.0 - std::acos(std::abs(point(0)) / point_radius);
    } else {
      theta = 3 * M_PI / 2.0 + std::acos(std::abs(point(0)) / point_radius);
    }
  }
  return (theta - beta) * turning_radius;
}

bool Navigation::CollisionCheck(const Vector2f& point, const Vector2f& turning_center, float small_radius, float large_radius) {
  // Consumes a points and the radii of turning and returns if a collisison will occur 
  float point_radius = std::sqrt(
    std::pow(turning_center(0) - point(0), 2) + 
    std::pow(turning_center(1) - point(1), 2));
  
  return point_radius >= small_radius && point_radius <= large_radius;
  
}

double Navigation::FreePathLength(float proposed_curvature) {
  // todo 
  if (proposed_curvature == 0) {
    return 0;
  }

  // Find the turning radii necessary to compute collision
  float turning_radius = std::abs(1.0 / proposed_curvature);
  float small_radius = turning_radius - WIDTH / 2.0;
  float mid_radius = std::sqrt(std::pow((LENGTH + WHEELBASE) / 2.0, 2) + std::pow(small_radius, 2));
  float large_radius = std::sqrt(std::pow(turning_radius + WIDTH / 2.0, 2) + std::pow((LENGTH + WHEELBASE) / 2.0, 2));
  float turning_center_x = 0;
  float turning_center_y = 0;
  float global_min_free_path = 100001.0;

  // mid_radius += 0;

  // Computer the center turning points
  if (proposed_curvature > 0) {
    turning_center_y = turning_radius;
  } else {
    turning_center_y = -1 * turning_radius;
  }
  Vector2f turning_center(turning_center_x, turning_center_y);

  for (auto point : point_cloud_) {
    if (CollisionCheck(point, turning_center, small_radius, large_radius)) {
      float proposed_path_length = PointFreePath(point, turning_center, small_radius, mid_radius, large_radius, turning_radius);
      global_min_free_path = std::min(global_min_free_path, proposed_path_length);
    }
  }
  if (global_min_free_path >= 100000) { // if we won't hit any points
    // set path length as a full circle length 
    global_min_free_path = turning_radius * M_PI * 2;
  }
  return global_min_free_path;
}


float Navigation::ComputeVelocity(float current_velocity, float free_path) {
  // Computes the next step of our 1D controller based on the current 
  // velocity and the free path of proposed arc 
  float minimum_distance = (-1 * (current_velocity * current_velocity)) / (2 * deceleration);
  if (free_path <= minimum_distance) {
    return max(0.0, current_velocity + ((1.0 / hertz) * deceleration));
  } else if (current_velocity < max_velocity) {
    return current_velocity + ((1.0 / hertz) * acceleration);
  } else {
    // We are at the max velocity
    return current_velocity;
  }
}

}  // namespace navigation
