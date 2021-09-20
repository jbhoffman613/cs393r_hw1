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
const float ACCELERATION = 4.0;
const float DECELERATION = -4.0;
const int HERTZ = 20;
const float MAX_VELOCITY = 1.0;

// constants in meters
const float FRONT_MARGIN = 0.4;
const float SIDE_MARGIN = 0.08;
const float WIDTH = 0.28 + SIDE_MARGIN; // 11 in - side to side of car
const float LENGTH = 0.51 + FRONT_MARGIN; // 20 - total front to back of car 
const float WHEELBASE = 0.33; // 13 in - back axel to front axel 
const float TRACK = 0.22; // 9 in - in between the wheels
const float SYSTEM_LATENCY = 1.0; // in seconds - TODO
const int CURVATURES = 15;

const float FLRR_DURATION = 2; // duration of J-turn segments
const float J_TURN_VELO = 0.5;
const float J_TURN_CURV = 2;

// actuation latency = system latency * 0.75
// const unsigned int QUEUE_LEN = ceil(SYSTEM_LATENCY*0.75 * HERTZ);
const unsigned int QUEUE_LEN = 1;

const float INITIAL_VELOCITY = 0;
const float INITIAL_CURVATURE = 0.0;

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
    state_(State::AUTO),
    j_turn_timer_(FLRR_DURATION * HERTZ) {
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);
  current_control_ = {INITIAL_VELOCITY, INITIAL_CURVATURE}; // initialize current velocity and curvature to 0
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

  if (!odom_initialized_) return;

  std::cout << "State: " << state_ << std::endl;

  if (state_ == State::AUTO) {
    RunAutonomous();
  } else if (state_ == State::FL) {
    RunRobot(0.5, -2);
  } else if (state_ == State::RR) {
    RunRobot(-0.5, 2);
  } else if (state_ == State::SANDBOX) {
      current_control_.velocity = -0.5;
      current_control_.curvature = 2;
      j_turn_timer_--;

      if (j_turn_timer_ <= 0) {
        current_control_.velocity = 0;
        current_control_.curvature = 0;
      }
  }

  PublishControl();
  PublishVis();

}

void Navigation::RunAutonomous() {  
  LatencyCompensation();

  vector<float> proposed_curvatures = ProposeCurvatures();

  auto ret = PickCurve(proposed_curvatures);
  PathOption chosen_path = ret.first;
  float max_free_path = ret.second;

  current_control_.velocity = ComputeVelocity(current_control_.velocity, chosen_path.free_path_length);
  current_control_.curvature = chosen_path.curvature;

  past_controls_.push(current_control_);

  cout << "CHOSE " << current_control_.curvature << endl << endl;

  if (max_free_path < 0.8) {
    state_ = State::RR;
    j_turn_timer_ = FLRR_DURATION * HERTZ;
  }
}

void Navigation::RunRobot(float velocity, float curvature) {
  if (GoStraightFreePath() > 2) {
    state_ = State::AUTO;
    return;
  }
  
  // Do Small Forward Left Turn
  current_control_.velocity = velocity;
  current_control_.curvature = curvature;

  // if (state_ == State::FL && FreePathLength(curvature) < 0.05) {
  //   state_ = State::RR;
  //   j_turn_timer_ = FLRR_DURATION * HERTZ;
  //   return;
  // }

  if (j_turn_timer_ <= 0) {
    state_ =  (state_ == State::FL) ? State::RR : State::FL;
    j_turn_timer_ = FLRR_DURATION * HERTZ;
  } else {
    j_turn_timer_--;
  }
}

void Navigation::PublishControl() {
  drive_msg_.curvature = current_control_.curvature;
  drive_msg_.velocity = current_control_.velocity;
  drive_msg_.header.stamp = ros::Time::now();
  drive_pub_.publish(drive_msg_);
}

void Navigation::PublishVis() {
  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();

  // draw the forward predicted point cloud in simulation
  // for (auto point : point_cloud_) {
  //   visualization::DrawPoint(point, 0x5eeb34, local_viz_msg_);
  // }

  // draw a box around where the car is
  // left side
  visualization::DrawLine(Vector2f( -1 * (LENGTH - WIDTH) / 2, WIDTH / 2 ),
                          Vector2f( (LENGTH + WIDTH) / 2, WIDTH / 2 ),
                          0xff000d,
                          local_viz_msg_);
  // back
  visualization::DrawLine(Vector2f( -1 * (LENGTH - WIDTH) / 2, WIDTH / 2 ),
                          Vector2f( -1 * (LENGTH - WIDTH) / 2, -1 * WIDTH / 2 ),
                          0xff000d,
                          local_viz_msg_);
  // right side
  visualization::DrawLine(Vector2f( -1 * (LENGTH - WIDTH) / 2, -1 * WIDTH / 2 ),
                          Vector2f( (LENGTH + WIDTH) / 2, -1 * WIDTH / 2 ),
                          0xff000d,
                          local_viz_msg_);
  // front
  visualization::DrawLine(Vector2f( (LENGTH + WIDTH) / 2, -1 * WIDTH / 2 ),
                          Vector2f( (LENGTH + WIDTH) / 2, WIDTH / 2 ),
                          0xff000d,
                          local_viz_msg_);

  // // straight line showing where we will go (if we are going in a straight path)
  // visualization::DrawLine(Vector2f( (LENGTH / 2) + (WIDTH / 2), 0 ),
  //                         Vector2f( (LENGTH / 2) + (WIDTH / 2) + chosen_path.free_path_length, 0),
  //                         0x0400ff,
  //                         local_viz_msg_);

  // Publish messages.
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
}

vector<float> Navigation::ProposeCurvatures() {
  vector<float> proposed_curvatures;
  float curvature_max = 1.2;
  // float curve_delta = 2.0 / (CURVATURES - 1.0);
  float curve_delta = (curvature_max*2) /  (CURVATURES - 1.0);
  for (int i = 0; i < CURVATURES; i++) {
    if (i == CURVATURES / 2) {
      proposed_curvatures.push_back(0);
    } else {
      proposed_curvatures.push_back(-1.0*curvature_max + i*curve_delta);
    }
  }
  return proposed_curvatures;
}

std::pair<PathOption, float> Navigation::PickCurve(vector<float> proposed_curves) {
  // using proposed_curves, create a vector of PathOptions
  vector<PathOption> path_options;
  float max_free_path = 0.0;

  for (auto curve : proposed_curves) {
    PathOption new_option;
    new_option.curvature = curve;
    new_option.free_path_length = FreePathLength(curve);
    max_free_path = std::max(max_free_path, new_option.free_path_length);
    new_option.clearance = CalculateClearance(curve);
    path_options.push_back(new_option);
  }

  // now pick a curve based on our reward function and return
  return {RewardFunction(path_options), max_free_path};
}

PathOption Navigation::RewardFunction(vector<PathOption> path_options) {
  float reward;
  float max_reward = -100000; // very small so we can choose paths with negative reward if we have to
  PathOption best_path;
  for (auto option : path_options) {
    // option.free_path_length = std::min(3.0, double(option.free_path_length));
    reward = ApplyRewardFunction(option);
    cout << "curve: " << option.curvature << ", free path: " << option.free_path_length << ", reward: " << reward << endl;
    if (reward > max_reward) {
      max_reward = reward;
      best_path = option;
    }
  }
  return best_path;
}

float Navigation::ApplyRewardFunction(PathOption option) {

  return (10.0 * option.free_path_length) + (-0.05 * std::abs(option.curvature)) + (2.0 * option.clearance);
  // return option.free_path_length;
}

void Navigation::LatencyCompensation() {
  float x, y;
  struct Control control;

  // if we haven't seen enough past controls, assume we haven't started moving
  // and skip latency compensation step
  if (past_controls_.size() < QUEUE_LEN) {
    return;
  }

  // obtain the control issued LATENCYVALUE ago
  control = past_controls_.front();

  // USED THROUGHOUT THE METHOD
  float distance_traveled = control.velocity * SYSTEM_LATENCY;

  // For when the car is going straight 
  if (control.curvature == 0.0) {
    for (unsigned int i = 0; i < point_cloud_.size(); i++) {
      x = point_cloud_[i](0);
      y = point_cloud_[i](1);
      point_cloud_[i] = Vector2f(x - distance_traveled, y);
    }
    return;
  }

  // forward predict position
  float turning_radius = 1.0 / control.curvature;
  float arc_radians = distance_traveled / turning_radius;
  float new_x = distance_traveled * std::cos(arc_radians);
  float new_y = distance_traveled * std::sin(arc_radians);

  // adjust each point in the point cloud based on forward predicted position
  for (unsigned int i = 0; i < point_cloud_.size(); i++) {
    x = point_cloud_[i](0);
    y = point_cloud_[i](1);
    point_cloud_[i] = Vector2f(x - new_x, y - new_y);
  }

  // pop the control we used
  past_controls_.pop();
}

double Navigation::PointFreePath(const Vector2f& point, const Vector2f& turning_center, float small_radius, float mid_radius, float large_radius, float turning_radius) {
  // Consumes a point and the three radii of turning. It computes the free path length that the car can turn 
  // without colliding with the point. 
  // NOTE: This method should only be used after having already checked that a collision will eventually occur. 
  float beta, theta, abs_y;

  visualization::DrawPoint(point, 0xff006a, local_viz_msg_);

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

  // TODO: check on the theta values we create here
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
    return GoStraightFreePath();
  }

  // Find the turning radii necessary to compute collision
  float turning_radius = std::abs(1.0 / proposed_curvature);
  float small_radius = turning_radius - WIDTH / 2.0;
  float mid_radius = std::sqrt(std::pow((LENGTH + WHEELBASE) / 2.0, 2) + std::pow(small_radius, 2));
  float large_radius = std::sqrt(std::pow(turning_radius + WIDTH / 2.0, 2) + std::pow((LENGTH + WHEELBASE) / 2.0, 2));
  float turning_center_x = 0;
  float turning_center_y = 0;
  float global_min_free_path = 100001.0;

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
    global_min_free_path = turning_radius * M_PI;
  }

  return global_min_free_path;
}

// TODO: hacked this together super fast, may need edits
double Navigation::CalculateClearance(float proposed_curvature) {

  float global_min = 10001.0;
  float new_min; 
  if (proposed_curvature == 0) {
    for (auto point : point_cloud_) {
      new_min = std::sqrt(point(0) * point(0) + point(1) * point(1));
      if (new_min < global_min) {
        global_min = new_min;
      }
    }
    return global_min;
  }

  // Find the turning radii necessary to compute collision
  float turning_radius = std::abs(1.0 / proposed_curvature);
  float turning_center_x = 0;
  float turning_center_y = 0;

  // Computer the center turning points
  if (proposed_curvature > 0) {
    turning_center_y = turning_radius;
  } else {
    turning_center_y = -1 * turning_radius;
  }

  for(auto point : point_cloud_) {
    new_min = std::sqrt(std::pow(turning_center_x - point(0), 2) + std::pow(turning_center_y - point(1), 2)) - turning_radius;
    if (new_min < global_min) {
      global_min = new_min;
    }
  }

  return global_min;
}


float Navigation::GoStraightFreePath() {
  // For going straight, takes all points in a point cloud and returns the distance that the 
  // car can travel forward. 
  
  float new_length;
  float global_min = 10001.0;
  for(auto point : point_cloud_) {
    // WE ADD IN EXTRA - TODO - MAY NEED TWEAK
    if (point(1) >= -1 * ((WIDTH / 2) + 0.01) && point(1) <= ((WIDTH / 2)+ 0.01)) {
      // TODO MAY NEED TO INCLUDE THE LENGTH OF THE CAR IN THE NEW LENGTH - POTENTIAL BUG
      new_length = point(0) - ((LENGTH / 2) + (WHEELBASE / 2));
      global_min = std::min(global_min, new_length);
    }
  }

  if (global_min >= 10000) {
      return 20.0;
    } else {
      return global_min;
    }
}


float Navigation::ComputeVelocity(float current_velocity, float free_path) {
  // Computes the next step of our 1D controller based on the current 
  // velocity and the free path of proposed arc 
  float minimum_distance = (-1 * (current_velocity * current_velocity)) / (2 * DECELERATION);
  if (free_path <= minimum_distance) {
    return max(0.0, current_velocity + ((1.0 / HERTZ) * DECELERATION));
  } else if (current_velocity < MAX_VELOCITY) {
    return current_velocity + ((1.0 / HERTZ) * ACCELERATION);
  } else {
    // We are at the max velocity
    return current_velocity;
  }
}

}  // namespace navigation
