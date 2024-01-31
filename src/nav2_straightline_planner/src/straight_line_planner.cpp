/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020 Shivang Patel
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Shivang Patel
 *
 * Reference tutorial:
 * https://navigation.ros.org/tutorials/docs/writing_new_nav2planner_plugin.html
 *********************************************************************/

#include <cmath>
#include <string>
#include <memory>
#include "nav2_util/node_utils.hpp"

#include "nav2_straightline_planner/straight_line_planner.hpp"

namespace nav2_straightline_planner
{

// Here's the easing function placed just inside the namespace but outside the class.
// double cubicEaseInOut(double t) {
//   if (t < 0.5) {
//     return 4.0 * t * t * t;
//   } else {
//     double f = ((2.0 * t) - 2.0);
//     return 0.5 * f * f * f + 1.0;
//   }
// }
  static geometry_msgs::msg::PoseStamped last_goal_;
  static nav_msgs::msg::Path last_global_path_;


double cubicEaseInOut(double t) {
  if (t < 0.25) {
    // Ease in quickly in the first quarter
    return 4 * t * t * t;
  } else if (t < 0.5) {
    // Linear interpolation for the second quarter
    return t - 0.125;
  } else {
    // Slow ease out over the second half
    double f = 2 * (t - 0.5);
    return 0.5 * (2 * f - f * f * f) + 0.375;
  }
}


void StraightLine::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  // Parameter initialization
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.01));
  node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
}

void StraightLine::cleanup()
{
  RCLCPP_INFO(
    node_->get_logger(), "CleaningUp plugin %s of type NavfnPlanner",
    name_.c_str());
}

void StraightLine::activate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Activating plugin %s of type NavfnPlanner",
    name_.c_str());
}

void StraightLine::deactivate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating plugin %s of type NavfnPlanner",
    name_.c_str());
}

nav_msgs::msg::Path StraightLine::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path global_path;

    // Check if the goal has changed
  if (!(last_goal_.header.frame_id.empty()) && (goal == last_goal_)) {
    // If the goal is the same, return the last global path
    return last_global_path_;
  }


  // Checking if the goal and start state is in the global frame
  if (start.header.frame_id != global_frame_) {
    RCLCPP_ERROR(
      node_->get_logger(), "Planner will only except start position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  if (goal.header.frame_id != global_frame_) {
    RCLCPP_INFO(
      node_->get_logger(), "Planner will only except goal position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  global_path.poses.clear();
  global_path.header.stamp = node_->now();
  global_path.header.frame_id = global_frame_;


  // calculating the number of loops for current value of interpolation_resolution_
  int total_number_of_loop = std::hypot(
    goal.pose.position.x - start.pose.position.x,
    goal.pose.position.y - start.pose.position.y) /
    0.04;
  double x_increment = (goal.pose.position.x - start.pose.position.x) / total_number_of_loop;
  double y_increment = (goal.pose.position.y - start.pose.position.y) / total_number_of_loop;

  for (int i = 0; i < total_number_of_loop; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = start.pose.position.x + x_increment * i;
    pose.pose.position.y = start.pose.position.y + y_increment * i;
    pose.pose.position.z = 0.0;

    // double t = static_cast<double>(i) / total_number_of_loop;
    double t = cubicEaseInOut(static_cast<double>(i) / total_number_of_loop);

    // SLERP for orientation
    double cosTheta = start.pose.orientation.x * goal.pose.orientation.x + 
                      start.pose.orientation.y * goal.pose.orientation.y +
                      start.pose.orientation.z * goal.pose.orientation.z + 
                      start.pose.orientation.w * goal.pose.orientation.w;
    //changable start and goal pose
    double ustartx = start.pose.orientation.x;
    double ustarty = start.pose.orientation.y;
    double ustartz = start.pose.orientation.z;
    double ustartw = start.pose.orientation.w;

    double ugoalx = goal.pose.orientation.x;
    double ugoaly = goal.pose.orientation.y;
    double ugoalz = goal.pose.orientation.z;
    double ugoalw = goal.pose.orientation.w;

    //invert 
    if (cosTheta < 0.0) {
        ugoalx = -goal.pose.orientation.x;
        ugoaly= -goal.pose.orientation.y;
        ugoalz = -goal.pose.orientation.z;
        ugoalw = -goal.pose.orientation.w;
        cosTheta = -cosTheta;
    }

    double scale0, scale1;
    if (cosTheta < 0.9999) {
        double theta = acos(cosTheta);
        double sinTheta = sin(theta);
        scale0 = sin((1.0 - t) * theta) / sinTheta;
        scale1 = sin(t * theta) / sinTheta;
    } else {
        scale0 = 1.0 - t;
        scale1 = t;
    }

    pose.pose.orientation.x = scale0 * ustartx + scale1 * ugoalx;
    pose.pose.orientation.y = scale0 * ustarty + scale1 * ugoaly;
    pose.pose.orientation.z = scale0 * ustartz + scale1 * ugoalz;
    pose.pose.orientation.w = scale0 * ustartw + scale1 * ugoalw;

    // pose.pose.orientation.x = 0.0;
    // pose.pose.orientation.y = 0.0;
    // pose.pose.orientation.z = 0.0;
    // pose.pose.orientation.w = 1.0;
    pose.header.stamp = node_->now();
    pose.header.frame_id = global_frame_;
    global_path.poses.push_back(pose);
  }

  geometry_msgs::msg::PoseStamped goal_pose = goal;
  goal_pose.header.stamp = node_->now();
  goal_pose.header.frame_id = global_frame_;
  global_path.poses.push_back(goal_pose);

  last_goal_ = goal;
  last_global_path_ = global_path;

  return global_path;
}

}  // namespace nav2_straightline_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_straightline_planner::StraightLine, nav2_core::GlobalPlanner)
