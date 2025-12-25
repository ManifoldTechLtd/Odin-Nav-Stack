/*
 * Copyright 2025 Manifold Tech Ltd.(www.manifoldtech.com.co)
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *   http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <map_planner/PlanPath.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <string>

class GoalStateMachine {
public:
  GoalStateMachine(ros::NodeHandle& nh, ros::NodeHandle& private_nh);

private:
  void arriveCallback(const std_msgs::EmptyConstPtr& msg);
  void goalCallback(const geometry_msgs::PoseStampedConstPtr& goal);
  bool getRobotPose(geometry_msgs::PoseStamped& pose) const;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber arrive_sub_;
  ros::Subscriber goal_sub_;
  ros::ServiceClient plan_client_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  geometry_msgs::PoseStamped last_goal_;
  bool have_goal_{false};
  double goal_tolerance_{0.3};
  std::string plan_service_name_{"/map_planner/plan"};
};