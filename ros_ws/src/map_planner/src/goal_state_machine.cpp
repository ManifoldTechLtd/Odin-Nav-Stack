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

#include "goal_state_machine.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

GoalStateMachine::GoalStateMachine(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
    : nh_(nh),
      private_nh_(private_nh),
      tf_listener_(tf_buffer_) {
  private_nh_.param("goal_tolerance", goal_tolerance_, goal_tolerance_);
  private_nh_.param("plan_service", plan_service_name_, plan_service_name_);
  arrive_sub_ = nh_.subscribe("/neupan/arrive", 1, &GoalStateMachine::arriveCallback, this);
  goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &GoalStateMachine::goalCallback, this);
  plan_client_ = nh_.serviceClient<map_planner::PlanPath>(plan_service_name_);
}

void GoalStateMachine::goalCallback(const geometry_msgs::PoseStampedConstPtr& goal) {
  geometry_msgs::PoseStamped map_goal;
  if (goal->header.frame_id.empty() || goal->header.frame_id == "map") {
    map_goal = *goal;
    map_goal.header.frame_id = "map";
  } else {
    try {
      tf_buffer_.transform(*goal, map_goal, "map", ros::Duration(0.2));
    } catch (const tf2::TransformException& ex) {
      ROS_WARN_THROTTLE(2.0, "Goal transform failed: %s", ex.what());
      return;
    }
  }
  last_goal_ = map_goal;
  have_goal_ = true;
}

void GoalStateMachine::arriveCallback(const std_msgs::EmptyConstPtr&) {
  if (!have_goal_) {
    ROS_WARN_THROTTLE(2.0, "Arrival received without a stored goal.");
    return;
  }
  geometry_msgs::PoseStamped current_pose;
  if (!getRobotPose(current_pose)) return;

  const double dx = last_goal_.pose.position.x - current_pose.pose.position.x;
  const double dy = last_goal_.pose.position.y - current_pose.pose.position.y;
  const double distance = std::hypot(dx, dy);

  if (distance <= goal_tolerance_) {
    ROS_INFO("Robot is within %.2f m of goal.", goal_tolerance_);
    return;
  }

  if (!plan_client_.exists() && !plan_client_.waitForExistence(ros::Duration(0.5))) {
    ROS_WARN("Plan service unavailable.");
    return;
  }

  map_planner::PlanPath srv;
  srv.request.goal = last_goal_;
  if (plan_client_.call(srv)) {
    ROS_INFO("Requested replanning toward goal (distance %.2f m).", distance);
  } else {
    ROS_WARN("Failed to call plan service.");
  }
}

bool GoalStateMachine::getRobotPose(geometry_msgs::PoseStamped& pose) const {
  try {
    geometry_msgs::TransformStamped tf = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0), ros::Duration(0.2));
    pose.header = tf.header;
    pose.pose.position.x = tf.transform.translation.x;
    pose.pose.position.y = tf.transform.translation.y;
    pose.pose.position.z = tf.transform.translation.z;
    pose.pose.orientation = tf.transform.rotation;
    return true;
  } catch (const tf2::TransformException& ex) {
    ROS_WARN_THROTTLE(2.0, "TF lookup failed: %s", ex.what());
    return false;
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "goal_state_machine");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  GoalStateMachine gsm(nh, private_nh);
  ros::spin();
  return 0;
}
