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

#include "map_planner.h"
#include "ros/console.h"

#include <queue>
#include <unordered_map>
#include <vector>
#include <limits>
#include <cmath>

#include <std_msgs/Bool.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace {
struct Node {
  int index;
  double g;
  double f;
  bool operator>(const Node& other) const { return f > other.f; }
};

constexpr double SQRT2 = 1.41421356237;
}  // namespace

MapPlanner::MapPlanner(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
    : nh_(nh),
      private_nh_(private_nh),
      tf_listener_(tf_buffer_) {
  private_nh_.param("inflation_radius", inflation_radius_, inflation_radius_);
  private_nh_.param("obstacle_threshold", obstacle_threshold_, obstacle_threshold_);
  private_nh_.param("publish_path", publish_path_, publish_path_);
  private_nh_.param("service_name", plan_service_name_, plan_service_name_);
  path_pub_ = nh_.advertise<nav_msgs::Path>("initial_path", 1, true);
  inflated_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("inflated_map", 1, true);
  plan_result_pub_ = nh_.advertise<std_msgs::Bool>("/map_planner/result", 1, true);
  map_sub_ = nh_.subscribe("map", 1, &MapPlanner::mapCallback, this);
  goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &MapPlanner::goalCallback, this);
  plan_service_ = private_nh_.advertiseService(plan_service_name_, &MapPlanner::planService, this);
}

void MapPlanner::mapCallback(const nav_msgs::OccupancyGridConstPtr& msg) {
  map_ = *msg;
  if (map_.info.resolution <= 0.0) {
    ROS_WARN_THROTTLE(5.0, "Map resolution invalid.");
    map_ready_ = false;
    return;
  }
  inflation_cells_ = std::max(1, static_cast<int>(std::ceil(inflation_radius_ / map_.info.resolution)));
  inflateMap();
  map_ready_ = true;
  publishInflatedMap();
  ROS_INFO_ONCE("Inflated map ready for planning.");
}

void MapPlanner::publishInflatedMap() {
  if (!inflated_map_pub_) return;
  nav_msgs::OccupancyGrid inflated = map_;
  inflated.header.stamp = ros::Time::now();
  inflated.data = inflated_data_;
  inflated_map_pub_.publish(inflated);
}

void MapPlanner::publishPlanResult(bool success) {
  if (!plan_result_pub_) return;
  std_msgs::Bool msg;
  msg.data = success;
  plan_result_pub_.publish(msg);
}

void MapPlanner::goalCallback(const geometry_msgs::PoseStampedConstPtr& goal) {
  if (!map_ready_) {
    ROS_WARN_THROTTLE(2.0, "Map not ready for planning.");
    publishPlanResult(false);
    return;
  }
  geometry_msgs::PoseStamped start_pose;
  if (!getRobotPose(start_pose)) {
    ROS_WARN_THROTTLE(2.0, "Unable to get robot pose.");
    publishPlanResult(false);
    return;
  }
  geometry_msgs::PoseStamped goal_in_map = *goal;
  if (goal->header.frame_id != map_.header.frame_id) {
    try {
      // tf_buffer_.transform(*goal, goal_in_map, map_.header.frame_id, ros::Duration(0.1));
      geometry_msgs::TransformStamped tf_stamped =
      tf_buffer_.lookupTransform(map_.header.frame_id,        // target frame
                                goal->header.frame_id,      // source frame
                                ros::Time(0),               // latest available
                                ros::Duration(0.1));        // timeout
      tf2::doTransform(*goal, goal_in_map, tf_stamped);
    } catch (const tf2::TransformException& ex) {
      ROS_WARN_THROTTLE(2.0, "Goal transform failed: %s", ex.what());
      publishPlanResult(false);
      return;
    }
  }
  nav_msgs::Path path;
  const bool success = plan(start_pose, goal_in_map, path);
  publishPlanResult(success);
  if (!success) {
    ROS_WARN("Failed to plan a path.");
    return;
  }
  path_pub_.publish(path);
}

bool MapPlanner::getRobotPose(geometry_msgs::PoseStamped& pose) const {
  try {
    geometry_msgs::TransformStamped tf = tf_buffer_.lookupTransform(map_.header.frame_id, "base_link", ros::Time(0), ros::Duration(0.2));
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

bool MapPlanner::plan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, nav_msgs::Path& path) {
  int start_x, start_y, goal_x, goal_y;
  if (!worldToMap(start.pose.position, start_x, start_y) || !worldToMap(goal.pose.position, goal_x, goal_y)) {
    ROS_WARN("Start or goal outside the map.");
    return false;
  }
  const int width = static_cast<int>(map_.info.width);
  const int height = static_cast<int>(map_.info.height);
  const int start_index = toIndex(start_x, start_y);
  const int goal_index = toIndex(goal_x, goal_y);
  if (!isFree(start_index) || !isFree(goal_index)) {
    ROS_WARN("Start or goal is occupied.");
    return false;
  }

  std::vector<double> g_score(width * height, std::numeric_limits<double>::infinity());
  std::vector<int> came_from(width * height, -1);
  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_set;

  auto heuristic = [&](int mx, int my) {
    const double dx = static_cast<double>(mx - goal_x);
    const double dy = static_cast<double>(my - goal_y);
    return std::hypot(dx, dy);
  };

  g_score[start_index] = 0.0;
  open_set.push({start_index, 0.0, heuristic(start_x, start_y)});

  const int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  const int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  const double costs[8] = {1.0, 1.0, 1.0, 1.0, SQRT2, SQRT2, SQRT2, SQRT2};

  while (!open_set.empty()) {
    Node current = open_set.top();
    open_set.pop();
    if (current.index == goal_index) break;

    int cx = current.index % width;
    int cy = current.index / width;

    for (int i = 0; i < 8; ++i) {
      const int nx = cx + dx[i];
      const int ny = cy + dy[i];
      if (nx < 0 || ny < 0 || nx >= width || ny >= height) continue;
      const int n_index = toIndex(nx, ny);
      if (!isFree(n_index)) continue;

      const double tentative_g = g_score[current.index] + costs[i];
      if (tentative_g < g_score[n_index]) {
        came_from[n_index] = current.index;
        g_score[n_index] = tentative_g;
        const double f_score = tentative_g + heuristic(nx, ny);
        open_set.push({n_index, tentative_g, f_score});
      }
    }
  }

  if (came_from[goal_index] == -1 && goal_index != start_index) {
    return false;
  }

  std::vector<int> index_path;
  for (int current = goal_index; current != -1; current = came_from[current]) {
    index_path.push_back(current);
    if (current == start_index) break;
  }
  if (index_path.back() != start_index) return false;
  std::reverse(index_path.begin(), index_path.end());

  path.header.stamp = ros::Time::now();
  path.header.frame_id = map_.header.frame_id;
  path.poses.reserve(index_path.size());
  for (int idx : index_path) {
    const int mx = idx % width;
    const int my = idx / width;
    geometry_msgs::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position = mapToWorld(mx, my);
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  return true;
}

void MapPlanner::inflateMap() {
  inflated_data_ = map_.data;
  if (inflation_cells_ <= 0) return;

  const int width = static_cast<int>(map_.info.width);
  const int height = static_cast<int>(map_.info.height);
  std::vector<int8_t> result = inflated_data_;

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const int index = toIndex(x, y);
      if (map_.data[index] < obstacle_threshold_ || map_.data[index] < 0) continue;
      for (int dy = -inflation_cells_; dy <= inflation_cells_; ++dy) {
        for (int dx = -inflation_cells_; dx <= inflation_cells_; ++dx) {
          const int nx = x + dx;
          const int ny = y + dy;
          if (nx < 0 || ny < 0 || nx >= width || ny >= height) continue;
          if (std::hypot(dx, dy) * map_.info.resolution > inflation_radius_) continue;
          result[toIndex(nx, ny)] = 100;
        }
      }
    }
  }
  inflated_data_.swap(result);
}

bool MapPlanner::worldToMap(const geometry_msgs::Point& point, int& mx, int& my) const {
  if (!map_ready_) return false;
  const double origin_x = map_.info.origin.position.x;
  const double origin_y = map_.info.origin.position.y;
  const double resolution = map_.info.resolution;

  mx = static_cast<int>(std::floor((point.x - origin_x) / resolution));
  my = static_cast<int>(std::floor((point.y - origin_y) / resolution));
  return mx >= 0 && my >= 0 && mx < static_cast<int>(map_.info.width) && my < static_cast<int>(map_.info.height);
}

geometry_msgs::Point MapPlanner::mapToWorld(int mx, int my) const {
  geometry_msgs::Point point;
  point.x = map_.info.origin.position.x + (mx + 0.5) * map_.info.resolution;
  point.y = map_.info.origin.position.y + (my + 0.5) * map_.info.resolution;
  point.z = 0.0;
  return point;
}

bool MapPlanner::isFree(int index) const {
  const int8_t value = inflated_data_[index];
  if (value < 0) return false;
  return value < obstacle_threshold_;
}

bool MapPlanner::planService(map_planner::PlanPath::Request& req, map_planner::PlanPath::Response& res) {
  if (!map_ready_) {
    ROS_WARN_THROTTLE(2.0, "Map not ready for planning.");
    publishPlanResult(false);
    return false;
  }
  geometry_msgs::PoseStamped start_pose;
  if (!getRobotPose(start_pose)) {
    ROS_WARN_THROTTLE(2.0, "Unable to get robot pose.");
    publishPlanResult(false);
    return false;
  }
  geometry_msgs::PoseStamped goal = req.goal;
  if (goal.header.frame_id.empty()) {
    goal.header.frame_id = map_.header.frame_id;
  }
  if (goal.header.frame_id != map_.header.frame_id) {
    ROS_WARN("Goal frame (%s) does not match map frame (%s).", goal.header.frame_id.c_str(), map_.header.frame_id.c_str());
    publishPlanResult(false);
    return false;
  }
  nav_msgs::Path path;
  const bool success = plan(start_pose, goal, path);
  publishPlanResult(success);
  if (!success) {
    ROS_WARN("Failed to plan a path.");
    return false;
  }
  res.path = path;
  if (publish_path_) path_pub_.publish(path);
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "map_planner");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  MapPlanner planner(nh, private_nh);
  ros::spin();
  return 0;
}
