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
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>

#include <string>
#include <utility>
#include <vector>

class Fake360Node {
public:
  Fake360Node();

private:
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void updateMap(const sensor_msgs::LaserScan& scan, const tf2::Transform& tf_map_laser);
  void publishFakeScan(const ros::Time& stamp);
  bool worldToMap(double wx, double wy, int& mx, int& my) const;
  void setFree(const std::vector<std::pair<int, int>>& cells);
  void setOccupied(int mx, int my);
  int index(int mx, int my) const;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber scan_sub_;
  ros::Publisher map_pub_;
  ros::Publisher fake_scan_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  nav_msgs::OccupancyGrid map_;
  double max_fake_range_;
  double range_min_;
  double ray_step_;
  int output_points_;
  std::string map_frame_;
  std::string base_frame_;
  std::string scan_topic_;
  std::string fake_scan_topic_;
  std::string map_topic_;
};
