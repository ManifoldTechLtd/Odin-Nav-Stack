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

#include "fake360.h"

#include <algorithm>
#include <cmath>

#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace {
constexpr double kPi = 3.14159265358979323846;
}

Fake360Node::Fake360Node() : nh_(), pnh_("~"), tf_listener_(tf_buffer_) {
  map_frame_ = pnh_.param<std::string>("map_frame", "odom");
  base_frame_ = pnh_.param<std::string>("base_frame", "base_link");
  scan_topic_ = pnh_.param<std::string>("input_scan_topic", "/scan_input");
  fake_scan_topic_ = pnh_.param<std::string>("fake_scan_topic", "/scan_360");
  map_topic_ = pnh_.param<std::string>("map_topic", "/fake_map");
  max_fake_range_ = pnh_.param("max_fake_range", 30.0);
  range_min_ = pnh_.param("range_min", 0.05);
  output_points_ = std::max(1, pnh_.param("output_points", 720));
  double resolution = pnh_.param("resolution", 0.05);
  int width = pnh_.param("width", 400);
  int height = pnh_.param("height", 400);
  double origin_x = pnh_.param("origin_x", -width * resolution / 2.0);
  double origin_y = pnh_.param("origin_y", -height * resolution / 2.0);

  map_.header.frame_id = map_frame_;
  map_.info.resolution = resolution;
  map_.info.width = width;
  map_.info.height = height;
  map_.info.origin.position.x = origin_x;
  map_.info.origin.position.y = origin_y;
  map_.info.origin.orientation.w = 1.0;
  map_.data.assign(width * height, -1);

  ray_step_ = std::max(0.01, resolution * 0.5);

  map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(map_topic_, 1, true);
  fake_scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>(fake_scan_topic_, 1);
  scan_sub_ = nh_.subscribe(scan_topic_, 1, &Fake360Node::scanCallback, this);
}

void Fake360Node::scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
  geometry_msgs::TransformStamped tf_msg;
  try {
    tf_msg = tf_buffer_.lookupTransform(map_frame_, msg->header.frame_id,
                                        msg->header.stamp, ros::Duration(0.1));
  } catch (const tf2::TransformException &ex) {
    ROS_WARN_THROTTLE(1.0, "TF lookup failed: %s", ex.what());
    return;
  }

  tf2::Transform tf_map_laser;
  tf2::fromMsg(tf_msg.transform, tf_map_laser);

  updateMap(*msg, tf_map_laser);

  map_.header.stamp = msg->header.stamp;
  map_.info.map_load_time = msg->header.stamp;
  map_pub_.publish(map_);

  publishFakeScan(msg->header.stamp);
}

void Fake360Node::updateMap(const sensor_msgs::LaserScan &scan,
                            const tf2::Transform &tf_map_laser) {
  tf2::Vector3 origin = tf_map_laser.getOrigin();
  int origin_mx, origin_my;
  if (!worldToMap(origin.x(), origin.y(), origin_mx, origin_my)) {
    return;
  }

  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    const float raw_range = scan.ranges[i];
    double range = std::isfinite(raw_range) ? static_cast<double>(raw_range)
                                            : scan.range_max;
    bool has_hit = std::isfinite(raw_range) && raw_range >= scan.range_min &&
                   raw_range < scan.range_max;
    double angle =
        scan.angle_min + static_cast<double>(i) * scan.angle_increment;

    tf2::Vector3 direction(std::cos(angle), std::sin(angle), 0.0);
    double traveled = std::min(static_cast<double>(scan.range_max), range);

    std::vector<std::pair<int, int>> ray_cells;
    ray_cells.emplace_back(origin_mx, origin_my);

    double free_limit = has_hit ? std::max(range - 1e-3, 0.0) : traveled;
    for (double dist = ray_step_; dist <= free_limit; dist += ray_step_) {
      tf2::Vector3 point = tf_map_laser * (direction * dist);
      int mx, my;
      if (!worldToMap(point.x(), point.y(), mx, my)) {
        continue;
      }
      ray_cells.emplace_back(mx, my);
    }

    setFree(ray_cells);

    if (has_hit) {
      tf2::Vector3 hit_point =
          tf_map_laser * (direction * static_cast<double>(raw_range));
      int mx, my;
      if (worldToMap(hit_point.x(), hit_point.y(), mx, my)) {
        setOccupied(mx, my);
      }
    }
  }
}

void Fake360Node::publishFakeScan(const ros::Time &stamp) {
  geometry_msgs::TransformStamped tf_msg;
  try {
    tf_msg = tf_buffer_.lookupTransform(map_frame_, base_frame_, stamp,
                                        ros::Duration(0.05));
  } catch (const tf2::TransformException &ex) {
    ROS_WARN_THROTTLE(1.0, "TF lookup for base failed: %s", ex.what());
    return;
  }

  tf2::Transform tf_map_base;
  tf2::fromMsg(tf_msg.transform, tf_map_base);
  tf2::Vector3 origin = tf_map_base.getOrigin();

  sensor_msgs::LaserScan scan;
  scan.header.stamp = stamp;
  scan.header.frame_id = base_frame_;
  scan.angle_min = -kPi;
  scan.angle_max = kPi;
  scan.angle_increment =
      (scan.angle_max - scan.angle_min) / static_cast<double>(output_points_);
  scan.range_min = range_min_;
  scan.range_max = max_fake_range_;
  scan.ranges.assign(output_points_, static_cast<float>(max_fake_range_));

  int origin_mx, origin_my;
  if (!worldToMap(origin.x(), origin.y(), origin_mx, origin_my)) {
    fake_scan_pub_.publish(scan);
    return;
  }

  tf2::Matrix3x3 rotation(tf_map_base.getRotation());

  for (int i = 0; i < output_points_; ++i) {
    double angle = scan.angle_min + i * scan.angle_increment;
    tf2::Vector3 dir_base(std::cos(angle), std::sin(angle), 0.0);
    tf2::Vector3 dir_map = rotation * dir_base;

    double measured = max_fake_range_;
    for (double dist = range_min_; dist <= max_fake_range_; dist += ray_step_) {
      tf2::Vector3 point = origin + dir_map * dist;
      int mx, my;
      if (!worldToMap(point.x(), point.y(), mx, my)) {
        measured = max_fake_range_;
        break;
      }

      int idx = index(mx, my);
      int value = map_.data[idx];
      if (value == 100) {
        measured = dist;
        break;
      }
      if (value == -1) {
        measured = max_fake_range_;
        break;
      }
    }

    scan.ranges[i] = static_cast<float>(std::min(measured, max_fake_range_));
  }

  fake_scan_pub_.publish(scan);
}

bool Fake360Node::worldToMap(double wx, double wy, int &mx, int &my) const {
  double origin_x = map_.info.origin.position.x;
  double origin_y = map_.info.origin.position.y;

  if (wx < origin_x || wy < origin_y) {
    return false;
  }

  mx = static_cast<int>((wx - origin_x) / map_.info.resolution);
  my = static_cast<int>((wy - origin_y) / map_.info.resolution);

  if (mx < 0 || my < 0 || mx >= static_cast<int>(map_.info.width) ||
      my >= static_cast<int>(map_.info.height)) {
    return false;
  }
  return true;
}

void Fake360Node::setFree(const std::vector<std::pair<int, int>> &cells) {
  for (const auto &cell : cells) {
    int idx = index(cell.first, cell.second);
    if (idx >= 0) {
      map_.data[idx] = 0;
    }
  }
}

void Fake360Node::setOccupied(int mx, int my) {
  int idx = index(mx, my);
  if (idx >= 0) {
    map_.data[idx] = 100;
  }
}

int Fake360Node::index(int mx, int my) const {
  if (mx < 0 || my < 0 || mx >= static_cast<int>(map_.info.width) ||
      my >= static_cast<int>(map_.info.height)) {
    return -1;
  }
  return my * map_.info.width + mx;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "fake360_node");
  Fake360Node node;
  ros::spin();
  return 0;
}
