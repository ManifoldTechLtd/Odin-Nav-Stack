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
  // F2: fake_map staleness threshold. Default 2.0s — long enough to bridge
  // fake360's 20Hz publish rate even under load, short enough that we don't
  // plan around a person who's already walked past.
  private_nh_.param("fake_map_max_age_sec", fake_map_max_age_sec_, fake_map_max_age_sec_);
  // Path B: Laplacian smoothing of the densified A* path. Set to 0 to
  // disable. See header for tuning notes.
  private_nh_.param("smoothing_iterations", smoothing_iterations_, smoothing_iterations_);
  private_nh_.param("smoothing_max_deviation_m", smoothing_max_deviation_m_, smoothing_max_deviation_m_);
  // Path C: distance-to-obstacle graduated A* cost. See header.
  private_nh_.param("obstacle_cost_weight", obstacle_cost_weight_, obstacle_cost_weight_);
  private_nh_.param("obstacle_cost_safe_distance", obstacle_cost_safe_distance_m_, obstacle_cost_safe_distance_m_);
  private_nh_.param("line_of_sight_safe_distance", line_of_sight_safe_distance_m_, line_of_sight_safe_distance_m_);
  // Heading-aware A* (Jun 9 g6 bag): see header. weight=0 disables.
  private_nh_.param("start_heading_penalty_weight", start_heading_penalty_weight_, start_heading_penalty_weight_);
  private_nh_.param("start_heading_penalty_radius_m", start_heading_penalty_radius_m_, start_heading_penalty_radius_m_);
  path_pub_ = nh_.advertise<nav_msgs::Path>("initial_path", 1, true);
  inflated_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("inflated_map", 1, true);
  plan_result_pub_ = nh_.advertise<std_msgs::Bool>("/map_planner/result", 1, true);
  map_sub_ = nh_.subscribe("map", 1, &MapPlanner::mapCallback, this);
  // F2: subscribe to fake360's accumulated grid. Queue depth 1 because we
  // only ever care about the latest; we don't need backlog.
  fake_map_sub_ = nh_.subscribe("/fake_map", 1, &MapPlanner::fakeMapCallback, this);
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
  buildPlanData();         // F2: bake fake360 cells (if any) into plan_data_
  publishInflatedMap();    // publishes plan_data_ (the actual planning grid)
  ROS_INFO_ONCE("Inflated map ready for planning.");
}

void MapPlanner::fakeMapCallback(const nav_msgs::OccupancyGridConstPtr& msg) {
  // F2: cache the latest fake360 grid. The merge into plan_data_ happens
  // lazily in buildPlanData() (called by plan() entry and on map updates),
  // not every fake_map publish, to keep CPU bounded.
  fake_map_ = *msg;
  fake_map_ready_ = (fake_map_.info.resolution > 0.0);
  if (map_ready_ && fake_map_ready_) {
    // Lightweight re-merge so the published /inflated_map reflects the
    // current dynamic layer for visualization. ~5000 cell ops at 20 Hz =
    // 100 kops/s, negligible.
    buildPlanData();
    publishInflatedMap();
  }
}

void MapPlanner::buildPlanData() {
  // F2: start from the static inflated map. plan_data_ is the buffer A*
  // actually reads via isFree().
  plan_data_ = inflated_data_;

  // Inject fake_map dynamic obstacles (best-effort). Each `return` inside
  // the lambda just falls through to the unconditional distance-transform
  // rebuild below — we always want a fresh distance map for the current
  // plan_data_, even when fake_map is unavailable / stale.
  auto inject_fake_map = [&]() {
    if (!fake_map_ready_) return;

    // Ignore stale fake_map (fake360 typically publishes at 20Hz; if we
    // haven't seen one in > fake_map_max_age_sec_, assume the upstream
    // pipeline is dead and fall back to static-only planning).
    const ros::Time now = ros::Time::now();
    if (!fake_map_.header.stamp.isZero()
        && (now - fake_map_.header.stamp).toSec() > fake_map_max_age_sec_) {
      ROS_WARN_THROTTLE(5.0,
          "fake_map stale (%.1fs old), planning with static map only.",
          (now - fake_map_.header.stamp).toSec());
      return;
    }

    // Resolve TF from fake_map's frame (usually "odom") to /map's frame.
    // Without a valid TF we can't safely transform cells, so we fall back
    // to static-only.
    geometry_msgs::TransformStamped tf_msg;
    try {
      tf_msg = tf_buffer_.lookupTransform(
          map_.header.frame_id,           // target: /map's frame
          fake_map_.header.frame_id,      // source: fake_map's frame (odom)
          ros::Time(0),                   // latest available
          ros::Duration(0.05));
    } catch (const tf2::TransformException& ex) {
      ROS_WARN_THROTTLE(5.0,
          "Cannot merge fake_map: TF %s -> %s failed (%s).",
          fake_map_.header.frame_id.c_str(),
          map_.header.frame_id.c_str(), ex.what());
      return;
    }

    const double cx = tf_msg.transform.translation.x;
    const double cy = tf_msg.transform.translation.y;
    // 2D yaw from quaternion (z-axis rotation only).
    const double qx = tf_msg.transform.rotation.x;
    const double qy = tf_msg.transform.rotation.y;
    const double qz = tf_msg.transform.rotation.z;
    const double qw = tf_msg.transform.rotation.w;
    const double yaw = std::atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz));
    const double cos_y = std::cos(yaw);
    const double sin_y = std::sin(yaw);

    const int fw = static_cast<int>(fake_map_.info.width);
    const int fh = static_cast<int>(fake_map_.info.height);
    const double fres = fake_map_.info.resolution;
    const double fox = fake_map_.info.origin.position.x;
    const double foy = fake_map_.info.origin.position.y;
    const int mw = static_cast<int>(map_.info.width);
    const int mh = static_cast<int>(map_.info.height);

    // Iterate fake_map cells once. Only OCCUPIED cells (value == 100) inject
    // new obstacles; free and unknown cells are NOT used to override /map.
    // For each occupied cell, transform its world center (odom frame) into
    // /map's world frame, locate the corresponding /map cell, and inflate
    // around it using the same inflation_cells_ used for the static map.
    for (int fy = 0; fy < fh; ++fy) {
      for (int fx = 0; fx < fw; ++fx) {
        const int fidx = fy * fw + fx;
        if (fake_map_.data[fidx] != 100) continue;
        const double wx_odom = fox + (fx + 0.5) * fres;
        const double wy_odom = foy + (fy + 0.5) * fres;
        // Rigid-body 2D transform: p_map = R(yaw) * p_odom + (cx, cy).
        const double wx_map = cos_y * wx_odom - sin_y * wy_odom + cx;
        const double wy_map = sin_y * wx_odom + cos_y * wy_odom + cy;
        // Locate corresponding /map cell.
        const int mx = static_cast<int>(
            std::floor((wx_map - map_.info.origin.position.x) / map_.info.resolution));
        const int my = static_cast<int>(
            std::floor((wy_map - map_.info.origin.position.y) / map_.info.resolution));
        if (mx < 0 || my < 0 || mx >= mw || my >= mh) continue;
        // Inflate the dynamic obstacle by the same radius as static.
        for (int dy = -inflation_cells_; dy <= inflation_cells_; ++dy) {
          for (int dx = -inflation_cells_; dx <= inflation_cells_; ++dx) {
            const int nx = mx + dx;
            const int ny = my + dy;
            if (nx < 0 || ny < 0 || nx >= mw || ny >= mh) continue;
            if (std::hypot(dx, dy) * map_.info.resolution > inflation_radius_) continue;
            plan_data_[ny * mw + nx] = 100;
          }
        }
      }
    }
  };
  inject_fake_map();

  // Path C: rebuild the distance-to-blocked transform on the FINAL
  // plan_data_ (static inflated + any merged fake360 dynamic obstacles).
  // Distance is used by both A* edge cost and string-pull line-of-sight.
  computeDistanceTransform();
}

void MapPlanner::publishInflatedMap() {
  if (!inflated_map_pub_) return;
  nav_msgs::OccupancyGrid inflated = map_;
  inflated.header.stamp = ros::Time::now();
  // F2: publish the MERGED plan_data_ so visualizers (foxglove) show what
  // A* actually sees, including dynamic obstacles.
  inflated.data = plan_data_.empty() ? inflated_data_ : plan_data_;
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
  // F2: rebuild plan_data_ at the entry of every plan() so A* uses the
  // freshest fake360 snapshot. We do NOT publish here — fakeMapCallback
  // already publishes when fake_map updates.
  buildPlanData();

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

  // Heading-aware A*: extract start yaw from the input pose's quaternion.
  // Used by headingAlignFactor() to bias the first ~radius_m of search
  // toward the direction the dog is currently facing.
  const auto& q = start.pose.orientation;
  const double start_yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));

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

      // Path C: graduated penalty for cells close to obstacles. Pure step
      // cost (1.0 / SQRT2) made A* hug the inflation boundary, producing
      // wall-edge paths that NeuPAN then tracked into corner-stuck wobble.
      // cellCostMultiplier returns 1.0 for cells outside the safe distance,
      // up to (1 + obstacle_cost_weight_) at the inflation boundary.
      // Heading-aware A* (Jun 9 g6 bag): additional multiplier biasing the
      // first start_heading_penalty_radius_m_ of search toward the dog's
      // current yaw, so a replan triggered while facing left does not
      // emit a path heading right (which NeuPAN then can't execute).
      // headingAlignFactor returns 1.0 outside the radius / when disabled.
      const double tentative_g =
          g_score[current.index]
          + costs[i]
                * cellCostMultiplier(n_index)
                * headingAlignFactor(n_index, start_index, start_yaw);
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

  // String-pull on the inflated grid so straight-line goals emit two
  // waypoints and obstacle-bypass goals emit a minimal corner sequence.
  // Without this, 8-connected A* produces 45-degree zigzag waypoints which
  // make NeuPAN visibly wobble even on trivially-clear paths.
  simplifyPath(index_path);

  path.header.stamp = ros::Time::now();
  path.header.frame_id = map_.header.frame_id;

  // Densify the sparse simplified waypoints back to a dense pose sequence at
  // approximately one-cell spacing. This preserves the clean string-pulled
  // geometry (straight segments + minimal corners) but restores the high pose
  // count that downstream NeuPAN relies on: NeuPAN's check_curve_arrive uses
  // (point_index >= len(curve) - arrive_index_threshold - 2) as a fallback
  // arrival condition, which immediately fires for sparse paths and was
  // causing the robot to declare "arrive at end of path" without moving.
  const double step = map_.info.resolution;
  path.poses.reserve(index_path.size() * 4);
  for (size_t k = 0; k + 1 < index_path.size(); ++k) {
    const int x0 = index_path[k] % width;
    const int y0 = index_path[k] / width;
    const int x1 = index_path[k + 1] % width;
    const int y1 = index_path[k + 1] / width;
    const geometry_msgs::Point p0 = mapToWorld(x0, y0);
    const geometry_msgs::Point p1 = mapToWorld(x1, y1);
    const double seg_len = std::hypot(p1.x - p0.x, p1.y - p0.y);
    const int n_steps = std::max(1, static_cast<int>(std::ceil(seg_len / step)));
    for (int s = 0; s < n_steps; ++s) {
      const double t = static_cast<double>(s) / static_cast<double>(n_steps);
      geometry_msgs::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = p0.x + t * (p1.x - p0.x);
      pose.pose.position.y = p0.y + t * (p1.y - p0.y);
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
  }
  // Always include the final waypoint exactly.
  {
    const int xf = index_path.back() % width;
    const int yf = index_path.back() / width;
    geometry_msgs::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position = mapToWorld(xf, yf);
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  // Path B: round the corners of the densified polyline so MPC sees a
  // geometrically smooth reference instead of polyline kinks. No-op on
  // already-straight paths (Laplacian is zero on collinear points).
  smoothPath(path.poses);
  return true;
}

void MapPlanner::smoothPath(std::vector<geometry_msgs::PoseStamped>& poses) const {
  // Path B: iterative Laplacian smoothing with two safety constraints:
  //   1. Per-point collision check: never move a point into an occupied
  //      cell (read via isFree on the merged plan_data_, so this respects
  //      both static obstacles and fake360 dynamic obstacles).
  //   2. Bounded deviation: each point may not drift more than
  //      smoothing_max_deviation_m_ from its original position over all
  //      passes. Keeps smoothing local; prevents pathological pull-toward-
  //      a-shortcut behavior on long paths.
  //
  // Endpoints (start, goal) are never moved — they must match the robot
  // pose and the requested goal exactly.
  if (smoothing_iterations_ <= 0) return;
  if (poses.size() < 3) return;

  // Snapshot of original positions for the deviation check.
  std::vector<geometry_msgs::Point> original;
  original.reserve(poses.size());
  for (const auto& p : poses) original.push_back(p.pose.position);

  const double max_dev_sq = smoothing_max_deviation_m_ * smoothing_max_deviation_m_;
  const int n = static_cast<int>(poses.size());

  auto isFreeWorld = [&](double wx, double wy) -> bool {
    geometry_msgs::Point q;
    q.x = wx; q.y = wy; q.z = 0.0;
    int mx, my;
    if (!worldToMap(q, mx, my)) return false;
    return isFree(toIndex(mx, my));
  };

  for (int iter = 0; iter < smoothing_iterations_; ++iter) {
    // Out-of-place to avoid using already-updated neighbors within a pass.
    std::vector<geometry_msgs::Point> next(poses.size());
    next.front() = poses.front().pose.position;
    next.back()  = poses.back().pose.position;
    for (int i = 1; i < n - 1; ++i) {
      const auto& p_prev = poses[i - 1].pose.position;
      const auto& p_cur  = poses[i].pose.position;
      const auto& p_next = poses[i + 1].pose.position;
      // Laplacian / [0.25, 0.5, 0.25] kernel.
      const double cx = 0.25 * p_prev.x + 0.5 * p_cur.x + 0.25 * p_next.x;
      const double cy = 0.25 * p_prev.y + 0.5 * p_cur.y + 0.25 * p_next.y;
      // Deviation check vs original position.
      const double odx = cx - original[i].x;
      const double ody = cy - original[i].y;
      if (odx * odx + ody * ody > max_dev_sq) {
        // Cap movement at the deviation boundary along the direction we
        // wanted to go. Keeps smoothing monotonic and stable.
        const double scale = smoothing_max_deviation_m_ / std::sqrt(odx * odx + ody * ody);
        next[i].x = original[i].x + odx * scale;
        next[i].y = original[i].y + ody * scale;
        next[i].z = 0.0;
      } else {
        next[i].x = cx;
        next[i].y = cy;
        next[i].z = 0.0;
      }
      // Collision check: if the candidate would land in an occupied cell,
      // keep the previous (pre-pass) position unchanged for this point.
      if (!isFreeWorld(next[i].x, next[i].y)) {
        next[i] = p_cur;
      }
    }
    // Commit the pass.
    for (int i = 0; i < n; ++i) poses[i].pose.position = next[i];
  }
}

// Path C: 2-pass Chamfer 1-SQRT2 distance transform on plan_data_.
// Linear-time (O(N) per pass). After this call distance_to_blocked_[idx]
// is the approximate Euclidean distance, in METERS, from cell `idx` to
// the nearest cell with !isFree() (i.e. an original obstacle or an
// inflated/dynamic obstacle cell). Cells without a valid plan_data_ map
// get an empty distance_to_blocked_, in which case the cost multiplier
// degenerates to 1.0 (= legacy A*).
void MapPlanner::computeDistanceTransform() {
  if (plan_data_.empty()) {
    distance_to_blocked_.clear();
    return;
  }
  const int width = static_cast<int>(map_.info.width);
  const int height = static_cast<int>(map_.info.height);
  const double res = map_.info.resolution;
  const int N = width * height;

  // Sentinel "infinity" that fits comfortably in a float, big enough that
  // it cannot be the winner of any std::min on real maps (a 1000x1000 map
  // at 0.05m res has max chord ~70m).
  constexpr float INF_F = 1.0e9f;
  distance_to_blocked_.assign(N, INF_F);
  for (int idx = 0; idx < N; ++idx) {
    if (!isFree(idx)) distance_to_blocked_[idx] = 0.0f;
  }

  const float d1 = static_cast<float>(res);
  const float d2 = static_cast<float>(res * SQRT2);

  // Forward pass: top-to-bottom, left-to-right. Each cell looks at four
  // neighbors already updated this pass: NW, N, NE, W.
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const int idx = y * width + x;
      if (distance_to_blocked_[idx] == 0.0f) continue;
      float d = distance_to_blocked_[idx];
      if (x > 0) d = std::min(d, distance_to_blocked_[idx - 1] + d1);
      if (y > 0) {
        if (x > 0)         d = std::min(d, distance_to_blocked_[idx - width - 1] + d2);
        d = std::min(d, distance_to_blocked_[idx - width] + d1);
        if (x < width - 1) d = std::min(d, distance_to_blocked_[idx - width + 1] + d2);
      }
      distance_to_blocked_[idx] = d;
    }
  }
  // Backward pass: bottom-to-top, right-to-left. Looks at SE, S, SW, E.
  for (int y = height - 1; y >= 0; --y) {
    for (int x = width - 1; x >= 0; --x) {
      const int idx = y * width + x;
      if (distance_to_blocked_[idx] == 0.0f) continue;
      float d = distance_to_blocked_[idx];
      if (x < width - 1) d = std::min(d, distance_to_blocked_[idx + 1] + d1);
      if (y < height - 1) {
        if (x < width - 1) d = std::min(d, distance_to_blocked_[idx + width + 1] + d2);
        d = std::min(d, distance_to_blocked_[idx + width] + d1);
        if (x > 0)         d = std::min(d, distance_to_blocked_[idx + width - 1] + d2);
      }
      distance_to_blocked_[idx] = d;
    }
  }
}

double MapPlanner::cellCostMultiplier(int index) const {
  // Disabled (weight == 0) or distance map not built: legacy step cost.
  if (obstacle_cost_weight_ <= 0.0 || obstacle_cost_safe_distance_m_ <= 0.0) return 1.0;
  if (distance_to_blocked_.empty()) return 1.0;
  if (index < 0 || index >= static_cast<int>(distance_to_blocked_.size())) return 1.0;
  const float d = distance_to_blocked_[index];
  if (d >= static_cast<float>(obstacle_cost_safe_distance_m_)) return 1.0;
  // Linear from 1+weight at d==0 down to 1.0 at d==safe_distance.
  const double frac = (obstacle_cost_safe_distance_m_ - static_cast<double>(d))
                      / obstacle_cost_safe_distance_m_;
  return 1.0 + obstacle_cost_weight_ * frac;
}

double MapPlanner::headingAlignFactor(int n_index, int start_index, double start_yaw) const {
  // Disabled: weight == 0 -> pure cost-A* (current shipping behavior).
  if (start_heading_penalty_weight_ <= 0.0 || start_heading_penalty_radius_m_ <= 0.0) return 1.0;
  const int width = static_cast<int>(map_.info.width);
  const double res = map_.info.resolution;
  const double dx_cells = static_cast<double>(n_index % width - start_index % width);
  const double dy_cells = static_cast<double>(n_index / width - start_index / width);
  const double dist_m = std::hypot(dx_cells, dy_cells) * res;
  // Outside the bias radius: don't constrain — A* picks freely so honestly
  // shorter detours win.
  if (dist_m >= start_heading_penalty_radius_m_) return 1.0;
  // angle from start to candidate cell (world frame; map x/y aligned with world).
  const double angle_to_n = std::atan2(dy_cells, dx_cells);
  double diff = angle_to_n - start_yaw;
  while (diff > M_PI) diff -= 2.0 * M_PI;
  while (diff < -M_PI) diff += 2.0 * M_PI;
  // 1 - cos(diff): 0 at aligned, 2 at 180°. Smooth, no sharp cutoff.
  // Linearly fade with distance so very near the start the penalty is
  // strongest (forcing aligned exit) and at radius_m it vanishes.
  const double fade = 1.0 - dist_m / start_heading_penalty_radius_m_;
  return 1.0 + start_heading_penalty_weight_ * (1.0 - std::cos(diff)) * fade;
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
  // F2: A* reads the merged plan_data_ (= inflated_data_ + fake360 dynamic
  // cells). plan_data_ is rebuilt at the entry of plan() and on every
  // fake_map / map callback. Falls back to inflated_data_ if plan_data_
  // hasn't been built yet (e.g. very first frame before any plan call).
  const auto& grid = plan_data_.empty() ? inflated_data_ : plan_data_;
  if (index < 0 || index >= static_cast<int>(grid.size())) return false;
  const int8_t value = grid[index];
  if (value < 0) return false;
  return value < obstacle_threshold_;
}

bool MapPlanner::hasLineOfSight(int x0, int y0, int x1, int y1,
                                double min_safe_dist_m) const {
  // Standard integer Bresenham. Returns true iff every cell touched by the
  // line from (x0,y0) to (x1,y1) is isFree() in the inflated grid AND
  // (when min_safe_dist_m > 0) has distance_to_blocked_ >= min_safe_dist_m.
  // Path C: the distance check stops simplifyPath from short-cutting a
  // centered cost-A* path back to the wall-adjacent diagonals.
  const int width = static_cast<int>(map_.info.width);
  const int height = static_cast<int>(map_.info.height);
  if (x0 < 0 || y0 < 0 || x0 >= width || y0 >= height) return false;
  if (x1 < 0 || y1 < 0 || x1 >= width || y1 >= height) return false;

  const bool check_distance =
      (min_safe_dist_m > 0.0) && !distance_to_blocked_.empty();
  const float min_d_f = static_cast<float>(min_safe_dist_m);

  int dx = std::abs(x1 - x0);
  int dy = std::abs(y1 - y0);
  int sx = x0 < x1 ? 1 : -1;
  int sy = y0 < y1 ? 1 : -1;
  int err = dx - dy;
  int x = x0;
  int y = y0;
  while (true) {
    const int idx = toIndex(x, y);
    if (!isFree(idx)) return false;
    if (check_distance && distance_to_blocked_[idx] < min_d_f) return false;
    if (x == x1 && y == y1) return true;
    const int e2 = 2 * err;
    if (e2 > -dy) { err -= dy; x += sx; }
    if (e2 <  dx) { err += dx; y += sy; }
  }
}

void MapPlanner::simplifyPath(std::vector<int>& index_path) const {
  if (index_path.size() <= 2) return;
  const int width = static_cast<int>(map_.info.width);

  std::vector<int> simplified;
  simplified.reserve(index_path.size());
  simplified.push_back(index_path.front());

  size_t i = 0;
  while (i + 1 < index_path.size()) {
    // Look from the far end backward; the first j with line-of-sight from i
    // becomes the next waypoint. This is the funnel/string-pulling step:
    // collapses long straight runs to a single segment.
    size_t j = index_path.size() - 1;
    while (j > i + 1) {
      const int xi = index_path[i] % width;
      const int yi = index_path[i] / width;
      const int xj = index_path[j] % width;
      const int yj = index_path[j] / width;
      // Path C: require the shortcut to stay at least
      // line_of_sight_safe_distance_m_ away from any obstacle. Without this
      // string-pull would happily collapse a centered cost-A* path back to
      // a wall-adjacent diagonal whenever that diagonal is binary-free.
      if (hasLineOfSight(xi, yi, xj, yj, line_of_sight_safe_distance_m_)) break;
      --j;
    }
    simplified.push_back(index_path[j]);
    i = j;
  }
  index_path.swap(simplified);
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
