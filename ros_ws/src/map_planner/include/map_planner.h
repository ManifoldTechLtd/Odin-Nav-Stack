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
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <map_planner/PlanPath.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <string>
#include <vector>

class MapPlanner {
public:
  MapPlanner(ros::NodeHandle& nh, ros::NodeHandle& private_nh);

private:
  void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg);
  // F2: dynamic obstacles from fake360's accumulated occupancy grid (in odom
  // frame). Merged into plan_data_ before each A* search so the planner
  // avoids people/dynamic obstacles that aren't in the static /map.
  void fakeMapCallback(const nav_msgs::OccupancyGridConstPtr& msg);
  void goalCallback(const geometry_msgs::PoseStampedConstPtr& goal);
  bool getRobotPose(geometry_msgs::PoseStamped& pose) const;
  bool plan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, nav_msgs::Path& path);
  bool planService(map_planner::PlanPath::Request& req, map_planner::PlanPath::Response& res);
  void inflateMap();
  // F2: rebuild plan_data_ = inflated_data_ + fake360 occupied cells (with
  // inflation), transformed from fake_map's frame into /map's frame.
  // Called from mapCallback, fakeMapCallback, and plan() entry.
  void buildPlanData();
  void publishInflatedMap();
  void publishPlanResult(bool success);
  bool worldToMap(const geometry_msgs::Point& point, int& mx, int& my) const;
  geometry_msgs::Point mapToWorld(int mx, int my) const;
  inline int toIndex(int mx, int my) const { return my * static_cast<int>(map_.info.width) + mx; }
  bool isFree(int index) const;
  // String-pulling on the A* output: drop intermediate waypoints when the
  // straight line between two non-adjacent waypoints crosses only free cells
  // in the inflated grid. Lets us emit clean straight lines on open ground
  // and minimal corner sequences around obstacles, instead of 8-connected
  // staircase paths that downstream MPC has to track wobbly.
  // Path C: when `min_safe_dist_m` > 0, the line of sight check requires
  // every cell on the Bresenham line to have `distance_to_blocked_` at
  // least `min_safe_dist_m`. Prevents string-pull from short-cutting a
  // centered cost-A* path back to the wall-hugging straight line.
  // Defaults to 0 (legacy binary free-only check).
  bool hasLineOfSight(int x0, int y0, int x1, int y1, double min_safe_dist_m = 0.0) const;
  void simplifyPath(std::vector<int>& index_path) const;
  // Path C: Chamfer 1-SQRT2 distance transform from blocked cells (any
  // cell with !isFree). Populates `distance_to_blocked_` in METERS.
  // O(N) 2-pass, called from buildPlanData(). Approximate Euclidean,
  // good enough for a smooth obstacle-distance cost gradient.
  void computeDistanceTransform();
  // Path C: linear penalty multiplier on A* edge cost.
  //   d >= obstacle_cost_safe_distance_m_  ->  1.0  (no penalty)
  //   d == 0                                ->  1 + obstacle_cost_weight_
  //   linear interpolation in between
  // Cells without a computed distance (distance_to_blocked_ empty) get
  // multiplier 1.0.
  double cellCostMultiplier(int index) const;
  // Heading-aware A* (Jun 9 g6 bag): bias the first ~radius_m of the search
  // toward the robot's current heading so a replan triggered while the dog
  // faces e.g. left does not emit a path that initially heads right. Without
  // this, A* picks shortest-distance bypass which is often opposite to the
  // dog's current yaw, NeuPAN then must turn 100°+ against a near obstacle
  // -> stuck oscillation. Returns 1.0 when feature disabled or cell is
  // farther than `start_heading_penalty_radius_m_` from the start.
  // Otherwise: 1 + alpha * (1 - cos(diff)) * (1 - d/radius), where alpha is
  // `start_heading_penalty_weight_` and diff is the angle between
  // (start->cell) direction and `start_yaw`.
  double headingAlignFactor(int n_index, int start_index, double start_yaw) const;
  // Path B: Laplacian smoothing on the densified polyline. Each interior
  // point is replaced with 0.25*prev + 0.5*self + 0.25*next, repeated for
  // `smoothing_iterations_` passes, with a per-point collision check so
  // smoothed points never enter occupied cells. On a perfectly straight
  // segment the Laplacian is zero, so this is a no-op for clear paths;
  // sharp A* corners (45-deg or fake360 bypass arcs) get rounded into
  // gentle curves the MPC can track without yawing back and forth.
  void smoothPath(std::vector<geometry_msgs::PoseStamped>& poses) const;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber map_sub_;
  ros::Subscriber fake_map_sub_;
  ros::Subscriber goal_sub_;
  ros::Publisher path_pub_;
  ros::Publisher inflated_map_pub_;
  ros::Publisher plan_result_pub_;
  ros::ServiceServer plan_service_;
  bool publish_path_{true};
  std::string plan_service_name_{"plan"};

  nav_msgs::OccupancyGrid map_;
  std::vector<int8_t> inflated_data_;  // static base: inflated /map
  // F2: cached fake360 grid (odom frame). Refreshed by fakeMapCallback.
  nav_msgs::OccupancyGrid fake_map_;
  bool fake_map_ready_{false};
  // F2: A* reads from plan_data_ = inflated_data_ + fake360 dynamic cells.
  // Rebuilt from scratch each time either source updates.
  std::vector<int8_t> plan_data_;
  bool map_ready_{false};
  double inflation_radius_{0.25};
  int obstacle_threshold_{50};
  int inflation_cells_{1};
  // F2: cells fetched from fake360 older than this are ignored. Guards
  // against planning around an obstacle that's been stale for several
  // seconds (the dog may have already moved past it; fake360's cell decay
  // typically expires occupied cells in ~3s anyway).
  double fake_map_max_age_sec_{2.0};
  // Path B: number of Laplacian smoothing passes on the densified path.
  // 0 disables smoothing (legacy behavior). 20-30 rounds the typical
  // A* 45-deg corner and fake360 bypass arcs without significant
  // deviation from the planned route. Each pass is O(N) over the polyline.
  int smoothing_iterations_{20};
  // Path B: maximum lateral deviation any single point is allowed to
  // accumulate from its original densified position over all smoothing
  // passes. Caps how far smoothing can pull a point away from the A*
  // route, so smoothing stays a "rounding" operation rather than a
  // shortcut around obstacles (string-pull already did that part safely
  // on the inflated grid; smoothing must not reintroduce risk).
  double smoothing_max_deviation_m_{0.20};

  // Path C: distance-to-obstacle graduated cost (Sep 6 field bag).
  // Bag analysis showed A* uses pure step-distance costs (1.0 / SQRT2)
  // which makes shortest paths hug the inflation boundary. The dog then
  // followed wall-edge paths and got stuck at corners (NeuPAN's MPC
  // bounced between the inside-corner wall and the path it could not
  // safely overshoot). The fix is a graduated penalty: cells closer than
  // `obstacle_cost_safe_distance_m_` to any blocked cell get higher
  // edge cost, so A* prefers centered paths. `obstacle_cost_weight_` is
  // the max multiplier (cost = base * (1 + weight) at the inflation
  // boundary). Set weight to 0 to disable.
  double obstacle_cost_weight_{3.0};
  double obstacle_cost_safe_distance_m_{0.5};
  // Path C: minimum distance the string-pull line of sight check
  // requires along the Bresenham segment. Without this, simplifyPath
  // would short-cut a centered A* path back to wall-adjacent diagonals
  // (cost-A* finds the centered path but string-pull would undo it).
  // Smaller than `obstacle_cost_safe_distance_m_` so straight corridors
  // still simplify cleanly. Set to 0 to restore legacy string-pull.
  double line_of_sight_safe_distance_m_{0.25};
  // Path C: distance-to-blocked in METERS, indexed by toIndex().
  // 0 for cells with !isFree, positive elsewhere. Rebuilt by
  // computeDistanceTransform() inside buildPlanData(). Empty until the
  // first plan_data_ is built.
  std::vector<float> distance_to_blocked_;

  // Heading-aware A* (Jun 9): tunables for the start-heading bias that
  // discourages first-segment paths that point opposite to the dog's
  // current yaw. Set weight to 0 to disable (recovers pure cost-A*).
  // Field-default 2.0 / 1.0m: enough to flip a same-cost left/right tie
  // toward the side the dog is already facing, but small enough that an
  // honestly shorter detour will still win.
  double start_heading_penalty_weight_{2.0};
  double start_heading_penalty_radius_m_{1.0};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};
