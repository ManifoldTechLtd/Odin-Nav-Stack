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

#include "model_planner/dwa_planner.h"
#include <ros/ros.h>
#include <cmath>
#include <limits>
#include <algorithm>

namespace model_planner {

static inline double clamp(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }

DWAPlanner::DWAPlanner(const LocalCostmap& costmap) : costmap_(costmap) {}

void DWAPlanner::initialize() {
    ROS_INFO("DWA Planner initialized");
}

bool DWAPlanner::plan(const Eigen::Vector3d& current_pose,
                      const Eigen::Vector3d& current_velocity,
                      const std::vector<std::pair<float, float>>& reference_path,
                      Eigen::Vector3d& cmd_vel) {
    reference_path_ = reference_path;
    best_traj_.clear();

    // dynamic window based on current velocity and acceleration limits
    double v_lo = current_velocity(0) - ax_max_ * sim_time_;
    double v_hi = current_velocity(0) + ax_max_ * sim_time_;
    double v_min = allow_backward_ ? std::max(-vx_max_, v_lo) : std::max(0.0, v_lo);
    double v_max = std::min(vx_max_, v_hi);
    double w_min = current_velocity(2) - alpha_max_ * sim_time_;
    double w_max = current_velocity(2) + alpha_max_ * sim_time_;

    if (!allow_backward_) {
        v_min = std::max(0.0, v_min);
    }
    v_max = std::max(v_min, v_max);
    w_min = std::max(-omega_max_, w_min);
    w_max = std::min( omega_max_, w_max);

    // sampling
    int Nv = std::max(1, v_samples_);
    int Nw = std::max(1, w_samples_);

    double best_score = -std::numeric_limits<double>::infinity();
    Eigen::Vector2d best_cmd(0, 0); // v, w

    for (int iv = 0; iv < Nv; ++iv) {
        double v = v_min + (v_max - v_min) * (Nv == 1 ? 0.0 : (double)iv / (Nv - 1));
        for (int iw = 0; iw < Nw; ++iw) {
            double w = w_min + (w_max - w_min) * (Nw == 1 ? 0.0 : (double)iw / (Nw - 1));

            std::vector<DWATrajPoint> traj;
            if (!simulateTrajectory(v, w, traj)) {
                continue; // collision during simulation
            }
            double score = scoreTrajectory(traj);
            if (score > best_score) {
                best_score = score;
                best_cmd << v, w;
                best_traj_ = std::move(traj);
            }
        }
    }

    if (best_score == -std::numeric_limits<double>::infinity()) {
        // rotate recovery: turn towards goal if enabled
        if (enable_rotate_recovery_ && !reference_path_.empty()) {
            const Eigen::Vector2d goal(reference_path_.back().first, reference_path_.back().second);
            double yaw_to_goal = std::atan2(goal(1), goal(0)); // current pose is origin
            double w = clamp(yaw_to_goal, -omega_max_, omega_max_);
            cmd_vel(0) = 0.0;
            cmd_vel(1) = 0.0;
            cmd_vel(2) = (w >= 0 ? 1.0 : -1.0) * 0.6 * omega_max_;
            best_traj_.clear();
            DWATrajPoint p0(0,0,0,0); best_traj_.push_back(p0);
            return true;
        }
        cmd_vel.setZero();
        return false;
    }

    cmd_vel(0) = best_cmd(0);
    cmd_vel(1) = 0.0; // differential drive assumption
    cmd_vel(2) = best_cmd(1);
    return true;
}

bool DWAPlanner::simulateTrajectory(double v, double w, std::vector<DWATrajPoint>& out) const {
    out.clear();
    double x = 0.0, y = 0.0, th = 0.0; // base frame, current pose is origin
    double t = 0.0;
    int steps = std::max(1, (int)std::round(sim_time_ / dt_));
    if (no_simulation_) {
        steps = 1; // only predict one step ahead
    }
    out.reserve(steps + 1);
    out.emplace_back(x, y, th, t);

    for (int i = 0; i < steps; ++i) {
        // simple unicycle model integration
        x += v * std::cos(th) * dt_;
        y += v * std::sin(th) * dt_;
        th += w * dt_;
        t += dt_;
        DWATrajPoint p(x, y, th, t);
        out.push_back(p);

        // collision check for this pose footprint
        if (obstacleCost(p.pose) >= 1e5) {
            return false;
        }
    }
    return true;
}


double DWAPlanner::scoreTrajectory(const std::vector<DWATrajPoint>& traj) const {
    if (traj.empty()) return -std::numeric_limits<double>::infinity();

    // weights from members
    const double w_clearance = w_clearance_;
    const double w_path = w_path_;
    const double w_heading = w_heading_;
    const double w_velocity = w_velocity_;

    // clearance: min obstacle cost along trajectory (higher better)
    double min_obs_cost = std::numeric_limits<double>::infinity();
    for (const auto& p : traj) {
        double c = obstacleCost(p.pose);
        min_obs_cost = std::min(min_obs_cost, c);
    }
    double clearance_score = (min_obs_cost >= 1e5) ? -1e3 : (1.0 / (1.0 + min_obs_cost));

    // path distance: average distance to nearest reference point (lower better)
    double avg_path_dist = 0.0;
    for (const auto& p : traj) {
        avg_path_dist += pathDistCost(p.pose);
    }
    avg_path_dist /= traj.size();
    double path_score = 1.0 / (1.0 + avg_path_dist);

    // heading: angle to goal from last point
    double heading_score = 0.0;
    if (!reference_path_.empty()) {
        const auto& last = traj.back();
        Eigen::Vector2d goal(reference_path_.back().first, reference_path_.back().second);
        double yaw_to_goal = std::atan2(goal(1) - last.pose(1), goal(0) - last.pose(0));
        double diff = std::fabs(std::atan2(std::sin(yaw_to_goal - last.theta), std::cos(yaw_to_goal - last.theta)));
        heading_score = 1.0 - clamp(diff / M_PI, 0.0, 1.0); // closer to 0 angle is better
        // boost for strong misalignment to encourage turning in place
        if (diff > heading_align_thresh_) {
            heading_score *= heading_boost_;
        }
    }

    // velocity: prefer higher speed magnitude (allow backward)
    double vel_score = traj.size() >= 2 ? ( (traj[1].pose - traj[0].pose).norm() / dt_ ) : 0.0;
    vel_score = clamp(vel_score / (vx_max_ + 1e-6), 0.0, 1.0);

    double total = w_clearance * clearance_score + w_path * path_score + w_heading * heading_score + w_velocity * vel_score;
    return total;
}

double DWAPlanner::obstacleCost(const Eigen::Vector2d& pose) const {
    int gx, gy;
    if (!costmap_.pointToGrid(pose(0), pose(1), gx, gy)) {
        return 1e6; // outside map
    }
    // check footprint circle
    const double r = robot_radius_ + obstacle_margin_;
    const double res = static_cast<double>(costmap_.getResolution());
    const double denom = std::max<double>(res, 1e-3);
    const int cells = std::max(8, (int)std::ceil(2 * M_PI * r / denom));
    for (int i = 0; i < cells; ++i) {
        double a = 2 * M_PI * i / cells;
        double x = pose(0) + r * std::cos(a);
        double y = pose(1) + r * std::sin(a);
        int cx, cy;
        if (!costmap_.pointToGrid(x, y, cx, cy)) return 1e6;
        if (costmap_.getCost(cx, cy) > 100) return 1e6;
    }
    // center cost scaled
    uint8_t c = costmap_.getCost(gx, gy);
    double norm = (double)c / 255.0;
    return norm * norm;
}

double DWAPlanner::pathDistCost(const Eigen::Vector2d& p) const {
    if (reference_path_.empty()) return 0.0;
    double best = std::numeric_limits<double>::infinity();
    for (const auto& rp : reference_path_) {
        double dx = p(0) - rp.first;
        double dy = p(1) - rp.second;
        double d2 = dx*dx + dy*dy;
        if (d2 < best) best = d2;
    }
    return std::sqrt(best);
}

} // namespace model_planner
