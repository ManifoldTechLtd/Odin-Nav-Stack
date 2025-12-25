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

#ifndef DWA_PLANNER_H
#define DWA_PLANNER_H

#include <vector>
#include <Eigen/Dense>
#include <memory>
#include "local_costmap.h"

namespace model_planner {

struct DWATrajPoint {
    Eigen::Vector2d pose;  // x, y in base frame
    double theta;          // yaw
    double time;           // time from start
    DWATrajPoint(double x=0, double y=0, double t=0, double ti=0) : pose(x,y), theta(t), time(ti) {}
};

class DWAPlanner {
public:
    explicit DWAPlanner(const LocalCostmap& costmap);

    void initialize();

    // plan returns true if a feasible command is found
    bool plan(const Eigen::Vector3d& current_pose,
              const Eigen::Vector3d& current_velocity,
              const std::vector<std::pair<float, float>>& reference_path,
              Eigen::Vector3d& cmd_vel);

    const std::vector<DWATrajPoint>& getTrajectory() const { return best_traj_; }

    // limits
    void setMaxVelocity(double vx_max, double vy_max, double omega_max) {
        vx_max_ = vx_max; vy_max_ = vy_max; omega_max_ = omega_max;
    }
    void setAcceleration(double ax_max, double ay_max, double alpha_max) {
        ax_max_ = ax_max; ay_max_ = ay_max; alpha_max_ = alpha_max;
    }
    void setRobotRadius(double r) { robot_radius_ = r; }
    void setObstacleMargin(double m) { obstacle_margin_ = m; }

    // tuning
    void setSimTime(double T) { sim_time_ = T; }
    void setSimDt(double dt) { dt_ = dt; }
    void setVelocitySamples(int nv, int nw) { v_samples_ = nv; w_samples_ = nw; }
    void setNoSimulation(bool v) { no_simulation_ = v; }
    void setAllowBackward(bool v) { allow_backward_ = v; }
    void setHeadingAlignThresh(double v) { heading_align_thresh_ = v; }
    void setHeadingBoost(double v) { heading_boost_ = v; }
    void setEnableRotateRecovery(bool v) { enable_rotate_recovery_ = v; }

    // weights
    void setWeights(double w_clearance, double w_path, double w_heading, double w_velocity) {
        w_clearance_ = w_clearance; w_path_ = w_path; w_heading_ = w_heading; w_velocity_ = w_velocity;
    }

private:
    const LocalCostmap& costmap_;

    // params
    double vx_max_{1.0};
    double vy_max_{0.0};
    double omega_max_{1.0};
    double ax_max_{0.5};
    double ay_max_{0.0};
    double alpha_max_{0.5};
    double robot_radius_{0.2};
    double obstacle_margin_{0.1};

    double sim_time_{2.0};
    double dt_{0.1};
    int v_samples_{5};
    int w_samples_{9};
    bool no_simulation_{false};
    bool allow_backward_{true};
    bool enable_rotate_recovery_{true};
    double heading_align_thresh_{0.7}; // rad
    double heading_boost_{1.5};

    std::vector<std::pair<float,float>> reference_path_;
    std::vector<DWATrajPoint> best_traj_;

    // weight params
    double w_clearance_{3.0};
    double w_path_{2.0};
    double w_heading_{1.0};
    double w_velocity_{0.5};

    // helpers
    bool simulateTrajectory(double v, double w, std::vector<DWATrajPoint>& out) const;
    double scoreTrajectory(const std::vector<DWATrajPoint>& traj) const;
    double obstacleCost(const Eigen::Vector2d& p) const;
    double pathDistCost(const Eigen::Vector2d& p) const;
};

} // namespace model_planner

#endif // DWA_PLANNER_H
