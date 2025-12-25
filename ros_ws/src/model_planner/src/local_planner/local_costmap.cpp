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

#include "model_planner/local_costmap.h"
#include <cmath>
#include <algorithm>
#include <ros/ros.h>

namespace model_planner {

LocalCostmap::LocalCostmap(int width, int height, float resolution, float origin_x, float origin_y)
    : width_(width), height_(height), resolution_(resolution),
      data_(width * height, 0), decay_factor_(0.95f),
      robot_x_(origin_x), robot_y_(origin_y), robot_yaw_(0.0f) {}

void LocalCostmap::setRobotPose(float robot_x, float robot_y, float robot_yaw) {
    robot_x_ = robot_x;
    robot_y_ = robot_y;
    robot_yaw_ = robot_yaw;
}

void LocalCostmap::updateFromScan(const std::vector<float>& ranges,
                                  float angle_min, float angle_max, float angle_increment,
                                  float range_min, float range_max) {
    // 首先应用衰减（保留旧数据但降低代价值）
    applyDecay();
    
    // 调试输出，正常运行时注释掉
    static int update_count = 0;
    // update_count++;
    // if (update_count % 50 == 0) {
    //     std::cout << "[LocalCostmap] ===== Update #" << update_count << " =====" << std::endl;
    //     std::cout << "[LocalCostmap] Robot Pose: x=" << robot_x_ << ", y=" << robot_y_ 
    //               << ", yaw=" << robot_yaw_ << " rad (" << (robot_yaw_ * 180 / 3.14159) << " deg)" << std::endl;
    //     std::cout << "[LocalCostmap] Map size: " << width_ << " x " << height_ 
    //               << ", resolution: " << resolution_ << " m/cell" << std::endl;
    // }
    
    // 整个流程：
    // 1. 激光扫描是在机器人坐标系中（极坐标）
    // 2. 转换为全局坐标系（笛卡尔坐标）
    // 3. 再转换为栅格坐标
    
    int obstacle_count = 0;
    // 遍历所有激光扫描数据
    for (size_t i = 0; i < ranges.size(); ++i) {
        float range = ranges[i];
        
        // 检查范围有效性
        if (range < range_min || range > range_max || std::isnan(range) || std::isinf(range)) {
            continue;
        }
        
        // 计算激光的角度
        float angle = angle_min + i * angle_increment;
        
        // 将极坐标转换为笛卡尔坐标（机器人坐标系）
        float x_robot = range * std::cos(angle);
        float y_robot = range * std::sin(angle);
        
        // 转换为全局坐标系
        float x_global = x_robot * std::cos(robot_yaw_) - y_robot * std::sin(robot_yaw_) + robot_x_;
        float y_global = x_robot * std::sin(robot_yaw_) + y_robot * std::cos(robot_yaw_) + robot_y_;
        
        // 转换为栅格坐标
        int grid_x, grid_y;
        if (globalToGrid(x_global, y_global, grid_x, grid_y)) {
            // 标记为障碍物
            setCost(grid_x, grid_y, 255);
            obstacle_count++;
            
            // 调试输出，正常运行时注释掉
            // if (update_count % 50 == 0 && obstacle_count <= 5) {
            //     std::cout << "[LocalCostmap] Obstacle " << obstacle_count << ": robot(" << x_robot << "," << y_robot 
            //               << ") -> global(" << x_global << "," << y_global << ") -> grid(" << grid_x << "," << grid_y << ")" << std::endl;
            // }
            
            // 在激光束路径上标记为自由空间（Bresenham线算法）
            int grid_robot_x, grid_robot_y;
            if (globalToGrid(robot_x_, robot_y_, grid_robot_x, grid_robot_y)) {
                int start_x = grid_robot_x;
                int start_y = grid_robot_y;
                int end_x = grid_x;
                int end_y = grid_y;
            
            // Bresenham线算法
            int dx = std::abs(end_x - start_x);
            int dy = std::abs(end_y - start_y);
            int sx = (end_x > start_x) ? 1 : -1;
            int sy = (end_y > start_y) ? 1 : -1;
            int err = dx - dy;
            
            int x = start_x;
            int y = start_y;
            
                while (true) {
                    // 标记为自由空间（但不覆盖已标记的障碍物）
                    if (getCost(x, y) < 200) {
                        setCost(x, y, 0);
                    }
                    
                    if (x == end_x && y == end_y) {
                        break;
                    }
                    
                    int e2 = 2 * err;
                    if (e2 > -dy) {
                        err -= dy;
                        x += sx;
                    }
                    if (e2 < dx) {
                        err += dx;
                        y += sy;
                    }
                }
            }
        }
    }
    
    // 调试输出，正常运行时注释掉
    // if (update_count % 50 == 0) {
    //     std::cout << "[LocalCostmap] Found " << obstacle_count << " obstacles in this scan" << std::endl;
    // }
}

void LocalCostmap::clearMemory() {
    std::fill(data_.begin(), data_.end(), 0);
}

void LocalCostmap::applyDecay() {
    // 对所有栅格应用衰减因子
    for (auto& cost : data_) {
        if (cost > 0) {
            // 衰减代价值（但保留一定的记忆）
            cost = static_cast<uint8_t>(cost * decay_factor_);
        }
    }
}

uint8_t LocalCostmap::getCost(int x, int y) const {
    if (!isInBounds(x, y)) {
        return 0;
    }
    return data_[y * width_ + x];
}

void LocalCostmap::setCost(int x, int y, uint8_t cost) {
    if (!isInBounds(x, y)) {
        return;
    }
    data_[y * width_ + x] = cost;
}

bool LocalCostmap::globalToGrid(float global_x, float global_y, int& grid_x, int& grid_y) const {
    // 获取地图中心
    int center_x = width_ / 2;
    int center_y = height_ / 2;
    
    // 相对于机器人位置的坐标差
    float dx = global_x - robot_x_;
    float dy = global_y - robot_y_;
    
    // 转换为栅格坐标（以机器人为中心）
    grid_x = center_x + static_cast<int>(dx / resolution_);
    grid_y = center_y + static_cast<int>(dy / resolution_);
    
    bool in_bounds = isInBounds(grid_x, grid_y);
    
    // 调试输出，正常运行时注释掉
    // static int debug_count = 0;
    // if (debug_count++ % 500 == 0) {
    //     std::cout << "[GlobalToGrid] global(" << global_x << "," << global_y 
    //               << ") robot(" << robot_x_ << "," << robot_y_ 
    //               << ") delta(" << dx << "," << dy 
    //               << ") -> grid(" << grid_x << "," << grid_y 
    //               << ") [" << (in_bounds ? "IN" : "OUT") << "]" << std::endl;
    // }
    
    return in_bounds;
}

void LocalCostmap::gridToGlobal(int grid_x, int grid_y, float& global_x, float& global_y) const {
    // 获取地图中心
    int center_x = width_ / 2;
    int center_y = height_ / 2;
    
    // 相对于中心的栅格偏移
    float dx = (grid_x - center_x + 0.5f) * resolution_;
    float dy = (grid_y - center_y + 0.5f) * resolution_;
    
    // 转换为全局坐标系（以机器人为中心）
    global_x = robot_x_ + dx;
    global_y = robot_y_ + dy;
}

bool LocalCostmap::pointToGrid(float robot_x, float robot_y, int& grid_x, int& grid_y) const {
    // 兼容层：将机器人坐标系转换为全局坐标系
    float cos_yaw = std::cos(robot_yaw_);
    float sin_yaw = std::sin(robot_yaw_);
    float global_x = robot_x_ + robot_x * cos_yaw - robot_y * sin_yaw;
    float global_y = robot_y_ + robot_x * sin_yaw + robot_y * cos_yaw;
    
    // 调试输出，正常运行时注释掉
    // static int debug_count = 0;
    // if (debug_count++ % 500 == 0) {
    //     ROS_INFO("[PointToGrid] robot(%.3f,%.3f) + pose(%.3f,%.3f,%.3f) -> global(%.3f,%.3f)",
    //              robot_x, robot_y, robot_x_, robot_y_, robot_yaw_, global_x, global_y);
    // }
    
    // 转换为栅格坐标
    return globalToGrid(global_x, global_y, grid_x, grid_y);
}

void LocalCostmap::gridToPoint(int grid_x, int grid_y, float& robot_x, float& robot_y) const {
    // 兼容层：将栅格坐标转换为全局坐标系，再转换为机器人坐标系
    float global_x, global_y;
    gridToGlobal(grid_x, grid_y, global_x, global_y);
    
    // 转换为机器人坐标系
    float dx = global_x - robot_x_;
    float dy = global_y - robot_y_;
    float cos_yaw = std::cos(robot_yaw_);
    float sin_yaw = std::sin(robot_yaw_);
    robot_x = dx * cos_yaw + dy * sin_yaw;
    robot_y = -dx * sin_yaw + dy * cos_yaw;
}

void LocalCostmap::inflate(float radius) {
    // 膨胀半径转换为栅格数
    int inflate_radius = static_cast<int>(std::ceil(radius / resolution_));
    
    // 创建临时地图用于膨胀
    std::vector<uint8_t> inflated_data = data_;
    
    // 遍历所有栅格
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            // 如果当前栅格是障碍物
            if (data_[y * width_ + x] >= 200) {
                // 膨胀周围栅格
                for (int dy = -inflate_radius; dy <= inflate_radius; ++dy) {
                    for (int dx = -inflate_radius; dx <= inflate_radius; ++dx) {
                        int nx = x + dx;
                        int ny = y + dy;
                        
                        if (isInBounds(nx, ny)) {
                            // 计算距离
                            int dist_sq = dx * dx + dy * dy;
                            int radius_sq = inflate_radius * inflate_radius;
                            
                            if (dist_sq <= radius_sq) {
                                // 根据距离设置代价值
                                int dist = static_cast<int>(std::sqrt(dist_sq));
                                uint8_t cost = static_cast<uint8_t>(
                                    254 * (1.0f - static_cast<float>(dist) / inflate_radius)
                                );
                                inflated_data[ny * width_ + nx] = 
                                    std::max(inflated_data[ny * width_ + nx], cost);
                            }
                        }
                    }
                }
            }
        }
    }
    
    data_ = inflated_data;
}

bool LocalCostmap::isInBounds(int x, int y) const {
    return x >= 0 && x < width_ && y >= 0 && y < height_;
}

} // namespace model_planner
