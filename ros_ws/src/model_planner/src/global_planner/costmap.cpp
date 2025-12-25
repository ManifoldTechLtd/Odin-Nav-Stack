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

#include "model_planner/costmap.h"

namespace model_planner {

Costmap::Costmap(int width, int height, float resolution, float origin_x, float origin_y)
    : width_(width), height_(height), resolution_(resolution),
      origin_x_(origin_x), origin_y_(origin_y),
      data_(width * height, 0) {}

uint8_t Costmap::getCost(int x, int y) const {
    if (!isInBounds(x, y)) {
        return 0;
    }
    return data_[y * width_ + x];
}

void Costmap::setCost(int x, int y, uint8_t cost) {
    if (!isInBounds(x, y)) {
        return;
    }
    data_[y * width_ + x] = cost;
}

bool Costmap::worldToGrid(float world_x, float world_y, int& grid_x, int& grid_y) const {
    // 将世界坐标转换为相对于原点的坐标
    float rel_x = world_x - origin_x_;
    float rel_y = world_y - origin_y_;
    
    // 转换为栅格坐标
    grid_x = static_cast<int>(rel_x / resolution_);
    grid_y = static_cast<int>(rel_y / resolution_);
    
    return isInBounds(grid_x, grid_y);
}

void Costmap::gridToWorld(int grid_x, int grid_y, float& world_x, float& world_y) const {
    // 将栅格坐标转换为世界坐标
    world_x = origin_x_ + (grid_x + 0.5f) * resolution_;
    world_y = origin_y_ + (grid_y + 0.5f) * resolution_;
}

void Costmap::inflate(float radius, uint8_t obstacle_threshold) {
    // 膨胀半径转换为栅格数
    int inflate_radius = static_cast<int>(std::ceil(radius / resolution_));
    
    // 创建临时地图用于膨胀
    std::vector<uint8_t> inflated_data = data_;
    
    // 遍历所有栅格
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            // 如果当前栅格是障碍物
            if (data_[y * width_ + x] >= obstacle_threshold) {
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

void Costmap::clear() {
    std::fill(data_.begin(), data_.end(), 0);
}

bool Costmap::isInBounds(int x, int y) const {
    return x >= 0 && x < width_ && y >= 0 && y < height_;
}

} // namespace model_planner
