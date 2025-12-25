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

#include <ros/ros.h>
#include "model_planner/astar.h"
#include <queue>
#include <set>
#include <cmath>

namespace model_planner {

AStar::AStar(const Costmap& costmap, bool allow_diagonal, uint8_t obstacle_threshold)
    : costmap_(costmap), allow_diagonal_(allow_diagonal),
      obstacle_threshold_(obstacle_threshold), path_cost_(0) {}

bool AStar::plan(float start_x, float start_y, float goal_x, float goal_y,
                 std::vector<std::pair<float, float>>& path) {
    path.clear();
    path_cost_ = 0;

    // 将世界坐标转换为栅格坐标
    int start_gx, start_gy, goal_gx, goal_gy;
    if (!costmap_.worldToGrid(start_x, start_y, start_gx, start_gy)) {
        ROS_WARN("Start position out of bounds");
        return false;
    }
    if (!costmap_.worldToGrid(goal_x, goal_y, goal_gx, goal_gy)) {
        ROS_WARN("Goal position out of bounds");
        return false;
    }

    // 检查起点和终点是否可通行
    if (!isWalkable(start_gx, start_gy) || !isWalkable(goal_gx, goal_gy)) {
        ROS_WARN("Start or goal is not walkable");
        return false;
    }

    // 优先队列：存储待处理节点
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_set;
    
    // 已访问集合
    std::set<std::pair<int, int>> closed_set;
    
    // 创建起点节点
    Node start_node(start_gx, start_gy, 0, heuristic(start_gx, start_gy, goal_gx, goal_gy));
    open_set.push(start_node);

    std::shared_ptr<Node> current = nullptr;
    
    // A*主循环
    while (!open_set.empty()) {
        // 取出代价最小的节点
        Node current_node = open_set.top();
        open_set.pop();
        
        current = std::make_shared<Node>(current_node);

        // 检查是否到达终点
        if (current->x == goal_gx && current->y == goal_gy) {
            path = reconstructPath(current);
            path_cost_ = current->g;
            return true;
        }

        // 检查是否已访问
        if (closed_set.count({current->x, current->y})) {
            continue;
        }
        closed_set.insert({current->x, current->y});

        // 获取邻近节点
        auto neighbors = getNeighbors(current->x, current->y);
        
        for (const auto& neighbor : neighbors) {
            int nx = neighbor.first;
            int ny = neighbor.second;

            // 跳过已访问的节点
            if (closed_set.count({nx, ny})) {
                continue;
            }

            // 跳过不可通行的节点
            if (!isWalkable(nx, ny)) {
                continue;
            }

            // 计算到邻近节点的代价
            float dx = nx - current->x;
            float dy = ny - current->y;
            float move_cost = std::sqrt(dx * dx + dy * dy);
            
            // 添加地形代价
            uint8_t cell_cost = costmap_.getCost(nx, ny);
            move_cost *= (1.0f + cell_cost / 255.0f);
            
            float new_g = current->g + move_cost;
            float h = heuristic(nx, ny, goal_gx, goal_gy);
            
            // 创建邻近节点
            Node neighbor_node(nx, ny, new_g, h);
            neighbor_node.parent = current;
            
            open_set.push(neighbor_node);
        }
    }

    ROS_WARN("No path found");
    return false;
}

float AStar::heuristic(int x1, int y1, int x2, int y2) const {
    // 欧几里得距离启发式
    float dx = x2 - x1;
    float dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

std::vector<std::pair<int, int>> AStar::getNeighbors(int x, int y) const {
    std::vector<std::pair<int, int>> neighbors;
    
    // 四方向邻近
    int dx[] = {0, 1, 0, -1};
    int dy[] = {1, 0, -1, 0};
    
    for (int i = 0; i < 4; ++i) {
        int nx = x + dx[i];
        int ny = y + dy[i];
        if (costmap_.getWidth() > nx && nx >= 0 && costmap_.getHeight() > ny && ny >= 0) {
            neighbors.push_back({nx, ny});
        }
    }
    
    // 对角线邻近
    if (allow_diagonal_) {
        int dx_diag[] = {1, 1, -1, -1};
        int dy_diag[] = {1, -1, 1, -1};
        
        for (int i = 0; i < 4; ++i) {
            int nx = x + dx_diag[i];
            int ny = y + dy_diag[i];
            if (costmap_.getWidth() > nx && nx >= 0 && costmap_.getHeight() > ny && ny >= 0) {
                neighbors.push_back({nx, ny});
            }
        }
    }
    
    return neighbors;
}

bool AStar::isWalkable(int x, int y) const {
    if (x < 0 || x >= costmap_.getWidth() || y < 0 || y >= costmap_.getHeight()) {
        return false;
    }
    return costmap_.getCost(x, y) < obstacle_threshold_;
}

std::vector<std::pair<float, float>> AStar::reconstructPath(const std::shared_ptr<Node>& node) {
    std::vector<std::pair<float, float>> path;
    
    std::shared_ptr<Node> current = node;
    while (current != nullptr) {
        float world_x, world_y;
        costmap_.gridToWorld(current->x, current->y, world_x, world_y);
        path.push_back({world_x, world_y});
        current = current->parent;
    }
    
    // 反转路径使其从起点到终点
    std::reverse(path.begin(), path.end());
    
    return path;
}

} // namespace model_planner
