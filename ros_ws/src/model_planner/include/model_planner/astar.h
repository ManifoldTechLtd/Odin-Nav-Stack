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

#ifndef ASTAR_H
#define ASTAR_H

#include <vector>
#include <queue>
#include <cmath>
#include <memory>
#include "costmap.h"

namespace model_planner {

/**
 * @struct Node
 * @brief A*算法中的节点
 */
struct Node {
    int x, y;           // 栅格坐标
    float g;            // 从起点到该节点的代价
    float h;            // 从该节点到终点的启发式代价
    float f;            // f = g + h
    std::shared_ptr<Node> parent;

    Node(int x_, int y_, float g_ = 0, float h_ = 0)
        : x(x_), y(y_), g(g_), h(h_), f(g_ + h_), parent(nullptr) {}

    bool operator>(const Node& other) const {
        return f > other.f;
    }
};

/**
 * @class AStar
 * @brief A*路径规划算法实现
 */
class AStar {
public:
    /**
     * @brief 构造函数
     * @param costmap 代价地图
     * @param allow_diagonal 是否允许对角线移动
     * @param obstacle_threshold 障碍物阈值
     */
    AStar(const Costmap& costmap, bool allow_diagonal = true, uint8_t obstacle_threshold = 200);

    /**
     * @brief 规划路径
     * @param start_x 起点X坐标（世界坐标）
     * @param start_y 起点Y坐标（世界坐标）
     * @param goal_x 终点X坐标（世界坐标）
     * @param goal_y 终点Y坐标（世界坐标）
     * @param path 输出路径（世界坐标）
     * @return 是否规划成功
     */
    bool plan(float start_x, float start_y, float goal_x, float goal_y,
              std::vector<std::pair<float, float>>& path);

    /**
     * @brief 获取最后一次规划的代价
     * @return 路径代价
     */
    float getPathCost() const { return path_cost_; }

private:
    const Costmap& costmap_;
    bool allow_diagonal_;
    uint8_t obstacle_threshold_;
    float path_cost_;

    /**
     * @brief 计算启发式代价（欧几里得距离）
     */
    float heuristic(int x1, int y1, int x2, int y2) const;

    /**
     * @brief 获取邻近节点
     */
    std::vector<std::pair<int, int>> getNeighbors(int x, int y) const;

    /**
     * @brief 检查节点是否可通行
     */
    bool isWalkable(int x, int y) const;

    /**
     * @brief 从终点回溯路径
     */
    std::vector<std::pair<float, float>> reconstructPath(const std::shared_ptr<Node>& node);
};

} // namespace model_planner

#endif // ASTAR_H
