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

#ifndef COSTMAP_H
#define COSTMAP_H

#include <vector>
#include <cmath>
#include <algorithm>
#include <cstdint>

namespace model_planner {

/**
 * @class Costmap
 * @brief 代价地图类，用于存储和操作栅格地图
 * 
 * 代价值范围: 0-255
 * - 0: 自由空间
 * - 1-254: 膨胀区域
 * - 255: 障碍物
 */
class Costmap {
public:
    /**
     * @brief 构造函数
     * @param width 地图宽度（栅格数）
     * @param height 地图高度（栅格数）
     * @param resolution 分辨率（米/栅格）
     * @param origin_x 原点X坐标（米）
     * @param origin_y 原点Y坐标（米）
     */
    Costmap(int width, int height, float resolution, float origin_x = 0.0f, float origin_y = 0.0f);
    
    ~Costmap() = default;

    // 基本操作
    /**
     * @brief 获取指定坐标的代价值
     * @param x 栅格X坐标
     * @param y 栅格Y坐标
     * @return 代价值 (0-255)
     */
    uint8_t getCost(int x, int y) const;

    /**
     * @brief 设置指定坐标的代价值
     * @param x 栅格X坐标
     * @param y 栅格Y坐标
     * @param cost 代价值
     */
    void setCost(int x, int y, uint8_t cost);

    /**
     * @brief 将世界坐标转换为栅格坐标
     * @param world_x 世界坐标X
     * @param world_y 世界坐标Y
     * @param grid_x 输出栅格坐标X
     * @param grid_y 输出栅格坐标Y
     * @return 是否在地图范围内
     */
    bool worldToGrid(float world_x, float world_y, int& grid_x, int& grid_y) const;

    /**
     * @brief 将栅格坐标转换为世界坐标
     * @param grid_x 栅格坐标X
     * @param grid_y 栅格坐标Y
     * @param world_x 输出世界坐标X
     * @param world_y 输出世界坐标Y
     */
    void gridToWorld(int grid_x, int grid_y, float& world_x, float& world_y) const;

    /**
     * @brief 膨胀障碍物
     * @param radius 膨胀半径（米）
     * @param obstacle_threshold 障碍物阈值（代价值 >= 此值被认为是障碍物）
     */
    void inflate(float radius, uint8_t obstacle_threshold = 200);

    /**
     * @brief 清空地图
     */
    void clear();

    /**
     * @brief 获取地图数据指针
     * @return 地图数据指针
     */
    uint8_t* getData() { return data_.data(); }
    const uint8_t* getData() const { return data_.data(); }

    // Getter
    int getWidth() const { return width_; }
    int getHeight() const { return height_; }
    float getResolution() const { return resolution_; }
    float getOriginX() const { return origin_x_; }
    float getOriginY() const { return origin_y_; }

private:
    int width_;
    int height_;
    float resolution_;
    float origin_x_;
    float origin_y_;
    std::vector<uint8_t> data_;

    /**
     * @brief 检查坐标是否在地图范围内
     */
    bool isInBounds(int x, int y) const;
};

} // namespace model_planner

#endif // COSTMAP_H
