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

#ifndef LOCAL_COSTMAP_H
#define LOCAL_COSTMAP_H

#include <vector>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>
#include <ctime>

namespace model_planner {

/**
 * @class LocalCostmap
 * @brief 局部动态代价地图
 * 
 * 功能:
 * - 从激光扫描数据构建局部栅格地图
 * - 支持实时更新
 * - 以全局坐标系为基准，保持障碍物位置固定
 * - 支持衰减记忆：旧数据保留但逐步衰减
 */
class LocalCostmap {
public:
    /**
     * @brief 构造函数
     * @param width 地图宽度（栅格数）
     * @param height 地图高度（栅格数）
     * @param resolution 分辨率（米/栅格）
     * @param origin_x 地图原点X坐标（米）
     * @param origin_y 地图原点Y坐标（米）
     */
    LocalCostmap(int width, int height, float resolution, float origin_x, float origin_y);

    ~LocalCostmap() = default;

    /**
     * @brief 设置当前机器人位置（全局坐标系）
     * @param robot_x 机器人 X 位置（米）
     * @param robot_y 机器人 Y 位置（米）
     * @param robot_yaw 机器人 朝向角（弧度）
     */
    void setRobotPose(float robot_x, float robot_y, float robot_yaw);

    /**
     * @brief 从激光扫描数据更新代价地图
     * @param ranges 激光扫描范围数据
     * @param angle_min 最小角度（弧度）
     * @param angle_max 最大角度（弧度）
     * @param angle_increment 角度增量（弧度）
     * @param range_min 最小距离（米）
     * @param range_max 最大距离（米）
     */
    void updateFromScan(const std::vector<float>& ranges,
                       float angle_min, float angle_max, float angle_increment,
                       float range_min, float range_max);

    /**
     * @brief 获取指定坐标的代价值
     * @param x 栅格X坐标（相对于地图中心）
     * @param y 栅格Y坐标（相对于地图中心）
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
     * @brief 将全局坐标系下的点转换为栅格坐标
     * @param global_x 全局坐标系下的X（米）
     * @param global_y 全局坐标系下的Y（米）
     * @param grid_x 输出栅格X坐标
     * @param grid_y 输出栅格Y坐标
     * @return 是否在地图范围内
     */
    bool globalToGrid(float global_x, float global_y, int& grid_x, int& grid_y) const;

    /**
     * @brief 将机器人坐标系下的点转换为栅格坐标（兼容层）
     * 内部会转换为全局坐标系后再转换为栅格坐标
     * @param robot_x 机器人坐标系下的X（米）
     * @param robot_y 机器人坐标系下的Y（米）
     * @param grid_x 输出栅格X坐标
     * @param grid_y 输出栅格Y坐标
     * @return 是否在地图范围内
     */
    bool pointToGrid(float robot_x, float robot_y, int& grid_x, int& grid_y) const;

    /**
     * @brief 将栅格坐标转换为全局坐标系下的点
     * @param grid_x 栅格X坐标
     * @param grid_y 栅格Y坐标
     * @param global_x 输出全局坐标系下的X（米）
     * @param global_y 输出全局坐标系下的Y（米）
     */
    void gridToGlobal(int grid_x, int grid_y, float& global_x, float& global_y) const;

    /**
     * @brief 将栅格坐标转换为机器人坐标系下的点（兼容层）
     * 内部会先转换为全局坐标系，再转换为机器人坐标系
     * @param grid_x 栅格X坐标
     * @param grid_y 栅格Y坐标
     * @param robot_x 输出机器人坐标系下的X（米）
     * @param robot_y 输出机器人坐标系下的Y（米）
     */
    void gridToPoint(int grid_x, int grid_y, float& robot_x, float& robot_y) const;

    /**
     * @brief 膨胀障碍物
     * @param radius 膨胀半径（米）
     */
    void inflate(float radius);

    /**
     * @brief 设置衰减因子（0-1，越小衰减越快）
     * @param decay_factor 衰减因子，默认0.95
     */
    void setDecayFactor(float decay_factor) { decay_factor_ = decay_factor; }

    /**
     * @brief 获取衰减因子
     */
    float getDecayFactor() const { return decay_factor_; }

    /**
     * @brief 获取当前机器人位置
     */
    void getRobotPose(float& x, float& y, float& yaw) const {
        x = robot_x_;
        y = robot_y_;
        yaw = robot_yaw_;
    }

    /**
     * @brief 应用衰减（每个更新周期调用一次）
     */
    void applyDecay();

    /**
     * @brief 清空地图
     */
    void clearMemory();

    /**
     * @brief 获取地图数据指针
     */
    uint8_t* getData() { return data_.data(); }
    const uint8_t* getData() const { return data_.data(); }

    // Getter
    int getWidth() const { return width_; }
    int getHeight() const { return height_; }
    float getResolution() const { return resolution_; }

private:
    int width_;
    int height_;
    float resolution_;
    std::vector<uint8_t> data_;
    float decay_factor_;  // 衰减因子 (0-1)
    
    // 当前机器人位置（全局坐标系）
    float robot_x_;
    float robot_y_;
    float robot_yaw_;

    /**
     * @brief 检查坐标是否在地图范围内
     */
    bool isInBounds(int x, int y) const;

    /**
     * @brief 获取地图中心坐标
     */
    void getCenterGrid(int& cx, int& cy) const {
        cx = width_ / 2;
        cy = height_ / 2;
    }
};

} // namespace model_planner

#endif // LOCAL_COSTMAP_H
