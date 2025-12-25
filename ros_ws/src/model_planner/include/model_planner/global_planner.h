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

#ifndef GLOBAL_PLANNER_H
#define GLOBAL_PLANNER_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_ros/transform_listener.h>
#include <memory>
#include "costmap.h"
#include "astar.h"

namespace model_planner {

/**
 * @class GlobalPlanner
 * @brief 全局规划器
 * 
 * 功能:
 * - 订阅 /map 话题获取全局地图
 * - 对地图进行膨胀处理
 * - 使用A*算法规划全局路径
 * - 订阅目标点，发布规划的路径
 */
class GlobalPlanner {
public:
    /**
     * @brief 构造函数
     * @param nh ROS节点句柄
     */
    GlobalPlanner(ros::NodeHandle& nh);

    /**
     * @brief 初始化规划器
     * @return 初始化是否成功
     */
    bool initialize();

    /**
     * @brief 规划路径
     * @param start_x 起点X坐标
     * @param start_y 起点Y坐标
     * @param goal_x 终点X坐标
     * @param goal_y 终点Y坐标
     * @param path 输出路径
     * @return 规划是否成功
     */
    bool planPath(float start_x, float start_y, float goal_x, float goal_y,
                  std::vector<std::pair<float, float>>& path);

private:
    ros::NodeHandle nh_;
    ros::Subscriber map_sub_;
    ros::Subscriber goal_sub_;
    ros::Publisher path_pub_;
    ros::Publisher global_costmap_pub_;
    
    std::unique_ptr<Costmap> costmap_;
    std::unique_ptr<AStar> astar_;
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // 参数
    float inflation_radius_;
    std::string map_frame_;
    std::string robot_frame_;

    // 回调函数
    /**
     * @brief 地图回调函数
     */
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    /**
     * @brief 目标点回调函数
     */
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    /**
     * @brief 获取当前机器人位置
     */
    bool getRobotPose(float& x, float& y);
};

} // namespace model_planner

#endif // GLOBAL_PLANNER_H
