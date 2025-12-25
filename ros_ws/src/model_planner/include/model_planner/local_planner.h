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

#ifndef LOCAL_PLANNER_H
#define LOCAL_PLANNER_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <memory>
#include "local_costmap.h"
#include "dwa_planner.h"

namespace model_planner {

/**
 * @class LocalPlanner
 * @brief 局部规划器
 * 
 * 功能:
 * - 订阅 /scan 话题获取激光扫描数据
 * - 构建实时局部代价地图
 * - 使用TEB方法进行局部规划
 * - 发布速度命令 /cmd_vel
 */
class LocalPlanner {
public:
    /**
     * @brief 构造函数
     * @param nh ROS节点句柄
     */
    LocalPlanner(ros::NodeHandle& nh);

    /**
     * @brief 初始化规划器
     * @return 初始化是否成功
     */
    bool initialize();

    /**
     * @brief 主循环处理函数
     */
    void spin();

private:
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;
    ros::Subscriber path_sub_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher local_path_pub_;
    ros::Publisher local_costmap_pub_;
    
    std::unique_ptr<LocalCostmap> local_costmap_;
    std::unique_ptr<DWAPlanner> dwa_planner_;
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // 参数
    int costmap_width_;
    int costmap_height_;
    float costmap_resolution_;
    float inflation_radius_;
    std::string base_frame_;
    std::string scan_frame_;

    // 状态
    std::vector<std::pair<float, float>> reference_path_;
    Eigen::Vector3d current_pose_;
    Eigen::Vector3d current_velocity_;
    bool has_reference_path_;
    bool has_scan_;
    std::string path_frame_;
    
    // 目标点信息
    Eigen::Vector2d goal_position_;
    bool has_goal_;
    float goal_arrival_threshold_;  // 到达目标点的距离阈值（米）

    // 回调函数
    /**
     * @brief 激光扫描回调函数
     */
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

    /**
     * @brief 参考路径回调函数
     */
    void pathCallback(const nav_msgs::Path::ConstPtr& msg);

    /**
     * @brief 更新机器人位置
     */
    bool updateRobotPose();

    /**
     * @brief 执行规划和控制循环
     */
    void planAndControl();
};

} // namespace model_planner

#endif // LOCAL_PLANNER_H
