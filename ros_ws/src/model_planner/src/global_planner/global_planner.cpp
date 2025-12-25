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

#include "model_planner/global_planner.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ros/ros.h>

namespace model_planner {

GlobalPlanner::GlobalPlanner(ros::NodeHandle& nh)
    : nh_(nh), tf_listener_(tf_buffer_) {
    // 初始化参数
    nh_.param("inflation_radius", inflation_radius_, 0.2f);
    nh_.param("map_frame", map_frame_, std::string("map"));
    nh_.param("robot_frame", robot_frame_, std::string("base_link"));
}

bool GlobalPlanner::initialize() {
    // 订阅地图话题
    map_sub_ = nh_.subscribe("/map", 1, &GlobalPlanner::mapCallback, this);
    
    // 订阅目标点话题
    goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &GlobalPlanner::goalCallback, this);
    
    // 发布规划的路径
    path_pub_ = nh_.advertise<nav_msgs::Path>("/plan", 1);
    // 发布全局代价地图（可视化）
    global_costmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/global_costmap", 1, true);
    
    ROS_INFO("Global Planner initialized");
    return true;
}

bool GlobalPlanner::planPath(float start_x, float start_y, float goal_x, float goal_y,
                             std::vector<std::pair<float, float>>& path) {
    if (!astar_) {
        ROS_WARN("A* planner not initialized");
        return false;
    }
    
    return astar_->plan(start_x, start_y, goal_x, goal_y, path);
}

void GlobalPlanner::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    ROS_INFO("Received map: %d x %d, resolution: %.3f", 
             msg->info.width, msg->info.height, msg->info.resolution);
    
    // 创建代价地图
    costmap_ = std::make_unique<Costmap>(
        msg->info.width, msg->info.height,
        msg->info.resolution,
        msg->info.origin.position.x,
        msg->info.origin.position.y
    );
    
    // 将OccupancyGrid数据转换为Costmap
    for (size_t i = 0; i < msg->data.size(); ++i) {
        int x = i % msg->info.width;
        int y = i / msg->info.width;
        
        // 将 [-1, 100] 映射到 [0, 255]
        int8_t occ_value = msg->data[i];
        uint8_t cost_value = 0;
        
        if (occ_value >= 0) {
            cost_value = static_cast<uint8_t>(occ_value * 2.55f);
        } else {
            cost_value = 0;  // 未知区域视为自由空间
        }
        
        costmap_->setCost(x, y, cost_value);
    }
    
    // 膨胀障碍物
    ROS_INFO("Inflating obstacles with radius: %.3f m", inflation_radius_);
    costmap_->inflate(inflation_radius_);
    
    // 创建A*规划器
    astar_ = std::make_unique<AStar>(*costmap_);
    
    ROS_INFO("Global costmap updated and A* planner created");

    // 发布全局代价地图供RViz显示
    nav_msgs::OccupancyGrid viz;
    viz.header.stamp = ros::Time::now();
    viz.header.frame_id = map_frame_;
    viz.info.width = costmap_->getWidth();
    viz.info.height = costmap_->getHeight();
    viz.info.resolution = costmap_->getResolution();
    viz.info.origin.position.x = costmap_->getOriginX();
    viz.info.origin.position.y = costmap_->getOriginY();
    viz.info.origin.position.z = 0.0;
    viz.info.origin.orientation.w = 1.0;

    viz.data.resize(viz.info.width * viz.info.height);
    const uint8_t* src = costmap_->getData();
    for (size_t i = 0; i < viz.data.size(); ++i) {
        // 将 [0,255] 线性映射到 [0,100]
        viz.data[i] = static_cast<int8_t>(std::round(static_cast<double>(src[i]) * (100.0 / 255.0)));
    }
    global_costmap_pub_.publish(viz);
}

void GlobalPlanner::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (!astar_) {
        ROS_WARN("A* planner not initialized, cannot plan");
        return;
    }
    
    ROS_INFO("Received goal: (%.3f, %.3f)", msg->pose.position.x, msg->pose.position.y);
    
    // 获取当前机器人位置
    float start_x, start_y;
    if (!getRobotPose(start_x, start_y)) {
        ROS_WARN("Cannot get robot pose");
        return;
    }
    
    // 规划路径
    std::vector<std::pair<float, float>> path;
    if (planPath(start_x, start_y, msg->pose.position.x, msg->pose.position.y, path)) {
        ROS_INFO("Path found with %zu waypoints", path.size());
        
        // 发布路径
        nav_msgs::Path path_msg;
        path_msg.header.frame_id = map_frame_;
        path_msg.header.stamp = ros::Time::now();
        
        for (const auto& waypoint : path) {
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = map_frame_;
            pose.header.stamp = ros::Time::now();
            pose.pose.position.x = waypoint.first;
            pose.pose.position.y = waypoint.second;
            pose.pose.position.z = 0;
            pose.pose.orientation.w = 1.0;
            
            path_msg.poses.push_back(pose);
        }
        
        path_pub_.publish(path_msg);
    } else {
        ROS_WARN("Failed to plan path");
    }
}

bool GlobalPlanner::getRobotPose(float& x, float& y) {
    try {
        // 获取从map到base_link的变换
        geometry_msgs::TransformStamped transform = 
            tf_buffer_.lookupTransform(map_frame_, robot_frame_, ros::Time(0));
        
        x = transform.transform.translation.x;
        y = transform.transform.translation.y;
        
        return true;
    } catch (tf2::TransformException& ex) {
        ROS_WARN("Cannot get robot pose: %s", ex.what());
        return false;
    }
}

} // namespace model_planner
