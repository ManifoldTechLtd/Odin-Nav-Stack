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
#include "model_planner/global_planner.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "global_planner_node");
    ros::NodeHandle nh("~");
    
    // 创建全局规划器
    model_planner::GlobalPlanner planner(nh);
    
    // 初始化规划器
    if (!planner.initialize()) {
        ROS_ERROR("Failed to initialize global planner");
        return 1;
    }
    
    // 运行ROS循环
    ros::spin();
    
    return 0;
}
