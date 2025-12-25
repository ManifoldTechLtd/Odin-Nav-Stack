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

#include "model_planner/local_planner.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ros/ros.h>

namespace model_planner {

LocalPlanner::LocalPlanner(ros::NodeHandle& nh)
    : nh_(nh), tf_listener_(tf_buffer_),
      current_pose_(Eigen::Vector3d::Zero()),
      current_velocity_(Eigen::Vector3d::Zero()),
      has_reference_path_(false),
      has_scan_(false),
      goal_position_(Eigen::Vector2d::Zero()),
      has_goal_(false),
      goal_arrival_threshold_(0.2f) {
    // 初始化参数
    nh_.param("costmap/width", costmap_width_, 40);
    nh_.param("costmap/height", costmap_height_, 40);
    nh_.param("costmap/resolution", costmap_resolution_, 0.1f);
    nh_.param("inflation_radius", inflation_radius_, 0.3f);
    nh_.param("base_frame", base_frame_, std::string("base_link"));
    nh_.param("scan_frame", scan_frame_, std::string("odin1_base_link"));
}

bool LocalPlanner::initialize() {
    // 创建局部代价地图（初始位置任意，会在spin()中动态更新）
    local_costmap_ = std::make_unique<LocalCostmap>(
        costmap_width_, costmap_height_, costmap_resolution_, 
        0.0f, 0.0f  // 临时初始位置，会在第一次spin()中更新到实际机器人位置
    );
    std::cout << "[INIT] LocalCostmap created with temporary origin (0, 0)" << std::endl;
    std::cout << "[INIT] Actual origin will be set to robot position on first spin()" << std::endl;

    // 读取衰减因子参数
    float decay_factor = 0.92f;
    nh_.param("costmap/decay_factor", decay_factor, decay_factor);
    local_costmap_->setDecayFactor(decay_factor);
    ROS_INFO("Local costmap decay factor set to: %.3f", decay_factor);

    // 创建DWA规划器
    dwa_planner_ = std::make_unique<DWAPlanner>(*local_costmap_);
    dwa_planner_->initialize();

    // 读取并设置DWA规划器参数（带默认值）
    double robot_radius = 0.2;          nh_.param("dwa/robot_radius", robot_radius, robot_radius);
    double obstacle_margin = 0.1;       nh_.param("dwa/obstacle_margin", obstacle_margin, obstacle_margin);

    double max_vel_x = 1.0;             nh_.param("dwa/max_vel_x", max_vel_x, max_vel_x);
    double max_omega = 1.0;             nh_.param("dwa/max_omega", max_omega, max_omega);
    double max_acc_x = 0.5;             nh_.param("dwa/max_acc_x", max_acc_x, max_acc_x);
    double max_alpha = 0.5;             nh_.param("dwa/max_alpha", max_alpha, max_alpha);

    double sim_time = 2.0;              nh_.param("dwa/sim_time", sim_time, sim_time);
    double sim_dt = 0.1;                nh_.param("dwa/sim_dt", sim_dt, sim_dt);
    int v_samples = 6;                  nh_.param("dwa/v_samples", v_samples, v_samples);
    int w_samples = 11;                 nh_.param("dwa/w_samples", w_samples, w_samples);
    bool no_simulation = false;         nh_.param("dwa/no_simulation", no_simulation, no_simulation);
    bool allow_backward = true;         nh_.param("dwa/allow_backward", allow_backward, allow_backward);
    bool enable_rotate_recovery = true; nh_.param("dwa/enable_rotate_recovery", enable_rotate_recovery, enable_rotate_recovery);
    double heading_align_thresh = 0.7;  nh_.param("dwa/heading_align_thresh", heading_align_thresh, heading_align_thresh);
    double heading_boost = 1.5;         nh_.param("dwa/heading_boost", heading_boost, heading_boost);

    double w_clearance = 3.0;           nh_.param("dwa/weights/clearance", w_clearance, w_clearance);
    double w_path = 2.0;                nh_.param("dwa/weights/path", w_path, w_path);
    double w_heading = 1.0;             nh_.param("dwa/weights/heading", w_heading, w_heading);
    double w_velocity = 0.5;            nh_.param("dwa/weights/velocity", w_velocity, w_velocity);

    dwa_planner_->setRobotRadius(robot_radius);
    dwa_planner_->setObstacleMargin(obstacle_margin);
    dwa_planner_->setMaxVelocity(max_vel_x, 0.0, max_omega);
    dwa_planner_->setAcceleration(max_acc_x, 0.0, max_alpha);
    dwa_planner_->setSimTime(sim_time);
    dwa_planner_->setSimDt(sim_dt);
    dwa_planner_->setVelocitySamples(v_samples, w_samples);
    dwa_planner_->setNoSimulation(no_simulation);
    dwa_planner_->setWeights(w_clearance, w_path, w_heading, w_velocity);
    dwa_planner_->setAllowBackward(allow_backward);
    dwa_planner_->setEnableRotateRecovery(enable_rotate_recovery);
    dwa_planner_->setHeadingAlignThresh(heading_align_thresh);
    dwa_planner_->setHeadingBoost(heading_boost);

    // 订阅激光扫描话题
    scan_sub_ = nh_.subscribe("/scan", 1, &LocalPlanner::scanCallback, this);

    // 订阅全局规划的路径
    path_sub_ = nh_.subscribe("/plan", 1, &LocalPlanner::pathCallback, this);

    // 发布速度命令
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    // 发布局部路径和局部代价地图（可视化）
    local_path_pub_ = nh_.advertise<nav_msgs::Path>("/local_plan", 1);
    local_costmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/local_costmap", 1);

    ROS_INFO("Local Planner initialized with memory-based costmap (decay_factor=%.3f)", decay_factor);
    return true;
}

void LocalPlanner::spin() {
    ros::Rate rate(20);
    static int spin_count = 0;
    static bool costmap_initialized = false;

    while (ros::ok()) {
        ros::spinOnce();
        spin_count++;

        // 更新机器人位置
        if (!updateRobotPose()) {
            if (spin_count % 100 == 0) {
                std::cout << "[SPIN] ERROR: Cannot get robot pose" << std::endl;
            }
            rate.sleep();
            continue;
        }
        
        // 首次初始化代价地图中心到机器人位置
        if (!costmap_initialized) {
            std::cout << "[SPIN] Initializing costmap center to robot position: (" 
                      << current_pose_(0) << ", " << current_pose_(1) << ")" << std::endl;
            local_costmap_->setRobotPose(current_pose_(0), current_pose_(1), current_pose_(2));
            costmap_initialized = true;
        } else {
            // 持续更新机器人位置，使地图框跟随机器人
            local_costmap_->setRobotPose(current_pose_(0), current_pose_(1), current_pose_(2));
        }
        
        // 调试信息 每100次打印一次，正常运行注释掉
        // if (spin_count % 100 == 0) {
        //     std::cout << "\n[SPIN #" << spin_count << "] ========== SPIN UPDATE ==========" << std::endl;
        //     std::cout << "[SPIN #" << spin_count << "] Robot Pose (global): (" << current_pose_(0) 
        //               << ", " << current_pose_(1) << ", " << current_pose_(2) << ")" << std::endl;
        //     std::cout << "[SPIN #" << spin_count << "] Costmap center should be at robot position" << std::endl;
        //     std::cout << "[SPIN #" << spin_count << "] has_scan=" << has_scan_ 
        //               << " has_reference_path=" << has_reference_path_ << std::endl;
        // }

        // 执行规划和控制
        if (has_reference_path_ && has_scan_) {
            planAndControl();
        } else if (has_reference_path_ && !has_scan_) {
            ROS_WARN_THROTTLE(2.0, "Waiting for scan data. has_reference_path_=%d, has_scan_=%d", 
                             has_reference_path_, has_scan_);
        }

        rate.sleep();
    }
}

void LocalPlanner::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    static int scan_count = 0;
    scan_count++;
    
    // 调试信息，正常运行注释即可
    // std::cout << "\n[SCAN #" << scan_count << "] Received scan:" << std::endl;
    // std::cout << "  Frame: " << msg->header.frame_id << std::endl;
    // std::cout << "  Ranges: " << msg->ranges.size() << std::endl;
    // std::cout << "  Range limits: [" << msg->range_min << ", " << msg->range_max << "]" << std::endl;
    // std::cout << "  Angle: [" << msg->angle_min << ", " << msg->angle_max 
    //           << "], increment: " << msg->angle_increment << std::endl;
    
    // 统计有效扫描点
    int valid_count = 0;
    for (const auto& r : msg->ranges) {
        if (r >= msg->range_min && r <= msg->range_max && !std::isnan(r) && !std::isinf(r)) {
            valid_count++;
        }
    }
    // 统计有效扫描点数量，正常运行注释掉
    //std::cout << "  Valid ranges: " << valid_count << " / " << msg->ranges.size() << std::endl;
    
    // ========== 第一步：检查扫描数据是否需要坐标变换 ==========
    std::vector<float> ranges_in_base_frame = msg->ranges;
    
    // 如果扫描数据不是在base_frame中，需要进行坐标变换
    if (msg->header.frame_id != base_frame_) {
        ROS_DEBUG("Scan frame '%s' differs from base_frame '%s', attempting transformation",
                  msg->header.frame_id.c_str(), base_frame_.c_str());
        
        try {
            // 查询从扫描框到base_frame的变换
            geometry_msgs::TransformStamped tf_scan_to_base =
                tf_buffer_.lookupTransform(base_frame_, msg->header.frame_id, 
                                         msg->header.stamp, ros::Duration(0.1));
            
            // 提取变换的平移量
            double offset_x = tf_scan_to_base.transform.translation.x;
            double offset_y = tf_scan_to_base.transform.translation.y;
            
            // 提取旋转角
            tf2::Quaternion quat(tf_scan_to_base.transform.rotation.x,
                                tf_scan_to_base.transform.rotation.y,
                                tf_scan_to_base.transform.rotation.z,
                                tf_scan_to_base.transform.rotation.w);
            double roll, pitch, yaw;
            tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
            
            ROS_DEBUG("Scan to base transform: offset=(%.3f, %.3f), yaw=%.3f",
                     offset_x, offset_y, yaw);
            
            // 对每个激光点进行坐标变换
            // 注意：激光扫描是极坐标，需要先转换为笛卡尔坐标
            for (size_t i = 0; i < msg->ranges.size(); ++i) {
                float range = msg->ranges[i];
                
                // 检查范围有效性
                if (range < msg->range_min || range > msg->range_max || 
                    std::isnan(range) || std::isinf(range)) {
                    continue;
                }
                
                // 计算激光的角度
                float angle = msg->angle_min + i * msg->angle_increment;
                
                // 极坐标转笛卡尔坐标（在扫描框中）
                float x_scan = range * std::cos(angle);
                float y_scan = range * std::sin(angle);
                
                // 应用旋转变换
                float x_rotated = x_scan * std::cos(yaw) - y_scan * std::sin(yaw);
                float y_rotated = x_scan * std::sin(yaw) + y_scan * std::cos(yaw);
                
            }
            
            
            ROS_DEBUG("Successfully transformed scan data from '%s' to '%s'",
                     msg->header.frame_id.c_str(), base_frame_.c_str());
        } catch (const tf2::TransformException& ex) {
            ROS_WARN_THROTTLE(1.0, "Failed to transform scan from '%s' to '%s': %s",
                             msg->header.frame_id.c_str(), base_frame_.c_str(), ex.what());
            ROS_WARN_THROTTLE(1.0, "Local costmap will be incorrect! Please check your TF setup.");
            // 继续使用原始扫描数据（可能不正确）
        }
    }
    
    // ========== 第二步：从激光扫描数据更新局部代价地图 ==========
    local_costmap_->updateFromScan(
        msg->ranges,
        msg->angle_min,
        msg->angle_max,
        msg->angle_increment,
        msg->range_min,
        msg->range_max
    );

    // 膨胀障碍物
    local_costmap_->inflate(inflation_radius_);

    has_scan_ = true;
    // 调试信息
    // std::cout << "  Costmap updated, has_scan_=true" << std::endl;

    // 发布局部代价地图
    nav_msgs::OccupancyGrid grid;
    grid.header.stamp = ros::Time::now();
    grid.header.frame_id = "map";
    grid.info.width = local_costmap_->getWidth();
    grid.info.height = local_costmap_->getHeight();
    grid.info.resolution = local_costmap_->getResolution();
    
    // 关键：地图原点应该以机器人为中心
    // 地图中心在栅格坐标 (20, 20)，对应全局坐标 (robot_x, robot_y)
    // 所以地图左下角的全局坐标应该是：
    float map_origin_x = current_pose_(0) - 0.5 * grid.info.width * grid.info.resolution;
    float map_origin_y = current_pose_(1) - 0.5 * grid.info.height * grid.info.resolution;
    
    grid.info.origin.position.x = map_origin_x;
    grid.info.origin.position.y = map_origin_y;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;

    grid.data.resize(grid.info.width * grid.info.height);
    const uint8_t* src = local_costmap_->getData();
    int non_zero_count = 0;
    for (size_t i = 0; i < grid.data.size(); ++i) {
        grid.data[i] = static_cast<int8_t>(std::round(static_cast<double>(src[i]) * (100.0 / 255.0)));
        if (grid.data[i] > 0) non_zero_count++;
    }
    
    static int publish_count = 0;
    if (publish_count++ % 100 == 0) {
        std::cout << "  [PUBLISH] Costmap origin: (" << map_origin_x << ", " << map_origin_y 
                  << ") robot: (" << current_pose_(0) << ", " << current_pose_(1) << ")" << std::endl;
        std::cout << "  [PUBLISH] Non-zero cells: " << non_zero_count << " / " 
                  << grid.data.size() << std::endl;
    }
    local_costmap_pub_.publish(grid);
}

void LocalPlanner::pathCallback(const nav_msgs::Path::ConstPtr& msg) {
    // 将路径转换为本地格式
    reference_path_.clear();
    path_frame_ = msg->header.frame_id;

    for (const auto& pose : msg->poses) {
        reference_path_.push_back({
            static_cast<float>(pose.pose.position.x),
            static_cast<float>(pose.pose.position.y)
        });
    }

    // 提取目标点（路径的最后一个点）
    if (!reference_path_.empty()) {
        goal_position_(0) = reference_path_.back().first;
        goal_position_(1) = reference_path_.back().second;
        has_goal_ = true;
        ROS_INFO("Goal position set to: (%.3f, %.3f)", goal_position_(0), goal_position_(1));
    }

    has_reference_path_ = true;

    ROS_INFO("Received reference path with %zu waypoints", reference_path_.size());
}


bool LocalPlanner::updateRobotPose() {
    static int pose_count = 0;
    pose_count++;
    try {
        const std::string global_frame = "map";
        const std::string robot_frame  = "base_link";

        if (pose_count % 100 == 0) {
            std::cout << "[POSE] Looking up transform from " << global_frame << " to " << robot_frame << std::endl;
        }

        geometry_msgs::TransformStamped tf =
            tf_buffer_.lookupTransform(global_frame, robot_frame, ros::Time(0), ros::Duration(0.1));

        const auto& t = tf.transform.translation;
        const auto& q = tf.transform.rotation;

        current_pose_(0) = t.x;
        current_pose_(1) = t.y;

        tf2::Quaternion quat(q.x, q.y, q.z, q.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        current_pose_(2) = yaw;

        if (pose_count % 100 == 0) {
            std::cout << "[POSE] Got robot pose: (" << current_pose_(0) << ", " 
                      << current_pose_(1) << ", " << current_pose_(2) << ")" << std::endl;
        }
        return true;
    } catch (const tf2::TransformException& ex) {
        if (pose_count % 50 == 0) {
            std::cout << "[POSE] ERROR: " << ex.what() << std::endl;
        }
        return false;
    }
}

void LocalPlanner::planAndControl() {
    ROS_DEBUG_THROTTLE(2.0, "planAndControl called");
    // 规划轨迹
    Eigen::Vector3d cmd_vel;
    
    // ========== 第一步：检查是否到达目标点 ==========
    if (has_goal_) {
        // 计算当前位置到目标点的距离
        double dx = current_pose_(0) - goal_position_(0);
        double dy = current_pose_(1) - goal_position_(1);
        double distance_to_goal = std::sqrt(dx * dx + dy * dy);
        
        if (distance_to_goal < goal_arrival_threshold_) {
            ROS_INFO("Goal reached! Distance: %.3f m", distance_to_goal);
            
            // 停止规划，清空路径
            has_reference_path_ = false;
            has_goal_ = false;
            reference_path_.clear();
            
            // 发布空路径以清空RViz中的可视化
            nav_msgs::Path empty_path;
            empty_path.header.frame_id = path_frame_;
            empty_path.header.stamp = ros::Time::now();
            local_path_pub_.publish(empty_path);
            
            // 发布停止命令
            geometry_msgs::Twist stop_cmd;
            stop_cmd.linear.x = 0;
            stop_cmd.linear.y = 0;
            stop_cmd.angular.z = 0;
            cmd_vel_pub_.publish(stop_cmd);
            
            return;
        }
    }
    
    // ========== 第二步：验证参考路径的坐标系 ==========
    if (path_frame_.empty()) {
        ROS_WARN_THROTTLE(1.0, "Reference path frame is empty, assuming 'map'");
        path_frame_ = "map";
    }
    
    // ========== 第三步：将全局参考路径转换到机器人基座坐标系，供DWA使用 ==========
    std::vector<std::pair<float, float>> reference_path_local;
    reference_path_local.reserve(reference_path_.size());
    bool ref_local_ok = false;
    
    try {
        if (path_frame_ != base_frame_) {
            // 需要进行坐标变换
            geometry_msgs::TransformStamped tf_glb_to_base =
                tf_buffer_.lookupTransform(base_frame_, path_frame_, ros::Time(0), ros::Duration(0.1));
            
            for (const auto& p : reference_path_) {
                geometry_msgs::PoseStamped pin, pout;
                pin.header.frame_id = path_frame_;
                pin.pose.position.x = p.first;
                pin.pose.position.y = p.second;
                pin.pose.orientation.w = 1.0;
                tf2::doTransform(pin, pout, tf_glb_to_base);
                reference_path_local.emplace_back(static_cast<float>(pout.pose.position.x),
                                                 static_cast<float>(pout.pose.position.y));
            }
            ref_local_ok = true;
            ROS_DEBUG("Transformed reference path from '%s' to '%s'", path_frame_.c_str(), base_frame_.c_str());
        } else {
            // 路径已经在base_frame中
            reference_path_local = reference_path_;
            ref_local_ok = true;
            ROS_DEBUG("Reference path already in '%s' frame", base_frame_.c_str());
        }
    } catch (const tf2::TransformException& ex) {
        ROS_WARN_THROTTLE(1.0, "Failed to transform reference path from '%s' to '%s': %s", 
                          path_frame_.c_str(), base_frame_.c_str(), ex.what());
        ref_local_ok = false;
    }

    if (!ref_local_ok) {
        // 无法保证坐标一致性，跳过规划，仅在全局坐标系可视化参考路径
        ROS_WARN_THROTTLE(1.0, "Cannot transform reference path, publishing fallback visualization");
        nav_msgs::Path path_msg;
        path_msg.header.frame_id = path_frame_;  // 使用原始frame
        path_msg.header.stamp = ros::Time::now();
        path_msg.poses.reserve(reference_path_.size());
        for (const auto& p : reference_path_) {
            geometry_msgs::PoseStamped ps;
            ps.header = path_msg.header;
            ps.pose.position.x = p.first;
            ps.pose.position.y = p.second;
            ps.pose.orientation.w = 1.0;
            path_msg.poses.push_back(ps);
        }
        local_path_pub_.publish(path_msg);
        return;
    }

    // ========== 第四步：在基座坐标系中规划 ==========
    // 当前位置视为原点（在base_link系中）
    Eigen::Vector3d current_pose_local = Eigen::Vector3d::Zero();
    if (dwa_planner_->plan(current_pose_local, current_velocity_, reference_path_local, cmd_vel)) {
        // ========== 第四步：将DWA轨迹变换回全局坐标系发布 ==========
        const auto& traj = dwa_planner_->getTrajectory();
        nav_msgs::Path path_msg;
        path_msg.header.frame_id = path_frame_;  // 始终在原始全局frame中发布
        path_msg.header.stamp = ros::Time::now();
        path_msg.poses.reserve(traj.size());

        try {
            if (path_frame_ != base_frame_) {
                // 需要将轨迹从base_frame变换回path_frame
                geometry_msgs::TransformStamped tf_base_to_glb =
                    tf_buffer_.lookupTransform(path_frame_, base_frame_, ros::Time(0), ros::Duration(0.1));
                
                for (const auto& v : traj) {
                    geometry_msgs::PoseStamped pin, pout;
                    pin.header.frame_id = base_frame_;
                    pin.pose.position.x = v.pose(0);
                    pin.pose.position.y = v.pose(1);
                    pin.pose.orientation.w = 1.0;
                    tf2::doTransform(pin, pout, tf_base_to_glb);
                    path_msg.poses.push_back(pout);
                }
                ROS_DEBUG("Transformed DWA trajectory from '%s' to '%s'", base_frame_.c_str(), path_frame_.c_str());
            } else {
                // 轨迹已经在正确的frame中
                for (const auto& v : traj) {
                    geometry_msgs::PoseStamped ps;
                    ps.header = path_msg.header;
                    ps.pose.position.x = v.pose(0);
                    ps.pose.position.y = v.pose(1);
                    ps.pose.orientation.w = 1.0;
                    path_msg.poses.push_back(ps);
                }
            }
        } catch (const tf2::TransformException& ex) {
            ROS_WARN_THROTTLE(1.0, "Failed to transform DWA trajectory from '%s' to '%s': %s",
                              base_frame_.c_str(), path_frame_.c_str(), ex.what());
            // 回退：直接在base_frame发布（这会导致轨迹随机器人移动，需要修复TF）
            ROS_ERROR("CRITICAL: Publishing local_plan in base_frame due to TF failure. "
                     "This is incorrect! Please check your TF setup.");
            path_msg.header.frame_id = base_frame_;
            for (const auto& v : traj) {
                geometry_msgs::PoseStamped ps;
                ps.header = path_msg.header;
                ps.pose.position.x = v.pose(0);
                ps.pose.position.y = v.pose(1);
                ps.pose.orientation.w = 1.0;
                path_msg.poses.push_back(ps);
            }
        }
        local_path_pub_.publish(path_msg);
        // ========== 第五步：发布速度命令 ==========
        geometry_msgs::Twist twist_msg;
        twist_msg.linear.x = cmd_vel(0);
        twist_msg.linear.y = cmd_vel(1);
        twist_msg.angular.z = cmd_vel(2);
        cmd_vel_pub_.publish(twist_msg);
        
        // 更新速度估计（简单的一阶低通滤波）
        current_velocity_ = 0.9 * current_velocity_ + 0.1 * cmd_vel;
    } else {
        // 规划失败，停止机器人
        ROS_WARN_THROTTLE(1.0, "DWA planning failed");
        geometry_msgs::Twist twist_msg;
        twist_msg.linear.x = 0;
        twist_msg.linear.y = 0;
        twist_msg.angular.z = 0;
        cmd_vel_pub_.publish(twist_msg);
    }
}

} // namespace model_planner
