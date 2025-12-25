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

/*
Relocalize from Rviz given initial pose.
Crop an area from global point cloud
Perform ICP between current point cloud and croped point cloud
*/

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/TransformStamped.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <algorithm>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <string>
#include <vector>
#include <limits>

class RelocalizationNode
{
private:
    ros::NodeHandle nh_;
    ros::Publisher grid_map_pub_;
    ros::Publisher pointcloud_pub_;
    ros::Subscriber cloud_slam_sub_;
    ros::Subscriber pose_estimate_sub_;
    ros::Timer grid_map_timer_;
    ros::Timer pointcloud_timer_;
    ros::Timer tf_timer_;
    
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    geometry_msgs::TransformStamped map_to_odom_transform_;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr slam_cloud_base_;
    nav_msgs::OccupancyGrid grid_map_;
    sensor_msgs::PointCloud2 cloud_msg_;
    
    // Parameters
    double min_height_;
    double max_height_;
    double resolution_;
    double voxel_size_;
    double local_crop_size_;
    double crop_x_min_, crop_x_max_;
    double crop_y_min_, crop_y_max_;
    std::string map_file_path_;
    std::string default_map_filename_;
    
    // ICP Parameters
    int max_icp_iterations_;
    double transformation_epsilon_;
    double euclidean_fitness_epsilon_;
    double max_correspondence_distance_;
    double lambda_initial_;  // LM algorithm parameter
    
public:
    RelocalizationNode() : nh_("~"), tf_listener_(tf_buffer_)
    {
        // Initialize point clouds
        cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        slam_cloud_base_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        
        // Get parameters
        nh_.param<double>("min_height", min_height_, -0.5);
        nh_.param<double>("max_height", max_height_, 2.0);
        nh_.param<double>("resolution", resolution_, 0.05);
        nh_.param<double>("voxel_size", voxel_size_, 0.1);
        nh_.param<double>("local_crop_size", local_crop_size_, 3.0);
        nh_.param<double>("crop_x_min", crop_x_min_, 0.0);
        nh_.param<double>("crop_x_max", crop_x_max_, 4.0);
        nh_.param<double>("crop_y_min", crop_y_min_, -3.0);
        nh_.param<double>("crop_y_max", crop_y_max_, 3.0);
        nh_.param<std::string>("map_file_path", map_file_path_, "");
        nh_.param<std::string>("default_map_filename", default_map_filename_, "default_map.pcd");
        
        // ICP parameters
        nh_.param<int>("max_icp_iterations", max_icp_iterations_, 50);
        nh_.param<double>("transformation_epsilon", transformation_epsilon_, 1e-6);
        nh_.param<double>("euclidean_fitness_epsilon", euclidean_fitness_epsilon_, 1e-6);
        nh_.param<double>("max_correspondence_distance", max_correspondence_distance_, 1.0);
        nh_.param<double>("lambda_initial", lambda_initial_, 0.01);
        
        // Publishers
        grid_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);
        pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/map_pointcloud", 1);
        
        // Subscribers
        // cloud_slam_sub_ = nh_.subscribe("/odin1/cloud_slam", 1, &RelocalizationNode::cloudSlamCallback, this);
        // pose_estimate_sub_ = nh_.subscribe("/initialpose", 1, &RelocalizationNode::poseEstimateCallback, this);
        
        // Initialize map to odom transform (all zeros by default)
        initializeMapToOdomTransform();
        
        // Load point cloud
        if (loadPointCloud())
        {
            // Create grid map
            createGridMap();
            
            // Timers
            grid_map_timer_ = nh_.createTimer(ros::Duration(0.1), &RelocalizationNode::publishGridMap, this); // 10Hz
            pointcloud_timer_ = nh_.createTimer(ros::Duration(1.0), &RelocalizationNode::publishPointCloud, this); // 1Hz
            tf_timer_ = nh_.createTimer(ros::Duration(0.01), &RelocalizationNode::publishTransform, this); // 100Hz odom frequency
            
            ROS_INFO("Relocalization node initialized successfully");
            ROS_INFO("Loaded %zu points from %s", cloud_->size(), map_file_path_.c_str());
            ROS_INFO("Grid map size: %d x %d, resolution: %.3f", grid_map_.info.width, grid_map_.info.height, resolution_);
        }
        else
        {
            ROS_ERROR("Failed to load point cloud. Node will not function properly.");
        }
    }
    
    bool loadPointCloud()
    {
        // If no specific path provided, use default filename in maps folder
        if (map_file_path_.empty())
        {
            std::string package_path = ros::package::getPath("pcd2pgm");
            if (package_path.empty())
            {
                ROS_ERROR("Could not find package pcd2pgm");
                return false;
            }
            map_file_path_ = package_path + "/maps/" + default_map_filename_;
        }
        
        ROS_INFO("Loading point cloud from: %s", map_file_path_.c_str());
        
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_file_path_, *cloud_) == -1)
        {
            ROS_ERROR("Could not read file %s", map_file_path_.c_str());
            return false;
        }
        
        // Remove NaN points
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud_, *cloud_, indices);
        
        ROS_INFO("Successfully loaded %zu points", cloud_->size());
        return true;
    }
    
    void createGridMap()
    {
        if (cloud_->empty())
        {
            ROS_ERROR("Point cloud is empty, cannot create grid map");
            return;
        }
        
        // Find point cloud bounds
        float min_x = std::numeric_limits<float>::max();
        float max_x = std::numeric_limits<float>::lowest();
        float min_y = std::numeric_limits<float>::max();
        float max_y = std::numeric_limits<float>::lowest();
        
        for (const auto& point : cloud_->points)
        {
            if (point.z >= min_height_ && point.z <= max_height_)
            {
                min_x = std::min(min_x, point.x);
                max_x = std::max(max_x, point.x);
                min_y = std::min(min_y, point.y);
                max_y = std::max(max_y, point.y);
            }
        }
        
        // Add some padding
        float padding = 1.0;
        min_x -= padding;
        min_y -= padding;
        max_x += padding;
        max_y += padding;
        
        // Calculate grid dimensions
        int width = static_cast<int>((max_x - min_x) / resolution_) + 1;
        int height = static_cast<int>((max_y - min_y) / resolution_) + 1;
        
        // Initialize grid map
        grid_map_.header.frame_id = "map";
        grid_map_.info.resolution = resolution_;
        grid_map_.info.width = width;
        grid_map_.info.height = height;
        grid_map_.info.origin.position.x = min_x;
        grid_map_.info.origin.position.y = min_y;
        grid_map_.info.origin.position.z = 0.0;
        grid_map_.info.origin.orientation.w = 1.0;
        
        // Initialize all cells as unknown (-1)
        grid_map_.data.resize(width * height, -1);
        
        // Fill grid map
        std::vector<std::vector<bool>> occupied(height, std::vector<bool>(width, false));
        std::vector<std::vector<bool>> has_point(height, std::vector<bool>(width, false));
        for (const auto& point : cloud_->points)
        {
            
                int grid_x = static_cast<int>((point.x - min_x) / resolution_);
                int grid_y = static_cast<int>((point.y - min_y) / resolution_);
                
                if (grid_x >= 0 && grid_x < width && grid_y >= 0 && grid_y < height)
                {
                    has_point[grid_y][grid_x] = true;
                }
        }
        for (const auto& point : cloud_->points)
        {
            if (point.z >= min_height_ && point.z <= max_height_)
            {
                int grid_x = static_cast<int>((point.x - min_x) / resolution_);
                int grid_y = static_cast<int>((point.y - min_y) / resolution_);
                
                if (grid_x >= 0 && grid_x < width && grid_y >= 0 && grid_y < height)
                {
                    occupied[grid_y][grid_x] = true;
                }
            }
        }
        
        // Convert to occupancy grid format
        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
                int index = y * width + x;
                if (occupied[y][x])
                {
                    grid_map_.data[index] = 100; // Occupied
                }
                else
                {
                    if(has_point[y][x])
                    {grid_map_.data[index] = 0; }
                    else{
                        grid_map_.data[index] = -1; // Unknown
                    }
                }
            }
        }
        
        ROS_INFO("Grid map created: %d x %d cells, origin: (%.2f, %.2f)", 
                 width, height, min_x, min_y);
    }
    
    void publishGridMap(const ros::TimerEvent&)
    {
        grid_map_.header.stamp = ros::Time::now();
        grid_map_pub_.publish(grid_map_);
    }
    
    void publishPointCloud(const ros::TimerEvent&)
    {
        if (cloud_->empty())
            return;
            
        pcl::toROSMsg(*cloud_, cloud_msg_);
        cloud_msg_.header.stamp = ros::Time::now();
        cloud_msg_.header.frame_id = "map";
        pointcloud_pub_.publish(cloud_msg_);
    }
    
    void initializeMapToOdomTransform()
    {
        // Initialize map to odom transform with all zeros (identity transform)
        map_to_odom_transform_.header.frame_id = "map";
        map_to_odom_transform_.child_frame_id = "odom";
        
        // Translation (all zeros)
        map_to_odom_transform_.transform.translation.x = 0.0;
        map_to_odom_transform_.transform.translation.y = 0.0;
        map_to_odom_transform_.transform.translation.z = 0.0;
        
        // Rotation (identity quaternion)
        map_to_odom_transform_.transform.rotation.x = 0.0;
        map_to_odom_transform_.transform.rotation.y = 0.0;
        map_to_odom_transform_.transform.rotation.z = 0.0;
        map_to_odom_transform_.transform.rotation.w = 1.0;
        
        ROS_INFO("Initialized map to odom transform (identity)");
    }
    
    void publishTransform(const ros::TimerEvent&)
    {
                ros::Duration offset(0.05);
        map_to_odom_transform_.header.stamp = ros::Time::now()+offset;
        tf_broadcaster_.sendTransform(map_to_odom_transform_);
    }
    
    void cloudSlamCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        try
        {
            // 检查点云是否来自odom坐标系
            if (msg->header.frame_id != "odom")
            {
                ROS_WARN_THROTTLE(5.0, "Received point cloud from frame '%s', expected 'odom'", 
                                  msg->header.frame_id.c_str());
            }
            
            // 转换为PCL格式进行降采样
            pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::fromROSMsg(*msg, *input_cloud);
            
            // 降采样处理
            pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
            voxel_filter.setInputCloud(input_cloud);
            voxel_filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
            voxel_filter.filter(*downsampled_cloud);
            
            ROS_DEBUG("Downsampled point cloud from %zu to %zu points", 
                     input_cloud->size(), downsampled_cloud->size());
            
            // 查找从odom到odin1_base_link的变换
            geometry_msgs::TransformStamped odom_to_base_transform;
            try
            {
                odom_to_base_transform = tf_buffer_.lookupTransform(
                    "odin1_base_link", "odom", ros::Time(0), ros::Duration(1.0));//获取odom到odin1_base_link的变换
            }
            catch (tf2::TransformException& ex)
            {
                ROS_WARN_THROTTLE(5.0, "Could not transform from odom to odin1_base_link: %s", ex.what());
                return;
            }
            
            // 将几何变换转换为Eigen变换矩阵
            Eigen::Affine3f transform = Eigen::Affine3f::Identity();
            
            // 设置平移
            transform.translation() << 
                odom_to_base_transform.transform.translation.x,
                odom_to_base_transform.transform.translation.y,
                odom_to_base_transform.transform.translation.z;
            
            // 设置旋转（从四元数转换为旋转矩阵）
            tf2::Quaternion q(
                odom_to_base_transform.transform.rotation.x,
                odom_to_base_transform.transform.rotation.y,
                odom_to_base_transform.transform.rotation.z,
                odom_to_base_transform.transform.rotation.w
            );
            
            tf2::Matrix3x3 mat(q);
            Eigen::Matrix3f rotation_matrix;
            for (int i = 0; i < 3; ++i)
            {
                for (int j = 0; j < 3; ++j)
                {
                    rotation_matrix(i, j) = mat[i][j];
                }
            }
            transform.rotate(rotation_matrix);
            
            // 使用PCL变换点云从odom坐标系到odin1_base_link坐标系
            pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::transformPointCloud(*downsampled_cloud, *transformed_cloud, transform);
            
            // 对变换后的点云进行范围截取
            pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::CropBox<pcl::PointXYZ> crop_filter;
            crop_filter.setInputCloud(transformed_cloud);
            
            // 设置截取范围：使用参数配置的范围
            Eigen::Vector4f min_point(crop_x_min_, crop_y_min_, min_height_, 1.0);
            Eigen::Vector4f max_point(crop_x_max_, crop_y_max_, max_height_, 1.0);
            
            crop_filter.setMin(min_point);
            crop_filter.setMax(max_point);
            crop_filter.filter(*cropped_cloud);
            
            // 更新SLAM点云（在odin1_base_link坐标系下，已截取范围）
            *slam_cloud_base_ = *cropped_cloud;
            
            ROS_DEBUG("Transformed and cropped SLAM point cloud: %zu -> %zu points", 
                     transformed_cloud->size(), slam_cloud_base_->size());
        }
        catch (const std::exception& e)
        {
            ROS_ERROR("Error in cloudSlamCallback: %s", e.what());
        }
    }
    
    void poseEstimateCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
    {
        try
        {
            ROS_INFO("Received 2D pose estimate for relocalization");
            
            // 检查slam点云是否可用
            if (slam_cloud_base_->empty())
            {
                ROS_WARN("SLAM point cloud is empty, cannot perform relocalization");
                return;
            }
            
            // 从map点云中截取局部点云
            pcl::PointCloud<pcl::PointXYZ>::Ptr local_map_cloud(new pcl::PointCloud<pcl::PointXYZ>());
            if (!extractLocalMapCloud(msg->pose.pose, local_map_cloud))
            {
                ROS_WARN("Failed to extract local map cloud");
                return;
            }
            
            // 执行6DOF ICP匹配
            Eigen::Matrix4f transformation_matrix;
            if (!perform6DOFICP(local_map_cloud, slam_cloud_base_, msg->pose.pose, transformation_matrix))
            {
                ROS_WARN("6DOF ICP alignment failed");
                return;
            }
            //transformation_matrix变换方向是将odin1_base_link的点变换到map坐标系下
            // 更新map到odom的变换
            updateMapToOdomTransform(transformation_matrix);
            
            ROS_INFO("Relocalization successful, updated map to odom transform");
        }
        catch (const std::exception& e)
        {
            ROS_ERROR("Error in poseEstimateCallback: %s", e.what());
        }
    }
    
    bool extractLocalMapCloud(const geometry_msgs::Pose& pose, 
                             pcl::PointCloud<pcl::PointXYZ>::Ptr& local_cloud)
    {
        if (cloud_->empty())
        {
            ROS_ERROR("Map point cloud is empty");
            return false;
        }
        
        // 使用CropBox滤波器截取局部点云
        pcl::CropBox<pcl::PointXYZ> crop_filter;
        crop_filter.setInputCloud(cloud_);
        
        // 设置截取范围（以pose为中心的正负local_crop_size_范围）
        Eigen::Vector4f min_point(
            pose.position.x - local_crop_size_,
            pose.position.y - local_crop_size_,
            min_height_,
            1.0
        );
        Eigen::Vector4f max_point(
            pose.position.x + local_crop_size_,
            pose.position.y + local_crop_size_,
            max_height_,
            1.0
        );
        
        crop_filter.setMin(min_point);
        crop_filter.setMax(max_point);
        crop_filter.filter(*local_cloud);
        
        ROS_INFO("Extracted local map cloud with %zu points around pose (%.2f, %.2f)", 
                 local_cloud->size(), pose.position.x, pose.position.y);
        
        return !local_cloud->empty();
    }
    
    // 6DOF ICP helper functions
    Eigen::Matrix4f create6DOFTransform(double x, double y, double z, double roll, double pitch, double yaw)
    {
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        
        // 设置平移部分
        transform(0, 3) = x;
        transform(1, 3) = y;
        transform(2, 3) = z;

        // 计算旋转矩阵 (ZYX顺序：先绕Z轴转yaw，再绕Y轴转pitch，最后绕X轴转roll)
        float cr = cos(roll);
        float sr = sin(roll);
        float cp = cos(pitch);
        float sp = sin(pitch);
        float cy = cos(yaw);
        float sy = sin(yaw);

        // 构建旋转矩阵
        transform(0, 0) = cy * cp;
        transform(0, 1) = cy * sp * sr - sy * cr;
        transform(0, 2) = cy * sp * cr + sy * sr;
        
        transform(1, 0) = sy * cp;
        transform(1, 1) = sy * sp * sr + cy * cr;
        transform(1, 2) = sy * sp * cr - cy * sr;
        
        transform(2, 0) = -sp;
        transform(2, 1) = cp * sr;
        transform(2, 2) = cp * cr;

        return transform;
    }
    
    // 保留4DOF函数以兼容性
    Eigen::Matrix4f create4DOFTransform(double x, double y, double z, double yaw)
    {
        return create6DOFTransform(x, y, z, 0.0, 0.0, yaw);
    }
    
    void extract4DOFParameters(const Eigen::Matrix4f& transform, double& x, double& y, double& z, double& yaw)
    {
        x = transform(0, 3);
        y = transform(1, 3);
        z = transform(2, 3);
        yaw = atan2(transform(1, 0), transform(0, 0));
    }
    
    double computePointToPointDistance(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2)
    {
        return sqrt((p1.x - p2.x) * (p1.x - p2.x) + 
                   (p1.y - p2.y) * (p1.y - p2.y) + 
                   (p1.z - p2.z) * (p1.z - p2.z));
    }
    
    bool perform6DOFICP(pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud,
                        const geometry_msgs::Pose& initial_pose,
                        Eigen::Matrix4f& final_transformation)
    {
        if (source_cloud->empty() || target_cloud->empty())
        {
            ROS_ERROR("Source or target cloud is empty");
            return false;
        }
        
        // 构建KD树用于最近邻搜索
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(target_cloud);
        
        // 初始化6DOF参数 (x, y, z, roll, pitch, yaw)
        double x = initial_pose.position.x;
        double y = initial_pose.position.y;
        double z = initial_pose.position.z;
        
        tf2::Quaternion q(
            initial_pose.orientation.x,
            initial_pose.orientation.y,
            initial_pose.orientation.z,
            initial_pose.orientation.w
        );
        tf2::Matrix3x3 mat(q);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);
        
        double lambda = lambda_initial_;
        double prev_error = std::numeric_limits<double>::max();
        bool converged = false;
        
        ROS_INFO("Starting 6DOF ICP with initial pose: x=%.3f, y=%.3f, z=%.3f, roll=%.3f, pitch=%.3f, yaw=%.3f", 
                 x, y, z, roll, pitch, yaw);
        
        for (int iter = 0; iter < max_icp_iterations_ && !converged; ++iter)
        {
            // 创建当前变换矩阵
            Eigen::Matrix4f current_transform = create6DOFTransform(x, y, z, roll, pitch, yaw);
            
            // 变换源点云
            pcl::PointCloud<pcl::PointXYZ> transformed_source;
            pcl::transformPointCloud(*source_cloud, transformed_source, current_transform);
            
            // 寻找对应点对
            std::vector<int> correspondences;
            std::vector<float> distances;
            double total_error = 0.0;
            int valid_correspondences = 0;
            
            for (size_t i = 0; i < transformed_source.size(); ++i)
            {
                std::vector<int> nearest_indices(1);
                std::vector<float> nearest_distances(1);
                
                if (kdtree.nearestKSearch(transformed_source[i], 1, nearest_indices, nearest_distances) > 0)
                {
                    if (nearest_distances[0] < max_correspondence_distance_ * max_correspondence_distance_)
                    {
                        correspondences.push_back(nearest_indices[0]);
                        distances.push_back(sqrt(nearest_distances[0]));
                        total_error += nearest_distances[0];
                        valid_correspondences++;
                    }
                    else
                    {
                        correspondences.push_back(-1);
                        distances.push_back(0.0);
                    }
                }
                else
                {
                    correspondences.push_back(-1);
                    distances.push_back(0.0);
                }
            }
            
            if (valid_correspondences < 10)
            {
                ROS_WARN("Too few correspondences (%d), ICP failed", valid_correspondences);
                return false;
            }
            
            double mean_error = total_error / valid_correspondences;
            
            ROS_DEBUG("Iteration %d: mean_error=%.6f, correspondences=%d", 
                     iter, mean_error, valid_correspondences);
            
            // 检查收敛
            if (iter > 0 && fabs(prev_error - mean_error) < euclidean_fitness_epsilon_)
            {
                converged = true;
                ROS_INFO("6DOF ICP converged after %d iterations with error %.6f", iter, mean_error);
                break;
            }
            
            // 构建线性方程组 Ax = b 求解增量
            Eigen::Matrix<double, 6, 6> A = Eigen::Matrix<double, 6, 6>::Zero();
            Eigen::Matrix<double, 6, 1> b = Eigen::Matrix<double, 6, 1>::Zero();
            
            for (size_t i = 0; i < source_cloud->size(); ++i)
            {
                if (correspondences[i] >= 0)
                {
                    const pcl::PointXYZ& src_pt = source_cloud->points[i];
                    const pcl::PointXYZ& tgt_pt = target_cloud->points[correspondences[i]];
                    const pcl::PointXYZ& trans_pt = transformed_source.points[i];
                    
                    // 计算雅可比矩阵 (对 x, y, z, roll, pitch, yaw 的偏导数)
                    Eigen::Matrix<double, 3, 6> jacobian;
                    jacobian.setZero();
                    
                    // 对 x, y, z 的偏导数
                    jacobian(0, 0) = 1.0;  // dx/dx
                    jacobian(1, 1) = 1.0;  // dy/dy
                    jacobian(2, 2) = 1.0;  // dz/dz
                    
                    // 对 roll, pitch, yaw 的偏导数（参考localization.cpp的实现）
                    double cr = cos(roll);
                    double sr = sin(roll);
                    double cp = cos(pitch);
                    double sp = sin(pitch);
                    double cy = cos(yaw);
                    double sy = sin(yaw);
                    
                    // 对 roll 的偏导数
                    jacobian(0, 3) = (cy * sp * cr + sy * sr) * src_pt.y + (-cy * sp * sr + sy * cr) * src_pt.z;
                    jacobian(1, 3) = (sy * sp * cr - cy * sr) * src_pt.y + (-sy * sp * sr - cy * cr) * src_pt.z;
                    jacobian(2, 3) = cp * cr * src_pt.y - cp * sr * src_pt.z;
                    
                    // 对 pitch 的偏导数
                    jacobian(0, 4) = -cy * sp * src_pt.x + cy * cp * sr * src_pt.y + cy * cp * cr * src_pt.z;
                    jacobian(1, 4) = -sy * sp * src_pt.x + sy * cp * sr * src_pt.y + sy * cp * cr * src_pt.z;
                    jacobian(2, 4) = -cp * src_pt.x - sp * sr * src_pt.y - sp * cr * src_pt.z;
                    
                    // 对 yaw 的偏导数
                    jacobian(0, 5) = -sy * cp * src_pt.x + (-sy * sp * sr - cy * cr) * src_pt.y + (-sy * sp * cr + cy * sr) * src_pt.z;
                    jacobian(1, 5) = cy * cp * src_pt.x + (cy * sp * sr - sy * cr) * src_pt.y + (cy * sp * cr + sy * sr) * src_pt.z;
                    jacobian(2, 5) = 0.0;
                    
                    // 残差向量
                    Eigen::Vector3d residual;
                    residual(0) = tgt_pt.x - trans_pt.x;
                    residual(1) = tgt_pt.y - trans_pt.y;
                    residual(2) = tgt_pt.z - trans_pt.z;
                    
                    // 累加到法方程
                    A += jacobian.transpose() * jacobian;
                    b += jacobian.transpose() * residual;
                }
            }
            
            // Levenberg-Marquardt阻尼
            A.diagonal() += lambda * Eigen::Matrix<double, 6, 1>::Ones();
            
            // 求解线性方程组
            Eigen::Matrix<double, 6, 1> delta = A.ldlt().solve(b);
            
            // 检查解的有效性
            if (!delta.allFinite())
            {
                ROS_WARN("Invalid solution in ICP iteration %d", iter);
                lambda *= 10.0;
                continue;
            }
            
            // 更新参数
            double step_size = 1.0;
            double new_x = x + step_size * delta(0);
            double new_y = y + step_size * delta(1);
            double new_z = z + step_size * delta(2);
            double new_roll = roll + step_size * delta(3);
            double new_pitch = pitch + step_size * delta(4);
            double new_yaw = yaw + step_size * delta(5);
            
            // 限制角度在[-π, π]范围内
            while (new_roll > M_PI) new_roll -= 2.0 * M_PI;
            while (new_roll < -M_PI) new_roll += 2.0 * M_PI;
            while (new_pitch > M_PI) new_pitch -= 2.0 * M_PI;
            while (new_pitch < -M_PI) new_pitch += 2.0 * M_PI;
            while (new_yaw > M_PI) new_yaw -= 2.0 * M_PI;
            while (new_yaw < -M_PI) new_yaw += 2.0 * M_PI;
            
            // 检查参数变化是否足够小
            double param_change = sqrt(delta(0)*delta(0) + delta(1)*delta(1) + 
                                     delta(2)*delta(2) + delta(3)*delta(3) +
                                     delta(4)*delta(4) + delta(5)*delta(5));
            
            if (param_change < transformation_epsilon_)
            {
                converged = true;
                ROS_INFO("6DOF ICP converged due to small parameter change: %.8f", param_change);
                break;
            }
            
            // 更新参数
            x = new_x;
            y = new_y;
            z = new_z;
            roll = new_roll;
            pitch = new_pitch;
            yaw = new_yaw;
            
            // 调整lambda参数
            if (mean_error < prev_error)
            {
                lambda *= 0.1;  // 误差减小，减少阻尼
            }
            else
            {
                lambda *= 10.0; // 误差增大，增加阻尼
            }
            
            prev_error = mean_error;
        }
        
        // 生成最终变换矩阵
        final_transformation = create6DOFTransform(x, y, z, roll, pitch, yaw);
        
        if (!converged)
        {
            ROS_WARN("6DOF ICP did not converge after %d iterations", max_icp_iterations_);
        }
        
        ROS_INFO("Final 6DOF ICP result: x=%.3f, y=%.3f, z=%.3f, roll=%.3f, pitch=%.3f, yaw=%.3f", 
                 x, y, z, roll, pitch, yaw);
        
        return converged || max_icp_iterations_ > 10; // 即使没完全收敛，如果迭代次数足够也认为有效
    }
    
    void updateMapToOdomTransform(const Eigen::Matrix4f& map_to_base_transform)
    {
        try
        {
            // 获取T_base_odom变换
            geometry_msgs::TransformStamped base_to_odom_transform;
            base_to_odom_transform = tf_buffer_.lookupTransform(
                "odin1_base_link", "odom", ros::Time(0), ros::Duration(1.0));
            
            // 将Eigen矩阵转换为Eigen::Isometry3d用于变换组合 T_map_odom
            Eigen::Isometry3d map_to_base = Eigen::Isometry3d::Identity();
            map_to_base.matrix() = map_to_base_transform.cast<double>();
            
            // 将base_to_odom变换转换为Eigen::Isometry3d
            Eigen::Isometry3d base_to_odom = Eigen::Isometry3d::Identity();
            
            // 设置平移
            base_to_odom.translation() << 
                base_to_odom_transform.transform.translation.x,
                base_to_odom_transform.transform.translation.y,
                base_to_odom_transform.transform.translation.z;
            
            // 设置旋转
            tf2::Quaternion q_base_odom(
                base_to_odom_transform.transform.rotation.x,
                base_to_odom_transform.transform.rotation.y,
                base_to_odom_transform.transform.rotation.z,
                base_to_odom_transform.transform.rotation.w
            );
            
            tf2::Matrix3x3 mat_base_odom(q_base_odom);
            Eigen::Matrix3d rotation_base_odom;
            for (int i = 0; i < 3; ++i)
            {
                for (int j = 0; j < 3; ++j)
                {
                    rotation_base_odom(i, j) = mat_base_odom[i][j];
                }
            }
            base_to_odom.rotate(rotation_base_odom);
            
            // 计算map到odom的变换：T_map_odom = T_map_base * T_base_odom
            Eigen::Isometry3d map_to_odom = map_to_base * base_to_odom;
            
            // 更新map_to_odom_transform_
            map_to_odom_transform_.transform.translation.x = map_to_odom.translation().x();
            map_to_odom_transform_.transform.translation.y = map_to_odom.translation().y();
            map_to_odom_transform_.transform.translation.z = map_to_odom.translation().z();
            
            Eigen::Quaterniond q_result(map_to_odom.rotation());
            map_to_odom_transform_.transform.rotation.x = q_result.x();
            map_to_odom_transform_.transform.rotation.y = q_result.y();
            map_to_odom_transform_.transform.rotation.z = q_result.z();
            map_to_odom_transform_.transform.rotation.w = q_result.w();
            
            ROS_INFO("Updated map to odom transform: translation(%.3f, %.3f, %.3f)",
                     map_to_odom_transform_.transform.translation.x,
                     map_to_odom_transform_.transform.translation.y,
                     map_to_odom_transform_.transform.translation.z);
        }
        catch (tf2::TransformException& ex)
        {
            ROS_ERROR("Failed to update map to odom transform: %s", ex.what());
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "relocal_node");
    
    try
    {
        RelocalizationNode node;
        ros::spin();
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("Exception in relocal_node: %s", e.what());
        return -1;
    }
    
    return 0;
}
