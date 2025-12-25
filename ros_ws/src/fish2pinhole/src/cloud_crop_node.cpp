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
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <cmath>

// 自定义点云结构
namespace odin1_ros
{
  struct EIGEN_ALIGN16 Point
  {
    float x;
    float y;
    float z;
    std::uint8_t intensity;
    std::uint16_t confidence;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
} // namespace odin1_ros

// 注册点云结构到PCL
POINT_CLOUD_REGISTER_POINT_STRUCT(odin1_ros::Point,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (std::uint8_t, intensity, intensity)
                                  (std::uint16_t, confidence, confidence))

class CloudCropNode
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // 订阅和发布
    ros::Subscriber cloud_sub_;
    ros::Publisher scan_pub_;
    ros::Publisher filtered_cloud_pub_;  // 发布滤波后的点云
    
    // 参数
    double min_confidence_;
    double z_min_ratio_;  // z的最小值相对于x的比例
    double z_max_ratio_;  // z的最大值相对于x的比例
    double scan_angle_range_;  // 激光扫描角度范围（度）
    double scan_min_range_;
    double scan_max_range_;
    double scan_resolution_;  // 角度分辨率（度）
    double max_forward_distance_;  // 最大前方距离（米）
    int min_intensity_;  // 最小强度阈值
    std::string input_topic_;
    std::string output_topic_;
    std::string filtered_cloud_topic_;  // 滤波后点云话题
    std::string frame_id_;

public:
    CloudCropNode() : private_nh_("~")
    {
        // 读取参数
        private_nh_.param<double>("min_confidence", min_confidence_, 30.0);
        private_nh_.param<double>("z_min_ratio", z_min_ratio_, -1.0);  // z >= -x
        private_nh_.param<double>("z_max_ratio", z_max_ratio_, 1.0);   // z <= x
        private_nh_.param<double>("scan_angle_range", scan_angle_range_, 40.0);  // ±40度
        private_nh_.param<double>("scan_min_range", scan_min_range_, 0.1);
        private_nh_.param<double>("scan_max_range", scan_max_range_, 20.0);
        private_nh_.param<double>("scan_resolution", scan_resolution_, 0.5);  // 0.5度分辨率
        private_nh_.param<double>("max_forward_distance", max_forward_distance_, 0.6);  // 最大前方距离
        private_nh_.param<int>("min_intensity", min_intensity_, 220);  // 最小强度阈值
        private_nh_.param<std::string>("input_topic", input_topic_, "/odin1/cloud_raw");
        private_nh_.param<std::string>("output_topic", output_topic_, "/scan");
        private_nh_.param<std::string>("filtered_cloud_topic", filtered_cloud_topic_, "/odin1/cloud_filtered");
        private_nh_.param<std::string>("frame_id", frame_id_, "odin_link");
        
        // 初始化订阅和发布
        cloud_sub_ = nh_.subscribe(input_topic_, 1, &CloudCropNode::cloudCallback, this);
        scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>(output_topic_, 1);
        filtered_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(filtered_cloud_topic_, 1);
        
        ROS_INFO("CloudCrop node initialized");
        ROS_INFO("Input topic: %s", input_topic_.c_str());
        ROS_INFO("Output topic: %s", output_topic_.c_str());
        ROS_INFO("Filtered cloud topic: %s", filtered_cloud_topic_.c_str());
        ROS_INFO("Min confidence: %.1f", min_confidence_);
        ROS_INFO("Z range ratio: [%.1f, %.1f] * |x|", z_min_ratio_, z_max_ratio_);
        ROS_INFO("Scan angle range: ±%.1f degrees", scan_angle_range_);
        ROS_INFO("Max forward distance: %.2f meters", max_forward_distance_);
        ROS_INFO("Min intensity threshold: %d", min_intensity_);
    }
    
    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
    {
        // 转换为PCL点云
        pcl::PointCloud<odin1_ros::Point> cloud;
        pcl::fromROSMsg(*cloud_msg, cloud);
        
        // 过滤点云
        pcl::PointCloud<odin1_ros::Point> filtered_cloud;
        filterPointCloud(cloud, filtered_cloud);
        
        // 发布滤波后的点云（用于调试）
        sensor_msgs::PointCloud2 filtered_cloud_msg;
        pcl::toROSMsg(filtered_cloud, filtered_cloud_msg);
        filtered_cloud_msg.header = cloud_msg->header;
        filtered_cloud_msg.header.frame_id = frame_id_;
        filtered_cloud_pub_.publish(filtered_cloud_msg);
        
        
        // 转换为LaserScan
        sensor_msgs::LaserScan scan_msg;
        convertToLaserScan(filtered_cloud, cloud_msg->header, scan_msg);
        
        // 发布LaserScan
        scan_pub_.publish(scan_msg);
    }
    
private:
    /**
     * 功能：根据多个条件过滤点云
     * 输入：const pcl::PointCloud<odin1_ros::Point>& input (原始点云)
     * 输出：pcl::PointCloud<odin1_ros::Point>& output (过滤后点云)
     * 过滤条件：
     * 1. 跳过无效点 (!isfinite)
     * 2. 置信度 >= min_confidence_
     * 3. Z轴范围：z_min_ratio_ <= z <= z_max_ratio_
     * 4. 角度范围：只保留 ±scan_angle_range_ 范围内的点
     * 5. 距离范围：scan_min_range_ <= distance <= scan_max_range_
     * 6. 强度阈值：intensity >= 100
     */
    void filterPointCloud(const pcl::PointCloud<odin1_ros::Point>& input,
                         pcl::PointCloud<odin1_ros::Point>& output)
    {
        output.clear();
        output.header = input.header;
        
        int total_points = 0;
        int filtered_by_validity = 0;
        int filtered_by_confidence = 0;
        int filtered_by_z_range = 0;
        int filtered_by_angle_range = 0;
        int filtered_by_distance_range = 0;
        int filtered_by_intensity = 0;
        int valid_points = 0;
        
        // 计算角度范围（弧度）
        double angle_range_rad = scan_angle_range_ * M_PI / 180.0;
        
        for (const auto& point : input.points)
        {
            total_points++;
            
            // 1. 跳过无效点
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
            {
                filtered_by_validity++;
                continue;
            }
            
            // 2. 过滤confidence
            if (point.confidence < min_confidence_)
            {
                filtered_by_confidence++;
                continue;
            }
            
            // 3. 过滤Z轴范围
            if (point.z < z_min_ratio_ || point.z > z_max_ratio_)
            {
                filtered_by_z_range++;
                continue;
            }
            
            // 4. 过滤角度范围（只保留 ±scan_angle_range_ 范围内的点）
            double angle = std::atan2(point.y, point.x);
            if (angle < -angle_range_rad || angle > angle_range_rad)
            {
                filtered_by_angle_range++;
                continue;
            }
            
            // 5. 过滤距离范围（scan_min_range_ <= distance <= scan_max_range_）
            double dis = sqrt(point.x*point.x + point.y*point.y);
            if (dis < scan_min_range_ || dis > scan_max_range_)
            {
                filtered_by_distance_range++;
                continue;
            }
            // 6. 分段强度过滤
            // 前方 0.5m 以内：过滤强度 < 150 的点
            // 0.5m 以外：过滤强度 < 120 的点
            if (dis <= 0.8)
            {
                // 前方 0.5m 以内，需要更高的强度阈值
                if ((int)point.intensity < 200)
                {
                    filtered_by_intensity++;
                    continue;
                }
            }
            else
            {
                // 0.5m 以外，使用较低的强度阈值
                if ((int)point.intensity < 120)
                {
                    filtered_by_intensity++;
                    continue;
                }
            }
            // 通过所有过滤条件的点
            output.points.push_back(point);
            valid_points++;
        }
        
        output.width = output.points.size();
        output.height = 1;
        output.is_dense = false;
        
        // 打印过滤统计信息
        ROS_DEBUG("Point cloud filtering stats:");
        ROS_DEBUG("  Total points: %d", total_points);
        ROS_DEBUG("  Filtered by validity: %d", filtered_by_validity);
        ROS_DEBUG("  Filtered by confidence: %d", filtered_by_confidence);
        ROS_DEBUG("  Filtered by Z-range: %d", filtered_by_z_range);
        ROS_DEBUG("  Filtered by angle range (±%.1f°): %d", scan_angle_range_, filtered_by_angle_range);
        ROS_DEBUG("  Filtered by distance range (%.1f-%.1fm): %d", scan_min_range_, scan_max_range_, filtered_by_distance_range);
        ROS_DEBUG("  Filtered by intensity (<100): %d", filtered_by_intensity);
        ROS_DEBUG("  Valid points: %d (%.1f%%)", valid_points, 
                 total_points > 0 ? (100.0 * valid_points / total_points) : 0.0);
    }
    
    void convertToLaserScan(const pcl::PointCloud<odin1_ros::Point>& cloud,
                           const std_msgs::Header& header,
                           sensor_msgs::LaserScan& scan)
    {
        // 设置LaserScan参数
        double angle_range_rad = scan_angle_range_ * M_PI / 180.0;
        double resolution_rad = scan_resolution_ * M_PI / 180.0;
        int num_readings = static_cast<int>(2 * angle_range_rad / resolution_rad) + 1;
        
        scan.header = header;
        scan.header.frame_id = frame_id_;
        scan.angle_min = -angle_range_rad;
        scan.angle_max = angle_range_rad;
        scan.angle_increment = resolution_rad;
        scan.time_increment = 0.0;
        scan.scan_time = 0.1;  // 假设10Hz
        scan.range_min = scan_min_range_;
        scan.range_max = scan_max_range_;
        // if you want to keep the nearest point
        // you should initial the ranges as infinity
        scan.ranges.assign(num_readings, std::numeric_limits<float>::infinity());
        
        // if you want to keep the farest point to filter the noisy point
        // you should initial the ranges as min range
        //scan.ranges.assign(num_readings, scan_max_range_);
        
        scan.intensities.assign(num_readings, 0.0);
        
        // 将点云转换为LaserScan
        for (const auto& point : cloud.points)
        {
            // 计算角度（相对于x轴）
            double angle = std::atan2(point.y, point.x);
            
            // 检查角度是否在范围内
            if (angle < -angle_range_rad || angle > angle_range_rad)
                continue;
            
            // 计算距离
            double range = std::sqrt(point.x * point.x + point.y * point.y);
            
            // 检查距离是否在范围内
            if (range < scan_min_range_ || range > scan_max_range_)
                continue;
            
            // 计算在scan数组中的索引
            int index = static_cast<int>((angle - scan.angle_min) / scan.angle_increment);
            
            // 确保索引在有效范围内
            if (index >= 0 && index < num_readings)
            {
                // 如果这个角度已经有更近的点，则保留更far的
                if (range < scan.ranges[index] || std::isinf(scan.ranges[index]))
                {
                    scan.ranges[index] = range;
                    scan.intensities[index] = point.intensity;
                }
            }
        }
        
        // 将无穷大的值替换为最大范围
        /*
        for (auto& range : scan.ranges)
        {
            if (std::isinf(range))
            {
                //range = scan.range_max;
            }
        }*/
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cloud_crop_node");
    
    CloudCropNode node;
    
    ros::spin();
    
    return 0;
}
