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
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>

class Fish2PinholeNode
{
public:
    Fish2PinholeNode(ros::NodeHandle &nh) : nh_(nh)
    {
        nh_.getParam("/cam_0/image_width", image_width);
        nh_.getParam("/cam_0/image_height", image_height);
        nh_.getParam("/cam_0/k2", k2);
        nh_.getParam("/cam_0/k3", k3);
        nh_.getParam("/cam_0/k4", k4);
        nh_.getParam("/cam_0/k5", k5);
        nh_.getParam("/cam_0/k6", k6);
        nh_.getParam("/cam_0/k7", k7);
        nh_.getParam("/cam_0/p1", p1);
        nh_.getParam("/cam_0/p2", p2);
        nh_.getParam("/cam_0/A11", A11);
        nh_.getParam("/cam_0/A12", A12);
        nh_.getParam("/cam_0/A22", A22);
        nh_.getParam("/cam_0/u0", u0);
        nh_.getParam("/cam_0/v0", v0);
        nh_.getParam("odin1_topic", odin1_topic);
        nh_.getParam("pinhole_topic", pinhole_topic);

        
        compressed_sub_ = nh_.subscribe(odin1_topic + "/compressed", 1, &Fish2PinholeNode::compressedImageCallback, this);
        //image_raw_sub_ = nh_.subscribe(odin1_topic, 1, &Fish2PinholeNode::imageCallback, this);
        
        image_pub_ = nh_.advertise<sensor_msgs::Image>(pinhole_topic, 1);
        compressed_pub_ = nh_.advertise<sensor_msgs::CompressedImage>(pinhole_topic + "/compressed", 1);
        camera_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(pinhole_topic + "/camera_info", 1);

        map_x = cv::Mat::zeros(image_height, image_width, CV_32FC1);
        map_y = cv::Mat::zeros(image_height, image_width, CV_32FC1);

        for (int u = 0; u < image_width; ++u)
        {
            for (int v = 0; v < image_height; ++v)
            {
                //new fx=image_width/2.0,fy=image_height/2.0, cx=image_width/2, cy=image_height/2
                double x = (u - image_width/2.0) / (image_width/2.0);
                double y = (v - image_height/2.0) / (image_height/2.0);
                
                double r = sqrt(x * x + y * y);
                if  (r == 0) {
                    map_x.at<float>(v, u) = u;
                    map_y.at<float>(v, u) = v;
                    continue;
                }
                double theta = atan(r);
                double theta_d = theta * (1.0 + k2 * std::pow(theta, 2) +
                                          k3 * std::pow(theta, 3) +
                                          k4 * std::pow(theta, 4) +
                                          k5 * std::pow(theta, 5) +
                                          k6 * std::pow(theta, 6) +
                                          k7 * std::pow(theta, 7));
                
                double xd = (theta_d / r) * x;
                double yd = (theta_d / r) * y;
                double u_d = A11 * xd + A12 * yd + u0;
                double v_d = A22 * yd + v0;
                
                if (u_d >= 0 && u_d < image_width && v_d >= 0 && v_d < image_height)
                {
                    map_x.at<float>(v, u) = u_d;
                    map_y.at<float>(v, u) = v_d;
                }
                else
                {
                    map_x.at<float>(v, u) = u;
                    map_y.at<float>(v, u) = v;
                }
            }
        }
    }

    void compressedImageCallback(const sensor_msgs::CompressedImageConstPtr &msg)
    {
        try
        {
            cv_bridge::CvImagePtr cv_ptr_compressed = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            processAndPublishImage(cv_ptr_compressed->image, msg->header);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }

    void imageCallback(const sensor_msgs::ImageConstPtr &msg)
    {
        try
        {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            processAndPublishImage(cv_ptr->image, msg->header);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }

    void processAndPublishImage(cv::Mat image, const std_msgs::Header &header)
    {
        auto outheader=header;
        outheader.frame_id = "camera_link";
        cv::Mat undistorted_image;
        cv::remap(image, undistorted_image, map_x, map_y,
                  cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
        sensor_msgs::ImagePtr output_msg = cv_bridge::CvImage(outheader, "bgr8", undistorted_image).toImageMsg();
        image_pub_.publish(output_msg);
        sensor_msgs::CompressedImagePtr compressed_msg = cv_bridge::CvImage(outheader, "bgr8", undistorted_image).toCompressedImageMsg();
        compressed_pub_.publish(compressed_msg);
        
        // 发布相机信息
        sensor_msgs::CameraInfo camera_info_msg = createCameraInfo(outheader);
        camera_info_pub_.publish(camera_info_msg);
    }

    sensor_msgs::CameraInfo createCameraInfo(const std_msgs::Header &header)
    {
        sensor_msgs::CameraInfo camera_info;
        camera_info.header = header;
        camera_info.height = image_height;
        camera_info.width = image_width;
        
        // 设置相机内参矩阵 K (3x3)
        // 针孔相机模型: fx=image_width/2.0, fy=image_height/2.0, cx=image_width/2, cy=image_height/2
        camera_info.K[0] = image_width / 2.0;  // fx
        camera_info.K[1] = 0.0;
        camera_info.K[2] = image_width / 2.0;  // cx
        camera_info.K[3] = 0.0;
        camera_info.K[4] = image_height / 2.0; // fy
        camera_info.K[5] = image_height / 2.0; // cy
        camera_info.K[6] = 0.0;
        camera_info.K[7] = 0.0;
        camera_info.K[8] = 1.0;
        
        // 设置投影矩阵 P (3x4)
        camera_info.P[0] = image_width / 2.0;  // fx
        camera_info.P[1] = 0.0;
        camera_info.P[2] = image_width / 2.0;  // cx
        camera_info.P[3] = 0.0;
        camera_info.P[4] = 0.0;
        camera_info.P[5] = image_height / 2.0; // fy
        camera_info.P[6] = image_height / 2.0; // cy
        camera_info.P[7] = 0.0;
        camera_info.P[8] = 0.0;
        camera_info.P[9] = 0.0;
        camera_info.P[10] = 1.0;
        camera_info.P[11] = 0.0;
        
        // 设置畸变模型（针孔相机模型下为空）
        camera_info.distortion_model = "plumb_bob";
        camera_info.D.resize(5);
        camera_info.D[0] = 0.0; // k1
        camera_info.D[1] = 0.0; // k2
        camera_info.D[2] = 0.0; // p1
        camera_info.D[3] = 0.0; // p2
        camera_info.D[4] = 0.0; // k3
        
        // 设置修正矩阵 R (3x3) - 单位矩阵
        camera_info.R[0] = 1.0;
        camera_info.R[1] = 0.0;
        camera_info.R[2] = 0.0;
        camera_info.R[3] = 0.0;
        camera_info.R[4] = 1.0;
        camera_info.R[5] = 0.0;
        camera_info.R[6] = 0.0;
        camera_info.R[7] = 0.0;
        camera_info.R[8] = 1.0;
        
        return camera_info;
    }

private:
    ros::NodeHandle &nh_;
    int image_width, image_height;
    double k2, k3, k4, k5, k6, k7, p1, p2;
    double A11, A12, A22;
    double u0, v0;
    ros::Subscriber compressed_sub_;
    ros::Subscriber image_raw_sub_;
    ros::Publisher image_pub_;
    ros::Publisher compressed_pub_;  
    ros::Publisher camera_info_pub_; 
    std::string odin1_topic, pinhole_topic;

    cv::Mat map_x, map_y;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fish2pinhole_node");
    ros::NodeHandle nh("~");
    Fish2PinholeNode fish2pinhole_node(nh);

    ros::spin();
    return 0;
}
