# Cloud Crop Node 使用说明

## 功能概述
`cloud_crop_node` 是一个点云视场角裁切节点，用于从输入的 SLAM 点云中提取指定视场角范围内的点云数据。

## 主要特性
- 订阅 `/odin1/cloud_slam` 点云话题
- 使用可配置的视场角参数（左右上下角度范围）
- 在 body 坐标系下构建四棱锥裁切区域
- 自动监控 TF 变换，将四棱锥变换到点云坐标系
- 发布裁切后的点云和变换后的四棱锥形状
- 实时计算处理耗时统计

## 使用方法

### 启动节点
```bash
# 启动云裁切节点
roslaunch fish2pinhole cloud_crop.launch

# 或者与其他节点一起启动
roslaunch fish2pinhole cloud_crop.launch &
roslaunch fish2pinhole start.launch
```

### 参数配置
在 `launch/cloud_crop.launch` 中可以配置以下参数：

```xml
<!-- 视场角参数 (度) -->
<param name="fov_left" value="-45.0" />     <!-- 左视场角 -->
<param name="fov_right" value="45.0" />     <!-- 右视场角 -->
<param name="fov_up" value="30.0" />        <!-- 上视场角 -->
<param name="fov_down" value="-30.0" />     <!-- 下视场角 -->

<!-- 最大距离 (米) -->
<param name="max_distance" value="15.0" />

<!-- 坐标系名称 -->
<param name="body_frame" value="odin1_base_link" />
<param name="cloud_frame" value="odom" />
```

### 话题接口

#### 订阅话题
- `/odin1/cloud_slam` (sensor_msgs/PointCloud2): 输入的 SLAM 点云

#### 发布话题
- `/odin1/cloud_cropped` (sensor_msgs/PointCloud2): 裁切后的点云
- `/odin1/frustum` (geometry_msgs/PolygonStamped): 变换后的四棱锥形状

## 技术实现
- 使用 PCL 库的 `CropHull` 和 `ConvexHull` 进行高效裁切
- 基于 TF2 的坐标变换监控
- 使用 Eigen 矩阵进行点云变换
- chrono 库进行微秒级计时统计

## 性能优化
- 预先构建四棱锥凸包，避免重复计算
- 使用 PCL 原生函数提高处理效率
- 最小化内存分配和拷贝操作

## 编译依赖
- PCL (Point Cloud Library)
- tf2_eigen
- sensor_msgs
- geometry_msgs
- tf2_ros

## 调试信息
节点会在 DEBUG 级别输出处理耗时信息：
```bash
# 启用调试输出
rosrun fish2pinhole cloud_crop_node _log_level:=debug
```