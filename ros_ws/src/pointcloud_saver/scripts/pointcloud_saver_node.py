#!/usr/bin/env python3
'''
Copyright 2025 Manifold Tech Ltd.(www.manifoldtech.com.co)
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
   http://www.apache.org/licenses/LICENSE-2.0
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
'''


import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from pointcloud_saver.srv import SaveMap, SaveMapResponse
from std_srvs.srv import Trigger, TriggerResponse
import tf2_ros
from datetime import datetime
import os
import struct
import threading


class PointCloudSaver:
    def __init__(self):
        rospy.init_node("pointcloud_saver_node", anonymous=False)

        # 参数
        self.cloud_topic = rospy.get_param("~cloud_topic", "/odin1/cloud_slam")
        self.target_frame = rospy.get_param("~target_frame", "map")
        self.save_dir = rospy.get_param("~save_dir", os.path.expanduser("~/maps"))
        self.max_points = rospy.get_param("~max_points", 10000000)
        self.auto_publish = rospy.get_param("~auto_publish", True)
        self.online_voxel_size = rospy.get_param("~online_voxel_size", 0.0)
        self.save_rgb = rospy.get_param("~save_rgb", True)
        self.max_stat_points = rospy.get_param("~max_stat_points", 200000)
        self.pcd_binary = rospy.get_param("~pcd_binary", True)

        # 状态
        self.is_recording = False
        # 存储结构
        # 1) 在线体素下采样：用字典存储体素的累计信息，降低内存占用
        #    key: (vx, vy, vz) -> value: [sum_x, sum_y, sum_z, sum_r, sum_g, sum_b, count]
        # 2) 非在线体素：使用 numpy 数组存储，dtype np.float32 / np.uint8
        self.voxel_map = (
            {} if self.online_voxel_size and self.online_voxel_size > 0 else None
        )
        self.map_lock = threading.Lock() if self.voxel_map is not None else None
        self.points_xyz = (
            np.empty((0, 3), dtype=np.float32) if not self.voxel_map else None
        )
        self.points_rgb = (
            np.empty((0, 3), dtype=np.uint8)
            if (not self.voxel_map and self.save_rgb)
            else None
        )
        self.frame_count = 0

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # 创建保存目录
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
            rospy.loginfo(f"创建保存目录: {self.save_dir}")

        # 订阅点云
        self.cloud_sub = rospy.Subscriber(
            self.cloud_topic, PointCloud2, self.cloud_callback, queue_size=10
        )

        # 发布累积点云（用于可视化）
        if self.auto_publish:
            self.accumulated_pub = rospy.Publisher(
                "/accumulated_map", PointCloud2, queue_size=1
            )
            # 定时发布
            self.pub_timer = rospy.Timer(
                rospy.Duration(2.0), self.publish_accumulated_cloud
            )

        # 服务
        self.start_srv = rospy.Service(
            "~start_recording", Trigger, self.start_recording_callback
        )
        self.stop_srv = rospy.Service(
            "~stop_recording", Trigger, self.stop_recording_callback
        )
        self.save_srv = rospy.Service("~save_map", SaveMap, self.save_map_callback)
        self.clear_srv = rospy.Service("~clear_map", Trigger, self.clear_map_callback)

        rospy.loginfo("=" * 60)
        rospy.loginfo("点云地图保存节点已启动")
        rospy.loginfo(f"订阅话题: {self.cloud_topic}")
        rospy.loginfo(f"目标坐标系: {self.target_frame}")
        rospy.loginfo(f"保存目录: {self.save_dir}")
        rospy.loginfo(
            f"在线体素下采样: {'开启' if self.voxel_map is not None else '关闭'} (voxel={self.online_voxel_size}m)"
        )
        rospy.loginfo(f"保存 RGB: {'是' if self.save_rgb else '否'}")
        rospy.loginfo(f"统计滤波上限: {self.max_stat_points} 点 (超过将自动跳过)")
        rospy.loginfo(f"PCD 二进制保存: {'是' if self.pcd_binary else '否'}")
        rospy.loginfo("=" * 60)
        rospy.loginfo("服务列表:")
        rospy.loginfo("  - ~/start_recording : 开始记录点云")
        rospy.loginfo("  - ~/stop_recording  : 停止记录点云")
        rospy.loginfo("  - ~/save_map        : 保存地图为PCD文件")
        rospy.loginfo("  - ~/clear_map       : 清空累积的点云")
        rospy.loginfo("=" * 60)

    def cloud_callback(self, msg):
        """点云回调函数"""
        if not self.is_recording:
            return

        try:
            # 当前累计点数（根据存储模式统计）
            if self.voxel_map is not None:
                if self.map_lock is not None:
                    with self.map_lock:
                        current_points = len(self.voxel_map)
                else:
                    current_points = len(self.voxel_map)
            else:
                current_points = int(self.points_xyz.shape[0])
            # 检查点数限制
            if current_points >= self.max_points:
                if self.frame_count % 100 == 0:
                    rospy.logwarn(f"已达到最大点数限制 {self.max_points}，停止累积")
                return

            # 获取 TF 变换
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    msg.header.frame_id,
                    rospy.Time(0),
                    rospy.Duration(1.0),
                )
            except Exception as e:
                if self.frame_count % 50 == 0:
                    rospy.logwarn(f"TF 查询失败: {e}")
                return

            # 提取变换参数
            tx = transform.transform.translation.x
            ty = transform.transform.translation.y
            tz = transform.transform.translation.z
            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w

            # 四元数转旋转矩阵
            R = self.quaternion_to_rotation_matrix(qx, qy, qz, qw)

            # 读取点云数据
            # 为减少中间对象开销，逐点处理
            new_xyz = []  # 临时列表，最后一次性拼接到 numpy 数组
            new_rgb = []  # 临时列表
            for point in pc2.read_points(msg, skip_nans=True):
                x, y, z = point[:3]

                # 转换到目标坐标系
                point_local = np.array([x, y, z])
                point_map = R @ point_local + np.array([tx, ty, tz])

                # 颜色（如果需要保存）
                if self.save_rgb:
                    if len(point) >= 4:
                        rgb = point[3]
                        if isinstance(rgb, float):
                            rgb_int = struct.unpack("I", struct.pack("f", rgb))[0]
                            r = (rgb_int >> 16) & 0xFF
                            g = (rgb_int >> 8) & 0xFF
                            b = rgb_int & 0xFF
                        else:
                            r = g = b = 255
                    else:
                        r = g = b = 255

                if self.voxel_map is not None and self.online_voxel_size > 0:
                    # 在线体素下采样：累积体素的和与计数，最后发布/保存时取质心
                    vx = int(np.floor(point_map[0] / self.online_voxel_size))
                    vy = int(np.floor(point_map[1] / self.online_voxel_size))
                    vz = int(np.floor(point_map[2] / self.online_voxel_size))
                    key = (vx, vy, vz)
                    if self.map_lock is not None:
                        with self.map_lock:
                            if key not in self.voxel_map:
                                if self.save_rgb:
                                    self.voxel_map[key] = [
                                        float(point_map[0]),
                                        float(point_map[1]),
                                        float(point_map[2]),
                                        float(r),
                                        float(g),
                                        float(b),
                                        1.0,
                                    ]
                                else:
                                    self.voxel_map[key] = [
                                        float(point_map[0]),
                                        float(point_map[1]),
                                        float(point_map[2]),
                                        1.0,
                                    ]
                            else:
                                acc = self.voxel_map[key]
                                acc[0] += float(point_map[0])
                                acc[1] += float(point_map[1])
                                acc[2] += float(point_map[2])
                                if self.save_rgb:
                                    acc[3] += float(r)
                                    acc[4] += float(g)
                                    acc[5] += float(b)
                                    acc[6] += 1.0
                                else:
                                    acc[3] += 1.0
                    else:
                        # 极端情况（无锁）
                        if key not in self.voxel_map:
                            if self.save_rgb:
                                self.voxel_map[key] = [
                                    float(point_map[0]),
                                    float(point_map[1]),
                                    float(point_map[2]),
                                    float(r),
                                    float(g),
                                    float(b),
                                    1.0,
                                ]
                            else:
                                self.voxel_map[key] = [
                                    float(point_map[0]),
                                    float(point_map[1]),
                                    float(point_map[2]),
                                    1.0,
                                ]
                        else:
                            acc = self.voxel_map[key]
                            acc[0] += float(point_map[0])
                            acc[1] += float(point_map[1])
                            acc[2] += float(point_map[2])
                            if self.save_rgb:
                                acc[3] += float(r)
                                acc[4] += float(g)
                                acc[5] += float(b)
                                acc[6] += 1.0
                            else:
                                acc[3] += 1.0
                else:
                    # 非在线体素模式：临时列表收集，后续一次性拼接为 np.float32 数组
                    new_xyz.append([point_map[0], point_map[1], point_map[2]])
                    if self.save_rgb:
                        new_rgb.append([r, g, b])

            # 写入存储结构
            if self.voxel_map is None:
                if new_xyz:
                    new_xyz_np = np.array(new_xyz, dtype=np.float32)
                    self.points_xyz = np.concatenate(
                        [self.points_xyz, new_xyz_np], axis=0
                    )
                    if self.save_rgb:
                        new_rgb_np = np.array(new_rgb, dtype=np.uint8)
                        self.points_rgb = np.concatenate(
                            [self.points_rgb, new_rgb_np], axis=0
                        )
            self.frame_count += 1

            if self.frame_count % 10 == 0:
                if self.voxel_map is not None:
                    rospy.loginfo(
                        f"已累积 {len(self.voxel_map)} 个体素中心，来自 {self.frame_count} 帧"
                    )
                else:
                    rospy.loginfo(
                        f"已累积 {self.points_xyz.shape[0]} 个点，来自 {self.frame_count} 帧"
                    )

        except Exception as e:
            rospy.logerr(f"处理点云时出错: {e}")

    def quaternion_to_rotation_matrix(self, qx, qy, qz, qw):
        """四元数转旋转矩阵"""
        R = np.array(
            [
                [
                    1 - 2 * (qy**2 + qz**2),
                    2 * (qx * qy - qz * qw),
                    2 * (qx * qz + qy * qw),
                ],
                [
                    2 * (qx * qy + qz * qw),
                    1 - 2 * (qx**2 + qz**2),
                    2 * (qy * qz - qx * qw),
                ],
                [
                    2 * (qx * qz - qy * qw),
                    2 * (qy * qz + qx * qw),
                    1 - 2 * (qx**2 + qy**2),
                ],
            ]
        )
        return R

    def publish_accumulated_cloud(self, event=None):
        """发布累积的点云用于可视化"""
        # 准备 XYZ 数据
        if self.voxel_map is not None:
            # 快照，避免遍历过程中大小变化
            if self.map_lock is not None:
                with self.map_lock:
                    values_snapshot = list(self.voxel_map.values())
            else:
                values_snapshot = list(self.voxel_map.values())
            if len(values_snapshot) == 0:
                return
            # 计算各体素质心
            centroids = np.zeros((len(values_snapshot), 3), dtype=np.float32)
            idx = 0
            for v in values_snapshot:
                if self.save_rgb:
                    count = max(1.0, v[6])
                    centroids[idx, 0] = v[0] / count
                    centroids[idx, 1] = v[1] / count
                    centroids[idx, 2] = v[2] / count
                else:
                    count = max(1.0, v[3])
                    centroids[idx, 0] = v[0] / count
                    centroids[idx, 1] = v[1] / count
                    centroids[idx, 2] = v[2] / count
                idx += 1
            points_to_publish = centroids
        else:
            if self.points_xyz is None or self.points_xyz.shape[0] == 0:
                return
            points_to_publish = self.points_xyz

        try:
            # 采样点云（如果太多）
            if points_to_publish.shape[0] > 100000:
                step = max(1, points_to_publish.shape[0] // 100000)
                points_to_publish = points_to_publish[::step]

            # 创建 PointCloud2 消息
            header = rospy.Header()
            header.stamp = rospy.Time.now()
            header.frame_id = self.target_frame

            # 转换为 PointCloud2 格式
            points_xyz = [
                (float(p[0]), float(p[1]), float(p[2])) for p in points_to_publish
            ]
            cloud_msg = pc2.create_cloud_xyz32(header, points_xyz)

            self.accumulated_pub.publish(cloud_msg)

        except Exception as e:
            rospy.logerr(f"发布累积点云时出错: {e}")

    def voxel_filter(self, points, voxel_size):
        """体素滤波 - 去除重复点"""
        rospy.loginfo(f"应用体素滤波，体素大小: {voxel_size}m")

        if len(points) == 0:
            return points

        # 使用字典存储体素，支持 3 或 6 分量
        voxel_dict = {}
        has_rgb = len(points[0]) >= 6

        for point in points:
            x = point[0]
            y = point[1]
            z = point[2]

            # 计算体素索引
            vx = int(x / voxel_size)
            vy = int(y / voxel_size)
            vz = int(z / voxel_size)
            voxel_key = (vx, vy, vz)

            if voxel_key not in voxel_dict:
                if has_rgb:
                    voxel_dict[voxel_key] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0]
                else:
                    voxel_dict[voxel_key] = [0.0, 0.0, 0.0, 0]
            acc = voxel_dict[voxel_key]
            acc[0] += float(x)
            acc[1] += float(y)
            acc[2] += float(z)
            if has_rgb:
                acc[3] += float(point[3])
                acc[4] += float(point[4])
                acc[5] += float(point[5])
                acc[6] += 1
            else:
                acc[3] += 1

        # 对每个体素取平均，保持与输入相同的结构
        filtered_points = []
        if has_rgb:
            for acc in voxel_dict.values():
                count = max(1, acc[6])
                avg_x = acc[0] / count
                avg_y = acc[1] / count
                avg_z = acc[2] / count
                avg_r = int(acc[3] / count)
                avg_g = int(acc[4] / count)
                avg_b = int(acc[5] / count)
                filtered_points.append((avg_x, avg_y, avg_z, avg_r, avg_g, avg_b))
        else:
            for acc in voxel_dict.values():
                count = max(1, acc[3])
                avg_x = acc[0] / count
                avg_y = acc[1] / count
                avg_z = acc[2] / count
                filtered_points.append((avg_x, avg_y, avg_z))

        rospy.loginfo(f"体素滤波: {len(points)} -> {len(filtered_points)} 点")
        return filtered_points

    def voxel_filter_fast(self, points, voxel_size):
        """使用 numpy 的快速体素滤波。支持 (N,3) 或 (N,6)。返回与输入维度一致的 numpy 数组。
        points: list/np.ndarray
        """
        if voxel_size <= 0:
            # 不做滤波
            return np.asarray(
                points, dtype=np.float32 if len(points[0]) == 3 else np.float32
            )

        # 转为 numpy，并拆分 xyz / rgb
        # 自动探测是否有 rgb
        has_rgb = len(points[0]) >= 6
        if has_rgb:
            # 构造两个数组以避免大对象复制
            if isinstance(points, np.ndarray):
                xyz = points[:, :3].astype(np.float32, copy=False)
                rgb = points[:, 3:6].astype(np.float32, copy=False)
            else:
                xyz = np.array([(p[0], p[1], p[2]) for p in points], dtype=np.float32)
                rgb = np.array([(p[3], p[4], p[5]) for p in points], dtype=np.float32)
        else:
            if isinstance(points, np.ndarray):
                xyz = points[:, :3].astype(np.float32, copy=False)
            else:
                xyz = np.array([(p[0], p[1], p[2]) for p in points], dtype=np.float32)
            rgb = None

        if xyz.shape[0] == 0:
            return np.empty((0, 6 if has_rgb else 3), dtype=np.float32)

        # 量化到体素索引
        vox = np.floor(xyz / float(voxel_size)).astype(np.int64)
        # 基于每行唯一组合做分组
        keys, inv, counts = np.unique(
            vox, axis=0, return_inverse=True, return_counts=True
        )

        # 对每个分量做组内求和后除以计数，得到质心
        cx = np.bincount(inv, weights=xyz[:, 0]) / counts
        cy = np.bincount(inv, weights=xyz[:, 1]) / counts
        cz = np.bincount(inv, weights=xyz[:, 2]) / counts
        centroids = np.stack([cx, cy, cz], axis=1).astype(np.float32)

        if has_rgb and rgb is not None:
            cr = np.bincount(inv, weights=rgb[:, 0]) / counts
            cg = np.bincount(inv, weights=rgb[:, 1]) / counts
            cb = np.bincount(inv, weights=rgb[:, 2]) / counts
            colors = np.stack([cr, cg, cb], axis=1).astype(np.float32)
            out = np.concatenate([centroids, colors], axis=1)
        else:
            out = centroids

        rospy.loginfo(f"体素滤波(FAST): {xyz.shape[0]} -> {out.shape[0]} 点")
        return out

    def statistical_outlier_filter(self, points, k=50, std_ratio=1.0):
        """统计滤波 - 去除离群点"""
        rospy.loginfo(f"应用统计滤波，k={k}, std_ratio={std_ratio}")

        if len(points) < k:
            return points
        if len(points) > self.max_stat_points:
            rospy.logwarn(
                f"点数 {len(points)} 超过统计滤波上限 {self.max_stat_points}，跳过统计滤波以节省内存/时间"
            )
            return points

        # 转换为 numpy 数组
        points_xyz = np.array([(p[0], p[1], p[2]) for p in points])

        # 计算每个点到最近 k 个点的平均距离
        from scipy.spatial import cKDTree

        tree = cKDTree(points_xyz)

        mean_distances = []
        for i, point in enumerate(points_xyz):
            distances, _ = tree.query(point, k=k + 1)  # +1 因为包括自己
            mean_dist = np.mean(distances[1:])  # 排除自己
            mean_distances.append(mean_dist)

        mean_distances = np.array(mean_distances)

        # 计算阈值
        global_mean = np.mean(mean_distances)
        global_std = np.std(mean_distances)
        threshold = global_mean + std_ratio * global_std

        # 过滤离群点
        filtered_points = [
            points[i] for i in range(len(points)) if mean_distances[i] <= threshold
        ]

        rospy.loginfo(f"统计滤波: {len(points)} -> {len(filtered_points)} 点")
        return filtered_points

    def save_pcd(self, points, filename):
        """保存为 PCD 文件
        points: 列表或数组。
          - 若保存 RGB: [(x,y,z,r,g,b), ...]
          - 若不保存 RGB: [(x,y,z), ...]
        """
        rospy.loginfo(f"保存点云到: {filename}")
        has_rgb = self.save_rgb and (len(points) > 0) and (len(points[0]) >= 6)

        if not self.pcd_binary:
            # ASCII 写出（慢，但可读）
            with open(filename, "w") as f:
                f.write("# .PCD v0.7 - Point Cloud Data file format\n")
                f.write("VERSION 0.7\n")
                if has_rgb:
                    f.write("FIELDS x y z rgb\n")
                    f.write("SIZE 4 4 4 4\n")
                    f.write("TYPE F F F U\n")
                    f.write("COUNT 1 1 1 1\n")
                else:
                    f.write("FIELDS x y z\n")
                    f.write("SIZE 4 4 4\n")
                    f.write("TYPE F F F\n")
                    f.write("COUNT 1 1 1\n")
                f.write(f"WIDTH {len(points)}\n")
                f.write("HEIGHT 1\n")
                f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
                f.write(f"POINTS {len(points)}\n")
                f.write("DATA ascii\n")
                if has_rgb:
                    for point in points:
                        x, y, z, r, g, b = point
                        rgb_int = (int(r) << 16) | (int(g) << 8) | int(b)
                        f.write(
                            f"{float(x):.6f} {float(y):.6f} {float(z):.6f} {rgb_int}\n"
                        )
                else:
                    for point in points:
                        x, y, z = point
                        f.write(f"{float(x):.6f} {float(y):.6f} {float(z):.6f}\n")
            rospy.loginfo(f"保存完成: {len(points)} 个点 (ASCII)")
            return

        # 二进制写出（快且文件更小）
        N = len(points)
        with open(filename, "wb") as f:
            header = "# .PCD v0.7 - Point Cloud Data file format\n"
            header += "VERSION 0.7\n"
            if has_rgb:
                header += "FIELDS x y z rgb\n"
                header += "SIZE 4 4 4 4\n"
                header += "TYPE F F F U\n"
                header += "COUNT 1 1 1 1\n"
            else:
                header += "FIELDS x y z\n"
                header += "SIZE 4 4 4\n"
                header += "TYPE F F F\n"
                header += "COUNT 1 1 1\n"
            header += f"WIDTH {N}\n"
            header += "HEIGHT 1\n"
            header += "VIEWPOINT 0 0 0 1 0 0 0\n"
            header += f"POINTS {N}\n"
            header += "DATA binary\n"
            f.write(header.encode("ascii"))

            # 构造二进制数据块
            if has_rgb:
                # xyz float32 + rgb uint32
                buf = bytearray()
                pack_xyz = struct.Struct("<fff").pack
                pack_rgbu = struct.Struct("<I").pack
                for x, y, z, r, g, b in points:
                    f.write(pack_xyz(float(x), float(y), float(z)))
                    rgb_int = (int(r) << 16) | (int(g) << 8) | int(b)
                    f.write(pack_rgbu(rgb_int))
            else:
                pack_xyz = struct.Struct("<fff").pack
                for x, y, z in points:
                    f.write(pack_xyz(float(x), float(y), float(z)))
        rospy.loginfo(f"保存完成: {len(points)} 个点 (binary)")

    def start_recording_callback(self, req):
        """开始记录服务"""
        self.is_recording = True
        rospy.loginfo("开始记录点云")
        return TriggerResponse(success=True, message="开始记录点云")

    def stop_recording_callback(self, req):
        """停止记录服务"""
        self.is_recording = False
        # 统计当前累计数量（体素或点）
        if self.voxel_map is not None:
            if self.map_lock is not None:
                with self.map_lock:
                    count = len(self.voxel_map)
            else:
                count = len(self.voxel_map)
            rospy.loginfo(f"停止记录点云，共累积 {count} 个体素中心")
            return TriggerResponse(
                success=True, message=f"停止记录，累积 {count} 个体素中心"
            )
        else:
            count = int(self.points_xyz.shape[0]) if self.points_xyz is not None else 0
            rospy.loginfo(f"停止记录点云，共累积 {count} 个点")
            return TriggerResponse(success=True, message=f"停止记录，累积 {count} 个点")

    def clear_map_callback(self, req):
        """清空地图服务"""
        if self.voxel_map is not None:
            if self.map_lock is not None:
                with self.map_lock:
                    old_count = len(self.voxel_map)
                    self.voxel_map.clear()
            else:
                old_count = len(self.voxel_map)
                self.voxel_map.clear()
            self.frame_count = 0
            rospy.loginfo(f"清空累积点云，删除了 {old_count} 个体素中心")
            return TriggerResponse(
                success=True, message=f"已清空 {old_count} 个体素中心"
            )
        else:
            old_count = (
                int(self.points_xyz.shape[0]) if self.points_xyz is not None else 0
            )
            self.points_xyz = np.empty((0, 3), dtype=np.float32)
            if self.save_rgb:
                self.points_rgb = np.empty((0, 3), dtype=np.uint8)
            self.frame_count = 0
            rospy.loginfo(f"清空累积点云，删除了 {old_count} 个点")
            return TriggerResponse(success=True, message=f"已清空 {old_count} 个点")

    def save_map_callback(self, req):
        """保存地图服务"""
        try:
            rospy.loginfo("=" * 60)
            rospy.loginfo("开始保存地图...")

            # 准备原始点集合（根据存储模式提取）
            raw_points = []
            if self.voxel_map is not None:
                if self.map_lock is not None:
                    with self.map_lock:
                        values_snapshot = list(self.voxel_map.values())
                else:
                    values_snapshot = list(self.voxel_map.values())
                if len(values_snapshot) == 0:
                    return SaveMapResponse(
                        success=False,
                        message="没有累积的点云数据",
                        saved_path="",
                        total_points=0,
                        filtered_points=0,
                    )
                for v in values_snapshot:
                    if self.save_rgb:
                        count = max(1.0, v[6])
                        raw_points.append(
                            (
                                v[0] / count,
                                v[1] / count,
                                v[2] / count,
                                int(v[3] / count),
                                int(v[4] / count),
                                int(v[5] / count),
                            )
                        )
                    else:
                        count = max(1.0, v[3])
                        raw_points.append((v[0] / count, v[1] / count, v[2] / count))
            else:
                if self.points_xyz is None or self.points_xyz.shape[0] == 0:
                    return SaveMapResponse(
                        success=False,
                        message="没有累积的点云数据",
                        saved_path="",
                        total_points=0,
                        filtered_points=0,
                    )
                if (
                    self.save_rgb
                    and self.points_rgb is not None
                    and self.points_rgb.shape[0] == self.points_xyz.shape[0]
                ):
                    for i in range(self.points_xyz.shape[0]):
                        x, y, z = self.points_xyz[i]
                        r, g, b = self.points_rgb[i]
                        raw_points.append(
                            (float(x), float(y), float(z), int(r), int(g), int(b))
                        )
                else:
                    for i in range(self.points_xyz.shape[0]):
                        x, y, z = self.points_xyz[i]
                        raw_points.append((float(x), float(y), float(z)))

            if len(raw_points) == 0:
                return SaveMapResponse(
                    success=False,
                    message="没有累积的点云数据",
                    saved_path="",
                    total_points=0,
                    filtered_points=0,
                )

            total_points = len(raw_points)
            rospy.loginfo(f"原始点云: {total_points} 个点")

            # 复制点云
            filtered_points = list(raw_points)

            # 体素滤波（使用 numpy 加速）
            voxel_size = req.voxel_size if req.voxel_size > 0 else 0.05
            filtered_np = self.voxel_filter_fast(filtered_points, voxel_size)
            # 将 numpy 结果转换为后续保存需要的结构
            if self.save_rgb and filtered_np.shape[1] >= 6:
                filtered_points = [
                    (
                        float(p[0]),
                        float(p[1]),
                        float(p[2]),
                        int(p[3]),
                        int(p[4]),
                        int(p[5]),
                    )
                    for p in filtered_np
                ]
            else:
                filtered_points = [
                    (float(p[0]), float(p[1]), float(p[2])) for p in filtered_np
                ]

            # 统计滤波
            if req.apply_statistical_filter and len(filtered_points) > 100:
                try:
                    filtered_points = self.statistical_outlier_filter(filtered_points)
                except Exception as e:
                    rospy.logwarn(f"统计滤波失败: {e}，跳过")

            # 生成文件名
            if req.filename:
                filename = req.filename
                if not filename.endswith(".pcd"):
                    filename += ".pcd"
            else:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"map_{timestamp}.pcd"

            filepath = os.path.join(self.save_dir, filename)

            # 保存 PCD 文件（根据是否保存 RGB）
            self.save_pcd(filtered_points, filepath)

            rospy.loginfo("=" * 60)
            rospy.loginfo(f"地图保存成功!")
            rospy.loginfo(f"文件路径: {filepath}")
            rospy.loginfo(f"原始点数: {total_points}")
            rospy.loginfo(f"滤波后点数: {len(filtered_points)}")
            rospy.loginfo(f"压缩率: {len(filtered_points)/total_points*100:.1f}%")
            rospy.loginfo("=" * 60)

            return SaveMapResponse(
                success=True,
                message="地图保存成功",
                saved_path=filepath,
                total_points=total_points,
                filtered_points=len(filtered_points),
            )

        except Exception as e:
            rospy.logerr(f"保存地图失败: {e}")
            import traceback

            traceback.print_exc()
            return SaveMapResponse(
                success=False,
                message=f"保存失败: {str(e)}",
                saved_path="",
                total_points=0,
                filtered_points=0,
            )


def main():
    try:
        saver = PointCloudSaver()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
