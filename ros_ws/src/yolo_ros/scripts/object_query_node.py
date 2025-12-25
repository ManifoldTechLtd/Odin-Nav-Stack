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
from vision_msgs.msg import Detection3DArray, Detection2DArray
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped
import json
import threading
import sys
import re
import tf2_ros
import numpy as np
import time
import os
from contextlib import contextmanager

# Speech recognition dependency (optional)
try:
    from vosk import Model as VoskModel, KaldiRecognizer
    import sounddevice as sd
    import queue as pyqueue

    _VOICE_DEPS_OK = True
except Exception:
    _VOICE_DEPS_OK = False


class ObjectQueryNode:
    def __init__(self):
        rospy.init_node("object_query_node", anonymous=True)

        # Store the latest test results
        self.latest_detections = None  # 3D
        self.latest_detections_2d = None  # 2D
        self.class_names = {}
        self.lock = threading.Lock()

        # Subscribe to 3D detection results
        rospy.Subscriber(
            "/yolo_detections_3d",
            Detection3DArray,
            self.detections_callback,
            queue_size=10,
        )
        # Subscribe to 2D detection results (fallback for list display when no depth)
        rospy.Subscriber(
            "/yolo_detections",
            Detection2DArray,
            self.detections2d_callback,
            queue_size=10,
        )

        # Subscribe to class names
        rospy.Subscriber("/yolo_class_names", String, self.class_names_callback)

        # Publish RViz markers
        self.marker_pub = rospy.Publisher("/object_markers", MarkerArray, queue_size=10)

        # Publish query results (optional, for other nodes to subscribe)
        self.result_pub = rospy.Publisher("/object_query_result", String, queue_size=10)

        # Publish navigation goal point visualization
        self.goal_marker_pub = rospy.Publisher(
            "/navigation_goal_marker", Marker, queue_size=10
        )

        # Publish navigation goal to move_base_simple/goal
        self.goal_pub = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=10
        )

        # TF listener (for coordinate conversion)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Navigation parameters
        self.direction_distance = rospy.get_param(
            "~direction_distance", 1.0
        )  # Directional offset distance (meters)
        self.camera_frame = rospy.get_param("~camera_frame", "camera_link")
        self.target_frame = rospy.get_param("~target_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "odin1_base_link")
        # Home/Return origin instruction configuration
        self.home_frame = rospy.get_param(
            "~home_frame", "odom"
        )  # Default publish to odom origin
        self.home_x = float(rospy.get_param("~home_x", 0.0))
        self.home_y = float(rospy.get_param("~home_y", 0.0))
        self.home_yaw = float(rospy.get_param("~home_yaw", 0.0))  # Radians, can be 0
        # Voice parameters
        self.enable_voice = rospy.get_param("~enable_voice", False)
        self.voice_model_path = rospy.get_param(
            "~voice_model_path", rospy.get_param("~voice_model", "./voicemodel")
        )
        self.voice_sample_rate = int(rospy.get_param("~voice_sample_rate", 16000))
        self.voice_block_size = int(rospy.get_param("~voice_block_size", 4000))
        self.voice_device = rospy.get_param("~voice_device", None)
        self.voice_interval_sec = float(rospy.get_param("~voice_interval_sec", 3.0))
        # voice endpoint control
        self.voice_end_silence = float(
            rospy.get_param("~voice_end_silence", 0.8)
        )  # continuous silence threshold, determine end of sentence
        self.voice_min_duration = float(
            rospy.get_param("~voice_min_duration", 1.2)
        )  # minimum sentence duration
        self.voice_max_duration = float(
            rospy.get_param("~voice_max_duration", 4.0)
        )  # maximum sentence duration
        self.voice_debounce = float(
            rospy.get_param("~voice_debounce", 1.5)
        )  # cooldown after one recognition
        self.voice_partial = bool(rospy.get_param("~voice_partial", False))
        self._voice_thread = None
        self._voice_stop = threading.Event()
        self._voice_ready = False

        rospy.loginfo("Object Query Node initialized.")
        rospy.loginfo(f"Direction distance: {self.direction_distance}m")
        rospy.loginfo("Waiting for class names and detections...")

        # Wait for class names to load
        rospy.sleep(1.0)

        # Pre-initialize voice recognition (optional), but do not start thread by default, controlled by interactive interface
        self.voice_active = False
        if self.enable_voice:
            if not _VOICE_DEPS_OK:
                rospy.logwarn(
                    "Enable voice input parameter is true, but dependencies(vosk, sounddevice) are not installed, will only provide text mode."
                )
                self.enable_voice = False
            elif not os.path.isdir(self.voice_model_path):
                rospy.logwarn(
                    f"Enable voice input parameter is true, but model directory does not exist: {self.voice_model_path}, will only provide text mode."
                )
                self.enable_voice = False
            else:
                try:
                    rospy.loginfo(f"Loading Vosk model: {self.voice_model_path}")
                    self._vosk_model = VoskModel(self.voice_model_path)
                    self._recognizer = KaldiRecognizer(
                        self._vosk_model, self.voice_sample_rate
                    )
                    self._recognizer.SetWords(True)
                    self._audio_q = pyqueue.Queue()
                    self._voice_ready = True
                    rospy.loginfo(
                        "Voice recognition ready, can be enabled in the interactive interface."
                    )
                except Exception as e:
                    rospy.logwarn(
                        f"Failed to initialize voice model, will only provide text mode: {e}"
                    )
                    self.enable_voice = False

    def class_names_callback(self, msg):
        """receive class name mapping"""
        with self.lock:
            self.class_names = json.loads(msg.data)
            rospy.loginfo(f"Loaded {len(self.class_names)} class names.")

    def detections_callback(self, msg):
        """receive latest 3D detection results"""
        with self.lock:
            self.latest_detections = msg
            self._last_3d_count = len(msg.detections)
            self._last_3d_time = rospy.Time.now()
            # limit log frequency
            if (
                not hasattr(self, "_last_3d_log")
                or (rospy.Time.now() - self._last_3d_log).to_sec() > 2.0
            ):
                self._last_3d_log = rospy.Time.now()

    def detections2d_callback(self, msg):
        """receive latest 2D detection results (for fallback display when depth is unavailable)"""
        with self.lock:
            self.latest_detections_2d = msg
            self._last_2d_count = len(msg.detections)
            self._last_2d_time = rospy.Time.now()
            if (
                not hasattr(self, "_last_2d_log")
                or (rospy.Time.now() - self._last_2d_log).to_sec() > 2.0
            ):
                self._last_2d_log = rospy.Time.now()

    def find_object(self, object_name):
        """
        Find objects with the specified name
        Returns: [(class_id, class_name, x, y, z, score), ...]
        """
        with self.lock:
            if not self.class_names:
                rospy.logwarn("Class names not loaded yet.")
                return []

            if self.latest_detections is None:
                rospy.logwarn("No detections received yet.")
                return []

            # find object, support partial match and case-insensitive
            object_name_lower = object_name.lower().strip()
            found_objects = []

            for detection in self.latest_detections.detections:
                if len(detection.results) > 0:
                    class_id = detection.results[0].id
                    score = detection.results[0].score
                    class_name = self.class_names.get(
                        str(class_id), f"unknown_{class_id}"
                    )

                    # ensure class_name is string
                    class_name_str = str(class_name)

                    # check name match
                    if object_name_lower in class_name_str.lower():
                        # original in camera frame
                        x_cam = detection.bbox.center.position.x
                        y_cam = detection.bbox.center.position.y
                        z_cam = detection.bbox.center.position.z
                        # transform to map frame
                        map_pos = self.transform_camera_to_map((x_cam, y_cam, z_cam))
                        if map_pos is None:
                            continue
                        x_map, y_map, z_map = map_pos
                        found_objects.append(
                            (class_id, class_name_str, x_map, y_map, z_map, score)
                        )

            return found_objects

    def visualize_objects(self, objects, object_name):
        """Visualize found objects in RViz"""
        marker_array = MarkerArray()

        # Always visualize in map coordinate frame (find_object already outputs map coordinates)
        frame_id = "map"

        # Delete old markers
        delete_marker = Marker()
        delete_marker.header.frame_id = frame_id
        delete_marker.header.stamp = rospy.Time.now()
        delete_marker.ns = "detected_objects"
        delete_marker.id = 0
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        for idx, (class_id, class_name, x, y, z, score) in enumerate(objects):
            # Create            # 使用线框正方体（LINE_LIST）表示，颜色为青色
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = rospy.Time.now()
            marker.ns = "detected_objects"
            marker.id = idx
            marker.type = Marker.LINE_LIST
            marker.action = Marker.ADD

            # 线宽（仅使用 scale.x）
            marker.scale.x = 0.05

            # 颜色：青色
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 0.9

            # 以目标为中心构建正方体8个角点
            h = 0.3
            corners = [
                (x - h, y - h, z - h),  # 0
                (x + h, y - h, z - h),  # 1
                (x - h, y + h, z - h),  # 2
                (x + h, y + h, z - h),  # 3
                (x - h, y - h, z + h),  # 4
                (x + h, y - h, z + h),  # 5
                (x - h, y + h, z + h),  # 6
                (x + h, y + h, z + h),  # 7
            ]

            # 立方体的12条边（按顶点索引成对加入）
            edges = [
                (0, 1),
                (0, 2),
                (0, 4),
                (1, 3),
                (1, 5),
                (2, 3),
                (2, 6),
                (3, 7),
                (4, 5),
                (4, 6),
                (5, 7),
                (6, 7),
            ]

            pts = []
            for i, j in edges:
                p1 = Point()
                p1.x, p1.y, p1.z = corners[i]
                p2 = Point()
                p2.x, p2.y, p2.z = corners[j]
                pts.append(p1)
                pts.append(p2)
            marker.points = pts

            marker.lifetime = rospy.Duration(0)
            marker_array.markers.append(marker)

            # Create text marker
            text_marker = Marker()
            text_marker.header = marker.header
            text_marker.ns = "object_labels"
            text_marker.id = idx + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD

            text_marker.pose.position.x = x
            text_marker.pose.position.y = y
            text_marker.pose.position.z = z + 0.3
            text_marker.pose.orientation.w = 1.0

            text_marker.scale.z = 0.15

            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0

            text_marker.text = f"{class_name}\n({x:.2f}, {y:.2f}, {z:.2f})"
            text_marker.lifetime = rospy.Duration(0)
            marker_array.markers.append(text_marker)

        # Publish markers
        self.marker_pub.publish(marker_array)
        rospy.loginfo(
            f"Published {len(objects)} object markers to /object_markers, frame: {frame_id}"
        )

    def parse_navigation_command(self, command):
        """
        Parse natural language navigation commands, supporting simplified keywords and number selection (1-5).
        Returns: (action, object_name, direction, index) or None
        Note: index is 1-based (None means default nearest)
        """
        cmd_raw = command.strip()
        cmd = cmd_raw.lower()

        # Support direction keywords (includes abbreviations and synonyms)
        directions = {
            "right": "right",
            "r": "right",
            "left": "left",
            "l": "left",
            "front": "front",
            "f": "front",
            "forward": "front",
            "behind": "behind",
            "back": "behind",
            "b": "behind",
        }
        # Support Chinese direction keywords
        cn_directions = {
            "右边": "right",
            "右面": "right",
            "左边": "left",
            "左面": "left",
            "前边": "front",
            "前面": "front",
            "后边": "behind",
            "后面": "behind",
        }

        # Find direction (any occurrence is valid)
        found_direction = None
        for k, v in directions.items():
            if re.search(r"\b" + re.escape(k) + r"\b", cmd):
                found_direction = v
                break
        if not found_direction:
            for k, v in cn_directions.items():
                if k in cmd_raw:
                    found_direction = v
                    break

        if not found_direction:
            return None

        # Extract number (optional, max 1-5): supports #2, no.2, 2nd, second, 第2个, 第三 等
        index = None
        ordinals = {"first": 1, "second": 2, "third": 3, "fourth": 4, "fifth": 5}
        for word, num in ordinals.items():
            if re.search(r"\b" + word + r"\b", cmd):
                index = num
                break
        if index is None:
            m = re.search(r"(?:#|no\.?\s*)([1-5])", cmd)
            if m:
                index = int(m.group(1))
        if index is None:
            m = re.search(r"\b([1-5])(st|nd|rd|th)\b", cmd)
            if m:
                index = int(m.group(1))
        if index is None:
            m = re.search(r"第\s*([一二三四五1-5])\s*个?", cmd_raw)
            if m:
                ch = m.group(1)
                cn_map = {"一": 1, "二": 2, "三": 3, "四": 4, "五": 5}
                index = cn_map.get(ch, None) if ch in cn_map else int(ch)

        # Extract object name:
        # 1) Prioritize extracting from "of (the) <multi words>" until number or string ends
        # 1b) Chinese: Extract "到/去到/走到/移动到/运动到 … 的 <direction>" to extract …
        # 2) Otherwise match known classes (from self.class_names), prioritize longest match
        object_name = None

        # Known class names (from model mapping), sorted by length, prioritize multi-word names
        known_names = []
        try:
            if self.class_names:
                known_names = [str(v).lower() for v in self.class_names.values()]
        except Exception:
            known_names = []
        # Common fallback names (multi-word)
        fallback_names = [
            "person",
            "bottle",
            "wine glass",
            "cup",
            "fork",
            "knife",
            "spoon",
            "bowl",
            "banana",
            "apple",
            "sandwich",
            "orange",
            "broccoli",
            "carrot",
            "hot dog",
            "pizza",
            "donut",
            "cake",
            "chair",
            "couch",
            "potted plant",
            "bed",
            "dining table",
            "toilet",
            "tv",
            "laptop",
            "mouse",
            "remote",
            "keyboard",
            "cell phone",
            "microwave",
            "oven",
            "toaster",
            "sink",
            "refrigerator",
            "book",
            "clock",
            "vase",
            "scissors",
            "teddy bear",
            "hair drier",
            "toothbrush",
            "traffic light",
            "fire hydrant",
            "stop sign",
            "parking meter",
            "bench",
            "dog",
            "cat",
            "car",
            "bus",
            "truck",
            "motorcycle",
            "bicycle",
            "suitcase",
        ]
        for n in fallback_names:
            if n not in known_names:
                known_names.append(n)
        # Unique and sort by length, prioritize multi-word names
        known_names = sorted(
            list(dict.fromkeys(known_names)), key=lambda s: len(s), reverse=True
        )

        # Chinese object mapping (fixed words)
        cn_obj_map = {
            "人": "person",
            "瓶子": "bottle",
            "红酒杯": "wine glass",
            "杯子": "cup",
            "水杯": "cup",
            "叉子": "fork",
            "刀": "knife",
            "勺子": "spoon",
            "碗": "bowl",
            "香蕉": "banana",
            "苹果": "apple",
            "三明治": "sandwich",
            "橙子": "orange",
            "西兰花": "broccoli",
            "胡萝卜": "carrot",
            "热狗": "hot dog",
            "披萨": "pizza",
            "甜甜圈": "donut",
            "蛋糕": "cake",
            "椅子": "chair",
            "沙发": "couch",
            "盆栽": "potted plant",
            "床": "bed",
            "餐桌": "dining table",
            "厕所": "toilet",
            "电视": "tv",
            "笔记本": "laptop",
            "鼠标": "mouse",
            "遥控器": "remote",
            "键盘": "keyboard",
            "手机": "cell phone",
            "微波炉": "microwave",
            "烤箱": "oven",
            "烤面包机": "toaster",
            "水槽": "sink",
            "冰箱": "refrigerator",
            "书": "book",
            "时钟": "clock",
            "花瓶": "vase",
            "剪刀": "scissors",
            "泰迪熊": "teddy bear",
            "吹风机": "hair drier",
            "牙刷": "toothbrush",
            "红绿灯": "traffic light",
            "消防栓": "fire hydrant",
            "停车标志": "stop sign",
            "停车计时器": "parking meter",
            "长凳": "bench",
            "狗": "dog",
            "猫": "cat",
            "汽车": "car",
            "公交车": "bus",
            "卡车": "truck",
            "摩托车": "motorcycle",
            "自行车": "bicycle",
            "手提箱": "suitcase",
            "行李箱": "suitcase",
            "凳子": "bench",
        }
        # First try Chinese mode: 到/去到/走到/移动到/运动到 [第N个] 物体 的 方向
        m_cn = re.search(
            r"(?:到|去到|走到|移动到|运动到)\s*(第\s*([一二三四五1-5])\s*个)?\s*([\u4e00-\u9fa5A-Za-z ]+?)的\s*(右边|右侧|左边|左侧|前边|前面|后边|后面)",
            cmd_raw,
        )
        if m_cn:
            if index is None and m_cn.group(2):
                ch = m_cn.group(2)
                cn_map = {"一": 1, "二": 2, "三": 3, "四": 4, "五": 5}
                index = cn_map.get(ch, None) if ch in cn_map else int(ch)
            obj_phrase = m_cn.group(3).strip()
            # Find longest match in Chinese mapping
            chosen_cn = None
            for cn_name in sorted(
                cn_obj_map.keys(), key=lambda s: len(s), reverse=True
            ):
                if cn_name in obj_phrase:
                    chosen_cn = cn_name
                    break
            if chosen_cn:
                object_name = cn_obj_map[chosen_cn]

        # 1) of (the) <name...>
        # Capture fragment up to number or string end
        m = re.search(
            r"of\s+(?:the\s+)?([a-z0-9 _\-]+?)(?=\s*(?:#\d|no\.?\s*\d|\d(?:st|nd|rd|th)|第\s*\d\s*个|$))",
            cmd,
        )
        if m:
            candidate = re.sub(r"\s+", " ", m.group(1).strip())
            # Process leading index: "第N个 name" / "#N name" / "no.N name" / "Nst|nd|rd|th name"
            if candidate:
                # 第N个 name
                m_idx = re.match(r"^第\s*([1-5])\s*个\s+", candidate)
                if m_idx:
                    if index is None:
                        index = int(m_idx.group(1))
                    candidate = candidate[m_idx.end() :].strip()
                else:
                    # #N or no.N
                    m_idx = re.match(r"^(?:#|no\.?\s*)([1-5])\s+", candidate)
                    if m_idx:
                        if index is None:
                            index = int(m_idx.group(1))
                        candidate = candidate[m_idx.end() :].strip()
                    else:
                        # Nst|nd|rd|th name
                        m_idx = re.match(r"^([1-5])(st|nd|rd|th)\s+", candidate)
                        if m_idx:
                            if index is None:
                                index = int(m_idx.group(1))
                            candidate = candidate[m_idx.end() :].strip()
            # Find longest match in known names
            for name in known_names:
                if candidate and candidate in name:
                    object_name = name
                    break
            if object_name is None and candidate:
                object_name = candidate  # Fallback to original fragment

        # 2) If not matched, search for known classes in full sentence (prioritize multi-word long names)
        if not object_name:
            for name in known_names:
                # Use word boundary match: replace spaces in name with \\s+ to handle multiple spaces
                pat = r"\b" + re.sub(r"\s+", r"\\s+", re.escape(name)) + r"\b"
                if re.search(pat, cmd):
                    object_name = name
                    break

        if not object_name:
            return None

        return ("move", object_name, found_direction, index)

    def calculate_target_position(self, object_pos, direction, distance):
        """
        Calculate target position in camera frame based on object position and direction

        Camera frame definition:
        - X axis: right
        - Y axis: down
        - Z axis: forward (optical axis direction)

        Parameters:
            object_pos: (x, y, z) Object position in camera frame
            direction: 'right', 'left', 'front', 'behind'
            distance: Offset distance (meters)

        Returns:
            (x, y, z) Target position in camera frame
        """
        x, y, z = object_pos

        if direction == "right":
            # Right: X increases
            return (x + distance, y, z)
        elif direction == "left":
            # Left: X decreases
            return (x - distance, y, z)
        elif direction == "front":
            # Forward: Z decreases (closer to camera)
            return (x, y, z - distance)
        elif direction == "behind":
            # Backward: Z increases (farther from camera)
            return (x, y, z + distance)
        else:
            return (x, y, z)

    def calculate_target_position_relative_to_robot(
        self, object_pos_map, direction, distance
    ):
        """
        Calculate target position relative to robot in base_link frame based on object position and direction

        Rules (base_link frame, ROS standard):
        - front: x += d
        - behind: x -= d
        - left: y += d
        - right: y -= d

        Implementation: rotate base_link unit axis vectors to map frame, as offset direction base.
        """
        x_obj, y_obj, z_obj = object_pos_map
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame,  # map
                self.base_frame,  # base_link
                rospy.Time(0),
                rospy.Duration(0.5),
            )
        except Exception as e:
            rospy.logwarn(
                f"Failed to get {self.target_frame}->{self.base_frame} transform, using map direction offset: {e}"
            )
            return self.calculate_target_position_in_map(
                object_pos_map, direction, distance
            )

        q = tf.transform.rotation
        R = self.quaternion_to_rotation_matrix(
            q.x, q.y, q.z, q.w
        )  # 3x3, from base to map
        ex_map = np.array([R[0][0], R[1][0], 0.0])  # base x axis in map
        ey_map = np.array([R[0][1], R[1][1], 0.0])  # base y axis in map

        dx, dy = 0.0, 0.0
        if direction == "front":
            dx = distance
        elif direction == "behind":
            dx = -distance
        elif direction == "left":
            dy = distance
        elif direction == "right":
            dy = -distance

        offset_map = dx * ex_map + dy * ey_map
        target = (x_obj + float(offset_map[0]), y_obj + float(offset_map[1]), z_obj)
        return target

    def calculate_target_position_in_map(self, object_pos_map, direction, distance):
        """
        Calculate target position in map frame based on object position and direction

        Map frame definition (ROS standard):
        - X axis: forward
        - Y axis: left
        - Z axis: up

        Parameters:
            object_pos_map: (x, y, z) Object position in map frame
            direction: 'right', 'left', 'front', 'behind'
            distance: Offset distance (meters)

        Returns:
            (x, y, z) Target position in map frame
        """
        x, y, z = object_pos_map

        if direction == "right":
            # Right: Y decreases (map Y left is positive)
            return (x, y - distance, z)
        elif direction == "left":
            # Left: Y increases
            return (x, y + distance, z)
        elif direction == "front":
            # Forward: X increases
            return (x + distance, y, z)
        elif direction == "behind":
            # Behind: X decreases
            return (x - distance, y, z)
        else:
            return (x, y, z)

    def transform_camera_to_map(self, camera_pos):
        """
        Transform point from camera frame to map frame

        Parameters:
            camera_pos: (x, y, z) Point in camera frame

        Returns:
            (x, y, z) Point in map frame, or None if transformation fails
        """
        try:
            # Get camera_link -> map transform
            transform = self.tf_buffer.lookup_transform(
                self.target_frame, self.camera_frame, rospy.Time(0), rospy.Duration(1.0)
            )

            # translation
            tx = transform.transform.translation.x
            ty = transform.transform.translation.y
            tz = transform.transform.translation.z

            # rotation quaternion
            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w

            # quaternion to rotation matrix
            R = self.quaternion_to_rotation_matrix(qx, qy, qz, qw)

            # apply transform
            point_camera = np.array(camera_pos)
            point_map = R @ point_camera + np.array([tx, ty, tz])

            return tuple(point_map)

        except Exception as e:
            rospy.logerr(f"Failed to transform camera to map: {e}")
            return None

    def quaternion_to_rotation_matrix(self, qx, qy, qz, qw):
        """Convert quaternion to rotation matrix"""
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

    def send_navigation_goal(self, target_pos_map):
        """
        Send navigation goal to /move_base_simple/goal

        Parameters:
            target_pos_map: (x, y, z) Target position in map frame
        """
        # Create PoseStamped message
        goal = PoseStamped()
        goal.header.frame_id = self.target_frame
        goal.header.stamp = rospy.Time.now()

        # Set position
        goal.pose.position.x = target_pos_map[0]
        goal.pose.position.y = target_pos_map[1]
        goal.pose.position.z = 0.0  # Ground navigation, z=0

        # Set orientation (default orientation, can be calculated based on the angle to the target object)
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0

        # Publish to move_base_simple/goal
        try:
            # subs = self.goal_pub.get_num_connections()
            # if subs == 0:
            #     rospy.logwarn("/move_base_simple/goal has no subscribers. Please confirm that the navigation stack/rviz is subscribing to this topic.")
            self.goal_pub.publish(goal)
            rospy.loginfo(
                f"Published navigation goal to /move_base_simple/goal: ({target_pos_map[0]:.2f}, {target_pos_map[1]:.2f}, {target_pos_map[2]:.2f})"
            )
        except Exception as e:
            rospy.logerr(f"Failed to publish navigation goal: {e}")
            return False

        # clear previous navigation goal marker
        delete_marker = Marker()
        delete_marker.header.frame_id = self.target_frame
        delete_marker.header.stamp = rospy.Time.now()
        delete_marker.ns = "navigation_goal"
        delete_marker.id = 0
        delete_marker.action = Marker.DELETEALL
        self.goal_marker_pub.publish(delete_marker)

        marker = Marker()
        marker.header.frame_id = self.target_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "navigation_goal"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # arrow points to the target point
        marker.pose.position.x = target_pos_map[0]
        marker.pose.position.y = target_pos_map[1]
        marker.pose.position.z = 0.5  # slightly raise for better visibility
        marker.pose.orientation.w = 1.0

        # arrow size
        marker.scale.x = 1.0  # arrow length
        marker.scale.y = 0.2  # arrow width
        marker.scale.z = 0.2  # arrow height

        # red color
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.lifetime = rospy.Duration(0)  # marker lifetime

        self.goal_marker_pub.publish(marker)
        rospy.loginfo(
            "Published navigation goal marker to RViz (topic: /navigation_goal_marker)"
        )
        return True

    def _publish_goal(self, frame_id: str, x: float, y: float, yaw: float = 0.0):
        """Publish any coordinate frame goal to /move_base_simple/goal."""
        goal = PoseStamped()
        goal.header.frame_id = frame_id
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.position.z = 0.0
        # yaw to quaternion
        qz = np.sin(yaw * 0.5)
        qw = np.cos(yaw * 0.5)
        goal.pose.orientation.z = float(qz)
        goal.pose.orientation.w = float(qw)
        try:
            # if self.goal_pub.get_num_connections() == 0:
            #     rospy.logwarn("/move_base_simple/goal has no subscribers. Please confirm that the navigation stack/rviz is subscribing to this topic.")
            self.goal_pub.publish(goal)
            rospy.loginfo(
                f"[GOAL] Published: frame={frame_id} pos=({x:.2f},{y:.2f}) yaw={yaw:.2f}"
            )
            return True
        except Exception as e:
            rospy.logerr(f"Failed to publish navigation goal: {e}")
            return False

    def send_home_goal(self):
        """Publish navigation goal to return to home position (default odom origin)."""
        ok = self._publish_goal(
            self.home_frame, self.home_x, self.home_y, self.home_yaw
        )
        if ok:
            # Simple visualization (using goal coordinate frame)
            marker = Marker()
            marker.header.frame_id = self.home_frame
            marker.header.stamp = rospy.Time.now()
            marker.ns = "navigation_goal_to_home"
            marker.id = 0
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.position.x = self.home_x
            marker.pose.position.y = self.home_y
            marker.pose.position.z = 0.5
            marker.pose.orientation.w = 1.0
            marker.scale.x = 1.0
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.r = 0.0
            marker.color.g = 0.2
            marker.color.b = 1.0
            marker.color.a = 1.0
            self.goal_marker_pub.publish(marker)
        return ok

    def process_simple_command(self, text: str) -> bool:
        """Process simple commands that do not depend on detection results, such as 回来/回去 (publish to origin). Returns True if processed."""
        raw = (text or "").strip()
        low = raw.lower()
        # Chinese synonyms with English equivalents
        home_triggers = [
            "回来",
            "回去",
            "回到原点",
            "返回",
            "回家",
            "返回原点",
            "home",
            "go home",
            "back",
            "go back",
            "return",
        ]
        if any(k in raw for k in home_triggers) or any(
            k in low for k in ["home", "go home", "go back", "return"]
        ):
            rospy.loginfo(
                f"[CMD] Triggered home command -> publish {self.home_frame} origin ({self.home_x},{self.home_y})"
            )
            ok = self.send_home_goal()
            if ok:
                print(
                    f"Published home goal: frame={self.home_frame} ({self.home_x:.2f}, {self.home_y:.2f})"
                )
            else:
                print("Failed to publish home goal.")
            return True
        return False

    def process_navigation_command(self, command):
        """
        Process navigation command and send goal to move_base_simple/goal

        Parameters:
            command: Natural language command, e.g. "Move to the right of the person"

        Returns:
            True if successful, False if failed
        """
        print(f"\n{'='*60}")
        print(f"Processing navigation command: '{command}'")
        print(f"{'='*60}")

        # 1. Parse command
        parsed = self.parse_navigation_command(command)
        if not parsed:
            print("Failed to parse command")
            print(
                "Command format: Move to the [right/left/front/behind] of the [object] [#N | Nth | 第N个] / 走到[第N个]中文物体的[左/右/前/后]边"
            )
            print("Example 1: Move to the right of the person")
            print("Example 2: right person #2   (simple)")
            print("Example 3: go left chair 2nd  (second)")
            print("Example 4: behind car 3rd  (third)")
            print("Example 5: (CN)走到第三个人的左边")
            print("Example 6: (CN)运动到沙发的前边")
            return False

        action, object_name, direction, index = parsed
        print(f"Parsed command:")
        print(f"  - Action: {action}")
        print(f"  - Object: {object_name}")
        print(f"  - Direction: {direction}")
        if index:
            print(f"  - Index: {index}")

        # Find object (returns map frame position)
        objects = self.find_object(object_name)
        if not objects:
            print(f"Object '{object_name}' not found")
            return False

        # Sort objects by distance (closest first)
        def obj_dist_sq(o):
            # o = (class_id, class_name, x, y, z, score)
            return o[2] ** 2 + o[3] ** 2

        objects = sorted(objects, key=obj_dist_sq)

        # Select target: if index (1-5) is provided, select corresponding item, otherwise default to nearest (1st)
        if index is not None and 1 <= index <= 5 and index <= len(objects):
            chosen = objects[index - 1]
        else:
            chosen = objects[0]
        class_id, class_name, x_map, y_map, z_map, score = chosen
        print(f"\nFound object: {class_name}")
        print(f"  - Map frame position: ({x_map:.2f}, {y_map:.2f}, {z_map:.2f})")
        print(f"  - Confidence: {score:.2f}")

        # Calculate target position (offset from object in robot frame)
        target_map = self.calculate_target_position_relative_to_robot(
            (x_map, y_map, z_map), direction, self.direction_distance
        )
        print(f"\nCalculating target position (offset from object in robot frame):")
        print(f"  - Direction: {direction}")
        print(f"  - Distance: {self.direction_distance}m")
        print(
            f"  - Target position: ({target_map[0]:.2f}, {target_map[1]:.2f}, {target_map[2]:.2f})"
        )

        # Send navigation goal
        success = self.send_navigation_goal(target_map)
        if success:
            print(f"\nNavigation goal sent to /move_base_simple/goal")
            print(f"  - Target: ({target_map[0]:.2f}, {target_map[1]:.2f})")
            print(f"  - View red arrow marker in RViz")
            print(f"  - Topic: rostopic echo /move_base_simple/goal")
        else:
            print(f"\nFailed to send navigation goal")

        print(f"{'='*60}\n")
        return success

    def query_object(self, object_name):
        """Query object and output results"""
        print(f"\n{'='*60}")
        print(f"Query object: '{object_name}'")
        print(f"{'='*60}")

        # Preprocessing: support Chinese/mixed input, extract object phrase and map to English class name
        cmd_raw = str(object_name).strip()
        cmd = cmd_raw.lower()

        # Construct known English class name list (from class_names and common COCO)
        known_names = []
        try:
            if self.class_names:
                known_names = [str(v).lower() for v in self.class_names.values()]
        except Exception:
            known_names = []
        fallback_names = [
            "person",
            "bottle",
            "wine glass",
            "cup",
            "fork",
            "knife",
            "spoon",
            "bowl",
            "banana",
            "apple",
            "sandwich",
            "orange",
            "broccoli",
            "carrot",
            "hot dog",
            "pizza",
            "donut",
            "cake",
            "chair",
            "couch",
            "potted plant",
            "bed",
            "dining table",
            "toilet",
            "tv",
            "laptop",
            "mouse",
            "remote",
            "keyboard",
            "cell phone",
            "microwave",
            "oven",
            "toaster",
            "sink",
            "refrigerator",
            "book",
            "clock",
            "vase",
            "scissors",
            "teddy bear",
            "hair drier",
            "toothbrush",
            "traffic light",
            "fire hydrant",
            "stop sign",
            "parking meter",
            "bench",
            "dog",
            "cat",
            "car",
            "bus",
            "truck",
            "motorcycle",
            "bicycle",
            "suitcase",
        ]
        for n in fallback_names:
            if n not in known_names:
                known_names.append(n)
        known_names = sorted(
            list(dict.fromkeys(known_names)), key=lambda s: len(s), reverse=True
        )

        # Chinese object name mapping
        cn_obj_map = {
            "人": "person",
            "瓶子": "bottle",
            "红酒杯": "wine glass",
            "杯子": "cup",
            "水杯": "cup",
            "叉子": "fork",
            "刀": "knife",
            "勺子": "spoon",
            "碗": "bowl",
            "香蕉": "banana",
            "苹果": "apple",
            "三明治": "sandwich",
            "橙子": "orange",
            "西兰花": "broccoli",
            "胡萝卜": "carrot",
            "热狗": "hot dog",
            "披萨": "pizza",
            "甜甜圈": "donut",
            "蛋糕": "cake",
            "椅子": "chair",
            "沙发": "couch",
            "盆栽": "potted plant",
            "床": "bed",
            "餐桌": "dining table",
            "厕所": "toilet",
            "电视": "tv",
            "笔记本": "laptop",
            "鼠标": "mouse",
            "遥控器": "remote",
            "键盘": "keyboard",
            "手机": "cell phone",
            "微波炉": "microwave",
            "烤箱": "oven",
            "烤面包机": "toaster",
            "水槽": "sink",
            "冰箱": "refrigerator",
            "书": "book",
            "时钟": "clock",
            "花瓶": "vase",
            "剪刀": "scissors",
            "泰迪熊": "teddy bear",
            "吹风机": "hair drier",
            "牙刷": "toothbrush",
            "红绿灯": "traffic light",
            "消防栓": "fire hydrant",
            "停车标志": "stop sign",
            "停车计时器": "parking meter",
            "长凳": "bench",
            "狗": "dog",
            "猫": "cat",
            "汽车": "car",
            "公交车": "bus",
            "卡车": "truck",
            "摩托车": "motorcycle",
            "自行车": "bicycle",
            "手提箱": "suitcase",
            "行李箱": "suitcase",
        }

        # Try Chinese sentence structure: 到/去到/走到/移动到/运动到 [第N个] <object> 的 <direction>
        obj_phrase = None
        m_cn = re.search(
            r"(?:到|去到|走到|移动到|运动到)?\s*(第\s*[一二三四五1-5]\s*个)?\s*([\u4e00-\u9fa5A-Za-z ]+?)(?:的\s*(右边|右侧|左边|左侧|前边|前面|后边|后面))?$",
            cmd_raw,
        )
        if m_cn:
            obj_phrase = m_cn.group(2).strip()

        # Further cleaning: remove leading verb/number/"的"/trailing direction word
        def clean_cn_phrase(s):
            t = s
            t = re.sub(r"^(到|去到|走到|移动到|运动到)\s*", "", t)
            t = re.sub(r"^第\s*[一二三四五1-5]\s*个\s*", "", t)
            t = re.sub(r"\s*的\s*(右边|右侧|左边|左侧|前边|前面|后边|后面)\s*$", "", t)
            t = re.sub(r"^的+", "", t)
            return t.strip()

        if obj_phrase:
            obj_phrase = clean_cn_phrase(obj_phrase)

        # If no match, use original string
        if not obj_phrase:
            obj_phrase = clean_cn_phrase(cmd_raw)

        mapped_en = None
        # Chinese phrase mapping
        if re.search(r"[\u4e00-\u9fa5]", obj_phrase):
            for cn_name in sorted(
                cn_obj_map.keys(), key=lambda s: len(s), reverse=True
            ):
                if cn_name in obj_phrase:
                    mapped_en = cn_obj_map[cn_name]
                    break
        # English phrase mapping (longest first)
        if mapped_en is None:
            phrase_en = obj_phrase.lower()
            for name in known_names:
                pat = r"\b" + re.sub(r"\s+", r"\\s+", re.escape(name)) + r"\b"
                if re.search(pat, phrase_en):
                    mapped_en = name
                    break
            # Compatibility word matching: take first English word as candidate for partial matching
            if mapped_en is None:
                tokens = re.findall(r"[a-z]+", phrase_en)
                if tokens:
                    key = tokens[0]
                    for name in known_names:
                        if key in name:
                            mapped_en = name
                            break

        query_key = mapped_en if mapped_en else object_name
        if mapped_en and mapped_en != object_name:
            print(f"Normalized query keyword: '{object_name}' → '{mapped_en}'")

        objects = self.find_object(query_key)

        if not objects:
            print(f"Object '{object_name}' not found")
            print(f"\nAvailable object categories:")
            with self.lock:
                if self.latest_detections:
                    detected_classes = set()
                    for det in self.latest_detections.detections:
                        if len(det.results) > 0:
                            class_id = det.results[0].id
                            class_name = self.class_names.get(
                                str(class_id), f"unknown_{class_id}"
                            )
                            # English->Chinese mapping (common mappings used in parse)
                            en = str(class_name)
                            cn = {
                                "person": "人",
                                "bottle": "瓶子",
                                "wine glass": "红酒杯",
                                "cup": "杯子",
                                "fork": "叉子",
                                "knife": "刀",
                                "spoon": "勺子",
                                "bowl": "碗",
                                "banana": "香蕉",
                                "apple": "苹果",
                                "sandwich": "三明治",
                                "orange": "橙子",
                                "broccoli": "西兰花",
                                "carrot": "胡萝卜",
                                "hot dog": "热狗",
                                "pizza": "披萨",
                                "donut": "甜甜圈",
                                "cake": "蛋糕",
                                "chair": "椅子",
                                "couch": "沙发",
                                "potted plant": "盆栽",
                                "bed": "床",
                                "dining table": "餐桌",
                                "toilet": "厕所",
                                "tv": "电视",
                                "laptop": "笔记本",
                                "mouse": "鼠标",
                                "remote": "遥控器",
                                "keyboard": "键盘",
                                "cell phone": "手机",
                                "microwave": "微波炉",
                                "oven": "烤箱",
                                "toaster": "烤面包机",
                                "sink": "水槽",
                                "refrigerator": "冰箱",
                                "book": "书",
                                "clock": "时钟",
                                "vase": "花瓶",
                                "scissors": "剪刀",
                                "teddy bear": "泰迪熊",
                                "hair drier": "吹风机",
                                "toothbrush": "牙刷",
                                "traffic light": "红绿灯",
                                "fire hydrant": "消防栓",
                                "stop sign": "停车标志",
                                "parking meter": "停车计时器",
                                "bench": "长凳",
                                "dog": "狗",
                                "cat": "猫",
                                "car": "汽车",
                                "bus": "公交车",
                                "truck": "卡车",
                                "motorcycle": "摩托车",
                                "bicycle": "自行车",
                                "suitcase": "手提箱",
                            }.get(en, en)
                            detected_classes.add(f"{cn} ({en})")
                    if detected_classes:
                        for cls in sorted(detected_classes):
                            print(f"  - {cls}")
                    else:
                        print("  (No objects detected)")
                else:
                    print("  (Waiting for detection data...)")

            # Publish empty result
            self.result_pub.publish(
                json.dumps({"query": object_name, "found": False, "objects": []})
            )
        else:
            print(f"Found {len(objects)} '{object_name}' objects:")
            print(
                f"\n{'ID':<5} {'Class (CN/EN)':<30} {'X (m)':<10} {'Y (m)':<10} {'Z (m)':<10} {'Confidence':<10}"
            )
            print(f"{'-'*70}")

            results = []
            for idx, (class_id, class_name, x, y, z, score) in enumerate(objects, 1):
                en = str(class_name)
                cn = {
                    "person": "人",
                    "bottle": "瓶子",
                    "wine glass": "红酒杯",
                    "cup": "杯子",
                    "fork": "叉子",
                    "knife": "刀",
                    "spoon": "勺子",
                    "bowl": "碗",
                    "banana": "香蕉",
                    "apple": "苹果",
                    "sandwich": "三明治",
                    "orange": "橙子",
                    "broccoli": "西兰花",
                    "carrot": "胡萝卜",
                    "hot dog": "热狗",
                    "pizza": "披萨",
                    "donut": "甜甜圈",
                    "cake": "蛋糕",
                    "chair": "椅子",
                    "couch": "沙发",
                    "potted plant": "盆栽",
                    "bed": "床",
                    "dining table": "餐桌",
                    "toilet": "厕所",
                    "tv": "电视",
                    "laptop": "笔记本",
                    "mouse": "鼠标",
                    "remote": "遥控器",
                    "keyboard": "键盘",
                    "cell phone": "手机",
                    "microwave": "微波炉",
                    "oven": "烤箱",
                    "toaster": "烤面包机",
                    "sink": "水槽",
                    "refrigerator": "冰箱",
                    "book": "书",
                    "clock": "时钟",
                    "vase": "花瓶",
                    "scissors": "剪刀",
                    "teddy bear": "泰迪熊",
                    "hair drier": "吹风机",
                    "toothbrush": "牙刷",
                    "traffic light": "红绿灯",
                    "fire hydrant": "消防栓",
                    "stop sign": "停车标志",
                    "parking meter": "停车计时器",
                    "bench": "长凳",
                    "dog": "狗",
                    "cat": "猫",
                    "car": "汽车",
                    "bus": "公交车",
                    "truck": "卡车",
                    "motorcycle": "摩托车",
                    "bicycle": "自行车",
                    "suitcase": "手提箱",
                }.get(en, en)
                name_cn_en = f"{cn} / {en}" if cn != en else en
                print(
                    f"{idx:<5} {name_cn_en:<30} {x:<10.3f} {y:<10.3f} {z:<10.3f} {score:<10.2f}"
                )
                results.append(
                    {
                        "class_id": class_id,
                        "class_name": en,
                        "class_name_cn": cn if cn != en else None,
                        "position": {"x": x, "y": y, "z": z},
                        "score": score,
                    }
                )

            # Visualize objects in RViz
            self.visualize_objects(objects, object_name)
            print(f"\nMarked object positions in RViz (topic: /object_markers)")

            # Publish results
            self.result_pub.publish(
                json.dumps({"query": object_name, "found": True, "objects": results})
            )

        print(f"{'='*60}\n")

    def interactive_mode(self):
        """Interactive query mode"""
        print("\n" + "=" * 60)
        print("Object Query and Navigation Node - Interactive Mode")
        print("=" * 60)
        print("Functions:")
        print("  1. Query object: Enter the object name (for example: person)")
        print(
            "  2. Navigation command: Move to the [direction] of the [object] / Move to the [Nth] [object] of the [direction]"
        )
        print("     - Direction (English): right, left, front, behind")
        print("     - Direction (Chinese): 左边/右边/前边/后边")
        print("     - Example (English): Move to the right of the person")
        print("     - Example (Chinese): 运动到第3个人的左边 / 运动到沙发的前边")
        print(
            "  6. Input mode: 'voice on' to enable voice / 'voice off' to disable voice / 'mode' to switch mode"
        )
        print("  3. 'list' - List all detected objects")
        print("  4. 'classes' - List all COCO classes")
        print("  5. 'quit' or 'exit' - Exit")
        print("=" * 60 + "\n")

        # Initial selection mode (default text input). If voice is required, user inputs 'voice on' to start
        if self.enable_voice and self._voice_ready:
            print(
                "Note: The current voice dependency is not available; only text mode is supported."
            )
        else:
            print(
                "Note: The current voice dependency is not available; only text mode is supported."
            )

        while not rospy.is_shutdown():
            try:
                if self.voice_active:
                    # Allow keyboard commands to control/exit in voice mode
                    user_input = input(
                        "[Voice mode] Input control command (voice off/list/classes/quit) > "
                    ).strip()
                else:
                    user_input = input("Input command > ").strip()

                if not user_input:
                    continue

                if user_input.lower() in ["quit", "exit", "q"]:
                    print("Exit query node...")
                    break

                if user_input.lower() == "list":
                    self.list_detected_objects()
                    continue

                if user_input.lower() == "classes":
                    self.list_all_classes()
                    continue

                # Mode switch command
                if user_input.lower() in ["voice on", "voiceon", "voice_on"]:
                    if not (self.enable_voice and self._voice_ready):
                        print(
                            "Voice function is unavailable: missing dependencies or model not ready"
                        )
                        continue
                    if not self.voice_active:
                        self._voice_stop.clear()
                        self._start_voice_thread()
                        self.voice_active = True
                        print(
                            "Voice mode enabled, listening every %.1fs."
                            % self.voice_interval_sec
                        )
                    else:
                        print("Voice mode already enabled.")
                    continue
                if user_input.lower() in ["voice off", "voiceoff", "voice_off"]:
                    if self.voice_active:
                        self._stop_voice_thread()
                        self.voice_active = False
                        print("Voice mode disabled, returning to text input.")
                    else:
                        print("Voice mode not enabled.")
                    continue
                if user_input.lower() in ["mode"]:
                    if self.voice_active:
                        self._stop_voice_thread()
                        self.voice_active = False
                        print("Switched to text input mode.")
                    else:
                        if not (self.enable_voice and self._voice_ready):
                            print(
                                "Voice function is unavailable: missing dependencies or model not ready"
                            )
                        else:
                            self._start_voice_thread()
                            self.voice_active = True
                            print("Switched to voice mode.")
                    continue

                # First try to parse as navigation command (both Chinese and English are supported)
                # First handle simple commands (e.g. come back/go back)
                if self.process_simple_command(user_input):
                    continue
                parsed = self.parse_navigation_command(user_input)
                if parsed:
                    self.process_navigation_command(user_input)
                else:
                    # Normal query
                    self.query_object(user_input)

            except KeyboardInterrupt:
                print("\n\nExit the object query node...")
                break
            except Exception as e:
                rospy.logerr(f"Error: {e}")

    # ================ Voice Input ================
    @contextmanager
    def _raw_input_stream(self):
        """Context manager: create and close RawInputStream."""
        stream = sd.RawInputStream(
            samplerate=self.voice_sample_rate,
            blocksize=self.voice_block_size,
            device=self.voice_device,
            dtype="int16",
            channels=1,
            callback=lambda indata, frames, timeinfo, status: self._audio_q.put(
                bytes(indata)
            ),
        )
        try:
            stream.__enter__()
            yield stream
        finally:
            stream.__exit__(None, None, None)

    def _start_voice_thread(self):
        if self._voice_thread and self._voice_thread.is_alive():
            return
        self._voice_stop.clear()
        self._voice_thread = threading.Thread(
            target=self._voice_loop, name="voice_loop", daemon=True
        )
        self._voice_thread.start()

    def _stop_voice_thread(self):
        if self._voice_thread and self._voice_thread.is_alive():
            self._voice_stop.set()
            try:
                self._voice_thread.join(timeout=1.0)
            except Exception:
                pass

    def _voice_loop(self):
        if not self._voice_ready:
            return
        rospy.loginfo("Voice recognition thread started.")
        # Keep the audio stream open and end with one sentence based on the mute determination
        try:
            with self._raw_input_stream():
                utter_started = False
                utter_start_time = 0.0
                last_activity_time = 0.0
                last_partial_text = ""
                while not rospy.is_shutdown() and not self._voice_stop.is_set():
                    # Read audio block
                    try:
                        data = self._audio_q.get(timeout=0.2)
                    except Exception:
                        # Check if the silence ends
                        if (
                            utter_started
                            and (time.time() - last_activity_time)
                            >= self.voice_end_silence
                            and (time.time() - utter_start_time)
                            >= self.voice_min_duration
                        ):
                            final = json.loads(self._recognizer.FinalResult())
                            text = final.get("text", "").strip()
                            self._handle_voice_text(text)
                            # Reset
                            self._recognizer.Reset()
                            utter_started = False
                            last_partial_text = ""
                            time.sleep(self.voice_debounce)
                        continue

                    # Send audio to recognizer
                    if self._recognizer.AcceptWaveform(data):
                        res = json.loads(self._recognizer.Result())
                        text = res.get("text", "").strip()
                        if not utter_started:
                            utter_started = True
                            utter_start_time = time.time()
                        last_activity_time = time.time()
                        # Directly as a sentence ends
                        self._handle_voice_text(text)
                        self._recognizer.Reset()
                        utter_started = False
                        last_partial_text = ""
                        time.sleep(self.voice_debounce)
                        continue
                    else:
                        # Process partial results, update activity time
                        try:
                            partial_json = json.loads(self._recognizer.PartialResult())
                            partial = partial_json.get("partial", "")
                        except Exception:
                            partial = ""
                        if partial:
                            if not utter_started:
                                utter_started = True
                                utter_start_time = time.time()
                            # Only update activity time when there is a change
                            if partial != last_partial_text:
                                last_partial_text = partial
                                last_activity_time = time.time()
                        # End of sentence based on silence
                        if utter_started:
                            dur = time.time() - utter_start_time
                            since_last = time.time() - last_activity_time
                            if (
                                dur >= self.voice_min_duration
                                and since_last >= self.voice_end_silence
                            ) or dur >= self.voice_max_duration:
                                final = json.loads(self._recognizer.FinalResult())
                                text = final.get("text", "").strip()
                                self._handle_voice_text(text)
                                self._recognizer.Reset()
                                utter_started = False
                                last_partial_text = ""
                                time.sleep(self.voice_debounce)
        except Exception as e:
            rospy.logwarn(f"Voice recognition thread exception exit: {e}")

    def _handle_voice_text(self, recognized_text: str):
        if not recognized_text:
            return
        cmd_text = recognized_text
        rospy.loginfo(f"Voice recognition: {cmd_text}")
        try:
            parsed = self.parse_navigation_command(cmd_text)
            if parsed:
                self.process_navigation_command(cmd_text)
            else:
                self.query_object(cmd_text)
        except Exception as e:
            rospy.logwarn(f"Failed to process voice command: {e}")

    def list_detected_objects(self):
        """List all detected objects"""
        print(f"\n{'='*60}")
        print("Current detected objects:")
        print(f"{'='*60}")

        with self.lock:
            has3d = (
                self.latest_detections and len(self.latest_detections.detections) > 0
            )
            has2d = (
                self.latest_detections_2d
                and len(self.latest_detections_2d.detections) > 0
            )
            if not has3d and not has2d:
                print("(No objects detected)")
            else:
                print(f"\n{'Class':<15} {'Count':<10} {'Average Confidence':<15}")
                print(f"{'-'*40}")
                class_stats = {}
                src = None
                if has3d:
                    src = self.latest_detections.detections
                else:
                    src = self.latest_detections_2d.detections
                for det in src:
                    if len(det.results) > 0:
                        class_id = det.results[0].id
                        score = det.results[0].score
                        class_name = str(
                            self.class_names.get(str(class_id), f"unknown_{class_id}")
                        )
                        if class_name not in class_stats:
                            class_stats[class_name] = {"count": 0, "total_score": 0.0}
                        class_stats[class_name]["count"] += 1
                        class_stats[class_name]["total_score"] += score
                for class_name, stats in sorted(class_stats.items()):
                    avg_score = stats["total_score"] / max(1, stats["count"])
                    print(f"{class_name:<15} {stats['count']:<10} {avg_score:<15.2f}")
                # Debug information
                last2d = getattr(self, "_last_2d_time", None)
                last3d = getattr(self, "_last_3d_time", None)
                c2d = getattr(self, "_last_2d_count", None)
                c3d = getattr(self, "_last_3d_count", None)
                if last2d:
                    print(
                        f"\n[Debug] Last 2D detection: {c2d} objects @ {last2d.to_sec():.2f}"
                    )
                if last3d:
                    print(
                        f"[Debug] Last 3D detection: {c3d} objects @ {last3d.to_sec():.2f}"
                    )
                if not has3d and has2d:
                    print(
                        "Warning: Only 2D detection (no depth), list based on /yolo_detections. Navigation requires 3D detection."
                    )

        print(f"{'='*60}\n")

    def list_all_classes(self):
        """List all classes"""
        print(f"\n{'='*60}")
        print("All classes (COCO dataset):")
        print(f"{'='*60}\n")

        with self.lock:
            if not self.class_names:
                print("(Class names not loaded)")
            else:
                # Display in columns
                classes = sorted(self.class_names.values())
                cols = 4
                for i in range(0, len(classes), cols):
                    row = classes[i : i + cols]
                    print("  ".join(f"{cls:<18}" for cls in row))

        print(f"\n{'='*60}\n")


def main():
    try:
        node = ObjectQueryNode()

        # Wait for data to be ready
        rospy.sleep(2.0)

        # Start interactive mode
        node.interactive_mode()

    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
