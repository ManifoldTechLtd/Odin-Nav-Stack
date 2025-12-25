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

"""
TF relay node - Convert static extrinsics to dynamic TF, using sensor timestamps
Solve timestamp mismatch problem
"""
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
import numpy as np
import math


class TFRelayNode:
    def __init__(self):
        rospy.init_node("tf_relay_node", anonymous=False)

        # TF broadcaster and listener
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # last sensor timestamp used for publishing (to prevent TF_REPEATED_DATA due to duplicate timestamps)
        self.last_sensor_stamp = rospy.Time(0)

        # static extrinsics (from calib.yaml and configuration)
        # odin1_base_link -> lidar_link
        self.odin_to_lidar_trans = [0.00347, 0.03447, 0.02174]
        self.odin_to_lidar_rot = [0, 0, 0, 1]  # identity rotation

        # lidar_link -> camera_link (from Tcl_0 - O1-N090100030)
        # Tcl_0 = [[-0.01476, -0.99983, -0.01096, 0.04354],
        #          [0.00604, 0.01087, -0.99992, -0.01748],
        #          [0.99987, -0.01482, 0.00588, -0.02099]]
        # translation: [0.04354, -0.01748, -0.02099]
        # rotation: [-0.5063, 0.4978, -0.4978, 0.5063] (qx, qy, qz, qw)
        self.lidar_to_camera_trans = [0.04354, -0.01748, -0.02099]
        self.lidar_to_camera_rot = [-0.5063, 0.4978, -0.4978, 0.5063]  # qx, qy, qz, qw

        # publish rate
        self.rate = rospy.Rate(10)  # 10 Hz

        rospy.loginfo("[TF Relay] Node started, waiting for sensor TF...")

    def publish_tf_with_sensor_timestamp(self):
        """Publish static TF using sensor timestamps"""
        while not rospy.is_shutdown():
            try:
                # get latest sensor TF (using its timestamp)
                # query odom -> odin1_base_link transform to get sensor timestamp
                try:
                    transform = self.tf_buffer.lookup_transform(
                        "odom", "odin1_base_link", rospy.Time(0), rospy.Duration(0.1)
                    )
                    sensor_timestamp = transform.header.stamp
                except (
                    tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException,
                ):
                    # if odom->odin1_base_link is not found, try map->odom
                    try:
                        transform = self.tf_buffer.lookup_transform(
                            "map", "odom", rospy.Time(0), rospy.Duration(0.1)
                        )
                        sensor_timestamp = transform.header.stamp
                    except:
                        # if both odom->odin1_base_link and map->odom are not found, use current time
                        sensor_timestamp = rospy.Time.now()

                # only publish when sensor timestamp advances, to avoid duplicate timestamps
                if sensor_timestamp > self.last_sensor_stamp:
                    # publish odin1_base_link -> lidar_link (using sensor timestamp)
                    self.publish_static_transform(
                        parent_frame="odin1_base_link",
                        child_frame="lidar_link",
                        translation=self.odin_to_lidar_trans,
                        rotation=self.odin_to_lidar_rot,
                        timestamp=sensor_timestamp,
                    )

                    # publish lidar_link -> camera_link (using sensor timestamp)
                    self.publish_static_transform(
                        parent_frame="lidar_link",
                        child_frame="camera_link",
                        translation=self.lidar_to_camera_trans,
                        rotation=self.lidar_to_camera_rot,
                        timestamp=sensor_timestamp,
                    )

                    self.last_sensor_stamp = sensor_timestamp
                else:
                    # skip publishing if sensor timestamp does not advance, to avoid TF_REPEATED_DATA warnings
                    pass

                self.rate.sleep()

            except Exception as e:
                rospy.logwarn(f"[TF Relay] Error: {e}")
                self.rate.sleep()

    def publish_static_transform(
        self, parent_frame, child_frame, translation, rotation, timestamp
    ):
        """Publish single TF transform"""
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame

        # set translation
        t.transform.translation.x = float(translation[0])
        t.transform.translation.y = float(translation[1])
        t.transform.translation.z = float(translation[2])

        # set rotation (quaternion)
        t.transform.rotation.x = float(rotation[0])
        t.transform.rotation.y = float(rotation[1])
        t.transform.rotation.z = float(rotation[2])
        t.transform.rotation.w = float(rotation[3])

        self.tf_broadcaster.sendTransform(t)


def main():
    try:
        relay = TFRelayNode()
        relay.publish_tf_with_sensor_timestamp()
    except rospy.ROSInterruptException:
        rospy.loginfo("[TF Relay] Node shutdown")


if __name__ == "__main__":
    main()
