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

import time
import threading
from loguru import logger

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped


TOPIC_CMD_STR = "/cmd_str"
TOPIC_CMD_VEL = "/cmd_vel"
TOPIC_GOAL = "/move_base_simple/goal"
GOAL_FRAME_ID = "base_link"
ANGULAR_SPEED_DEG = 90.0  # degrees per second


class STRCMDController:
    def __init__(self):
        rospy.init_node("str_cmd_controller", anonymous=True)
        self.goal_pub = rospy.Publisher(TOPIC_GOAL, PoseStamped, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher(TOPIC_CMD_VEL, Twist, queue_size=1)
        self.str_cmd_sub = rospy.Subscriber(
            TOPIC_CMD_STR, String, self.str_cmd_cb, queue_size=1
        )

        self.command_thread = threading.Thread(target=self.command_worker, daemon=True)
        self.str_cmd: str = None
        self.cmd_lock = threading.Lock()

    def str_cmd_cb(self, msg: String):
        command = msg.data
        with self.cmd_lock:
            self.str_cmd = command

    def command_worker(self):
        while not rospy.is_shutdown():
            with self.cmd_lock:
                command = self.str_cmd
            if command is None:
                time.sleep(0.1)
                continue

            command = command.lower()
            logger.info(f"Executing command: {command}")
            if command == "stop":
                self.publish_stop()
                time.sleep(1.0)
            elif command == "forward":
                self.publish_goal(0.5)
                time.sleep(1.0)
            elif command == "left":
                self.publish_rotation(30)
            elif command == "right":
                self.publish_rotation(-30)
            else:
                logger.warning(f"Unknown command: {command}")

            with self.cmd_lock:
                self.str_cmd = None
            time.sleep(0.1)

    def publish_goal(self, x_offset: float):
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = GOAL_FRAME_ID
        goal.pose.position.x = x_offset
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)

    def publish_stop(self):
        self.cmd_vel_pub.publish(Twist())
        self.publish_goal(0.0)

    def publish_rotation(self, angle_degrees: float):
        self.publish_goal(0.0)
        time.sleep(0.25)

        t_start = time.time()
        t_need = abs(angle_degrees) / ANGULAR_SPEED_DEG
        angular_z = ANGULAR_SPEED_DEG * (3.14159265 / 180.0)
        if angle_degrees < 0:
            angular_z = -angular_z

        while time.time() - t_start < t_need:
            twist = Twist()
            twist.angular.z = angular_z
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)

        self.cmd_vel_pub.publish(Twist())
        time.sleep(0.25)

    def run(self):
        logger.info("str cmd controller start.")
        self.command_thread.start()
        rospy.spin()


if __name__ == "__main__":
    try:
        node = STRCMDController()
        node.run()
    except rospy.ROSInterruptException:
        pass
