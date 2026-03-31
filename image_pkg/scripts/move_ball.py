#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
特点：
1. 运动范围更大
2. 轨迹平滑，适合展示机器人跟踪
3. 带轻微随机扰动，不会显得机械
4. 比简单随机游走更容易展示“追踪效果”
"""

import rospy
import math
import random

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Twist


class BallMover:
    def __init__(self):
        rospy.init_node("move_ball_node")

        # 要运动的球名
        self.model_name = rospy.get_param("~model_name", "green_ball")

        # 运动中心
        self.center_x = rospy.get_param("~center_x", 2.8)
        self.center_y = rospy.get_param("~center_y", 0.0)

        # 运动幅度
        self.amp_x = rospy.get_param("~amp_x", 1.6)
        self.amp_y = rospy.get_param("~amp_y", 1.2)

        # 基础角速度（控制整体运动快慢）
        self.omega = rospy.get_param("~omega", 0.22)

        # 纵横方向附加扰动强度
        self.noise_amp = rospy.get_param("~noise_amp", 0.18)

        # z 高度
        self.z = rospy.get_param("~z", 0.0)

        # 起始时间
        self.start_time = rospy.Time.now().to_sec()

        rospy.loginfo("等待 /gazebo/set_model_state 服务...")
        rospy.wait_for_service("/gazebo/set_model_state")
        self.set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        rospy.loginfo("move_ball_node 启动成功，目标模型: %s", self.model_name)

    def run(self):
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            t = rospy.Time.now().to_sec() - self.start_time

            # 大范围平滑主轨迹
            x = self.center_x + self.amp_x * math.sin(self.omega * t)

            # y 方向叠加两个不同频率的项，让轨迹更自然
            y = self.center_y \
                + self.amp_y * math.sin(0.7 * self.omega * t + 1.2) \
                + self.noise_amp * math.sin(2.3 * self.omega * t + 0.8)

            # 再加一点缓慢变化的小扰动
            x += 0.08 * math.sin(1.7 * self.omega * t + 2.0)
            y += 0.06 * math.cos(1.9 * self.omega * t + 0.3)

            state = ModelState()
            state.model_name = self.model_name
            state.reference_frame = "world"

            state.pose.position.x = x
            state.pose.position.y = y
            state.pose.position.z = self.z

            # 姿态保持默认
            state.pose.orientation.x = 0.0
            state.pose.orientation.y = 0.0
            state.pose.orientation.z = 0.0
            state.pose.orientation.w = 1.0

            state.twist = Twist()

            try:
                self.set_state(state)
                rospy.loginfo_throttle(1.0, "%s 位置: x=%.2f, y=%.2f", self.model_name, x, y)
            except rospy.ServiceException as e:
                rospy.logerr_throttle(1.0, "设置模型状态失败: %s", e)

            rate.sleep()


if __name__ == "__main__":
    mover = BallMover()
    mover.run()