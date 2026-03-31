#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
plot_trajectory.py

功能：
1. 订阅 /odom 记录机器人在 x-y 平面上的运动轨迹
2. 可选：通过 Gazebo 服务记录目标球的位置轨迹
3. 结束时自动绘图并保存为 PNG
4. 适合用于生成报告中的 trajectory plot

使用方法：
1. 启动仿真与机器人追踪
2. 运行本脚本
3. 跑一段时间后 Ctrl+C 结束
4. 自动生成轨迹图文件
"""

import rospy
import math
import matplotlib
matplotlib.use("Agg")   # 不依赖 GUI
import matplotlib.pyplot as plt

from nav_msgs.msg import Odometry
from gazebo_msgs.srv import GetModelState


class TrajectoryRecorder:
    def __init__(self):
        rospy.init_node("plot_trajectory_node")

        # -----------------------------
        # 参数
        # -----------------------------
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.output_path = rospy.get_param("~output_path", "trajectory_plot.png")

        # 是否记录目标球轨迹
        self.record_target = rospy.get_param("~record_target", True)
        self.target_model_name = rospy.get_param("~target_model_name", "green_ball")

        # 轨迹记录间隔（秒）
        self.sample_interval = rospy.get_param("~sample_interval", 0.1)

        # 数据缓存
        self.robot_x = []
        self.robot_y = []

        self.target_x = []
        self.target_y = []

        self.last_sample_time = None

        # 订阅里程计
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=1)

        # Gazebo 获取模型状态服务
        self.get_model_state_srv = None
        if self.record_target:
            try:
                rospy.wait_for_service("/gazebo/get_model_state", timeout=5.0)
                self.get_model_state_srv = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
                rospy.loginfo("已连接 Gazebo 服务 /gazebo/get_model_state")
            except Exception as e:
                rospy.logwarn("无法连接 Gazebo model state 服务，将不记录目标轨迹: %s", e)
                self.record_target = False

        rospy.loginfo("Trajectory recorder started.")
        rospy.loginfo("Odom topic: %s", self.odom_topic)
        rospy.loginfo("Output path: %s", self.output_path)
        rospy.loginfo("Record target: %s", str(self.record_target))
        if self.record_target:
            rospy.loginfo("Target model name: %s", self.target_model_name)

        rospy.on_shutdown(self.on_shutdown)

    # =====================================================
    # 里程计回调
    # =====================================================
    def odom_callback(self, msg):
        now = rospy.Time.now().to_sec()

        if self.last_sample_time is None:
            self.last_sample_time = now

        # 按固定时间间隔采样，避免点太密
        if now - self.last_sample_time < self.sample_interval:
            return

        self.last_sample_time = now

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        self.robot_x.append(x)
        self.robot_y.append(y)

        # 记录目标球轨迹
        if self.record_target and self.get_model_state_srv is not None:
            try:
                res = self.get_model_state_srv(self.target_model_name, "")
                if res.success:
                    self.target_x.append(res.pose.position.x)
                    self.target_y.append(res.pose.position.y)
            except Exception as e:
                rospy.logwarn_throttle(2.0, "读取目标模型状态失败: %s", e)

    # =====================================================
    # 绘图
    # =====================================================
    def plot_and_save(self):
        if len(self.robot_x) < 2:
            rospy.logwarn("轨迹点太少，无法绘图")
            return

        plt.figure(figsize=(8, 6))

        # 机器人轨迹
        plt.plot(self.robot_x, self.robot_y, 'b-', linewidth=2, label='Robot Trajectory')
        plt.scatter(self.robot_x[0], self.robot_y[0], c='green', s=80, marker='o', label='Robot Start')
        plt.scatter(self.robot_x[-1], self.robot_y[-1], c='red', s=100, marker='*', label='Robot End')

        # 目标轨迹（可选）
        if self.record_target and len(self.target_x) > 1:
            plt.plot(self.target_x, self.target_y, 'm--', linewidth=2, label='Target Trajectory')
            plt.scatter(self.target_x[0], self.target_y[0], c='orange', s=80, marker='o', label='Target Start')
            plt.scatter(self.target_x[-1], self.target_y[-1], c='purple', s=80, marker='x', label='Target End')
        elif self.record_target and len(self.target_x) == 1:
            plt.scatter(self.target_x[0], self.target_y[0], c='orange', s=80, marker='o', label='Target Position')

        plt.xlabel("X Position (m)")
        plt.ylabel("Y Position (m)")
        plt.title("Robot Trajectory During Tracking")
        plt.axis("equal")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()

        plt.savefig(self.output_path, dpi=200)
        rospy.loginfo("轨迹图已保存到: %s", self.output_path)

    # =====================================================
    # 退出时处理
    # =====================================================
    def on_shutdown(self):
        self.plot_and_save()


if __name__ == "__main__":
    recorder = TrajectoryRecorder()
    rospy.spin()