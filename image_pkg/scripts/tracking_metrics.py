#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
tracking_metrics.py

兼容两种情况：
1. 仅 RGB-D 追踪
2. RGB-D + 激光兜底追踪

统计指标：
- Detection Success Rate
- Average Response Time
- Tracking Accuracy (Image RMSE)
- Tracking Accuracy (Distance RMSE)
- Average Task Completion Time
- Valid Depth Rate
- TF Transform Success Rate
- Effective Distance Availability Rate
- Laser Fallback Usage Rate

支持在达到目标距离后自动输出结果并退出。
"""

import rospy
import cv2
import numpy as np
import time
import math
import threading

from sensor_msgs.msg import Image, CameraInfo, LaserScan
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError

import tf2_ros
import tf2_geometry_msgs


class TrackingMetricsNode:
    def __init__(self):
        rospy.init_node("tracking_metrics_node")

        self.bridge = CvBridge()

        # -----------------------------
        # 参数
        # -----------------------------
        self.rgb_topic = rospy.get_param("~rgb_topic", "/camera/rgb/image_raw")
        self.depth_topic = rospy.get_param("~depth_topic", "/camera/depth/image_raw")
        self.info_topic = rospy.get_param("~info_topic", "/camera/rgb/camera_info")
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")

        self.target_frame = rospy.get_param("~target_frame", "base_link")
        self.source_frame = rospy.get_param("~source_frame", "camera_rgb_optical_frame")

        self.desired_distance = rospy.get_param("~desired_distance", 0.30)
        self.distance_threshold = rospy.get_param("~distance_threshold", 0.08)

        self.auto_stop_on_reached = rospy.get_param("~auto_stop_on_reached", True)
        self.enable_laser_fallback = rospy.get_param("~enable_laser_fallback", True)

        self.color_name = rospy.get_param("~target_color_name", "green")

        self.color_presets = {
            "green":  {"hmin": 35,  "hmax": 85,  "smin": 60,  "smax": 255, "vmin": 0, "vmax": 255},
            "orange": {"hmin": 5,   "hmax": 20,  "smin": 100, "smax": 255, "vmin": 0, "vmax": 255},
            "red":    {"hmin": 0,   "hmax": 10,  "smin": 80,  "smax": 255, "vmin": 0, "vmax": 255},
            "blue":   {"hmin": 100, "hmax": 130, "smin": 100, "smax": 255, "vmin": 0, "vmax": 255},
        }

        if self.color_name not in self.color_presets:
            rospy.logwarn("未知颜色 %s，默认使用 green", self.color_name)
            self.color_name = "green"

        # -----------------------------
        # 数据缓存
        # -----------------------------
        self.depth_image = None
        self.depth_encoding = None
        self.camera_matrix = None
        self.latest_scan = None
        self.lock = threading.Lock()

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # -----------------------------
        # 统计量
        # -----------------------------
        self.total_frames = 0
        self.detected_frames = 0
        self.valid_depth_frames = 0
        self.tf_success_frames = 0
        self.effective_distance_frames = 0
        self.laser_fallback_frames = 0

        self.response_times = []
        self.center_errors_px = []
        self.distance_errors = []

        self.task_started = False
        self.task_start_time = None
        self.task_completion_times = []

        self.in_reached_zone = False
        self.report_printed = False

        # 订阅
        rospy.Subscriber(self.depth_topic, Image, self.depth_callback, queue_size=1)
        rospy.Subscriber(self.info_topic, CameraInfo, self.camera_info_callback, queue_size=1)
        rospy.Subscriber(self.rgb_topic, Image, self.rgb_callback, queue_size=1)

        # 激光可选
        rospy.Subscriber(self.scan_topic, LaserScan, self.scan_callback, queue_size=1)

        rospy.loginfo("TrackingMetricsNode started.")
        rospy.loginfo("RGB topic: %s", self.rgb_topic)
        rospy.loginfo("Depth topic: %s", self.depth_topic)
        rospy.loginfo("Info topic: %s", self.info_topic)
        rospy.loginfo("Scan topic: %s", self.scan_topic)
        rospy.loginfo("Color for evaluation: %s", self.color_name)
        rospy.loginfo("Laser fallback enabled: %s", str(self.enable_laser_fallback))
        rospy.loginfo("Auto stop on reached: %s", str(self.auto_stop_on_reached))

        rospy.on_shutdown(self.on_shutdown)

    # =====================================================
    # 回调
    # =====================================================
    def depth_callback(self, msg):
        try:
            depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            with self.lock:
                self.depth_image = depth_img
                self.depth_encoding = msg.encoding
        except CvBridgeError as e:
            rospy.logerr("深度图转换失败: %s", e)

    def camera_info_callback(self, msg):
        with self.lock:
            self.camera_matrix = np.array(msg.K, dtype=np.float64).reshape(3, 3)

    def scan_callback(self, msg):
        with self.lock:
            self.latest_scan = msg

    def rgb_callback(self, msg):
        if self.report_printed:
            return

        start_time = time.perf_counter()

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("RGB图像转换失败: %s", e)
            return

        with self.lock:
            depth_image = None if self.depth_image is None else self.depth_image.copy()
            depth_encoding = self.depth_encoding
            camera_matrix = None if self.camera_matrix is None else self.camera_matrix.copy()
            latest_scan = self.latest_scan

        self.total_frames += 1

        image_h, image_w = cv_image.shape[:2]

        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv_image)
        v = cv2.equalizeHist(v)
        hsv_image = cv2.merge([h, s, v])

        preset = self.color_presets[self.color_name]

        # 红色双区间
        if self.color_name == "red":
            mask1 = cv2.inRange(
                hsv_image,
                (0, preset["smin"], preset["vmin"]),
                (10, preset["smax"], preset["vmax"])
            )
            mask2 = cv2.inRange(
                hsv_image,
                (170, preset["smin"], preset["vmin"]),
                (179, preset["smax"], preset["vmax"])
            )
            th_image = cv2.bitwise_or(mask1, mask2)
        else:
            th_image = cv2.inRange(
                hsv_image,
                (preset["hmin"], preset["smin"], preset["vmin"]),
                (preset["hmax"], preset["smax"], preset["vmax"])
            )

        kernel_open = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        kernel_close = cv2.getStructuringElement(cv2.MORPH_RECT, (11, 11))
        kernel_dilate = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

        th_image = cv2.morphologyEx(th_image, cv2.MORPH_OPEN, kernel_open)
        th_image = cv2.morphologyEx(th_image, cv2.MORPH_CLOSE, kernel_close)
        th_image = cv2.dilate(th_image, kernel_dilate, iterations=1)

        contours, _ = cv2.findContours(th_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected = False

        if len(contours) > 0:
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)

            if area >= 100:
                detected = True
                self.detected_frames += 1

                hull = cv2.convexHull(largest_contour)
                x, y, w, h = cv2.boundingRect(hull)

                if h < 0.9 * w:
                    target_h = int(1.10 * w)
                    h = target_h

                x_pad = int(0.04 * w)
                y_pad = int(0.04 * h)

                x = max(0, x - x_pad)
                y = max(0, y - y_pad)
                w = min(image_w - x, w + 2 * x_pad)
                h = min(image_h - y, h + 2 * y_pad)

                target_u = x + w // 2
                target_v = y + h // 2

                center_error_px = abs(target_u - image_w / 2.0)
                self.center_errors_px.append(center_error_px)

                if not self.task_started:
                    self.task_started = True
                    self.task_start_time = rospy.Time.now().to_sec()

                measured_Z = None
                distance_source = "none"

                # -----------------------------
                # 优先深度
                # -----------------------------
                if depth_image is not None and camera_matrix is not None:
                    depth_h, depth_w = depth_image.shape[:2]

                    if 0 <= target_u < depth_w and 0 <= target_v < depth_h:
                        Z_depth = self.get_average_depth(depth_image, target_u, target_v, depth_encoding, window_size=5)
                        if Z_depth is not None:
                            measured_Z = Z_depth
                            distance_source = "depth"
                            self.valid_depth_frames += 1

                # -----------------------------
                # 激光兜底（可选）
                # -----------------------------
                if measured_Z is None and self.enable_laser_fallback and latest_scan is not None and camera_matrix is not None:
                    Z_laser = self.get_laser_depth_for_target(latest_scan, target_u, camera_matrix, angle_window=3)
                    if Z_laser is not None:
                        measured_Z = Z_laser
                        distance_source = "laser"
                        self.laser_fallback_frames += 1

                # -----------------------------
                # 如果最终拿到可用距离
                # -----------------------------
                if measured_Z is not None and camera_matrix is not None:
                    self.effective_distance_frames += 1

                    fx = camera_matrix[0, 0]
                    fy = camera_matrix[1, 1]
                    cx = camera_matrix[0, 2]
                    cy = camera_matrix[1, 2]

                    X = (target_u - cx) * measured_Z / fx
                    Y = (target_v - cy) * measured_Z / fy
                    Z = measured_Z

                    p_cam = PointStamped()
                    p_cam.header.frame_id = self.source_frame
                    p_cam.header.stamp = rospy.Time(0)
                    p_cam.point.x = float(X)
                    p_cam.point.y = float(Y)
                    p_cam.point.z = float(Z)

                    try:
                        if self.tf_buffer.can_transform(
                            self.target_frame,
                            self.source_frame,
                            rospy.Time(0),
                            rospy.Duration(0.2)
                        ):
                            p_robot = self.tf_buffer.transform(
                                p_cam,
                                self.target_frame,
                                rospy.Duration(0.2)
                            )
                            self.tf_success_frames += 1

                            robot_x = p_robot.point.x
                            distance_error = robot_x - self.desired_distance
                            self.distance_errors.append(distance_error)

                            if abs(distance_error) < self.distance_threshold:
                                if not self.in_reached_zone and self.task_started:
                                    task_time = rospy.Time.now().to_sec() - self.task_start_time
                                    self.task_completion_times.append(task_time)
                                    self.in_reached_zone = True

                                    rospy.loginfo(
                                        "检测到到达目标条件：distance_error=%.4f, source=%s",
                                        distance_error, distance_source
                                    )

                                    if self.auto_stop_on_reached and not self.report_printed:
                                        self.print_report()
                                        self.report_printed = True
                                        rospy.signal_shutdown("Reached state detected, evaluation finished.")
                                        return
                            else:
                                self.in_reached_zone = False

                    except Exception:
                        pass

        if not detected:
            self.in_reached_zone = False

        end_time = time.perf_counter()
        response_ms = (end_time - start_time) * 1000.0
        self.response_times.append(response_ms)

    # =====================================================
    # 工具函数
    # =====================================================
    def get_depth_in_meters(self, depth_raw_value, encoding):
        if depth_raw_value is None:
            return None

        if encoding == "32FC1":
            z = float(depth_raw_value)
            if np.isnan(z) or np.isinf(z) or z <= 0:
                return None
            return z

        elif encoding == "16UC1":
            z = float(depth_raw_value) / 1000.0
            if z <= 0:
                return None
            return z

        else:
            z = float(depth_raw_value)
            if np.isnan(z) or np.isinf(z) or z <= 0:
                return None
            return z

    def get_average_depth(self, depth_img, u, v, encoding, window_size=5):
        h, w = depth_img.shape[:2]
        half = window_size // 2

        u_min = max(0, u - half)
        u_max = min(w, u + half + 1)
        v_min = max(0, v - half)
        v_max = min(h, v + half + 1)

        region = depth_img[v_min:v_max, u_min:u_max]

        values = []
        for row in region:
            for d in row:
                z = self.get_depth_in_meters(d, encoding)
                if z is not None:
                    values.append(z)

        if len(values) == 0:
            return None

        return float(np.mean(values))

    def get_laser_depth_for_target(self, scan_msg, target_u, camera_matrix, angle_window=3):
        if scan_msg is None or camera_matrix is None:
            return None

        fx = camera_matrix[0, 0]
        cx = camera_matrix[0, 2]

        angle = math.atan2((target_u - cx), fx)

        if scan_msg.angle_increment == 0:
            return None

        index = int(round((angle - scan_msg.angle_min) / scan_msg.angle_increment))

        if index < 0 or index >= len(scan_msg.ranges):
            return None

        start = max(0, index - angle_window)
        end = min(len(scan_msg.ranges), index + angle_window + 1)

        vals = []
        for r in scan_msg.ranges[start:end]:
            if not np.isnan(r) and not np.isinf(r) and r > 0:
                vals.append(r)

        if len(vals) == 0:
            return None

        return float(np.mean(vals))

    def rmse(self, values):
        if len(values) == 0:
            return None
        arr = np.array(values, dtype=np.float64)
        return float(np.sqrt(np.mean(arr ** 2)))

    # =====================================================
    # 打印报告
    # =====================================================
    def print_report(self):
        detection_success_rate = 0.0
        if self.total_frames > 0:
            detection_success_rate = 100.0 * self.detected_frames / self.total_frames

        avg_response_time = np.mean(self.response_times) if len(self.response_times) > 0 else 0.0
        center_rmse_px = self.rmse(self.center_errors_px)
        distance_rmse_m = self.rmse(self.distance_errors)
        avg_task_completion_time = np.mean(self.task_completion_times) if len(self.task_completion_times) > 0 else None

        valid_depth_rate = 0.0
        if self.detected_frames > 0:
            valid_depth_rate = 100.0 * self.valid_depth_frames / self.detected_frames

        tf_success_rate = 0.0
        if self.detected_frames > 0:
            tf_success_rate = 100.0 * self.tf_success_frames / self.detected_frames

        effective_distance_rate = 0.0
        if self.detected_frames > 0:
            effective_distance_rate = 100.0 * self.effective_distance_frames / self.detected_frames

        laser_fallback_usage_rate = 0.0
        if self.detected_frames > 0:
            laser_fallback_usage_rate = 100.0 * self.laser_fallback_frames / self.detected_frames

        rospy.loginfo("\n")
        rospy.loginfo("======================================================")
        rospy.loginfo("               Performance Metrics Report             ")
        rospy.loginfo("======================================================")
        rospy.loginfo("| %-36s | %-15s |", "Metric", "Value")
        rospy.loginfo("------------------------------------------------------")
        rospy.loginfo("| %-36s | %-15.2f |", "Detection Success Rate (%)", detection_success_rate)
        rospy.loginfo("| %-36s | %-15.2f |", "Average Response Time (ms)", avg_response_time)

        if center_rmse_px is not None:
            rospy.loginfo("| %-36s | %-15.2f |", "Tracking Accuracy RMSE (px)", center_rmse_px)
        else:
            rospy.loginfo("| %-36s | %-15s |", "Tracking Accuracy RMSE (px)", "N/A")

        if distance_rmse_m is not None:
            rospy.loginfo("| %-36s | %-15.4f |", "Distance Error RMSE (m)", distance_rmse_m)
        else:
            rospy.loginfo("| %-36s | %-15s |", "Distance Error RMSE (m)", "N/A")

        if avg_task_completion_time is not None:
            rospy.loginfo("| %-36s | %-15.2f |", "Average Task Completion Time (s)", avg_task_completion_time)
        else:
            rospy.loginfo("| %-36s | %-15s |", "Average Task Completion Time (s)", "N/A")

        rospy.loginfo("| %-36s | %-15.2f |", "Valid Depth Rate (%)", valid_depth_rate)
        rospy.loginfo("| %-36s | %-15.2f |", "TF Transform Success Rate (%)", tf_success_rate)
        rospy.loginfo("| %-36s | %-15.2f |", "Effective Distance Availability (%)", effective_distance_rate)
        rospy.loginfo("| %-36s | %-15.2f |", "Laser Fallback Usage Rate (%)", laser_fallback_usage_rate)
        rospy.loginfo("| %-36s | %-15d |", "Total Frames", self.total_frames)
        rospy.loginfo("| %-36s | %-15d |", "Detected Frames", self.detected_frames)
        rospy.loginfo("======================================================")
        rospy.loginfo("\n")

        print("\nTable 2: Performance Metrics")
        print("| Metric | Value |")
        print("| --- | --- |")
        print(f"| Detection Success Rate | {detection_success_rate:.2f}% |")
        print(f"| Average Response Time | {avg_response_time:.2f} ms |")

        if center_rmse_px is not None:
            print(f"| Tracking Accuracy (Image RMSE) | {center_rmse_px:.2f} px |")
        else:
            print(f"| Tracking Accuracy (Image RMSE) | N/A |")

        if distance_rmse_m is not None:
            print(f"| Tracking Accuracy (Distance RMSE) | {distance_rmse_m:.4f} m |")
        else:
            print(f"| Tracking Accuracy (Distance RMSE) | N/A |")

        if avg_task_completion_time is not None:
            print(f"| Average Task Completion Time | {avg_task_completion_time:.2f} s |")
        else:
            print(f"| Average Task Completion Time | N/A |")

        print(f"| Valid Depth Rate | {valid_depth_rate:.2f}% |")
        print(f"| TF Transform Success Rate | {tf_success_rate:.2f}% |")
        print(f"| Effective Distance Availability Rate | {effective_distance_rate:.2f}% |")
        print(f"| Laser Fallback Usage Rate | {laser_fallback_usage_rate:.2f}% |")
        print("")

    def on_shutdown(self):
        if not self.report_printed:
            self.print_report()
            self.report_printed = True


if __name__ == "__main__":
    node = TrackingMetricsNode()
    rospy.spin()