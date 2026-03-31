#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import threading
from enum import Enum

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist, PointStamped
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

import tf2_ros
import tf2_geometry_msgs


# =========================================================
# 一、HSV 阈值全局变量
# =========================================================
hue_min = 10
hue_max = 40
satu_min = 90
satu_max = 255
val_min = 1
val_max = 255

color_mode = 0
last_color_mode = -1
current_color_name = "manual"


# =========================================================
# 二、颜色预设
# 注意：红色虽然这里给了一个区间，但真正分割时会使用双区间
# =========================================================
COLOR_PRESETS = {
    0: {"name": "manual", "hmin": 10,  "hmax": 40,  "smin": 90,  "smax": 255, "vmin": 0,   "vmax": 255},
    1: {"name": "green",  "hmin": 35,  "hmax": 85,  "smin": 60,  "smax": 255, "vmin": 0,  "vmax": 255},
    2: {"name": "orange", "hmin": 5,   "hmax": 20,  "smin": 100, "smax": 255, "vmin": 0, "vmax": 255},
    3: {"name": "red",    "hmin": 0,   "hmax": 10,  "smin": 80,  "smax": 255, "vmin": 0,  "vmax": 255},
    4: {"name": "blue",   "hmin": 100, "hmax": 130, "smin": 100, "smax": 255, "vmin": 0,  "vmax": 255}
}


# =========================================================
# 三、状态机
# =========================================================
class RobotState(Enum):
    SEARCHING = 0
    ALIGNING = 1
    APPROACHING = 2
    REACHED = 3


# =========================================================
# 四、全局变量
# =========================================================
bridge = CvBridge()

depth_image = None
depth_encoding = None
camera_matrix = None

tf_buffer = None
tf_listener = None

target_frame = "base_link"
source_frame = "camera_rgb_optical_frame"

vel_cmd = Twist()
vel_pub = None
target_color_pub = None

img_rgb = None
img_hsv = None
img_result = None
img_depth_vis = None
img_lock = threading.Lock()

current_state = RobotState.SEARCHING
target_lost_count = 0
max_lost_frames = 10

desired_distance = 0.50
distance_threshold = 0.08
align_threshold = 0.06

angular_kp = 1.5
linear_kp = 1.0

max_linear_speed = 0.40
max_angular_speed = 0.8
search_angular_speed = 0.5


# =========================================================
# 五、颜色预设切换
# =========================================================
def apply_color_preset(mode):
    global hue_min, hue_max, satu_min, satu_max, val_min, val_max, current_color_name

    if mode not in COLOR_PRESETS:
        return

    preset = COLOR_PRESETS[mode]
    current_color_name = preset["name"]

    hue_min = preset["hmin"]
    hue_max = preset["hmax"]
    satu_min = preset["smin"]
    satu_max = preset["smax"]
    val_min = preset["vmin"]
    val_max = preset["vmax"]

    cv2.setTrackbarPos("hue_min", "Threshold", hue_min)
    cv2.setTrackbarPos("hue_max", "Threshold", hue_max)
    cv2.setTrackbarPos("satu_min", "Threshold", satu_min)
    cv2.setTrackbarPos("satu_max", "Threshold", satu_max)
    cv2.setTrackbarPos("val_min", "Threshold", val_min)
    cv2.setTrackbarPos("val_max", "Threshold", val_max)

    rospy.loginfo("切换目标颜色为: %s", current_color_name)

    if target_color_pub is not None:
        target_color_pub.publish(String(data=current_color_name))


# =========================================================
# 六、深度图回调
# =========================================================
def depth_callback(msg):
    global depth_image, depth_encoding, img_depth_vis

    try:
        depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        depth_encoding = msg.encoding

        rospy.loginfo_once("已收到深度图")
        rospy.loginfo_once("深度图编码: %s", depth_encoding)

        if depth_image is not None:
            depth_vis = depth_image.copy()
            depth_vis = np.nan_to_num(depth_vis, nan=0.0, posinf=0.0, neginf=0.0)

            depth_norm = cv2.normalize(depth_vis, None, 0, 255, cv2.NORM_MINMAX)
            depth_norm = depth_norm.astype(np.uint8)
            depth_color = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)

            with img_lock:
                img_depth_vis = depth_color

    except CvBridgeError as e:
        rospy.logerr("深度图转换失败: %s", e)


# =========================================================
# 七、相机内参回调
# =========================================================
def camera_info_callback(msg):
    global camera_matrix
    camera_matrix = np.array(msg.K, dtype=np.float64).reshape(3, 3)
    rospy.loginfo_once("已收到相机内参:")
    rospy.loginfo_once("\n%s", camera_matrix)


# =========================================================
# 八、深度值统一转换成米
# =========================================================
def get_depth_in_meters(depth_raw_value, encoding):
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


# =========================================================
# 九、邻域平均深度
# =========================================================
def get_average_depth(depth_img, u, v, window_size=5):
    global depth_encoding

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
            z = get_depth_in_meters(d, depth_encoding)
            if z is not None:
                values.append(z)

    if len(values) == 0:
        return None

    return float(np.mean(values))


# =========================================================
# 十、空回调
# =========================================================
def nothing(x):
    pass


# =========================================================
# 十一、RGB 回调
# =========================================================
def rgb_callback(msg):
    global hue_min, hue_max, satu_min, satu_max, val_min, val_max
    global depth_image, camera_matrix
    global tf_buffer, target_frame, source_frame
    global vel_cmd, vel_pub
    global img_rgb, img_hsv, img_result
    global current_state, target_lost_count
    global current_color_name

    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("RGB图像转换失败: %s", e)
        return

    image_h, image_w = cv_image.shape[:2]

    vel_cmd.linear.x = 0.0
    vel_cmd.angular.z = 0.0

    # BGR -> HSV
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # 提升亮度鲁棒性
    h, s, v = cv2.split(hsv_image)
    v = cv2.equalizeHist(v)
    hsv_image = cv2.merge([h, s, v])

    # =====================================================
    # 红色使用双区间 HSV 检测
    # =====================================================
    if current_color_name == "red":
        mask1 = cv2.inRange(
            hsv_image,
            (0, satu_min, val_min),
            (10, satu_max, val_max)
        )
        mask2 = cv2.inRange(
            hsv_image,
            (170, satu_min, val_min),
            (179, satu_max, val_max)
        )
        th_image = cv2.bitwise_or(mask1, mask2)
    else:
        th_image = cv2.inRange(
            hsv_image,
            (hue_min, satu_min, val_min),
            (hue_max, satu_max, val_max)
        )

    # 形态学处理
    kernel_open = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    kernel_close = cv2.getStructuringElement(cv2.MORPH_RECT, (11, 11))
    kernel_dilate = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

    th_image = cv2.morphologyEx(th_image, cv2.MORPH_OPEN, kernel_open)
    th_image = cv2.morphologyEx(th_image, cv2.MORPH_CLOSE, kernel_close)
    th_image = cv2.dilate(th_image, kernel_dilate, iterations=1)

    contours, _ = cv2.findContours(th_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 图像中心参考十字（黄色）
    cv2.line(cv_image, (image_w // 2 - 20, image_h // 2), (image_w // 2 + 20, image_h // 2), (0, 255, 255), 2)
    cv2.line(cv_image, (image_w // 2, image_h // 2 - 20), (image_w // 2, image_h // 2 + 20), (0, 255, 255), 2)

    cv2.putText(cv_image, f"Color: {current_color_name}", (10, 120),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)

    target_found = False

    if len(contours) > 0:
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)

        if area >= 100:
            target_found = True
            target_lost_count = 0

            # 先凸包，再矩形
            hull = cv2.convexHull(largest_contour)
            x, y, w, h = cv2.boundingRect(hull)

            # 高度不足时，适度向下补偿
            if h < 0.9 * w:
                target_h = int(1.10 * w)
                h = target_h

            # 轻微留边
            x_pad = int(0.04 * w)
            y_pad = int(0.04 * h)

            x = x - x_pad
            y = y - y_pad
            w = w + 2 * x_pad
            h = h + 2 * y_pad

            # 防止越界
            x = max(0, x)
            y = max(0, y)
            if x + w > image_w:
                w = image_w - x
            if y + h > image_h:
                h = image_h - y

            # 目标中心 = 绿色框中心 = 蓝色十字中心 = 深度采样中心
            target_u = x + w // 2
            target_v = y + h // 2

            # 画绿色框框选目标
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # 画蓝色十字标记目标中心
            cv2.line(cv_image, (target_u - 10, target_v), (target_u + 10, target_v), (255, 0, 0), 2)
            cv2.line(cv_image, (target_u, target_v - 10), (target_u, target_v + 10), (255, 0, 0), 2)

            rospy.loginfo_throttle(
                1.0,
                "目标颜色: %s, 像素坐标: (u=%d, v=%d), area=%.1f, w=%d, h=%d",
                current_color_name, target_u, target_v, area, w, h
            )

            if depth_image is not None and camera_matrix is not None:
                depth_h, depth_w = depth_image.shape[:2]

                if 0 <= target_u < depth_w and 0 <= target_v < depth_h:
                    Z = get_average_depth(depth_image, target_u, target_v, window_size=5)

                    if Z is not None:
                        fx = camera_matrix[0, 0]
                        fy = camera_matrix[1, 1]
                        cx = camera_matrix[0, 2]
                        cy = camera_matrix[1, 2]

                        X = (target_u - cx) * Z / fx
                        Y = (target_v - cy) * Z / fy

                        rospy.loginfo_throttle(
                            1.0,
                            "相机坐标系: X=%.3f m, Y=%.3f m, Z=%.3f m",
                            X, Y, Z
                        )

                        text_cam = "Cam: X:{:.2f} Y:{:.2f} Z:{:.2f}m".format(X, Y, Z)
                        cv2.putText(cv_image, text_cam, (x, max(y - 10, 20)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

                        p_cam = PointStamped()
                        p_cam.header.frame_id = source_frame
                        p_cam.header.stamp = rospy.Time(0)
                        p_cam.point.x = float(X)
                        p_cam.point.y = float(Y)
                        p_cam.point.z = float(Z)

                        try:
                            if tf_buffer is not None and tf_buffer.can_transform(
                                target_frame,
                                p_cam.header.frame_id,
                                rospy.Time(0),
                                rospy.Duration(0.2)
                            ):
                                p_robot = tf_buffer.transform(
                                    p_cam,
                                    target_frame,
                                    rospy.Duration(0.2)
                                )

                                robot_x = p_robot.point.x
                                robot_y = p_robot.point.y
                                robot_z = p_robot.point.z

                                rospy.loginfo_throttle(
                                    1.0,
                                    "机器人坐标系(%s): x=%.3f, y=%.3f, z=%.3f",
                                    target_frame, robot_x, robot_y, robot_z
                                )

                                text_robot = "Robot: X:{:.2f} Y:{:.2f} Z:{:.2f}m".format(
                                    robot_x, robot_y, robot_z
                                )
                                cv2.putText(cv_image, text_robot, (x, max(y - 30, 40)),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

                                image_center_x = image_w / 2.0
                                horizontal_error = (target_u - image_center_x) / image_center_x
                                distance_error = robot_x - desired_distance

                                # 真正接近目标时，提示已到达
                                if robot_x <= desired_distance + 0.03:
                                    rospy.loginfo_throttle(1.0, "距离太近，已到达目标位置")

                                if current_state == RobotState.SEARCHING:
                                    current_state = RobotState.ALIGNING
                                    rospy.loginfo("状态切换: SEARCHING -> ALIGNING")

                                if current_state == RobotState.ALIGNING:
                                    if abs(horizontal_error) > align_threshold:
                                        vel_cmd.angular.z = -angular_kp * horizontal_error
                                        vel_cmd.angular.z = max(-max_angular_speed,
                                                                min(max_angular_speed, vel_cmd.angular.z))

                                        if distance_error > 0.3:
                                            vel_cmd.linear.x = min(0.10, 0.25 * distance_error)
                                        else:
                                            vel_cmd.linear.x = 0.0
                                    else:
                                        current_state = RobotState.APPROACHING
                                        rospy.loginfo("状态切换: ALIGNING -> APPROACHING")

                                elif current_state == RobotState.APPROACHING:
                                    if abs(horizontal_error) > align_threshold * 3.0:
                                        current_state = RobotState.ALIGNING
                                        rospy.loginfo("状态切换: APPROACHING -> ALIGNING")
                                    elif abs(distance_error) < distance_threshold:
                                        current_state = RobotState.REACHED
                                        rospy.loginfo("状态切换: APPROACHING -> REACHED")
                                    else:
                                        vel_cmd.angular.z = -0.8 * angular_kp * horizontal_error
                                        vel_cmd.angular.z = max(-max_angular_speed,
                                                                min(max_angular_speed, vel_cmd.angular.z))

                                        if distance_error > 0:
                                            forward_scale = max(0.25, 1.0 - abs(horizontal_error))
                                            vel_cmd.linear.x = linear_kp * distance_error * forward_scale
                                            vel_cmd.linear.x = max(0.0,
                                                                   min(max_linear_speed, vel_cmd.linear.x))
                                        else:
                                            vel_cmd.linear.x = 0.0

                                elif current_state == RobotState.REACHED:
                                    vel_cmd.linear.x = 0.0
                                    vel_cmd.angular.z = 0.0

                                    if abs(horizontal_error) > align_threshold * 1.5:
                                        current_state = RobotState.ALIGNING
                                        rospy.loginfo("状态切换: REACHED -> ALIGNING")
                                    elif distance_error > distance_threshold:
                                        current_state = RobotState.APPROACHING
                                        rospy.loginfo("状态切换: REACHED -> APPROACHING")

                                state_text = "State: {}".format(current_state.name)
                                dist_text = "DistErr: {:.2f}".format(distance_error)
                                hori_text = "HorErr: {:.3f}".format(horizontal_error)

                                cv2.putText(cv_image, state_text, (10, 30),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                                cv2.putText(cv_image, dist_text, (10, 60),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                                cv2.putText(cv_image, hori_text, (10, 90),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

                            else:
                                rospy.logwarn_throttle(2.0, "TF不可用：%s -> %s", source_frame, target_frame)

                        except (tf2_ros.LookupException,
                                tf2_ros.ConnectivityException,
                                tf2_ros.ExtrapolationException) as e:
                            rospy.logwarn_throttle(2.0, "TF转换失败: %s", e)

                    else:
                        rospy.logwarn_throttle(1.0, "距离超出范围，可能被遮挡或距离太远或已到达目标")

                        image_center_x = image_w / 2.0
                        horizontal_error = (target_u - image_center_x) / image_center_x

                        # 先保持对准
                        vel_cmd.angular.z = -angular_kp * horizontal_error
                        vel_cmd.angular.z = max(-max_angular_speed,
                                                min(max_angular_speed, vel_cmd.angular.z))

                        # =================================================
                        # 远距离兜底策略：
                        # 如果目标面积较小，说明目标大概率较远，
                        # 此时即使没有有效深度，也允许小速度前进靠近
                        # =================================================
                        if area < 6000:
                            if abs(horizontal_error) < 0.12:
                                vel_cmd.linear.x = 0.10
                            else:
                                vel_cmd.linear.x = 0.0
                        else:
                            # 面积已经不小，说明可能较近，此时深度无效就不要贸然前进
                            vel_cmd.linear.x = 0.0

                else:
                    rospy.logwarn_throttle(1.0, "目标超出深度图范围")
            else:
                rospy.logwarn_throttle(1.0, "深度图或相机内参未就绪")

    if not target_found:
        target_lost_count += 1

        if target_lost_count > max_lost_frames:
            if current_state != RobotState.SEARCHING:
                rospy.loginfo("状态切换: %s -> SEARCHING", current_state.name)

            current_state = RobotState.SEARCHING
            vel_cmd.linear.x = 0.0
            vel_cmd.angular.z = search_angular_speed

    rospy.loginfo_throttle(
        1.0,
        "当前颜色=%s, 速度指令: linear.x=%.3f, angular.z=%.3f",
        current_color_name, vel_cmd.linear.x, vel_cmd.angular.z
    )

    if vel_pub is not None:
        vel_pub.publish(vel_cmd)

    with img_lock:
        img_rgb = cv_image.copy()
        img_hsv = hsv_image.copy()
        img_result = th_image.copy()


# =========================================================
# 十二、主函数
# =========================================================
if __name__ == "__main__":
    rospy.init_node("hsv_node", anonymous=True)

    rgb_topic = rospy.get_param("~rgb_topic", "/camera/rgb/image_raw")
    depth_topic = rospy.get_param("~depth_topic", "/camera/depth/image_raw")
    info_topic = rospy.get_param("~info_topic", "/camera/rgb/camera_info")
    target_frame = rospy.get_param("~target_frame", "base_link")
    source_frame = rospy.get_param("~source_frame", "camera_rgb_optical_frame")

    rospy.loginfo("===== TurtleBot3 HSV 状态机跟踪节点（单机器人最终版）启动 =====")
    rospy.loginfo("RGB topic: %s", rgb_topic)
    rospy.loginfo("Depth topic: %s", depth_topic)
    rospy.loginfo("Info topic: %s", info_topic)
    rospy.loginfo("Source frame: %s", source_frame)
    rospy.loginfo("Target frame: %s", target_frame)

    vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    target_color_pub = rospy.Publisher("/target_color", String, queue_size=10, latch=True)

    rospy.Subscriber(rgb_topic, Image, rgb_callback, queue_size=1)
    rospy.Subscriber(depth_topic, Image, depth_callback, queue_size=1)
    rospy.Subscriber(info_topic, CameraInfo, camera_info_callback, queue_size=1)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    cv2.namedWindow("Threshold", cv2.WINDOW_NORMAL)
    cv2.createTrackbar("color_mode", "Threshold", 0, 4, nothing)
    cv2.createTrackbar("hue_min", "Threshold", hue_min, 179, nothing)
    cv2.createTrackbar("hue_max", "Threshold", hue_max, 179, nothing)
    cv2.createTrackbar("satu_min", "Threshold", satu_min, 255, nothing)
    cv2.createTrackbar("satu_max", "Threshold", satu_max, 255, nothing)
    cv2.createTrackbar("val_min", "Threshold", val_min, 255, nothing)
    cv2.createTrackbar("val_max", "Threshold", val_max, 255, nothing)

    cv2.namedWindow("RGB", cv2.WINDOW_NORMAL)
    cv2.namedWindow("HSV", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Result", cv2.WINDOW_NORMAL)
    cv2.namedWindow("DepthVis", cv2.WINDOW_NORMAL)

    apply_color_preset(0)

    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        color_mode = cv2.getTrackbarPos("color_mode", "Threshold")

        if color_mode != last_color_mode:
            apply_color_preset(color_mode)
            last_color_mode = color_mode

        if color_mode == 0:
            hue_min = cv2.getTrackbarPos("hue_min", "Threshold")
            hue_max = cv2.getTrackbarPos("hue_max", "Threshold")
            satu_min = cv2.getTrackbarPos("satu_min", "Threshold")
            satu_max = cv2.getTrackbarPos("satu_max", "Threshold")
            val_min = cv2.getTrackbarPos("val_min", "Threshold")
            val_max = cv2.getTrackbarPos("val_max", "Threshold")
            current_color_name = "manual"

            if target_color_pub is not None:
                target_color_pub.publish(String(data=current_color_name))

        with img_lock:
            if img_rgb is not None:
                cv2.imshow("RGB", img_rgb)
            if img_hsv is not None:
                cv2.imshow("HSV", img_hsv)
            if img_result is not None:
                cv2.imshow("Result", img_result)
            if img_depth_vis is not None:
                cv2.imshow("DepthVis", img_depth_vis)

        cv2.waitKey(5)
        rate.sleep()

    cv2.destroyAllWindows()