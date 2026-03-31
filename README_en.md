**Read this in other languages: [English](README_en.md), [中文](README.md), [Français](README_fr.md), [Español](README_es.md), [العربية](README_ar.md), [Русский](README_ru.md), [日本语](README_jp.md)**
# 🎯 ROS-based TurtleBot3 Color Target Tracking and Control System 🤖



## 📋 Project Description

This project implements a complete ROS autonomous mobile robot system capable of real-time detection, tracking, and following of colored target objects using an RGB-D camera 🎨

The system employs HSV color space for target segmentation, combines depth information to estimate the target's 3D position, and achieves robust tracking control behavior through a finite state machine 🎯



## ✨ Features

| Feature | Description |
|:---|:---|
| 🎨 **Color Target Detection** | Robust target segmentation based on HSV color space |
| 📍 **3D Localization** | Uses RGB-D depth information to compute target position in camera frame and robot frame |
| 🔄 **State Machine Control** | Four-state tracking strategy: `SEARCHING` → `ALIGNING` → `APPROACHING` → `REACHED` |
| 🏃 **Dynamic Target Tracking** | Supports real-time following of moving targets |
| 🌈 **Multi-color Support** | Green 💚 / Orange 🧡 / Red ❤️ / Blue 💙 selectable (via OpenCV sliders) |
| 📡 **Lidar Fallback** (Beta) | Alternative distance measurement when depth is invalid |


## 📁 File Description

| File | Description | Version |
|:---|:---|:---:|
| `hsv_node_release.py` | ⭐ **Official tracking program** | ✅ |
| `hsv_node_beta.py` | 🧪 Beta version with lidar fallback | 🔬 |
| `turtlebot3_balls.launch` | 🌍 Gazebo simulation launch file (with colored balls) | - |
| `move_ball.py` | ⚽ Random movement program for target ball | - |
| `plot_trajectory.py` | 📈 Robot trajectory recording and plotting program | - |
| `tracking_metrics.py` | 📊 Performance metrics evaluation program | - |



## 🛠️ System Requirements

- 🐢 ROS Noetic
- 🤖 TurtleBot3
- 📷 OpenCV
- 🐍 Python 3



## ⚙️ Installation & Build

```bash
# Navigate to workspace
cd ~/catkin_ws/src

# Clone repository
git clone https://github.com/Tywh202/RGB-D-Based-Target-Tracking-and-Control-for-Mobile-Robots.git

# Build
cd ~/catkin_ws
catkin_make

# Setup environment
source devel/setup.bash
```



## 🚀 Quick Start

### 1️⃣ Launch Simulation Environment

```bash
export TURTLEBOT3_MODEL=waffle
roslaunch image_pkg turtlebot3_balls.launch
```

### 2️⃣ Start Target Tracking

**Official version** ⭐ (report based on this version):
```bash
rosrun image_pkg hsv_node_release.py
```

**Beta version** 🧪 (with lidar fallback):
```bash
rosrun image_pkg hsv_node_beta.py
```

### 3️⃣ Move Target Ball (Optional) ⚽

```bash
rosrun image_pkg move_ball.py _model_name:=green_ball
```

### 4️⃣ Record Robot Trajectory 📈

```bash
rosrun image_pkg plot_trajectory.py \
    _output_path:=~/trajectory.png \
    _target_model_name:=green_ball
```

### 5️⃣ Performance Evaluation 📊

```bash
rosrun image_pkg tracking_metrics.py \
    _target_color_name:=green \
    _auto_stop_on_reached:=true
```



## 🎮 Operation Instructions

After launching `hsv_node_release.py`, OpenCV windows will appear:

| Window | Function |
|:---|:---|
| 🎚️ **Threshold** | Select target color via slider (`0`=Manual, `1`=💚Green, `2`=🧡Orange, `3`=❤️Red, `4`=💙Blue) |
| 🖼️ **RGB** | Displays detection results (green box ✅, blue cross center 🔵, status info) |
| 📊 **Result** | Displays binary image after HSV threshold segmentation |

The robot will automatically enter the state cycle:

```
🔍 SEARCHING ──► 🎯 ALIGNING ──► 🏃 APPROACHING ──► ✅ REACHED
```



## 📷 Visualization Effects

### Target Detection Effect
![Detection Screenshot](docs/detection_screenshot.png)

### Robot Tracking Trajectory
![Trajectory Plot](docs/trajectory_plot.png)


https://github.com/user-attachments/assets/c4767ce2-2d3d-4d69-bc6a-8f826a56afd9


## 📚 References

- 📖 [ROS Wiki](http://wiki.ros.org/)
- 📖 [OpenCV Documentation](https://docs.opencv.org/)
- 📖 [wpr_simulation](https://github.com/6-robot/wpr_simulation.git)
