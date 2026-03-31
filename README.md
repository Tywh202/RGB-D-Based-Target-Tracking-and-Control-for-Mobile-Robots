**Read this in other languages: [English](README_en.md), [中文](README.md), [Français](README_fr.md), [Español](README_es.md), [العربية](README_ar.md), [Русский](README_ru.md), [日本语](README_jp.md)**
# 🎯 基于 ROS 的 TurtleBot3 颜色目标追踪与控制系统 🤖



## 📋 项目简介

本项目实现了一个完整的 ROS 自主移动机器人系统，能够利用 RGB-D 相机实时检测、追踪并跟随彩色目标物体 🎨

系统采用 HSV 颜色空间进行目标分割，结合深度信息估计目标三维位置，并通过有限状态机实现鲁棒的跟踪控制行为 🎯



## ✨ 功能特性

| 功能 | 说明 |
|:---|:---|
| 🎨 **颜色目标检测** | 基于 HSV 颜色空间的鲁棒目标分割 |
| 📍 **三维定位** | 利用 RGB-D 深度信息计算目标在相机坐标系和机器人坐标系下的位置 |
| 🔄 **状态机控制** | `SEARCHING` → `ALIGNING` → `APPROACHING` → `REACHED` 四状态跟踪策略 |
| 🏃 **动态目标跟踪** | 支持移动目标的实时跟随 |
| 🌈 **多颜色支持** | 绿色 💚 / 橙色 🧡 / 红色 ❤️ / 蓝色 💙 可选（通过 OpenCV 滑动条切换） |
| 📡 **激光雷达兜底**（测试版） | 深度无效时可用激光雷达替代测距 |


## 📁 文件说明

| 文件 | 说明 | 版本 |
|:---|:---|:---:|
| `hsv_node_release.py` | ⭐ **正式版追踪程序**| ✅ |
| `hsv_node_beta.py` | 🧪 测试版，添加激光雷达兜底功能 | 🔬 |
| `turtlebot3_balls.launch` | 🌍 Gazebo 仿真启动文件（含彩球生成） | - |
| `move_ball.py` | ⚽ 目标球随机移动程序 | - |
| `plot_trajectory.py` | 📈 机器人轨迹记录与绘图程序 | - |
| `tracking_metrics.py` | 📊 性能指标评测程序 | - |



## 🛠️ 系统要求

- 🐢 ROS Noetic
- 🤖 TurtleBot3
- 📷 OpenCV
- 🐍 Python 3



## ⚙️ 安装与编译

```bash
# 进入工作空间
cd ~/catkin_ws/src

# 克隆仓库
git clone https://github.com/your_username/image_pkg.git

# 编译
cd ~/catkin_ws
catkin_make

# 配置环境
source devel/setup.bash
```



## 🚀 快速开始

### 1️⃣ 启动仿真环境

```bash
export TURTLEBOT3_MODEL=waffle
roslaunch image_pkg turtlebot3_balls.launch
```

### 2️⃣ 启动目标追踪

**正式版** ⭐（报告基于此版本）：
```bash
rosrun image_pkg hsv_node_release.py
```

**测试版** 🧪（含激光兜底）：
```bash
rosrun image_pkg hsv_node_beta.py
```

### 3️⃣ 让目标球移动（可选）⚽

```bash
rosrun image_pkg move_ball.py _model_name:=green_ball
```

### 4️⃣ 记录机器人轨迹 📈

```bash
rosrun image_pkg plot_trajectory.py \
    _output_path:=~/trajectory.png \
    _target_model_name:=green_ball
```

### 5️⃣ 性能评测 📊

```bash
rosrun image_pkg tracking_metrics.py \
    _target_color_name:=green \
    _auto_stop_on_reached:=true
```



## 🎮 操作说明

启动 `hsv_node_release.py` 后，会弹出五个 OpenCV 窗口，其中比较重要的是以下三个：

| 窗口 | 功能 |
|:---|:---|
| 🎚️ **Threshold** | 通过滑动条选择目标颜色（`0`=手动，`1`=💚绿，`2`=🧡橙，`3`=❤️红，`4`=💙蓝） |
| 🖼️ **RGB** | 显示检测结果（绿色框 ✅、蓝色十字中心 🔵、状态信息） |
| 📊 **Result** | 显示 HSV 阈值分割后的二值图 |

机器人将自动进入状态循环：

```
🔍 SEARCHING ──► 🎯 ALIGNING ──► 🏃 APPROACHING ──► ✅ REACHED
```



## 📷 可视化效果

### 目标检测效果
![检测截图](docs/detection_screenshot.png)

### 机器人追踪轨迹
![轨迹图](docs/trajectory_plot.png)




## 📚 参考

- 📖 [ROS Wiki](http://wiki.ros.org/)
- 📖 [OpenCV Documentation](https://docs.opencv.org/)
- 📖 [wpr_simulation](https://github.com/6-robot/wpr_simulation.git)
