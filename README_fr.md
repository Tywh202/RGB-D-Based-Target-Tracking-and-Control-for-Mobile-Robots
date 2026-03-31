**Read this in other languages: [English](README_en.md), [中文](README.md), [Français](README_fr.md), [Español](README_es.md), [العربية](README_ar.md), [Русский](README_ru.md), [日本语](README_jp.md)**
# 🎯 Système de suivi et contrôle d'objets colorés basé sur ROS pour TurtleBot3 🤖



## 📋 Description du projet

Ce projet implémente un système complet de robot mobile autonome sous ROS, capable de détecter, suivre et poursuivre en temps réel des objets colorés à l’aide d’une caméra RGB-D 🎨

Le système utilise l’espace colorimétrique HSV pour la segmentation de l’objet cible, combine les informations de profondeur pour estimer sa position 3D, et met en œuvre un comportement de suivi robuste via une machine à états finis 🎯



## ✨ Fonctionnalités

| Fonctionnalité | Description |
|:---|:---|
| 🎨 **Détection d’objet coloré** | Segmentation robuste basée sur l’espace HSV |
| 📍 **Localisation 3D** | Utilisation des données de profondeur RGB-D pour calculer la position de l’objet dans le repère caméra puis dans le repère robot |
| 🔄 **Machine à états** | Stratégie de suivi en quatre états : `SEARCHING` → `ALIGNING` → `APPROACHING` → `REACHED` |
| 🏃 **Suivi d’objet dynamique** | Poursuite en temps réel d’objets mobiles |
| 🌈 **Support multi‑couleurs** | Vert 💚 / Orange 🧡 / Rouge ❤️ / Bleu 💙 sélectionnables via curseur OpenCV |
| 📡 **Secours Lidar** (version bêta) | Mesure de distance alternative par lidar lorsque la profondeur est invalide |


## 📁 Description des fichiers

| Fichier | Description | Version |
|:---|:---|:---:|
| `hsv_node_release.py` | ⭐ **Programme de suivi officiel** | ✅ |
| `hsv_node_beta.py` | 🧪 Version bêta avec secours lidar | 🔬 |
| `turtlebot3_balls.launch` | 🌍 Lancement de la simulation Gazebo (avec balles colorées) | - |
| `move_ball.py` | ⚽ Programme de déplacement aléatoire de la balle cible | - |
| `plot_trajectory.py` | 📈 Enregistrement et tracé de la trajectoire du robot | - |
| `tracking_metrics.py` | 📊 Évaluation des performances | - |



## 🛠️ Prérequis système

- 🐢 ROS Noetic
- 🤖 TurtleBot3
- 📷 OpenCV
- 🐍 Python 3



## ⚙️ Installation et compilation

```bash
# Se rendre dans l’espace de travail
cd ~/catkin_ws/src

# Cloner le dépôt
git clone https://github.com/Tywh202/RGB-D-Based-Target-Tracking-and-Control-for-Mobile-Robots.git

# Compiler
cd ~/catkin_ws
catkin_make

# Configurer l’environnement
source devel/setup.bash
```



## 🚀 Démarrage rapide

### 1️⃣ Lancer l’environnement de simulation

```bash
export TURTLEBOT3_MODEL=waffle
roslaunch image_pkg turtlebot3_balls.launch
```

### 2️⃣ Lancer le suivi de cible

**Version officielle** ⭐ (celle utilisée dans le rapport) :
```bash
rosrun image_pkg hsv_node_release.py
```

**Version bêta** 🧪 (avec secours lidar) :
```bash
rosrun image_pkg hsv_node_beta.py
```

### 3️⃣ Faire bouger la balle cible (optionnel) ⚽

```bash
rosrun image_pkg move_ball.py _model_name:=green_ball
```

### 4️⃣ Enregistrer la trajectoire du robot 📈

```bash
rosrun image_pkg plot_trajectory.py \
    _output_path:=~/trajectory.png \
    _target_model_name:=green_ball
```

### 5️⃣ Évaluation des performances 📊

```bash
rosrun image_pkg tracking_metrics.py \
    _target_color_name:=green \
    _auto_stop_on_reached:=true
```



## 🎮 Mode d’emploi

Après le lancement de `hsv_node_release.py`, trois fenêtres OpenCV apparaissent :

| Fenêtre | Fonction |
|:---|:---|
| 🎚️ **Threshold** | Sélection de la couleur cible via curseur (`0`=manuel, `1`=💚vert, `2`=🧡orange, `3`=❤️rouge, `4`=💙bleu) |
| 🖼️ **RGB** | Affichage de la détection (cadre vert ✅, croix bleue au centre 🔵, informations d’état) |
| 📊 **Result** | Image binaire après seuillage HSV |

Le robot entre automatiquement dans le cycle d’états :

```
🔍 SEARCHING ──► 🎯 ALIGNING ──► 🏃 APPROACHING ──► ✅ REACHED
```



## 📷 Visualisation

### Effet de la détection de cible
![Capture de la détection](docs/detection_screenshot.png)

### Trajectoire du robot
![Tracé de trajectoire](docs/trajectory_plot.png)


https://github.com/user-attachments/assets/c4767ce2-2d3d-4d69-bc6a-8f826a56afd9


## 📚 Références

- 📖 [ROS Wiki](http://wiki.ros.org/)
- 📖 [Documentation OpenCV](https://docs.opencv.org/)
- 📖 [wpr_simulation](https://github.com/6-robot/wpr_simulation.git)
