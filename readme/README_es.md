**Read this in other languages: [English](readme/README_en.md), [中文](README.md), [日本语](readme/README_jp.md), [Français](readme/README_fr.md), [Español](readme/README_es.md), [العربية](readme/README_ar.md), [Русский](readme/README_ru.md).**
# 🎯 Sistema de seguimiento y control de objetos de color basado en ROS para TurtleBot3 🤖



## 📋 Descripción del proyecto

Este proyecto implementa un sistema completo de robot móvil autónomo bajo ROS, capaz de detectar, rastrear y seguir en tiempo real objetos de color utilizando una cámara RGB-D 🎨

El sistema emplea el espacio de color HSV para la segmentación del objetivo, combina información de profundidad para estimar su posición 3D y logra un comportamiento de seguimiento robusto mediante una máquina de estados finitos 🎯



## ✨ Características

| Característica | Descripción |
|:---|:---|
| 🎨 **Detección de objetos de color** | Segmentación robusta basada en el espacio de color HSV |
| 📍 **Localización 3D** | Uso de la información de profundidad RGB-D para calcular la posición del objetivo en el marco de la cámara y del robot |
| 🔄 **Control por máquina de estados** | Estrategia de seguimiento en cuatro estados: `SEARCHING` → `ALIGNING` → `APPROACHING` → `REACHED` |
| 🏃 **Seguimiento de objetivos dinámicos** | Seguimiento en tiempo real de objetos en movimiento |
| 🌈 **Soporte para múltiples colores** | Verde 💚 / Naranja 🧡 / Rojo ❤️ / Azul 💙 seleccionables mediante deslizadores de OpenCV |
| 📡 **Respaldo con Láser** (versión beta) | Medición alternativa de distancia mediante láser cuando la profundidad no es válida |


## 📁 Descripción de archivos

| Archivo | Descripción | Versión |
|:---|:---|:---:|
| `hsv_node_release.py` | ⭐ **Programa de seguimiento oficial** | ✅ |
| `hsv_node_beta.py` | 🧪 Versión beta con respaldo por láser | 🔬 |
| `turtlebot3_balls.launch` | 🌍 Archivo de lanzamiento de simulación en Gazebo (con generación de bolas de colores) | - |
| `move_ball.py` | ⚽ Programa de movimiento aleatorio de la bola objetivo | - |
| `plot_trajectory.py` | 📈 Registro y graficado de la trayectoria del robot | - |
| `tracking_metrics.py` | 📊 Programa de evaluación de métricas de rendimiento | - |



## 🛠️ Requisitos del sistema

- 🐢 ROS Noetic
- 🤖 Entorno de simulación TurtleBot3
- 📷 OpenCV
- 🐍 Python 3



## ⚙️ Instalación y compilación

```bash
# Ir al espacio de trabajo
cd ~/catkin_ws/src

# Clonar el repositorio
git clone https://github.com/your_username/image_pkg.git

# Compilar
cd ~/catkin_ws
catkin_make

# Configurar el entorno
source devel/setup.bash
```



## 🚀 Inicio rápido

### 1️⃣ Iniciar el entorno de simulación

```bash
export TURTLEBOT3_MODEL=waffle
roslaunch image_pkg turtlebot3_balls.launch
```

### 2️⃣ Iniciar el seguimiento del objetivo

**Versión oficial** ⭐ (el informe se basa en esta versión):
```bash
rosrun image_pkg hsv_node_release.py
```

**Versión beta** 🧪 (con respaldo por láser):
```bash
rosrun image_pkg hsv_node_beta.py
```

### 3️⃣ Mover la bola objetivo (opcional) ⚽

```bash
rosrun image_pkg move_ball.py _model_name:=green_ball
```

### 4️⃣ Registrar la trayectoria del robot 📈

```bash
rosrun image_pkg plot_trajectory.py \
    _output_path:=~/trajectory.png \
    _target_model_name:=green_ball
```

### 5️⃣ Evaluación del rendimiento 📊

```bash
rosrun image_pkg tracking_metrics.py \
    _target_color_name:=green \
    _auto_stop_on_reached:=true
```



## 🎮 Instrucciones de uso

Tras iniciar `hsv_node_release.py`, aparecerán tres ventanas de OpenCV:

| Ventana | Función |
|:---|:---|
| 🎚️ **Threshold** | Seleccionar el color objetivo mediante deslizadores (`0`=manual, `1`=💚verde, `2`=🧡naranja, `3`=❤️rojo, `4`=💙azul) |
| 🖼️ **RGB** | Muestra el resultado de la detección (recuadro verde ✅, cruz azul en el centro 🔵, información de estado) |
| 📊 **Result** | Imagen binaria tras la segmentación por umbral HSV |

El robot entrará automáticamente en el ciclo de estados:

```
🔍 SEARCHING ──► 🎯 ALIGNING ──► 🏃 APPROACHING ──► ✅ REACHED
```



## 📷 Visualización

### Efecto de detección del objetivo
![Captura de detección](docs/detection_screenshot.png)

### Trayectoria de seguimiento del robot
![Gráfico de trayectoria](docs/trajectory_plot.png)




## 📚 Referencias

- 📖 [ROS Wiki](http://wiki.ros.org/)
- 📖 [Documentación de OpenCV](https://docs.opencv.org/)
- 📖 [wpr_simulation](https://github.com/6-robot/wpr_simulation.git)