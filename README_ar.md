**Read this in other languages: [English](README_en.md), [中文](README.md), [Français](README_fr.md), [Español](README_es.md), [العربية](README_ar.md), [Русский](README_ru.md), [日本语](README_jp.md)**
# 🎯 نظام تتبع والتحكم بالأجسام الملونة لـ TurtleBot3 القائم على ROS 🤖



## 📋 وصف المشروع

يقوم هذا المشروع بتنفيذ نظام كامل لروبوت متحرك ذاتي التحكم باستخدام ROS، قادر على اكتشاف وتتبع وملاحقة الأجسام الملونة في الوقت الفعلي باستخدام كاميرا RGB-D 🎨

يستخدم النظام فضاء الألوان HSV لتجزئة الهدف، ويجمع بين معلومات العمق لتقدير موقعه ثلاثي الأبعاد، ويحقق سلوك تتبع قوي عبر آلة حالات محدودة 🎯



## ✨ الميزات

| الميزة | الوصف |
|:---|:---|
| 🎨 **اكتشاف الأجسام الملونة** | تجزئة قوية للهدف باستخدام فضاء الألوان HSV |
| 📍 **تحديد الموضع ثلاثي الأبعاد** | استخدام معلومات العمق من كاميرا RGB-D لحساب موقع الهدف في إطار الكاميرا وإطار الروبوت |
| 🔄 **التحكم بآلة الحالات** | استراتيجية التتبع بأربع حالات: `SEARCHING` → `ALIGNING` → `APPROACHING` → `REACHED` |
| 🏃 **تتبع الأهداف الديناميكية** | ملاحقة الأجسام المتحركة في الوقت الفعلي |
| 🌈 **دعم ألوان متعددة** | أخضر 💚 / برتقالي 🧡 / أحمر ❤️ / أزرق 💙 يمكن اختيارها عبر شريط تمرير OpenCV |
| 📡 **الاحتياط بواسطة ليدار** (نسخة تجريبية) | قياس بديل للمسافة باستخدام الليدار عندما تكون معلومات العمق غير صالحة |


## 📁 وصف الملفات

| الملف | الوصف | الإصدار |
|:---|:---|:---:|
| `hsv_node_release.py` | ⭐ **برنامج التتبع الرسمي** | ✅ |
| `hsv_node_beta.py` | 🧪 نسخة تجريبية مع خاصية الاحتياط بواسطة الليدار | 🔬 |
| `turtlebot3_balls.launch` | 🌍 ملف تشغيل محاكاة Gazebo (مع توليد كرات ملونة) | - |
| `move_ball.py` | ⚽ برنامج تحريك الهدف (الكرة) بشكل عشوائي | - |
| `plot_trajectory.py` | 📈 برنامج تسجيل ورسم مسار الروبوت | - |
| `tracking_metrics.py` | 📊 برنامج تقييم مقاييس الأداء | - |



## 🛠️ متطلبات النظام

- 🐢 ROS Noetic
- 🤖 TurtleBot3
- 📷 OpenCV
- 🐍 Python 3



## ⚙️ التثبيت والبناء

```bash
# الانتقال إلى مساحة العمل
cd ~/catkin_ws/src

# استنساخ المستودع
git clone https://github.com/Tywh202/RGB-D-Based-Target-Tracking-and-Control-for-Mobile-Robots.git

# البناء
cd ~/catkin_ws
catkin_make

# إعداد البيئة
source devel/setup.bash
```



## 🚀 البدء السريع

### 1️⃣ تشغيل بيئة المحاكاة

```bash
export TURTLEBOT3_MODEL=waffle
roslaunch image_pkg turtlebot3_balls.launch
```

### 2️⃣ بدء تتبع الهدف

**الإصدار الرسمي** ⭐ (يعتمد التقرير على هذا الإصدار):
```bash
rosrun image_pkg hsv_node_release.py
```

**الإصدار التجريبي** 🧪 (مع الاحتياط بواسطة الليدار):
```bash
rosrun image_pkg hsv_node_beta.py
```

### 3️⃣ تحريك الكرة المستهدفة (اختياري) ⚽

```bash
rosrun image_pkg move_ball.py _model_name:=green_ball
```

### 4️⃣ تسجيل مسار الروبوت 📈

```bash
rosrun image_pkg plot_trajectory.py \
    _output_path:=~/trajectory.png \
    _target_model_name:=green_ball
```

### 5️⃣ تقييم الأداء 📊

```bash
rosrun image_pkg tracking_metrics.py \
    _target_color_name:=green \
    _auto_stop_on_reached:=true
```



## 🎮 تعليمات التشغيل

بعد تشغيل `hsv_node_release.py`، ستظهر ثلاث نوافذ لـ OpenCV:

| النافذة | الوظيفة |
|:---|:---|
| 🎚️ **Threshold** | اختيار اللون المستهدف عبر شريط التمرير (`0`=يدوي، `1`=💚أخضر، `2`=🧡برتقالي، `3`=❤️أحمر، `4`=💙أزرق) |
| 🖼️ **RGB** | عرض نتيجة الاكتشاف (إطار أخضر ✅، علامة صليب زرقاء في المركز 🔵، معلومات الحالة) |
| 📊 **Result** | صورة ثنائية بعد تجزئة عتبة HSV |

سيدخل الروبوت تلقائيًا في دورة الحالات:

```
🔍 SEARCHING ──► 🎯 ALIGNING ──► 🏃 APPROACHING ──► ✅ REACHED
```



## 📷 التصور البصري

### تأثير اكتشاف الهدف
![لقطة شاشة الاكتشاف](docs/detection_screenshot.png)

### مسار تتبع الروبوت
![رسم المسار](docs/trajectory_plot.png)




## 📚 المراجع

- 📖 [ROS Wiki](http://wiki.ros.org/)
- 📖 [وثائق OpenCV](https://docs.opencv.org/)
- 📖 [wpr_simulation](https://github.com/6-robot/wpr_simulation.git)
