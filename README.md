# مشروع TurtleBot3: الدليل الكامل

## المقدمة
هذا المشروع يحتوي على تعليمات شاملة لتشغيل TurtleBot3 باستخدام ROS 2، ويتضمن إعداد **Quick Start** على جهاز التحكّم (Remote PC)، بالإضافة إلى خطوات لمحاكاة الروبوت مع Gazebo، تشغيل **SLAM** لتوليد الخرائط، واختبار **Navigation** باستخدام Map محفوظ.

---

## المتطلبات الأساسية
- نظام تشغيل: Ubuntu (مثل 24.04 LTS)
- ROS 2 (Humble أو Jazzy)
- Gazebo (نسخة متوافقة مع ROS 2)

---

## 1. البدء السريع – Quick Start (Remote PC)
1. افتح الطرفية: `Ctrl + Alt + T`
2. ثبّت ROS 2 (مثل `ros-humble-desktop`)، وأضف السطر التالي إلى `~/.bashrc`:
   ```bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   source ~/.bashrc

ثبّت الحزم الضرورية:

sudo apt install ros-humble-gazebo-*
sudo apt install ros-humble-cartographer ros-humble-cartographer-ros
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup


أنشئ workspace وابنِ الحزمة:

source /opt/ros/humble/setup.bash
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src
git clone -b humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git
cd ~/turtlebot3_ws
colcon build --symlink-install
echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc


ضبط البيئة:

echo 'export ROS_DOMAIN_ID=30 # TURTLEBOT3' >> ~/.bashrc
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
source ~/.bashrc

2. المحاكاة باستخدام Gazebo

استنخ الحزمة الخاصة بالمحاكاة:

cd ~/turtlebot3_ws/src/
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/turtlebot3_ws
colcon build --symlink-install


تشغيل المحاكاة:

export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py


أو:

export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

3. محاكاة SLAM (Cartographer)

تشغيل Gazebo:

export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py


تشغيل SLAM:

export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True


تشغيل teleop:

export TURTLEBOT3_MODEL=burger
ros2 run turtlebot3_teleop teleop_keyboard


التحكم: w/a/s/d, x لتقليل السرعة, space للتوقف.

حفظ الخريطة:

ros2 run nav2_map_server map_saver_cli -f ~/map

4. محاكاة التنقل – Navigation2

تشغيل Navigation2 مع الخريطة:

export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=$HOME/map.yaml

5. ملخص تدفق العمل

إعداد PC (Quick Start)

إعداد الحزم والمحاكاة (Gazebo)

تشغيل SLAM وتوليد خريطة

تشغيل التنقل باستخدام الخريطة المحفوظة

6. ملاحظة إضافية

يمكن حفظ TURTLEBOT3_MODEL في ~/.bashrc لتجنّب إعادة تعريفه.
