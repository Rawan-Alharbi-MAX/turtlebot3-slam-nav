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

## تشغيل SLAM
![SLAM Demo](<img width="535" height="510" alt="save map" src="https://github.com/user-attachments/assets/afd94da9-bcd2-49fe-9b06-b0df2ccb3bb8" /> )
( <img width="564" height="523" alt="burger model" src="https://github.com/user-attachments/assets/4c684e5d-8b8f-4993-9d45-f819414d8392" /> )
(<img width="562" height="509" alt="waffle model" src="https://github.com/user-attachments/assets/73a68112-15d8-4a2e-98c6-ac5e3ee3b691" /> )
(<img width="562" height="509" alt="waffle_pi model" src="https://github.com/user-attachments/assets/d8bfea1e-9e97-4e03-ad20-48e72cd53aa8" /> ) 

## تشغيل Navigation
![Navigation Demo](<img width="538" height="510" alt="map" src="https://github.com/user-attachments/assets/893921a8-ab90-4301-a491-358981eed68b" /> )
(<img width="563" height="512" alt="point arrow" src="https://github.com/user-attachments/assets/ac35ab33-651b-4248-bea3-fc1c83b48106" /> )
(<img width="558" height="509" alt="point goal " src="https://github.com/user-attachments/assets/db5cdf12-70b3-466b-8478-55d13d6df1b0" /> )
(<img width="563" height="507" alt="collecting dots" src="https://github.com/user-attachments/assets/d81465b5-8591-4a96-ae6f-929058a10d57" /> )

