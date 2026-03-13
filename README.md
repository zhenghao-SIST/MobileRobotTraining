# README

This repository is for Mobile robot training

## SLAM cmd
```bash
#Laser and chassis
ros2 launch vehicle_nav2 laser_and_chassis.launch.py
#Joystick
ros2 launch vehicle_nav2 joy.launch.py
#SLAM
ros2 launch vehicle_nav2 slam.launch.py
```

## Navigation cmd
```bash
#Laser and chassis
ros2 launch vehicle_nav2 laser_and_chassis.launch.py
#Joystick
ros2 launch vehicle_nav2 joy.launch.py
#Nav2: behavior_tree cost_map amcl ...
ros2 launch vehicle_nav2 nav.launch.py
#Rviz if you need
rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
```
## Material
1. udev file  包括485 + laser + 手柄
2. 完整的git仓库

## package in Git repository
### vehicle_nav2
### chassis_controller
### joy_to_twist
### vehicle_urdf
### sllidar_ros2

## Prerequsite
``` bash
sudo apt install pythoh3-colcon-common-extensionssudo
sudo apt install ros-humble-xacro
sudo apt install ros-humble-joint-state-publisher-gui
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

# Install pymodbus
sudo apt install python3-pip
sudo apt remove brltty
pip install pymodbus
pip install pyserial
```

