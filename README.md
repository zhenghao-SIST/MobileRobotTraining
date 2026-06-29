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
Push save map button on Rviz after setting a path for saving. For example: ~/map. This will save map.pgm and map.yaml to your home directory. After saving, move this two files to vehicle_nav2/map directory.  
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
run install script
``` bash
./install_orangepi_ros2.sh
```
