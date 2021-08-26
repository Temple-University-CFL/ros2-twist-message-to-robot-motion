<p align="center">
  <h1 align="center">Generalized ROS 2 Package for Robot Movement</h1>
</p>

A Generalized ROS 2 Package to Move `Jetbot` or any `Adafruit MotorHat` driven Robot. The Package subscribes to `/cmd_vel` topic, to acquire steering and speed control instreuctions as geometry Twist message format.

## Colaborators
[Animesh Bala Ani](https://www.linkedin.com/in/ani717/)

## Download Package
Download package in a ROS2 workspace.
```
git clone https://github.com/ANI717/ros2-twist-to-jetbot-motion
```

## Install Dependency
Install `Adafruit-SSD1306` and `Adafruit_MotorHat`.
```
python3 -m pip install Adafruit-SSD1306
python3 -m pip install Adafruit_MotorHat
python3 -m pip install traitlets
python3 -m pip install multiexit
```
Install ROS2 dependency.
```
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

## Build, Source & Run Package
```
colcon build
source install/setup.bash
ros2 run ros2_twist_to_jetbot_motion execute
```
Contains one executable node named `execute`.

## Launch Package
```
ros2 launch ros2_twist_to_jetbot_motion twist_to_motion_launch.py
```

## Calibration
Modify `XCAL` and `ZCAL` values from `ros2_twist_to_jetbot_motion/twist_to_motion_function.py` script.
