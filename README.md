<p align="center">
  <h1 align="center">Generalized ROS 2 Package for Robot Movement</h1>
</p>

A Generalized ROS 2 Package to Move `Nvidia Jetbot` or any `Adafruit MotorHat` driven Robot. The Package subscribes to `/cmd_vel` topic, to acquire steering and speed control instreuctions as geometry Twist message format.

## Colaborators
[Animesh Bala Ani](https://www.linkedin.com/in/ani717/)

## Table of Contents 
* [Download Package](#download) <br/>
* [Install Dependency](#install) <br/>
* [Build, Source & Run Package](#run) <br/>
* [Launch Package](#launch) <br/>
* [Calibration](#calibration) <br/>

## Download Package <a name="download"></a>
Download package in a ROS2 workspace.
```
git clone https://github.com/ANI717/ros2-twist-to-jetbot-motion
```
Or update `.rosinstall` file with following command and run `rosws update` to clone this repository
```
- git: {local-name: src/deps/ros2-twist-to-jetbot-motion, uri: 'https://github.com/ANI717/ros2-twist-to-jetbot-motion.git', version: main}
```

## Install Dependency <a name="install"></a>
Install `traitlets`, `multiexit`, `Adafruit-SSD1306` and `Adafruit_MotorHat`.
```
python3 -m pip install Adafruit-SSD1306, Adafruit_MotorHat, traitlets, multiexit
```
Install ROS2 dependency.
```
rosdep update && rosdep install --from-paths src --ignore-src -r -y
```

## Build, Source & Run Package <a name="run"></a>
```
colcon build
source install/setup.bash
ros2 run ros2_twist_to_jetbot_motion execute
```
Contains one executable node named `execute`.

## Launch Package <a name="launch"></a>
```
ros2 launch ros2_twist_to_jetbot_motion twist_to_motion_launch.py
```

## Calibration <a name="calibration"></a>
Modify `XCAL` and `ZCAL` values from `ros2_twist_to_jetbot_motion/twist_to_motion_function.py` script.
