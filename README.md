<p align="center">
  <h1 align="center">A Generalized ROS 2 Package for Robot Movement</h1>
</p>

A generalized ROS2 package to move `Nvidia Jetbot` or any `Adafruit MotorHat` driven robot. The package subscribes to `/cmd_vel` topic, to acquire steering and speed control instructions in `geometry_msgs/Twist.msg` message format. `linear.x` controls forward-backward movement and steering is controlled by `angular.z`.

## Colaborators
[Animesh Bala Ani](https://www.linkedin.com/in/ani717/)

## Table of Contents 
* [Download Package](#download) <br/>
* [Install Dependency](#install) <br/>
* [Build, Source & Run Package](#run) <br/>
* [Launch Package](#launch) <br/>
* [Calibration](#calibration) <br/>
* [Demo](#demo) <br/>

## Download Package <a name="download"></a>
Download package in a ROS2 workspace.
```
git clone https://github.com/ANI717/ros2-twist-message-to-robot-motion
```
Or update `.rosinstall` file with following command and run `rosws update` to clone this repository
```
- git: {local-name: src/deps/ros2-twist-message-to-robot-motion, uri: 'https://github.com/ANI717/ros2-twist-message-to-robot-motion.git', version: main}
```

## Install Dependency <a name="install"></a>
Install `traitlets`, `multiexit`, `Adafruit-SSD1306` and `Adafruit_MotorHat`.
```
python3 -m pip install Adafruit-SSD1306 Adafruit_MotorHat traitlets multiexit
```
Install ROS2 dependency.
```
rosdep update && rosdep install --from-paths src --ignore-src -r -y
```

## Build, Source & Run Package <a name="run"></a>
```
colcon build && source install/setup.bash && ros2 run ros2_twist_message_to_robot_motion execute
```
Contains one executable node named `execute`.

## Launch Package <a name="launch"></a>
```
ros2 launch ros2_twist_message_to_robot_motion launch.py
```

## Calibration <a name="calibration"></a>
Modify `XCAL` and `ZCAL` values from `ros2_twist_message_to_robot_motion/adafruit_motion.py` script.

## Key Mapping <a name="key"></a>
`+ linear.x` : `Forward`<br/>
`- linear.x` : `Backward`<br/>
`+ angular.z`: `Right`<br/>
`- angular.z`: `Left`<br/>

## Demo <a name="demo"></a>
Demonstration of robot movement.<br/>
Direction: backward left.<br/>

<img src="https://github.com/ANI717/ani717_gif_repository/blob/main/ros2_twist_message_to_robot_motion/jetbot_motion.gif" alt="JupytarLab Interface" class="inline"/><br/>
