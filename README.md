<p align="center">
  <h1 align="center">ROS 2 Package for Robot Movement</h1>
</p>

ROS 2 Package to Move Jetbot from Geometry Twist Message.

## Colaborators
[Animesh Bala Ani](https://www.linkedin.com/in/ani717/)

## Download Package
Download package in a ROS workspace.
```
git clone https://github.com/ANI717/ros2-twist-to-jetbot-motion
```

## Install Dependency
Install `Adafruit-SSD1306` and `Adafruit_MotorHat`.
```
python3 -m pip install Adafruit-SSD1306
python3 -m pip install Adafruit_MotorHat
```
Install ROS2 dependency.
```
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

## Build, Source & Run Package
ros2 run ros2_deeplearn_twist execute
```
colcon build && . install/setup.bash && ros2 run ros2_deeplearn_twist execute
```

## Launch Package
ros2 launch ros2_deeplearn_twist deeplearn_twist.launch.py
```
colcon build && . install/setup.bash && ros2 launch ros2_deeplearn_twist deeplearn_twist.launch.py
```

## ROS 2 Package to capture image with camera
This ROS 2 built-in package can be used to capture image with a camera. It publishes captured images to ROS 2 /image topic. The ros2_deeplearn_twist package subscribes to this topic and creates twist message.
```
ros2 run image_tools cam2image
```
