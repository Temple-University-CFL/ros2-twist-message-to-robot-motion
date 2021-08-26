<p align="center">
  <h1 align="center">ROS 2 Deep Learning Package for Robot Movement</h1>
</p>

ROS 2 Package to Publish Twist Message from Subscribed Sensor Image Message using End to End Learning (Deep Learning) Method for Robot Movement.

## Colaborators
[Computer Fusion Laboratory (CFL) - Temple University College of Engineering](https://sites.temple.edu/cflab/people/)
* [Animesh Bala Ani](https://animeshani.com/)
* [Dr. Li Bai](https://engineering.temple.edu/about/faculty-staff/li-bai-lbai)

## Download Package
Download package in a ROS workspace.
```
git clone https://github.com/ANI717/ros2_deeplearn_twist
```

## Install Dependency
Install Torch, Torchvision and Opencv.
```
python3 -m pip install torch
python3 -m pip install torchvision
python3 -m pip install opencv-python
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
