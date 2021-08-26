#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""ROS2 Twist to Jetbot Move.

This script subscribes to "/cmd_vel" topic, reads Twist message, and moves the
Jetbot.

Revision History:
        2021-08-18 (Animesh): Baseline Software.

Example:
        $ ros2 run ros2_twist_to_jetbot_motion execute

"""


#___Import Modules:

import atexit
from Adafruit_MotorHAT import Adafruit_MotorHAT
import traitlets
from traitlets.config.configurable import Configurable, SingletonConfigurable

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


#___Global Variables:
SUBSCRIBE_TOPIC = '/cmd_vel'
XCAL = 0.25 #Calibration X
ZCAL = 0.25 #Calibration Z


#__Classes
class Twist_to_Move(Node):
    """TWIST to Jetbot Move Class.
    
    This class contains all methods to read TWIST message and move the Jetbot. 
    
    """

    def __init__(self):
        
        super().__init__('twist_to_move')
        
        # initialize robot
        self.robot = Robot()
        
        # initialize subscriber
        self.subscription = self.create_subscription(
            Twist,
            SUBSCRIBE_TOPIC,
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning


    def listener_callback(self, msg):
        
        # parses data from subscribed topic message
        x = XCAL*float(msg.linear.x)
        z = ZCAL*float(msg.angular.z)
        
        # control robot movement
        # both wheel same state
        if z == 0:
            # total stop
            if x == 0:
                self.robot.stop()
            else:
                self.robot.set_motors(x, x)
        
        # one wheel moving
        elif x == 0:
            # rotate right
            if z > 0:
                self.robot.set_motors(z, 0)
            # rotate left
            elif z < 0:
                self.robot.set_motors(0, -z)
        
        # moving forward
        elif x > 0:
            # rotate right
            if z > 0:
                self.robot.set_motors((x+z)/2, x/2)
            # rotate left
            elif z < 0:
                self.robot.set_motors(x/2, (x-z)/2)
        
        # moving backward
        elif x < 0:
            # rotate right
            if z > 0:
                self.robot.set_motors((x-z)/2, x/2)
            # rotate left
            elif z < 0:
                self.robot.set_motors(x/2, (x+z)/2)



# Jetbot Motor Driver Classes From NVIDIA-AI-IOT
# Ref: https://github.com/NVIDIA-AI-IOT/jetbot/blob/master/jetbot/motor.py
class Motor(Configurable):

    value = traitlets.Float()
    
    # config
    alpha = traitlets.Float(default_value=1.0).tag(config=True)
    beta = traitlets.Float(default_value=0.0).tag(config=True)

    def __init__(self, driver, channel, *args, **kwargs):
        super(Motor, self).__init__(*args, **kwargs)  # initializes traitlets

        self._driver = driver
        self._motor = self._driver.getMotor(channel)
        if(channel == 1):
            self._ina = 1
            self._inb = 0
        else:
            self._ina = 2
            self._inb = 3
        atexit.register(self._release)
        
    @traitlets.observe('value')
    def _observe_value(self, change):
        self._write_value(change['new'])

    def _write_value(self, value):
        """Sets motor value between [-1, 1]"""
        mapped_value = int(255.0 * (self.alpha * value + self.beta))
        speed = min(max(abs(mapped_value), 0), 255)
        self._motor.setSpeed(speed)
        if mapped_value < 0:
            self._motor.run(Adafruit_MotorHAT.FORWARD)
            # The two lines below are required for the Waveshare JetBot Board only
            self._driver._pwm.setPWM(self._ina,0,0)
            self._driver._pwm.setPWM(self._inb,0,speed*16)
        else:
            self._motor.run(Adafruit_MotorHAT.BACKWARD)
            # The two lines below are required for the Waveshare JetBot Board only
            self._driver._pwm.setPWM(self._ina,0,speed*16)
            self._driver._pwm.setPWM(self._inb,0,0)

    def _release(self):
        """Stops motor by releasing control"""
        self._motor.run(Adafruit_MotorHAT.RELEASE)
        # The two lines below are required for the Waveshare JetBot Board only
        self._driver._pwm.setPWM(self._ina,0,0)
        self._driver._pwm.setPWM(self._inb,0,0)


# Jetbot Robot Driver Classes From NVIDIA-AI-IOT
# Ref: https://github.com/NVIDIA-AI-IOT/jetbot/blob/master/jetbot/robot.py
class Robot(SingletonConfigurable):
    
    left_motor = traitlets.Instance(Motor)
    right_motor = traitlets.Instance(Motor)

    # config
    i2c_bus = traitlets.Integer(default_value=1).tag(config=True)
    left_motor_channel = traitlets.Integer(default_value=1).tag(config=True)
    left_motor_alpha = traitlets.Float(default_value=1.0).tag(config=True)
    right_motor_channel = traitlets.Integer(default_value=2).tag(config=True)
    right_motor_alpha = traitlets.Float(default_value=1.0).tag(config=True)
    
    def __init__(self, *args, **kwargs):
        super(Robot, self).__init__(*args, **kwargs)
        self.motor_driver = Adafruit_MotorHAT(i2c_bus=self.i2c_bus)
        self.left_motor = Motor(self.motor_driver, channel=self.left_motor_channel, alpha=self.left_motor_alpha)
        self.right_motor = Motor(self.motor_driver, channel=self.right_motor_channel, alpha=self.right_motor_alpha)
        
    def set_motors(self, left_speed, right_speed):
        self.left_motor.value = left_speed
        self.right_motor.value = right_speed
        
    def forward(self, speed=1.0, duration=None):
        self.left_motor.value = speed
        self.right_motor.value = speed

    def backward(self, speed=1.0):
        self.left_motor.value = -speed
        self.right_motor.value = -speed

    def left(self, speed=1.0):
        self.left_motor.value = -speed
        self.right_motor.value = speed

    def right(self, speed=1.0):
        self.left_motor.value = speed
        self.right_motor.value = -speed

    def stop(self):
        self.left_motor.value = 0
        self.right_motor.value = 0


#___Main Method:
def main(args=None):
    rclpy.init(args=args)

    twist_to_move = Twist_to_Move()

    rclpy.spin(twist_to_move)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    twist_to_move.destroy_node()
    rclpy.shutdown()


#___Driver Program:
if __name__ == '__main__':
    main()


#                                                                              
# end of file
"""ANI717"""
