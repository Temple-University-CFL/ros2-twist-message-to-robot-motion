#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""ROS2 Deeplearn Twist Launch Script.

This script launches "cam2image" node from "image_tools" package and "execute" 
node from "ros2_deeplearn_twist" package.

Revision History:
        2021-04-01 (Animesh): Baseline Software.

Example:
        $ ros2 launch ros2_deeplearn_twist deeplearn_twist.launch.py 

"""


#___Import Modules:
import launch
import launch_ros.actions


#___Functions
def generate_launch_description():
    """Launch Description Generating Function
    
    This function launches peoper nodes from proper packages.
    
    """

    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true. Default: true'),

        # nodes to launch
            
        launch_ros.actions.Node(
            package='ros2_deeplearn_twist',
            node_executable='execute',
            name='deeplearn_twist'
        ),
    
    ])    
    return ld


#___Driver Program: 
if __name__ == '__main__':  
    generate_launch_description()


#                                                                              
# end of file
"""ANI717"""