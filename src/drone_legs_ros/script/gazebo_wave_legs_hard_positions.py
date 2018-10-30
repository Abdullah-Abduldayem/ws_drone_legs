#!/usr/bin/env python  
import time
import sys
import os
from math import pi, cos, sin

import rospy
from gazebo_msgs.srv import SetModelConfiguration


"""
string model_name
string urdf_param_name
string[] joint_names
float64[] joint_positions
------------
bool success
string status_message
"""

def mainLoop(t_elapsed):
    rospy.wait_for_service('/gazebo/set_model_configuration')
    
    model_name = "drone_legs"
    model_param_name = "robot_description"
    
    angles = dict()
    angles["front_right_base_hip"] = 0
    angles["front_right_thigh_lift"] = sin(t_elapsed)
    angles["front_right_knee_lift"] = cos(t_elapsed)
    angles["front_left_base_hip"] = 0
    angles["front_left_thigh_lift"] = 0
    angles["front_left_knee_lift"] = 0
    angles["back_right_base_hip"] = 0
    angles["back_right_thigh_lift"] = 0
    angles["back_right_knee_lift"] = 0
    angles["back_left_base_hip"] = 0
    angles["back_left_thigh_lift"] = 0
    angles["back_left_knee_lift"] = 0
    
    joint_names = list(angles.keys())
    joint_positions = list(angles.values())

    try:
        set_model_configuration = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
        resp = set_model_configuration(model_name, model_param_name, joint_names, joint_positions)
        print "set model configuration status: ", resp.status_message
        return resp.success
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == '__main__':
    rospy.init_node('servo_to_tf')

    update_rate = 1/10

    t = 0.0
    while not rospy.is_shutdown():
        mainLoop(t)
        t += update_rate
        rospy.sleep(update_rate)
