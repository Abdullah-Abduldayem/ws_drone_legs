#!/usr/bin/env python  
import time
import sys
import os
from math import pi, cos, sin

import rospy
import tf_conversions

import tf2_ros
import geometry_msgs.msg
from sensor_msgs.msg import JointState


def mainLoop(t_elapsed):
    t = JointState()
    
    a1 = sin(t_elapsed)
    a2 = pi/2 - a1

    angles = dict()
    angles["front_right_base_hip"] = 0
    angles["front_right_thigh_lift"] = a1
    angles["front_right_knee_lift"] = a2
    angles["front_left_base_hip"] = 0
    angles["front_left_thigh_lift"] = a1
    angles["front_left_knee_lift"] = a2
    angles["back_right_base_hip"] = 0
    angles["back_right_thigh_lift"] = a1
    angles["back_right_knee_lift"] = a2
    angles["back_left_base_hip"] = 0
    angles["back_left_thigh_lift"] = a1
    angles["back_left_knee_lift"] = a2



    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"

    t.name = list(angles.keys())
    t.position = list(angles.values())

    #t.name = ["front_right_base_hip", "front_right_thigh_lift", "front_right_knee_lift", "front_left_base_hip", "front_left_thigh_lift", "front_left_knee_lift", "back_right_base_hip", "back_right_thigh_lift", "back_right_knee_lift", "back_left_base_hip", "back_left_thigh_lift", "back_left_knee_lift"]
    #t.position = [0.2205398042820037, -0.7998494896039614, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    pub.publish(t)
    print(t)


if __name__ == '__main__':
    rospy.init_node('servo_to_tf')
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)

    t = 0.0
    while not rospy.is_shutdown():
        mainLoop(t)
        t += 0.1
        rospy.sleep(0.1)
