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


def main():
    t = JointState()
    
    angles = dict()
    angles["front_right_base_hip"] = 0
    angles["front_right_thigh_lift"] = 0
    angles["front_right_knee_lift"] = 0
    angles["front_left_base_hip"] = 0
    angles["front_left_thigh_lift"] = 0
    angles["front_left_knee_lift"] = 0
    angles["back_right_base_hip"] = 0
    angles["back_right_thigh_lift"] = 0
    angles["back_right_knee_lift"] = 0
    angles["back_left_base_hip"] = 0
    angles["back_left_thigh_lift"] = 0
    angles["back_left_knee_lift"] = 0



    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"

    t.name = list(angles.keys())
    t.position = list(angles.values())
    pub.publish(t)


if __name__ == '__main__':
    rospy.init_node('servo_to_tf')
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)

    rospy.sleep(0.5)

    main()
