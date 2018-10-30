#!/usr/bin/env python  
import time
import sys
import os

import rospy
import tf_conversions

import tf2_ros
import geometry_msgs.msg
from sensor_msgs.msg import JointState


def mainLoop():
    t = JointState()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.name = ["front_right_base_hip", "front_right_thigh_lift", "front_right_knee_lift", "front_left_base_hip", "front_left_thigh_lift", "front_left_knee_lift", "back_right_base_hip", "back_right_thigh_lift", "back_right_knee_lift", "back_left_base_hip", "back_left_thigh_lift", "back_left_knee_lift"]

    t.position = [0.2205398042820037, -0.7998494896039614, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    pub.publish(t)
    print(t)


def die():
    print("Interrupt detected")
    rospy.signal_shutdown('Quit')
    sys.exit(0)
    os._exit(0)


if __name__ == '__main__':
    rospy.init_node('servo_to_tf')
    rospy.on_shutdown(die)
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)

    while True:
        mainLoop()
        time.sleep(1)
