#!/usr/bin/env python  
import time
import sys
import os
from math import pi, cos, sin

import rospy
from drone_legs_ros.srv import LegCommand, LegVerticalThrust


"""
string model_name
string urdf_param_name
string[] joint_names
float64[] joint_positions
------------
bool success
string status_message
"""

def setThrust(thrust):
    if (thrust >= 0):  
        service_name = "/drone_legs/thrust_command"
        rospy.wait_for_service(service_name)
        
        
    
        try:
            thrust_command = rospy.ServiceProxy(service_name, LegVerticalThrust)
            resp = thrust_command(thrust)
            
            if (not resp.success):
                print ("thrust_command status: ", resp.status_message)
                
            return resp.success
        except rospy.ServiceException, e:
            print "thrust_command service call failed: %s"%e

def setAngles(a11, a12, a21, a22):
    service_name = "/drone_legs/leg_command"
    rospy.wait_for_service(service_name)
    
    angles = dict()
    angles["front_right_base_hip"] = 0
    angles["front_right_thigh_lift"] = a11
    angles["front_right_knee_lift"] = a12
    angles["front_left_base_hip"] = 0
    angles["front_left_thigh_lift"] = a21
    angles["front_left_knee_lift"] = a22
    angles["back_right_base_hip"] = 0
    angles["back_right_thigh_lift"] = a21
    angles["back_right_knee_lift"] = a22
    angles["back_left_base_hip"] = 0
    angles["back_left_thigh_lift"] = a21
    angles["back_left_knee_lift"] = a22
    
    joint_names = list(angles.keys())
    joint_positions = list(angles.values())

    try:
        leg_command = rospy.ServiceProxy(service_name, LegCommand)
        resp = leg_command(joint_names, joint_positions)
        
        if (not resp.success):
            print ("leg_command status: ", resp.status_message)
            
        return resp.success
    except rospy.ServiceException, e:
        print "leg_command service call failed: %s"%e


def mainLoop(t_elapsed):
    t_thresh = 1.4
    move_rate = 0.7
    
    
    dt = t_elapsed-t_thresh
    if (dt > 0):
        a11 = pi/4- dt*move_rate
        a11 = max(pi*0.05, a11)
        a12 = pi/2 - a11
        
        a21 = pi/4 + dt*move_rate
        a21 = min(pi*0.45, a21)
        a22 = pi/2 - a21
        
        
        setAngles(a11, a12, a21, a22)
    
        
        thrust = max(0.5, 1-dt)
        setThrust(thrust)

if __name__ == '__main__':
    rospy.init_node('servo_to_tf')

    update_rate = 0.01

    ## Set initial positions
    setAngles(pi/4, pi/4, pi/4, pi/4)
    rospy.sleep(2.0)

    setThrust(0.98)

    t = 0.0
    while not rospy.is_shutdown():
        mainLoop(t)
        t += update_rate
        rospy.sleep(update_rate)
