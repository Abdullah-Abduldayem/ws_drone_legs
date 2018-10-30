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

def mainLoop(t_elapsed):
    service_name = "/drone_legs/leg_command"
    rospy.wait_for_service(service_name)
    
    a1 = max(0, pi/2*sin(t_elapsed))
    a2 = max(0, pi/2*cos(t_elapsed*1.5))
    
    ## Ease in the value of a2, because it will try to contract at the very start 
    ## and launch the legs up if they're already contacting the floor
    if (t < 1.0):
        a2 = a2 * t
        
    
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
    
    joint_names = list(angles.keys())
    joint_positions = list(angles.values())

    try:
        leg_command = rospy.ServiceProxy(service_name, LegCommand)
        resp = leg_command(joint_names, joint_positions)
        
        if (not resp.success):
            print ("leg_command status: ", resp.status_message)
            
        #return resp.success
    except rospy.ServiceException, e:
        print "leg_command service call failed: %s"%e
        
        
        
        
    """
    thrust = max(0.00001, min(1, 1.5-0.1*t));
    
    if (thrust >= 0):  
        service_name = "/drone_legs/thrust_command"
        rospy.wait_for_service(service_name)
        
        
    
        try:
            thrust_command = rospy.ServiceProxy(service_name, LegVerticalThrust)
            resp = thrust_command(thrust)
            
            if (not resp.success):
                print ("thrust_command status: ", resp.status_message)
                
            #return resp.success
        except rospy.ServiceException, e:
            print "thrust_command service call failed: %s"%e
            
        #print(thrust)
    """
    

if __name__ == '__main__':
    rospy.init_node('servo_to_tf')

    update_rate = 0.01

    t = 0.0
    while not rospy.is_shutdown():
        mainLoop(t)
        t += update_rate
        rospy.sleep(update_rate)
