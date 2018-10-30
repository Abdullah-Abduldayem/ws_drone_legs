#!/usr/bin/env python  
from math import pi, cos, sin, acos

import rospy
from drone_legs_ros.single_leg import SingleLegControl
from drone_legs_ros.quadruped_controller import QuadrupedController

class LandingProcess(QuadrupedController):
    def __init__(self):
        self.initController()
        
        ## Set variables
        self.angular_speed = 0.7*2
        self.linear_speed = 0.01
        
        ## Set initial positions
        self.setAngles(0, 0)
        
        self.start()
    
    def update(self, t_elapsed):
        dt = self.update_interval
        
        leg = self.legs["back_right"]
        
        
        x_target = leg.length_thigh*0.5
        y_target = leg.length_foreleg*sin(t_elapsed)
        
        #x_target = 0
        #mult = max(0.5, sin(t_elapsed))
        #y_target = (leg.length_thigh+leg.length_foreleg)*mult
        
        leg.inverseKinematicsLocal(x_target, y_target)
        x, y = leg.getEffectorLocalPosition()
        
        print("x\ty\tx_tar\ty_tar")
        print("{:02.3f}\t{:02.3f}\t{:02.3f}\t{:02.3f}".format(x,y,x_target,y_target))
    
if __name__ == '__main__':
    rospy.init_node('reactive_landing')

    LandingProcess()
