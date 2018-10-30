#!/usr/bin/env python  
from math import pi, cos, sin, acos, atan2, sqrt


class SingleLegControl():
    length_thigh = 0.125
    length_foreleg = 0.093
        
    def __init__(self, name):
        self.name = name
        self.sensor = 0
        self.angle_base_hip = 0
        self.angle_thigh_lift = 0
        self.angle_knee_lift = 0
        self.angle_speed = pi/4
        
        self.angle_base_hip_target = 0
        self.angle_thigh_lift_target = 0
        self.angle_knee_lift_target = 0
        
        #self.length_thigh = 0.093
        #self.length_foreleg = 0.129
        
        self.length_thigh = 0.125
        self.length_foreleg = 0.093
        
        self.length_ratio = self.length_thigh/self.length_foreleg
        
        self.made_contact = False
    
        self.state = 0
    
    
    def getKneeAngleConstantRadius(self):
        """
        Constant radius = the toe's position from the base if we keep the legs
            perpendicular.
            
        Keeping a constant radius prevents skipping/slipping in the initial descent.
        
        Full equation:
        
                    k-L1*cos(theta)
        phi = acos(----------------) - theta
                         L2
                         
        If k > L2, there is a chance for the radius to be unreachable
        """
        theta = self.angle_thigh_lift_target
        return acos(self.length_ratio*(1-cos(theta))) - theta
    
    
    def getEffectorLocalPosition(self):
        """
        Perform forward kinematics and get the position of the end effector
        """
        
        x = self.length_thigh*cos(self.angle_thigh_lift) + self.length_foreleg*cos(self.angle_thigh_lift + self.angle_knee_lift)
        y = self.length_thigh*sin(self.angle_thigh_lift) + self.length_foreleg*sin(self.angle_thigh_lift + self.angle_knee_lift)
        
        return x, y
        
    
    def inverseKinematicsLocal(self, x_target, y_target):
        """
        The values of x_target and y_target are relative to the thigh joint.
        These are oriented the same as the base plate, so an additional transformation is needed
          to convert to global coordinated.
        """
        w = (x_target**2 + y_target**2 - self.length_thigh**2 - self.length_foreleg**2)/(2 * self.length_thigh* self.length_foreleg)
        
        ## Check if the position is reachable
        if w > 1 or w < -1:
            return False
        
        a2 = atan2(sqrt(1-w**2), w)
        a1 = atan2(y_target,x_target) - atan2(self.length_foreleg*sin(a2), self.length_thigh+self.length_foreleg*cos(a2))
        
        self.angle_thigh_lift_target = a1
        self.angle_knee_lift_target = a2
        
        return True
        