#!/usr/bin/env python  
from math import pi, cos, sin, acos

import rospy
from drone_legs_ros.single_leg import SingleLegControl
from drone_legs_ros.quadruped_controller import QuadrupedController

class LandingProcess(QuadrupedController):
    LEG_STATE_IDLE = 0
    LEG_STATE_EXPANSION_INITIAL = 1
    LEG_STATE_EXPANSION_REACH = 2
    LEG_STATE_LANDED = 3
    
    LEG_CONTACT_THRESHOLD = 0.01
    
    def __init__(self):
        self.initController()
        
        
        ## Set variables
        self.angular_speed = 0.7*2
        self.linear_speed = 0.01
        
        
        ## Set initial positions
        #self.setAngles(0, pi/2)
        x = pi/8
        self.setAngles(x, pi/2-x)
            
        ## Small delay before descending
        rospy.sleep(1.0)
        
        ##self.start()
        t = 0.0
        while not rospy.is_shutdown():            
            ## The function that computes new commands. Defined in derivative classes
            self.update(t)
            
            ## Send new commands
            self.updateAngles()
            #self.updateThrust()
            
            t += self.update_interval
            rospy.sleep(self.update_interval)



    
    def update(self, t_elapsed):
        ## Set new thrust
        #self.thrust = max(0.5, 1-t_elapsed)
        self.thrust = 0.95
        
        
        ## Initiate leg expansion on touchdown
        if (not self.initiated_touchdown):
            contact_made = False
            for leg in self.legs.values():
                if (leg.made_contact):
                    contact_made = True
            
            if (contact_made):
                self.initiated_touchdown = True
                
                for leg in self.legs.values():
                    if (not leg.made_contact and leg.state == LandingProcess.LEG_STATE_IDLE):
                        leg.state = LandingProcess.LEG_STATE_EXPANSION_INITIAL
        
        dt = self.update_interval
        for leg in self.legs.values():
            self.updateSingleLeg(leg, dt)



    def updateSingleLeg(self, leg, dt):
        msg = ""
        if (leg.sensor > LandingProcess.LEG_CONTACT_THRESHOLD):
            leg.made_contact = True
            leg.state = LandingProcess.LEG_STATE_LANDED
        
        if (leg.state == LandingProcess.LEG_STATE_LANDED):
            if (leg.made_contact):
                if (leg.sensor < LandingProcess.LEG_CONTACT_THRESHOLD):
                    ## Lost contact
                    ## TODO: re-execute reaching
                    leg.made_contact = False
                    #return
                    leg.state = LandingProcess.LEG_STATE_EXPANSION_INITIAL
        
        if (leg.state == LandingProcess.LEG_STATE_EXPANSION_INITIAL):
            ## Reach down while maintaining constant radial distance from the plane of drone
            
            leg.angle_thigh_lift_target += leg.angle_speed*dt
            leg.angle_knee_lift_target -= leg.angle_speed*dt
            
            if (leg.angle_thigh_lift_target > pi/2):
                leg.angle_thigh_lift_target = pi/2
                leg.state == LandingProcess.LEG_STATE_IDLE
                
            if (leg.angle_knee_lift_target < 0):
                leg.angle_knee_lift_target = 0
                leg.state == LandingProcess.LEG_STATE_IDLE
            
            """
            x, y = leg.getEffectorLocalPosition()
            y += self.linear_speed * dt
            
            success = leg.inverseKinematicsLocal(x, y)
            msg = "IK success"
            if (not success):
                msg = "IK FAILED"
                if x <= 0.1:
                    leg.state == LandingProcess.LEG_STATE_IDLE
                
                else:
                    leg.state == LandingProcess.LEG_STATE_EXPANSION_REACH
            """
        
        if (leg.state == LandingProcess.LEG_STATE_EXPANSION_REACH):
            ## Keep the knee locked at 190 degrees and swing the thigh down to make contact
            leg.angle_thigh_lift_target += leg.angle_speed*dt
            leg.angle_knee_lift_target = 0
            
            if leg.angle_thigh_lift_target > pi/2:
                leg.angle_thigh_lift_target = pi/2
                leg.angle_knee_lift_target = 0
                leg.state == LandingProcess.LEG_STATE_IDLE
    
        if (leg.state == LandingProcess.LEG_STATE_LANDED):
            pass
            ## Attempt balancing.
            ## 1) Check if the opposing leg landed
            ## 2) Compute the angle of the platform between these two
            ## 3) Perform rebalancing
    
        #print(leg.name, leg.sensor, leg.state, leg.angle_thigh_lift, leg.angle_thigh_lift_target)
        x, y = leg.getEffectorLocalPosition()
        
        if (leg.name == "back_left"):
            print("Name\t\tSens\tState\tx\ty\tMessage")
            
        print("{}\t{:02.3f}\t{}\t{:02.3f}\t{:02.3f}\t{}".format(leg.name, leg.sensor, leg.state, x, y, msg))
    
    
if __name__ == '__main__':
    rospy.init_node('reactive_landing')

    LandingProcess()
