#!/usr/bin/env python  
from math import pi, cos, sin, acos
import numpy as np

import rospy
from drone_legs_ros.single_leg import SingleLegControl
from drone_legs_ros.quadruped_controller import QuadrupedController
from numpy import isnan

class BalanceController(QuadrupedController):
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
        """
        a11 = (pi/2)*0.8
        a12 = pi/2 - a11
        a21 = 0
        a22 = 0
        """
        #a11 = pi/4
        #a12 = 0
        #a21 = 0
        #a22 = pi/2
        
        """
        a11 = a21 = pi/4
        a12 = a22 = pi/4
        
        self.legs["front_right"].angle_thigh_lift_target = a11
        self.legs["front_right"].angle_knee_lift_target = a12
        self.legs["front_left"].angle_thigh_lift_target = a21
        self.legs["front_left"].angle_knee_lift_target = a22
        self.legs["back_left"].angle_thigh_lift_target = a21
        self.legs["back_left"].angle_knee_lift_target = a22
        self.legs["back_right"].angle_thigh_lift_target = a11
        self.legs["back_right"].angle_knee_lift_target = a12
        """
        
        a11 = 0
        a12 = pi/2
        a21 = pi/4
        a22 = pi/4
        
        self.legs["front_right"].angle_thigh_lift_target = a11
        self.legs["front_right"].angle_knee_lift_target = a12
        self.legs["front_left"].angle_thigh_lift_target = a21
        self.legs["front_left"].angle_knee_lift_target = a22
        self.legs["back_left"].angle_thigh_lift_target = a21
        self.legs["back_left"].angle_knee_lift_target = a22
        self.legs["back_right"].angle_thigh_lift_target = a21
        self.legs["back_right"].angle_knee_lift_target = a22
        
        self.updateAngles()
            
        ## Small delay before descending
        rospy.sleep(0.5)
        
        ## Land
        self.thrust = 0
        self.updateThrust()
        
        ## Small delay before starting
        rospy.sleep(2.0)
        
        
        np.set_printoptions(linewidth=250)
        self.start()
    
    def update(self, t_elapsed):
        dt = self.update_interval
        
        sgn = 1
        if (t_elapsed%2 > 1):
            sgn = -1
        
        wave = sgn*sin(2*pi*t_elapsed)*0.5
        
        x_dot = 0 # wave
        y_dot = x_dot #wave
        z_dot = 0.2*wave
        r_dot = 0 #0.2*sgn*cos(2*pi*t_elapsed)
        p_dot = 0 #r_dot
        d1_dot = 0
        d2_dot = 0
        
        state_dot = np.reshape( np.array([x_dot, y_dot, z_dot, r_dot, p_dot, d1_dot, d2_dot]), (-1, 1) )
        
        
        ## Compute the change in joint angles
        J = self.getJacobian()

        ## Get the inverse Jacobian
        Jinv = np.linalg.pinv(J)
        Jinv[np.abs(Jinv)<1e-10] = 0 ## Cast anything below the specified threshold to zero
        
        Q_dot = np.dot(Jinv, state_dot) ## Perform matrix multiplication
        
        angles = np.reshape( np.array(self.getAngles()) , (-1, 1) )
        angles += Q_dot*dt
        self.setAngles(angles)
        
        
        
        """
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
        """
        
        
    def getJacobian(self):
        angles = self.getAngles()
        
        ## TODO: Get roll and pitch along the desired axes
        
        ## Create the Jacobian matrix
        J = np.zeros((7, 8), dtype=np.float64)
        
        ## Pre-compute some values
        roll = 0
        pitch = 0
        d = 0.155
        z31 = 0
        z32 = 0
        
        cr = cos(roll)
        sr = sin(roll)
        cp = cos(pitch)
        sp = sin(pitch)
        crcp = cr*cp
        crsp = cr*sp
        srcp = sr*cp
        srsp = sr*sp        
        
        ## Partials of x
        #J[0,:] = self.dx(angles, 1) + self.dx(angles, 2) + self.dx(angles, 3) + self.dx(angles, 4)
        J[0,0] = -self.getDist(1, angles, sin, full=1)*srsp + self.getDist(1, angles, cos, full=1)*crsp
        J[0,1] = -self.getDist(1, angles, sin, full=0)*srsp + self.getDist(1, angles, cos, full=0)*crsp
        J[0,2] =  self.getDist(2, angles, sin, full=1)*cp   + self.getDist(2, angles, cos, full=1)*crsp
        J[0,3] =  self.getDist(2, angles, sin, full=0)*cp   + self.getDist(2, angles, cos, full=0)*crsp
        J[0,4] =  self.getDist(3, angles, sin, full=1)*srsp + self.getDist(3, angles, cos, full=1)*crsp
        J[0,5] =  self.getDist(3, angles, sin, full=0)*srsp + self.getDist(3, angles, cos, full=0)*crsp
        J[0,6] = -self.getDist(4, angles, sin, full=1)*cp   + self.getDist(4, angles, cos, full=1)*crsp
        J[0,7] = -self.getDist(4, angles, sin, full=0)*cp   + self.getDist(4, angles, cos, full=0)*crsp
        
        ## Partials of y
        #J[1,:] = self.dy(angles, 1) + self.dy(angles, 2) + self.dy(angles, 3) + self.dy(angles, 4)
        J[1,0] = -self.getDist(1, angles, sin, full=1)*cr -self.getDist(1, angles, cos, full=1)*sr
        J[1,1] = -self.getDist(1, angles, sin, full=0)*cr -self.getDist(1, angles, cos, full=0)*sr
        J[1,2] =                                          -self.getDist(2, angles, cos, full=1)*sr
        J[1,3] =                                          -self.getDist(2, angles, cos, full=0)*sr
        J[1,4] =  self.getDist(3, angles, sin, full=1)*cr -self.getDist(3, angles, cos, full=1)*sr
        J[1,5] =  self.getDist(3, angles, sin, full=0)*cr -self.getDist(3, angles, cos, full=0)*sr
        J[1,6] =                                          -self.getDist(4, angles, cos, full=1)*sr
        J[1,7] =                                          -self.getDist(4, angles, cos, full=0)*sr
        
        ## Partials of z
        #J[2,:] = self.dz(angles, 1) + self.dz(angles, 2) + self.dz(angles, 3) + self.dz(angles, 4)
        J[2,0] = 0.25*(-self.getDist(1, angles, sin, full=1)*srcp + self.getDist(1, angles, cos, full=1)*crcp)
        J[2,1] = 0.25*(-self.getDist(1, angles, sin, full=0)*srcp + self.getDist(1, angles, cos, full=0)*crcp)
        J[2,2] = 0.25*(-self.getDist(2, angles, sin, full=1)*sp   + self.getDist(2, angles, cos, full=1)*crcp)
        J[2,3] = 0.25*(-self.getDist(2, angles, sin, full=0)*sp   + self.getDist(2, angles, cos, full=0)*crcp)
        J[2,4] = 0.25*( self.getDist(3, angles, sin, full=1)*srcp + self.getDist(3, angles, cos, full=1)*crcp)
        J[2,5] = 0.25*( self.getDist(3, angles, sin, full=0)*srcp + self.getDist(3, angles, cos, full=0)*crcp)
        J[2,6] = 0.25*( self.getDist(4, angles, sin, full=1)*sp   + self.getDist(4, angles, cos, full=1)*crcp)
        J[2,7] = 0.25*( self.getDist(4, angles, sin, full=0)*sp   + self.getDist(4, angles, cos, full=0)*crcp)
        
        
        ## Partials of roll
        #k1 = self.getDist(1, angles, cos, full=1)*crcp*srcp - self.getDist(3, angles, cos, full=1)*srcp
        #k = np.sqrt(d**2 - k1**2)
        
        
        k1 = self.getDist(1, angles, sin, full=1)*crcp - self.getDist(3, angles, sin, full=1)*crcp + self.getDist(1, angles, cos, full=1)*crcp*srcp - self.getDist(3, angles, cos, full=1)*srcp
        k = np.sqrt(d**2 - (z31 - k1)**2)
        
        if (np.isnan(k)):
            k = 0
        
        if (k>0):
            J[3,0] = -J[2,0]/k
            J[3,1] = -J[2,1]/k
            J[3,2] = 0
            J[3,3] = 0
            J[3,4] = J[2,4]/k
            J[3,5] = J[2,5]/k
            J[3,6] = 0
            J[3,7] = 0
            
            
            
        ## Partials of pitch
        k1 = self.getDist(1, angles, sin, full=1)*crcp - self.getDist(3, angles, sin, full=1)*crcp + self.getDist(1, angles, cos, full=1)*crcp*srcp - self.getDist(3, angles, cos, full=1)*srcp
        k = np.sqrt(d**2 - (z31 - k1)**2)
        
        if (np.isnan(k)):
            k = 0
        
        if (k>0):
            J[4,0] = 0
            J[4,1] = 0
            J[4,2] = -J[2,2]/k
            J[4,3] = -J[2,3]/k
            J[4,4] = 0
            J[4,5] = 0
            J[4,6] = J[2,6]/k
            J[4,7] = J[2,7]/k
            
            
            
        ## Partials of d1^2
        J[5,0] =   -self.getDist(1, angles, sin, full=1) + 2*self.getDist(1, angles, cos, full=1) * ( self.getDist(1, angles, sin, full=1) - self.getDist(3, angles, sin, full=1))
        J[5,1] =  2*self.getDist(1, angles, cos, full=0) * (self.getDist(1, angles, sin, full=1) - self.getDist(3, angles, sin, full=1)) - self.getDist(1, angles, sin, full=0)
        J[5,2] = 0
        J[5,3] = 0
        J[5,4] =    self.getDist(3, angles, sin, full=1) - 2*self.getDist(3, angles, cos, full=1) * ( self.getDist(1, angles, sin, full=1) - self.getDist(3, angles, sin, full=1))
        J[5,5] = -2*self.getDist(3, angles, cos, full=0) * (self.getDist(1, angles, sin, full=1) - self.getDist(3, angles, sin, full=1)) + self.getDist(3, angles, sin, full=0)
        J[5,6] = 0
        J[5,7] = 0
        
        
        ## Partials of d2^2
        J[6,0] = 0
        J[6,1] = 0
        J[6,2] =   -self.getDist(2, angles, sin, full=1) + 2*self.getDist(2, angles, cos, full=1) * ( self.getDist(2, angles, sin, full=1) - self.getDist(4, angles, sin, full=1))
        J[6,3] =  2*self.getDist(2, angles, cos, full=0) * (self.getDist(2, angles, sin, full=1) - self.getDist(4, angles, sin, full=1)) - self.getDist(2, angles, sin, full=0)
        J[6,4] = 0
        J[6,5] = 0
        J[6,6] =    self.getDist(4, angles, sin, full=1) - 2*self.getDist(4, angles, cos, full=1) * ( self.getDist(2, angles, sin, full=1) - self.getDist(4, angles, sin, full=1))
        J[6,7] = -2*self.getDist(4, angles, cos, full=0) * (self.getDist(2, angles, sin, full=1) - self.getDist(4, angles, sin, full=1)) + self.getDist(4, angles, sin, full=0)
        
        
        ## Cast anything below the specified threshold to zero
        J[np.abs(J)<1e-10] = 0
        
        return J

    def getAnglesFromLeg(self, angles, leg):
        idx = (leg-1)*2
        return angles[idx], angles[idx+1]
        
    def getDist(self, leg, angles, trig, full=0):
        idx = (leg-1)*2
        a1 = angles[idx]
        a2 = angles[idx+1]
        L1 = SingleLegControl.length_thigh
        L2 = SingleLegControl.length_foreleg
        
        dist = L2*trig(a1+a2)
        
        if (full>0):
            dist += L1*trig(a1)
            
        return dist
            
        
    
    
        
    """
    def dx(self, angles, leg):
        if (leg==1 or leg==3):
            return [0, 0]
        
        dx = np.array( self.dz(angles, leg) ) * 4
        if (leg==4):
            dx *= -1
            
        return list(dx)
    
    
    def dy(self, angles, leg):
        if (leg==2 or leg==4):
            return [0, 0]
        
        dy = np.array( self.dz(angles, leg) ) * 4
        if (leg==3):
            dy *= -1
            
        return list(dy)
        

    def dz(self, angles, leg):
        a1, a2 = self.getAnglesFromLeg(angles, leg)
        L1 = SingleLegControl.length_thigh
        L2 = SingleLegControl.length_foreleg
        
        dz2 = L2*cos(a1+a2)/4
        dz1 = L1*cos(a1)/4 + dz2
        
        return [dz1, dz2]
    """

    def getAngles(self):
        angles = []
        angles.append( self.legs["front_right"].angle_thigh_lift_target )
        angles.append( self.legs["front_right"].angle_knee_lift_target )
        angles.append( self.legs["front_left"].angle_thigh_lift_target )
        angles.append( self.legs["front_left"].angle_knee_lift_target )
        angles.append( self.legs["back_left"].angle_thigh_lift_target )
        angles.append( self.legs["back_left"].angle_knee_lift_target )
        angles.append( self.legs["back_right"].angle_thigh_lift_target )
        angles.append( self.legs["back_right"].angle_knee_lift_target )
        
        return angles
        
    def setAngles(self, angles):
        self.legs["front_right"].angle_thigh_lift_target = angles[0]
        self.legs["front_right"].angle_knee_lift_target = angles[1]
        self.legs["front_left"].angle_thigh_lift_target = angles[2]
        self.legs["front_left"].angle_knee_lift_target = angles[3]
        self.legs["back_left"].angle_thigh_lift_target = angles[4]
        self.legs["back_left"].angle_knee_lift_target = angles[5]
        self.legs["back_right"].angle_thigh_lift_target = angles[6]
        self.legs["back_right"].angle_knee_lift_target = angles[7]

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
    rospy.init_node('balance_test')

    BalanceController()
