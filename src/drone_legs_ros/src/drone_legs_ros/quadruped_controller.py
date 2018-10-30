from math import pi, cos, sin, acos

import rospy
from drone_legs_ros.srv import LegCommand, LegVerticalThrust
from drone_legs_ros.msg import LegJointAngles, LegPressureSensors
from drone_legs_ros.single_leg import SingleLegControl


class QuadrupedController():
    LEG_NAMES = ["front_right", "front_left", "back_right", "back_left"]
    
    def __init__(self):
        pass
        
    
    def initController(self):
        rospy.Subscriber("/drone_legs/joint_angles", LegJointAngles, self.callbackJointAngles)
        rospy.Subscriber("/drone_legs/pressure_sensors", LegPressureSensors, self.callbackPressureSensors)
    
        ## Declare variables
        self.legs = dict()
        self.update_interval = 0.01
        self.initiated_touchdown = False
        self.thrust = 1
        
        ## Initialize vector of legs
        for name in QuadrupedController.LEG_NAMES:
            self.legs[name] = SingleLegControl(name)
    
    
    def loop(self, t_elapsed):
        raise Exception("The function \"update(self, t_elapsed)\" must be defined in your derivative QuadrupedController")
    
        
    def start(self):
        ## Initiate loop
        t = 0.0
        while not rospy.is_shutdown():            
            ## The function that computes new commands. Defined in derivative classes
            self.update(t)
            
            ## Send new commands
            self.updateAngles()
            self.updateThrust()
            
            t += self.update_interval
            rospy.sleep(self.update_interval)
    
    
    def callbackJointAngles(self, msg):
        """
        JointAngle[] data
        --------
        string leg
        string type
        float64 angle
        """
        for datum in msg.data:
            leg = self.legs[datum.leg]
            
            if (datum.type == "base_hip"):
                leg.angle_base_hip = datum.angle
            elif (datum.type == "thigh_lift"):
                leg.angle_thigh_lift = datum.angle
            elif (datum.type == "knee_lift"):
                leg.angle_knee_lift = datum.angle
    
    
    def callbackPressureSensors(self, msg):
        """
        float64 front_right
        float64 front_left
        float64 back_right
        float64 back_left
        """
        self.legs["front_right"].sensor = msg.front_right
        self.legs["front_left"].sensor = msg.front_left
        self.legs["back_right"].sensor = msg.back_right
        self.legs["back_left"].sensor = msg.back_left
    
    
    def setAngles(self, a1, a2):
        service_name = "/drone_legs/leg_command"
        rospy.wait_for_service(service_name)
        
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
        
        
        for leg in self.legs.values():
            leg.angle_thigh_lift_target = a1
            leg.angle_knee_lift_target = a2
        
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
    
    
    def updateAngles(self):
        service_name = "/drone_legs/leg_command"
        rospy.wait_for_service(service_name)
        
        angles = dict()
        
        
        for name in QuadrupedController.LEG_NAMES:
            angles[name + "_base_hip"] = self.legs[name].angle_base_hip_target
            angles[name + "_thigh_lift"] = self.legs[name].angle_thigh_lift_target
            angles[name + "_knee_lift"] = self.legs[name].angle_knee_lift_target
        
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
            
            
    def updateThrust(self):
        if (self.thrust >= 0):  
            service_name = "/drone_legs/thrust_command"
            rospy.wait_for_service(service_name)
            
            
        
            try:
                thrust_command = rospy.ServiceProxy(service_name, LegVerticalThrust)
                resp = thrust_command(self.thrust)
                
                if (not resp.success):
                    print ("thrust_command status: ", resp.status_message)
                    
                return resp.success
            except rospy.ServiceException, e:
                print "thrust_command service call failed: %s"%e