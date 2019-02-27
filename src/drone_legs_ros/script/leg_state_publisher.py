#!/usr/bin/env python
import time
import os
import rospy, rospkg
import ssc32
import yaml
import math
import threading

from drone_legs_ros.msg import LegJointAngles, JointAngle, LegPressureSensors
from drone_legs_ros.srv import LegCommand, LegCommandResponse, LegVerticalThrust, LegVerticalThrustResponse
from drone_legs_ros.pressure_sensor_manager import PressureSensorManager

class LegStatePublisher:
    def __init__(self, rate=30):
        self.pub_sensors = rospy.Publisher("/drone_legs/pressure_sensors", LegPressureSensors, queue_size=10)
        self.pub_joints = rospy.Publisher("/drone_legs/joint_angles", LegJointAngles, queue_size=10)
        self.srv_joints = rospy.Service("/drone_legs/leg_command", LegCommand, self.service_joint_command)
        self.srv_thrust = rospy.Service("/drone_legs/thrust_command", LegVerticalThrust, self.service_thrust_command)

        self.lock = threading.Lock()

        ## Get path to config files
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('drone_legs_ros')
        pressure_config_path = os.path.join(pkg_path, "yaml", "pressure_sensor_calibration.yaml")
        board_config_path = os.path.join(pkg_path, "yaml", "servo_calibration.yaml")


        with open(board_config_path, 'r') as f:
            data = yaml.load(f.read())
        print(board_config_path)

        self.ssc = ssc32.SSC32(config=board_config_path)
        self.ssc.autocommit = None
        #self.ssc = ssc32.SSC32("/dev/ttyUSB0", 115200, count=32)
        rospy.loginfo("Connected to SSC32 servo board. Port: {}, Baud: {}, Version: {}".format(self.ssc.ser.port, self.ssc.ser.baudrate, self.ssc.get_firmware_version()))

        self.pressure_mgr = PressureSensorManager(pressure_config=pressure_config_path, ssc=self.ssc)


        ## Begin update loop
        ## Max rate is around 100Hz
        r = rospy.Rate(rate) # Hz
        while not rospy.is_shutdown():
            self.update()
            r.sleep()
        
    
    def update(self):
        ## Publish pressire sensor data
        self.lock.acquire()
        weights = self.pressure_mgr.getWeights(unit="grams")
        self.lock.release()

        msg_pressure = LegPressureSensors()
        msg_pressure.front_right = weights["front_right"]
        msg_pressure.front_left  = weights["front_left"]
        msg_pressure.back_left   = weights["back_left"]
        msg_pressure.back_right  = weights["back_right"]
        self.pub_sensors.publish(msg_pressure)


        ## Publish joint angle
        msg_joints = LegJointAngles()
        self.lock.acquire()
        for servo in self.ssc._servos:
            msg_joint = JointAngle()

            name = servo.name.lower().split("_")
            msg_joint.leg  = "_".join(name[0:2]) #"back_left"
            msg_joint.type = "_".join(name[2:])  #"thigh_lift"
            msg_joint.angle = servo.radians

            msg_joints.data.append(msg_joint)

        self.lock.release()
        self.pub_joints.publish(msg_joints)

    def service_joint_command(self, req):
        print("Service called")


        resp = LegCommandResponse()

        if (len(req.joint_names) != len(req.joint_positions)):
            resp.success = False
            resp.status_message = "Length of joint_names and joint_positions is not equal."
            return resp
        
        
        self.lock.acquire()

        for i in range(len(req.joint_names)):
            name = req.joint_names[i]
            angle = req.joint_positions[i]

            self.ssc[name].radians = angle

        self.ssc.commit()
        """
        for servo in self.ssc._servos:
            #print('query_pulse_width', self.ssc.query_pulse_width(joint))
            print("{}\t {}".format(servo.name, servo.degrees))
            servo.degrees = 40*abs(math.sin(time.time()))

        self.ssc.commit()
        """
        self.lock.release()

        print("Success")
        
        resp.success = True
        resp.status_message = ""
        return resp

    def service_thrust_command(self, req):
        print("Ignoring thrust command")


        resp = LegVerticalThrustResponse()
        resp.success = True
        resp.status_message = ""
        return resp

        


if __name__ == '__main__':
    node_name = "leg_state_publisher"
    rospy.init_node(node_name)
    rospy.loginfo("Started " + node_name)

    leg_state = LegStatePublisher(rate=30)