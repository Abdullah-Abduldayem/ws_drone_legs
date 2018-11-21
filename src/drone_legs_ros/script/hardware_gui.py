#!/usr/bin/env python
# -*- coding: utf-8 -*-

import wx
from math import radians, degrees, pi

import rospy
import rospkg
import os
import tf
from sensor_msgs.msg import Imu
from drone_legs_ros.quadruped_controller import QuadrupedController
from collections import deque

import time

class HardwareControlForm(wx.Frame):
    def __init__(self, *args, **kw):
        super(HardwareControlForm, self).__init__(*args, **kw)

        # get an instance of RosPack with the default search paths
        rospack = rospkg.RosPack()
        self.pkg_dir = rospack.get_path('drone_legs_ros')
        self.csv_dir = os.path.join(self.pkg_dir, "training_data", "landed.csv")

        self.num_samples = 0
        self.joint_names = [
            "front_right_thigh_lift",
            "front_right_knee_lift",
            "front_left_thigh_lift",
            "front_left_knee_lift",
            "back_right_thigh_lift",
            "back_right_knee_lift",
            "back_left_thigh_lift",
            "back_left_knee_lift"
            ]
        self.leg_names = [
            "front_right",
            "front_left",
            "back_left",
            "back_right"
        ]

        self.servo_controller = FullQuadrupedController()
        self.servo_controller.initController()
        self.servo_controller.initAngles()


        self.entries = []
        self.joints_target = None
        self.entry = None

        self.timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.OnTick, self.timer)
        self.timer.Start(20) #50Hz

        self.executing_control = False

        self.InitUI()

    def InitUI(self):

        pnl = wx.Panel(self)

        ##########
        # Create sliders for controlling joint angle
        ##########
        slider_pos = dict()
        slider_pos["front_right_thigh_lift"] = (150, 20)
        slider_pos["front_right_knee_lift"] = (180, 20)
        slider_pos["front_left_thigh_lift"] = (20, 20)
        slider_pos["front_left_knee_lift"] = (50, 20)
        slider_pos["back_right_thigh_lift"] = (150, 120)
        slider_pos["back_right_knee_lift"] = (180, 120)
        slider_pos["back_left_thigh_lift"] = (20, 120)
        slider_pos["back_left_knee_lift"] = (50, 120)

        slider_range = dict()
        slider_range["front_right_thigh_lift"] = (-20, 100, 45)
        slider_range["front_right_knee_lift"] = (-20, 180, 45)
        slider_range["front_left_thigh_lift"] = (-20, 100, 45)
        slider_range["front_left_knee_lift"] = (-20, 180, 45)
        slider_range["back_right_thigh_lift"] = (-20, 100, 45)
        slider_range["back_right_knee_lift"] = (-20, 180, 45)
        slider_range["back_left_thigh_lift"] = (-20, 100, 45)
        slider_range["back_left_knee_lift"] = (-20, 180, 45)

        slider_size = (30, 80)

        self.slider_ctrl = dict()
        self.slider_txt = dict()
        self.sensor_txt = dict()

        for joint_name in self.joint_names:
            sld = wx.Slider(pnl, value=slider_range[joint_name][2], minValue=slider_range[joint_name][0], maxValue=slider_range[joint_name][1], style=wx.SL_VERTICAL)
            sld.Bind(wx.EVT_SCROLL, self.OnSliderScroll)
            sld.SetLabel(joint_name)
            sld.SetPosition(slider_pos[joint_name])
            sld.SetSize(slider_size)
            self.slider_ctrl[joint_name] = sld

            if slider_pos[joint_name][1] < 100:
                txt_pos = (slider_pos[joint_name][0]+5, 0)
            else:
                txt_pos = (slider_pos[joint_name][0]+5, 200)

            self.slider_txt[joint_name] = wx.StaticText(pnl, label=str(slider_range[joint_name][2]))
            self.slider_txt[joint_name].SetPosition(txt_pos)
            self.slider_txt[joint_name].SetSize((40, 40))

        ## Pressure sensor labels
        for leg_name in self.leg_names:
            joint_name = leg_name+"_thigh_lift"

            if slider_pos[joint_name][0] < 100:
                x = 70
            else:
                x = 110

            if slider_pos[joint_name][1] < 100:
                y = 70
            else:
                y = 110

            self.sensor_txt[leg_name] = wx.StaticText(pnl, label="0")
            self.sensor_txt[leg_name].SetPosition((x,y))
            self.sensor_txt[leg_name].SetSize((40, 40))

        self.SetTitle('Leg Control GUI')
        self.Centre()

        ##########
        # Create buttons
        ##########
        button_size = (140, 40)
        btnExecuteControl1 = wx.Button(pnl, label='Start Control 1', pos=(220, 20), size=button_size)
        btnExecuteControl1.Bind(wx.EVT_BUTTON, self.OnControll1)

        btnResetPose = wx.Button(pnl, label='Reset Pose', pos=(220, 100), size=button_size)
        btnResetPose.Bind(wx.EVT_BUTTON, self.OnResetPose)

        self.txt_roll  = wx.StaticText(pnl, label=str("   Roll: 0"), pos=(80, 220))
        self.txt_pitch = wx.StaticText(pnl, label=str("Pitch: 0"), pos=(80, 240))
        self.txt_error = wx.StaticText(pnl, label=str("Error: None"), pos=(0, 260))

    def OnTick(self, e):
        for leg_name in self.leg_names:
            self.sensor_txt[leg_name].SetLabel("{:04d}".format(int(self.servo_controller.legs[leg_name].sensor)))

        error = ""

        if (self.servo_controller.rpy is None):
            error += "No roll or pitch data received"
        else:
            roll = self.servo_controller.rpy[0]
            pitch = self.servo_controller.rpy[1]

            self.txt_roll.SetLabel("   Roll: {:2.3f}".format(degrees(roll)))
            self.txt_pitch.SetLabel("Pitch: {:2.3f}".format(degrees(pitch)))

        if (self.executing_control):
            self.servo_controller.OnTick()

        self.txt_error.SetLabel("Error: " + error)

    def OnSliderScroll(self, e):
        obj = e.GetEventObject()
        val = obj.GetValue()

        joint_name = obj.GetLabel()
        self.slider_txt[joint_name].SetLabel(str(val))

        leg_type, joint_type = self.SplitLegName(joint_name)

        if (joint_type == "knee_lift"):
           self.servo_controller.legs[leg_type].angle_knee_lift_target = radians(val)
        elif (joint_type == "thigh_lift"):
           self.servo_controller.legs[leg_type].angle_thigh_lift_target = radians(val)
        else:
            raise Exception("Unknown joint type '{}'".format(joint_type))

        self.servo_controller.updateAngles()

    def OnResetPose(self, e):
        for joint_name in self.joint_names:
            val = 45
            self.slider_ctrl[joint_name].SetValue(val)
            self.slider_txt[joint_name].SetLabel(str(val))

            leg_type, joint_type = self.SplitLegName(joint_name)

            if (joint_type == "knee_lift"):
                self.servo_controller.legs[leg_type].angle_knee_lift_target = radians(val)
            elif (joint_type == "thigh_lift"):
                self.servo_controller.legs[leg_type].angle_thigh_lift_target = radians(val)

        self.servo_controller.updateAngles()

    def OnControll1(self, e):
        self.executing_control = not self.executing_control

        obj = e.GetEventObject()
        if (self.executing_control):
            obj.SetLabel("Stop Control 1")
        else:
            obj.SetLabel("Start Control 1")



    def GetCurrentJointAngles(self):
        angles = []
        for joint_name in self.joint_names:
            leg_type, joint_type = self.SplitLegName(joint_name)

            leg = self.servo_controller.legs[leg_type]
            if (joint_type == "thigh_lift"):
                angles.append(leg.angle_thigh_lift_target)
            elif (joint_type == "knee_lift"):
                angles.append(leg.angle_knee_lift_target)

        return angles

    def SplitLegName(self, name):
        fragments = name.split("_")
        leg_type = "_".join(fragments[0:2])
        joint_type = "_".join(fragments[2:4])

        return leg_type, joint_type



class FullQuadrupedController(QuadrupedController):
    ###########
    # This class collects the following data:
    # 1) The target state (joint angles)
    # 2) The current state (joint angles, joint speeds, IMU tilt, pressure sensor data?)
    ###########
    imu_data = None
    rpy = None


    def __init__(self):
        rospy.Subscriber("/mavros/imu/data", Imu, self.callbackImu)
        print("Initilized controller")

        ## -1 moves in negative (downward) direction, 1 moves in positive (upward) direction
        self.prev_dir = dict()
        self.prev_dir["front_left"] = 0
        self.prev_dir["front_right"] = 0
        self.prev_dir["back_right"] = 0
        self.prev_dir["back_left"] = 0

        self.pose_ori = [0, 0]

        self.leg_num = 0
        self.leg_names = [
            "front_right",
            "front_left",
            "back_left",
            "back_right"
        ]

        ## The "roll" and "pitch" if we rotate IMU data so it aligns with the legs
        self.modified_roll = 0
        self.modified_pitch = 0

        self.rad_inc = radians(2)
        self.processing_tick = False

    def initAngles(self):
        for key in self.legs:
            leg = self.legs[key]
            leg.angle_thigh_lift_target = radians(45)
            leg.angle_knee_lift_target = radians(45)

    def callbackImu(self, msg):
        ########
        # Quaternion orientation
        # xyz angular_velocity
        # xyz linear_acceleration
        ########
        self.imu_data = msg
        self.quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        self.rpy = tf.transformations.euler_from_quaternion(self.quat)
        self.angular_vel = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        self.linear_acc = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]

        ## To simplify the problem, let's look at the angle that two opposing legs contribute to
        ## To do this, we rotate the quat by 45 degrees since the legs are in an X-configuration
        q_rot = tf.transformations.quaternion_from_euler(0, 0, -pi / 4)
        q_new = tf.transformations.quaternion_multiply(self.quat, q_rot)
        rpy_new = tf.transformations.euler_from_quaternion(q_new)
        self.modified_roll = degrees(rpy_new[0])
        self.modified_pitch = degrees(rpy_new[1])

    def OnTick(self):
        if (self.processing_tick):
            return

        self.processing_tick = True
        print("{:2.3f}, {:2.3f}".format(self.modified_roll, self.modified_pitch))

        self.leg_num = 0
        self.pose_ori = [self.modified_roll, self.modified_pitch]
        dist_ori = self.pose_ori[0]**2 + self.pose_ori[1]**2


        for self.leg_num in range(4):
            leg_name = self.leg_names[self.leg_num]
            leg = self.legs[leg_name]
            leg_name_opp = self.leg_names[(self.leg_num+2)%4]
            leg_opp = self.legs[leg_name_opp]

            tilt_threshold = 2.5
            imbalance = 4
            if (leg_name == "front_left" or leg_name == "back_right") and abs(self.modified_roll) < tilt_threshold:
                ## We've reached our threshold, skip ahead
                pass

            elif (leg_name == "back_left" or leg_name == "front_right") and abs(self.modified_pitch) < tilt_threshold:
                ## We've reached our threshold, skip ahead
                pass

            elif (abs(self.modified_roll) - imbalance > abs(self.modified_pitch)) and (leg_name == "back_left" or leg_name == "front_right"):
                ## We have a big imbalance in roll, skip the pitch for now
                pass

            elif (abs(self.modified_pitch) - imbalance > abs(self.modified_roll)) and (leg_name == "front_left" or leg_name == "back_right"):
                ## We have a big imbalance in pitch, skip the roll for now
                pass

            else:
                start_dir = self.prev_dir[leg_name]
                if (start_dir == 0):
                    start_dir = -1

                if start_dir+self.prev_dir[leg_name_opp] == 0:
                    move_opp_leg = True
                else:
                    move_opp_leg = False

                ## Let's execute motion in the direction of the last step since the seemed to work
                ## Otherwise we'll try the opposite direction
                updated_dir = False
                for dir in [start_dir, -start_dir]:
                    leg.angle_thigh_lift_target += self.rad_inc*dir

                    if (leg.angle_thigh_lift_target > radians(80)):
                        ## Lower all legs. This guy is stretching too hard
                        for temp_leg_name in self.legs:
                            temp_leg = self.legs[temp_leg_name]

                            ## If the opposing leg is relatively unbent, don't touch it
                            if temp_leg_name == leg_name_opp and temp_leg.angle_thigh_lift_target < radians(70):
                                continue

                            temp_leg.angle_thigh_lift_target -= self.rad_inc*dir
                            temp_leg.angle_knee_lift_target += self.rad_inc*dir

                    elif (leg.angle_thigh_lift_target < radians(5)):
                        ## This leg is just too high. Stop raising it.
                        leg.angle_thigh_lift_target = radians(5)

                    leg.angle_knee_lift_target = pi/2 - leg.angle_thigh_lift_target

                    if (move_opp_leg):
                        leg_opp.angle_thigh_lift_target -= self.rad_inc*dir
                        leg_opp.angle_knee_lift_target = pi/2 - leg_opp.angle_thigh_lift_target

                    self.updateAngles()

                    ## Sleep to get the IMU data
                    time.sleep(0.2) ## 20Hz

                    ## If this is a good step, keep it
                    ## Otherwise, revert
                    dist = self.modified_roll**2 + self.modified_pitch**2
                    if (dist < dist_ori):
                        dist_ori = dist
                        self.prev_dir[leg_name] = dir
                        updated_dir = True
                        break

                    else:
                        ## Bad move. Let's go back to the original pose
                        leg.angle_thigh_lift_target -= self.rad_inc * dir
                        leg.angle_knee_lift_target = pi/2 - leg.angle_thigh_lift_target

                        if (move_opp_leg):
                            leg_opp.angle_thigh_lift_target += self.rad_inc * dir
                            leg_opp.angle_knee_lift_target = pi/2 - leg_opp.angle_thigh_lift_target
                """
                if (not updated_dir):
                    ## Moving this leg does nothing. It's probably in the air or at its limits
                    self.prev_dir[leg_name] = 0

                    ## Let's lower the leg if its probably in the air
                    if (leg.angle_thigh_lift_target < radians(70)):
                        leg.angle_thigh_lift_target += self.rad_inc
                        leg.angle_knee_lift_target = pi/2 - leg.angle_thigh_lift_target
                        self.updateAngles()
                """
        self.processing_tick = False


if __name__ == '__main__':
    rospy.init_node('rl_data_collector')

    app = wx.App()
    ex = HardwareControlForm(None)
    ex.Show()
    app.MainLoop()