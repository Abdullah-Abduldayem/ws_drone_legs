#!/usr/bin/env python
# -*- coding: utf-8 -*-

import wx
from math import radians, degrees

import time
import rospy
import rospkg
import os
import tf
from sensor_msgs.msg import Imu
from drone_legs_ros.quadruped_controller import QuadrupedController



class DataEntry():
    joints_target  = []
    joints_current = []
    quat_current = []
    angular_vel_current = []
    linear_acc_current = []

    def __init__(self, csv_line = None):
        if (csv_line is not None):
            csv_elements = csv_line.split(",")
            if len(csv_elements) != 26:
                raise Exception("Expecting {} elements in CSV line. Got {}. String: '{}'".format(26, len(csv_elements), csv_line))

            arr = [float(x) for x in csv_elements]
            self.joints_target = arr[0:8]
            self.joints_current = arr[8:16]
            self.quat_current = arr[16:20]
            self.angular_vel_current = arr[20:23]
            self.linear_acc_current = arr[23:26]


    def GetNetworkOutput(self):
        return self.joints_target

    def GetNetworkInput(self):
        return self.joints_current + self.quat_current + self.angular_vel_current + self.linear_acc_current

    def __str__(self):
        out_arr = self.GetNetworkOutput() + self.GetNetworkInput()

        out = ', '.join(str(x) for x in out_arr)
        return out


class DataCollectorForm(wx.Frame):
    def __init__(self, *args, **kw):
        super(DataCollectorForm, self).__init__(*args, **kw)

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

        self.entries = []
        self.joints_target = None
        self.entry = None

        self.timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.OnTick, self.timer)
        self.timer.Start(100)

        self.InitUI()

        self.OnLoad(None)

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
        slider_range["front_right_thigh_lift"] = (0, 90, 45)
        slider_range["front_right_knee_lift"] = (0, 180, 45)
        slider_range["front_left_thigh_lift"] = (0, 90, 45)
        slider_range["front_left_knee_lift"] = (0, 180, 45)
        slider_range["back_right_thigh_lift"] = (0, 90, 45)
        slider_range["back_right_knee_lift"] = (0, 180, 45)
        slider_range["back_left_thigh_lift"] = (0, 90, 45)
        slider_range["back_left_knee_lift"] = (0, 180, 45)

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

        self.SetTitle('Data Collector for landed RL')
        self.Centre()

        ##########
        # Create buttons
        ##########
        button_size = (140, 40)
        btnSetTarget = wx.Button(pnl, label='Set Target Pose', pos=(220, 20), size=button_size)
        btnSetTarget.Bind(wx.EVT_BUTTON, self.OnSetTarget)

        btnGetSample = wx.Button(pnl, label='Get Sample Pose', pos=(220, 60), size=button_size)
        btnGetSample.Bind(wx.EVT_BUTTON, self.OnGetSample)

        btnResetPose = wx.Button(pnl, label='Reset Pose', pos=(220, 100), size=button_size)
        btnResetPose.Bind(wx.EVT_BUTTON, self.OnResetPose)

        self.txt_num_samples = wx.StaticText(pnl, label=str("Total Samples: 0"), pos=(220, 140))

        btnLoad = wx.Button(pnl, label='Load Data', pos=(220, 180), size=button_size)
        btnLoad.Bind(wx.EVT_BUTTON, self.OnLoad)

        btnSave = wx.Button(pnl, label='Save Data', pos=(220, 220), size=button_size)
        btnSave.Bind(wx.EVT_BUTTON, self.OnSave)

        self.txt_roll  = wx.StaticText(pnl, label=str("   Roll: 0"), pos=(80, 220))
        self.txt_pitch = wx.StaticText(pnl, label=str("Pitch: 0"), pos=(80, 240))

    def OnTick(self, e):
        for leg_name in self.leg_names:
            self.sensor_txt[leg_name].SetLabel("{:04d}".format(int(self.servo_controller.legs[leg_name].sensor)))

        self.txt_roll.SetLabel("   Roll: {:2.1f}".format(degrees(self.servo_controller.rpy[0])))
        self.txt_pitch.SetLabel("Pitch: {:2.1f}".format(degrees(self.servo_controller.rpy[1])))

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

    def OnSetTarget(self, e):
        if (self.entry is None):
            wx.MessageBox('Get sample pose first', 'Warning', wx.OK | wx.ICON_WARNING)
            return

        self.entry.joints_target = self.GetCurrentJointAngles()

        self.entries.append(self.entry)
        self.UpdateTotalSamples()

    def OnGetSample(self, e):
        #if (self.joints_target is None):
        #    wx.MessageBox('Set target pose first', 'Warning', wx.OK | wx.ICON_WARNING)
        #    return

        entry = DataEntry()

        entry.joints_current = self.GetCurrentJointAngles()
        entry.quat_current = self.servo_controller.quat
        entry.angular_vel_current = self.servo_controller.angular_vel
        entry.linear_acc_current = self.servo_controller.linear_acc
        self.entry = entry

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

    def OnLoad(self, e):
        self.entries = []

        with open(self.csv_dir, "r") as csv:
            header = csv.readline()

            for line in csv:
                entry = DataEntry(line)
                self.entries.append(entry)

        self.UpdateTotalSamples()

    def OnSave(self, e):
        with open(self.csv_dir, "w") as csv:
            header = "target_1, target_2, target_3, target_4, target_5, target_6, target_7, target_8, "
            header += "current_1, current_2, current_3, current_4, current_5, current_6, current_7, current_8, "
            header += "quat_x, quat_y, quat_z, quat_w, "
            header += "ang_vel_x, ang_vel_y, ang_vel_z, "
            header += "lin_acc_x, lin_acc_y, lin_acc_z"
            csv.write(header + "\n")

            for entry in self.entries:
                csv.write(str(entry) + "\n")

    def UpdateTotalSamples(self):
        self.txt_num_samples.SetLabel("Total Samples: {}".format(len(self.entries)))

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



if __name__ == '__main__':
    rospy.init_node('rl_data_collector')

    app = wx.App()
    ex = DataCollectorForm(None)
    ex.Show()
    app.MainLoop()