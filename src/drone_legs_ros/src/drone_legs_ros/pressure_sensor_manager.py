#!/usr/bin/env python

import sys
import os
import time
from types import *
import select

import rospy
import rospkg
import numpy as np
import yaml

import ssc32

class PressureSensorManager:
    def __init__(self, pressure_config, board_config=None, ssc=None):
        assert type(pressure_config) is StringType, "'pressure_config' must be a string"

        ## Read existing calibration data
        self.pressure_config_path = pressure_config

        if (os.path.isfile(self.pressure_config_path)):
            with open(self.pressure_config_path, "r") as f:
                calib_data = f.read()

            self.calib_data = yaml.load(calib_data)

        else:
            self.calib_data = dict()


        ## Initialize SSC32 board
        if ssc:
            self.ssc = ssc
        else:
            if board_config:
                self.ssc = ssc32.SSC32(config=board_config)
            else:
                self.ssc = ssc32.SSC32("/dev/ttyUSB0", 115200, count=32)

        rospy.loginfo("Connected to SSC32 servo board. Port: {}, Baud: {}, Version: {}".format(self.ssc.ser.port, self.ssc.ser.baudrate, self.ssc.get_firmware_version()))

        ## Define our mapping between channel name and leg name
        self.channel_map = {
            "A": "front_right",
            "B": "front_left",
            "C": "back_left",
            "D": "back_right",
        }

    def calibrate(self):
        self.console = ConsoleManager()

        calib_weights = [0, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000]
        ports = self.channel_map.keys()
        ports.sort()

        for port in ports:
            calibration_values = []
            for weight in calib_weights:
                if (rospy.is_shutdown()):
                    print("Calibration interuupted by user. Exiting.")
                    sys.exit()

                ## Wait for ENTER to be pressed
                msg = "{:3}\tApply {}g of force to {} leg (channel {})."
                while (not self.console.heardEnter()):
                    reading = self.ssc.read_analog_input(port)
                    self.console.overwriteConsoleOutput('>' + msg.format(reading, weight, self.channel_map[port], port) + " [ENTER to continue]")

                ## Take a sample of readings
                rate = 30
                count = rate*3 # 3 seconds of data

                readings = []

                r = rospy.Rate(rate) # in Hz
                for _ in range(count):
                    if rospy.is_shutdown():
                        break

                    reading = self.ssc.read_analog_input(port)
                    self.console.overwriteConsoleOutput(' ' + msg.format(reading, weight, self.channel_map[port], port) + " [Reading for 3 seconds]")

                    readings.append(reading)
                    r.sleep()

                readings = np.array(readings)
                avg = np.mean(readings)
                std = np.std(readings)
                
                self.console.overwriteConsoleOutput("Mean: {:3.1f}, Std dev: {:2.1f}\n".format(avg, std))

                calibration_values.append([avg, std])

            calibration_values = np.array(calibration_values)

            ## Ratio between fixed resistance and the FSR resisitance.
            ## This is derived from the voltage divider formula and is
            ## directly related to weight
            R_Rs = calibration_values[:,0]/(255-calibration_values[:,0])

            ## Fit a straight line
            coeff = np.polyfit(calib_weights, R_Rs, 1)
            
            ## Determine a threshold for readings, below which it will
            ## automatically assign a value of "zero" weight
            #threshold = calibration_values[1][0]+calibration_values[1][1]
            threshold = 20

            data = dict()
            data["port"] = port
            data["coeff0"] = float(coeff[0])
            data["coeff1"] = float(coeff[1])
            data["threshold"] = float(threshold)

            self.calib_data[self.channel_map[port]] = data

        with open(self.pressure_config_path, 'w') as f:
            f.write(yaml.dump(self.calib_data, default_flow_style=False))

    def getWeights(self, unit="newtons"):
        ports = self.channel_map.keys()
        ports.sort()

        weights = dict()
        for port in ports:
            reading = self.ssc.read_analog_input(port)
            weight = self.readingToWeight(name=self.channel_map[port], reading=reading, unit=unit)
            
            weights[self.channel_map[port]] = int(weight)

        return weights

    def readingToWeight(self, name, reading, unit="newtons"):
        if (reading < self.calib_data[name]["threshold"]):
            return 0
        
        R_Rs = reading/float(255-reading)
        mass = (R_Rs-self.calib_data[name]["coeff1"])/self.calib_data[name]["coeff0"]

        if unit == "grams":
            return mass
        if unit == "newtons":
            return mass*0.00981
        raise Exception("Invalid units. Use either 'grams' or 'newtons'")

    
class ConsoleManager:
    def __init__(self):
        pass

    def heardEnter(self):
        i,o,e = select.select([sys.stdin],[],[],0.0001)
        for s in i:
            if s == sys.stdin:
                input = sys.stdin.readline()
                return True
        return False

    def overwriteConsoleOutput(self, msg):
        sys.stdout.write("\r")
        sys.stdout.write(" "*100) ## Overwrite the line with blanks
        sys.stdout.write("\r")
        sys.stdout.write(msg)
        sys.stdout.flush()



if __name__ == '__main__':
    rospy.init_node('pressure_sensor_calibration')

    ## Get path to config file
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('drone_legs_ros')
    pressure_config_path = os.path.join(pkg_path, "yaml", "pressure_sensor_calibration.yaml")
    board_config_path = os.path.join(pkg_path, "yaml", "servo_calibration.yaml")

    ## Create the calibrator
    manager = PressureSensorManager(board_config=board_config_path, pressure_config=pressure_config_path)
    #manager.calibrate()

    console_mgr = ConsoleManager()
    r = rospy.Rate(30) # Hz

    while not rospy.is_shutdown():
        weights = manager.getWeights(unit="grams")
        msg = ""
        for w in weights:
            msg += "{:4.0f}\t".format(w)

        console_mgr.overwriteConsoleOutput(msg)

        r.sleep()