#!/usr/bin/env python

import os
import sys
import rospy, rospkg

from drone_legs_ros.pressure_sensor_manager import PressureSensorManager, ConsoleManager

if __name__ == '__main__':
    rospy.init_node('pressure_sensor_calibration')

    ## Get path to config file
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('drone_legs_ros')
    pressure_config_path = os.path.join(pkg_path, "yaml", "pressure_sensor_calibration.yaml")
    board_config_path = os.path.join(pkg_path, "yaml", "servo_calibration.yaml")

    ## Create the calibrator
    manager = PressureSensorManager(board_config=board_config_path, pressure_config=pressure_config_path)
    manager.calibrate()

    """
    console_mgr = ConsoleManager()
    r = rospy.Rate(30) # Hz

    while not rospy.is_shutdown():
        weights = manager.getWeights(unit="grams")
        msg = ""
        for w in weights:
            msg += "{:4.0f}\t".format(w)

        console_mgr.overwriteConsoleOutput(msg)

        r.sleep()
    """