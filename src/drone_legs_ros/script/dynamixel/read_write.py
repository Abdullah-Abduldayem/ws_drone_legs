#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
# Copyright 2017 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

# Author: Ryu Woon Jung (Leon)

#
# *********     Read and Write Example      *********
#
#
# Available DXL model on this example : All models using Protocol 1.0
# This example is tested with a DXL MX-28, and an USB2DYNAMIXEL
# Be sure that DXL MX properties are already set as %% ID : 1 / Baudnum : 34 (Baudrate : 57600)
#

import os, subprocess
import platform
import time
from math import sin, cos, pi

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import *                    # Uses Dynamixel SDK library





# Define exceptions
class DxlError(Exception):
   """Base class for other Dynamixel exceptions"""
   pass

class DxlTxError(DxlError):
    """Base class for transmission errors"""
    pass

class DxlRxError(DxlError):
    """Base class for reception errors"""
    pass

class DxlPortBusyError(DxlTxError):
    def __init__(self):
        DxlTxError.__init__(self, "Port is in use")

class DxlTransmissionFailError(DxlTxError):
    def __init__(self):
        DxlTxError.__init__(self, "Failed to transmit instruction packet")

class DxlReceptionFailError(DxlRxError):
    def __init__(self):
        DxlRxError.__init__(self, "Failed to receive status packet")

class DxlWaitingError(DxlRxError):
    def __init__(self):
        DxlRxError.__init__(self, "Now receiving status packet")

class DxlTimeoutError(DxlRxError):
    def __init__(self):
        DxlRxError.__init__(self, "Timeout: No status packet")
        
class DxlCorruptPacketError(DxlRxError):
    def __init__(self):
        DxlRxError.__init__(self, "Received status packet is corrupt")
        
class DxlProtocolCommandError(DxlTxError):
    def __init__(self):
        DxlTxError.__init__(self, "Protocol does not support this function")

class DxlInstructionError(DxlTxError):
   def __init__(self):
        DxlTxError.__init__(self, "Undefined instruction used or action command is delivered without reg_write command")

class DxlOverloadError(DxlRxError):
   def __init__(self):
        DxlRxError.__init__(self, "Overload: present load cannot be controlled with set maximum torque")

class DxlChecksumError(DxlTxError):
   def __init__(self):
        DxlTxError.__init__(self, "Checksum of transmitted instruction packet is invalid")

class DxlRangeError(DxlRxError):
   def __init__(self):
        DxlRxError.__init__(self, "Command is out of range")

class DxlOverheatingError(DxlRxError):
   def __init__(self):
        DxlRxError.__init__(self, "Temperature is out of range of operating temperatures")

class DxlAngleLimitError(DxlRxError):
    def __init__(self):
        DxlRxError.__init__(self, "Goal position is outside CW and CCW angle limits")

class DxlInputVoltageError(DxlRxError):
   def __init__(self):
        DxlRxError.__init__(self, "Input voltage is outside operating voltage range")




class Motor:
    """
    When raw=True, the raw values are passed between the motors and the user
    When raw=False, the values are converted automatically to the following:
        Position: Degrees
        Speed: Degrees per second
        Current: Amps
        Voltage: Volts
    """
    
    def __init__(self, port_handler, protocol_version, id):
        self.port_handler = port_handler
        self.id = id
        self.packet_handler = PacketHandler(protocol_version)
        
        ## Format: [Address, length]
        self.CTRL_TORQUE_ENABLE      = [64, 1]
        self.CTRL_GOAL_POSITION      = [116, 2]
        self.CTRL_PRESENT_POSITION   = [132, 2]
        
        self.UNIT_ANGLE   = 0.06 # degrees per unit
        self.UNIT_CURRENT = 0.01 # amps per unit
        self.UNIT_VOLTAGE = 0.1  # volts per unit
        self.UNIT_SPEED   = 0.111*6 # (deg per second) per unit
        
        self.CENTER_POSITION = 2048
        self.CENTER_CURRENT = 512
        
        self.cw_angle_limit = None
        self.ccw_angle_limit = None
        
        
    def enable(self, mode="joint", min_angle=None, max_angle=None):
        self.port_handler.clearPort()
        
        if mode == "joint":
            self.ccw_angle_limit = max_angle
            self.cw_angle_limit = min_angle
            
            self.setCcwAngleLimit(self.ccw_angle_limit)
            self.setCwAngleLimit(self.cw_angle_limit)
            
        elif mode == "wheel":
            raise NotImplementedError("Wheel mode not implimented")
        
        else:
            raise ValueError("Invalid mode. Must be joint or wheel")
        
    def isJointMode(self):
        if self.ccw_angle_limit != 0 or self.cw_angle_limit != 0:
            return True
        
        return False

    def writeNByteTxRx(self, instruction, value):
        if (instruction[1] == 1):
            comm_result, error = self.packet_handler.write1ByteTxRx(self.port_handler, self.id, instruction[0], value)
        elif (instruction[1] == 2):
            comm_result, error = self.packet_handler.write2ByteTxRx(self.port_handler, self.id, instruction[0], value)
        elif (instruction[1] == 4):
            comm_result, error = self.packet_handler.write4ByteTxRx(self.port_handler, self.id, instruction[0], value)
        else:
            raise ValueError("Invalid instruction. Must have length 1, 2 or 4")
        
        self.handleErrors(comm_result, error)
    
    def readNByteTxRx(self, instruction):
        if (instruction[1] == 1):
            value, comm_result, error = self.packet_handler.read1ByteTxRx(self.port_handler, self.id, instruction[0])
        elif (instruction[1] == 2):
            value, comm_result, error =  self.packet_handler.read2ByteTxRx(self.port_handler, self.id, instruction[0])
        elif (instruction[1] == 4):
            value, comm_result, error =  self.packet_handler.read4ByteTxRx(self.port_handler, self.id, instruction[0])
        else:
            raise ValueError("Invalid instruction. Must have length 1, 2 or 4")
    
        self.handleErrors(comm_result, error)
        return value
    
    def handleErrors(self, comm_result, error):
        if comm_result == COMM_SUCCESS:
            pass
        elif comm_result == COMM_PORT_BUSY:
            raise DxlPortBusyError()
        elif comm_result == COMM_TX_FAIL:
            raise DxlTransmissionFailError()
        elif comm_result == COMM_RX_FAIL:
            raise DxlReceptionFailError()
        elif comm_result == COMM_TX_ERROR:
            raise DxlInstructionError()
        elif comm_result == COMM_RX_WAITING:
            raise DxlWaitingError()
        elif comm_result == COMM_RX_TIMEOUT:
            raise DxlTimeoutError()
        elif comm_result == COMM_RX_CORRUPT:
            raise DxlCorruptPacketError()
        elif comm_result == COMM_NOT_AVAILABLE:
            raise DxlProtocolCommandError()
        
        if error == 0:
            pass
        elif error & ERRBIT_VOLTAGE:
            raise DxlInputVoltageError()
        elif error & ERRBIT_ANGLE:
            raise DxlAngleLimitError()
        elif error & ERRBIT_OVERHEAT:
            raise DxlOverheatingError()
        elif error & ERRBIT_RANGE:
            raise DxlRangeError()
        elif error & ERRBIT_CHECKSUM:
            raise DxlChecksumError()
        elif error & ERRBIT_OVERLOAD:
            raise DxlOverloadError()
        elif error & ERRBIT_INSTRUCTION:
            raise DxlInstructionError()
    
    
    def getModelNumber(self):
        return self.readNByteTxRx(self.CTRL_MODEL_NUMBER)
        
    def getFirmwareVersion(self):
        return self.readNByteTxRx(self.CTRL_FIRMWARE_VERSION)
        
    def getId(self):
        return self.readNByteTxRx(self.CTRL_ID)
    
    def getBaudRate(self):
        return self.readNByteTxRx(self.CTRL_BAUD_RATE)
    
    def getReturnDelayTime(self):
        return self.readNByteTxRx(self.CTRL_RETURN_DELAY)
    
    def getCwAngleLimit(self, raw=False):
        val = self.readNByteTxRx(self.CTRL_CW_ANGLE_LIMIT)
        
        if not raw:
            val = float(val-self.CENTER_POSITION)*self.UNIT_ANGLE
        
        return val
    
    def getCcwAngleLimit(self, raw=False):
        val = self.readNByteTxRx(self.CTRL_CCW_ANGLE_LIMIT)
        
        if not raw:
            val = float(val-self.CENTER_POSITION)*self.UNIT_ANGLE
        
        return val
    
    def getDriveMode(self):
        return self.readNByteTxRx(self.CTRL_DRIVE_MODE)
    
    
    
    
    def getTorqueEnable(self):
        return self.readNByteTxRx(self.CTRL_TORQUE_ENABLE)
    
    def getLed(self):
        return self.readNByteTxRx(self.CTRL_LED)

    def getGoalPosition(self, raw=False):
        val = self.readNByteTxRx(self.CTRL_GOAL_POSITION)
        
        if not raw:
            val = float(val-self.CENTER_POSITION)*self.UNIT_ANGLE
        
        return val

    def getMovingSpeed(self):
        
        return self.readNByteTxRx(self.CTRL_MOVING_SPEED)
    
    def getTorqueLimit(self):
        return self.readNByteTxRx(self.CTRL_TORQUE_LIMIT)
    
    def getPresentPosition(self, raw=False):
        val = self.readNByteTxRx(self.CTRL_PRESENT_POSITION)
        
        if not raw:
            val = float(val-self.CENTER_POSITION)*self.UNIT_ANGLE
        
        return val
    
    
    def getPresentSpeed(self, raw=False):
        val = self.readNByteTxRx(self.CTRL_PRESENT_SPEED)
        
        if not raw:
            mag = val & 0x3FF
            if val & 0x400:
                mag = -mag
                
            val = float(mag)*self.UNIT_SPEED
        
        return val
    
    def getPresentLoad(self, raw=False):
        val = self.readNByteTxRx(self.CTRL_PRESENT_LOAD)
        
        if not raw:
            mag = val & 0x3FF
            if val & 0x400:
                mag = -mag
                
            val = float(mag)*self.UNIT_SPEED
        
        return val
    
    def getPresentVoltage(self):
        return self.readNByteTxRx(self.CTRL_PRESENT_VOLTAGE)
    
    def getPresentTemperature(self):
        return self.readNByteTxRx(self.CTRL_PRESENT_TEMPERATURE)
    
    def getSensedCurrent(self):
        return self.readNByteTxRx(self.CTRL_SENSED_CURRENT)
    
    
    
    
    def setBaudRate(self, value, raw=False):
        if not raw:
            value = int(round(2000000/value - 1))
            
        self.writeNByteTxRx(self.CTRL_BAUD_RATE, value)
    
    def setCwAngleLimit(self, value, raw=False):
        if not raw:
            self.cw_angle_limit = value
            value = int(value/self.UNIT_ANGLE + self.CENTER_POSITION)
        else:
            self.cw_angle_limit = float(value - self.CENTER_POSITION)*self.UNIT_ANGLE
        
        self.writeNByteTxRx(self.CTRL_CW_ANGLE_LIMIT, value)
    
    def setCcwAngleLimit(self, value, raw=False):
        if not raw:
            self.ccw_angle_limit = value
            value = int(value/self.UNIT_ANGLE + self.CENTER_POSITION)
        else:
            self.ccw_angle_limit = float(value - self.CENTER_POSITION)*self.UNIT_ANGLE
        
        self.writeNByteTxRx(self.CTRL_CCW_ANGLE_LIMIT, value)
    
    
    
    
    def setTorqueEnable(self, value):
        return self.writeNByteTxRx(self.CTRL_TORQUE_ENABLE, value)
    
    def setLED(self, value):
        self.writeNByteTxRx(self.CTRL_LED, value)
    
    def setGoalPosition(self, value, raw=False):
        if not raw:
            value = int(value/self.UNIT_ANGLE + self.CENTER_POSITION)
            
        self.writeNByteTxRx(self.CTRL_GOAL_POSITION, value)
    
    def setMovingSpeed(self, value, raw=False):
        if not raw:
            if self.isJointMode():
                value = int(value/self.UNIT_SPEED)
            else:
                raise NotImplementedError("Wheel mode currently not supported")
        
        self.writeNByteTxRx(self.CTRL_MOVING_SPEED, value)
    
    def setTorqueLimit(self, value, raw=False):
        if not raw:
            if (value < 0 or value > 1):
                raise ValueError("Torque should be between 0.0 and 1.0 when raw=False")
            value = int(value * 1023)
        else:
            if (value < 0 or value > 1023):
                raise ValueError("Torque should be between 0 and 1023 when raw=True")
            
        self.writeNByteTxRx(self.CTRL_TORQUE_LIMIT, value)
        

class EX106(Motor):
    def __init__(self, port_handler, id):
        Motor.__init__(self, port_handler, protocol_version=1.0, id=id)
        
        self.CTRL_MODEL_NUMBER          = [0, 2]
        self.CTRL_FIRMWARE_VERSION      = [2, 1]
        self.CTRL_ID                    = [3, 1]
        self.CTRL_BAUD_RATE             = [4, 1]
        self.CTRL_RETURN_DELAY          = [5, 1]
        self.CTRL_CW_ANGLE_LIMIT        = [6, 2]
        self.CTRL_CCW_ANGLE_LIMIT       = [8, 2]
        self.CTRL_DRIVE_MODE            = [10, 1]
        self.CTRL_MAX_LIMIT_TEMPERATURE = [11, 1]
        self.CTRL_MIN_LIMIT_VOLTAGE     = [12, 1]
        self.CTRL_MAX_LIMIT_VOLTAGE     = [13, 1]
        self.CTRL_MAX_TORQUE            = [14, 2]
        self.CTRL_STATUS_RETURN_LEVEL   = [16, 1]
        self.CTRL_ALARM_LED             = [17, 1]
        self.CTRL_ALARM_SHUTDOWN        = [18, 1]

        self.CTRL_TORQUE_ENABLE         = [24, 1]
        self.CTRL_LED                   = [25, 1]
        self.CTRL_CW_COMP_MARGIN        = [26, 1]
        self.CTRL_CCW_COMP_MARGIN       = [27, 1]
        self.CTRL_CW_COMP_SLOPE         = [28, 1]
        self.CTRL_CCW_COMP_SLOPE        = [29, 1]
        self.CTRL_GOAL_POSITION         = [30, 2]
        self.CTRL_MOVING_SPEED          = [32, 2]
        self.CTRL_TORQUE_LIMIT          = [34, 2]
        self.CTRL_PRESENT_POSITION      = [36, 2]
        self.CTRL_PRESENT_SPEED         = [38, 2]
        self.CTRL_PRESENT_LOAD          = [40, 2]
        self.CTRL_PRESENT_VOLTAGE       = [42, 1]
        self.CTRL_PRESENT_TEMPERATURE   = [43, 1]
        self.CTRL_REGISTERED_INSTRUCTION  = [44, 1]
        self.CTRL_MOVING                = [46, 1]
        self.CTRL_LOCK                  = [47, 1]
        self.CTRL_PUNCH                 = [48, 2]
        self.CTRL_SENSED_CURRENT        = [56, 2]
        
        
        self.UNIT_ANGLE   = 0.06 # degrees per unit
        self.UNIT_CURRENT = 0.01 # amps per unit
        self.UNIT_VOLTAGE = 0.1  # volts per unit
        self.UNIT_SPEED   = 0.111*6 # (deg per second) per unit
        
        self.CENTER_POSITION = 2048
        self.CENTER_CURRENT = 512
        




class LegController():
    def __init__(self, port_handler):
        self.port_handler = port_handler
        self.packet_handler = PacketHandler(1.0)
        
        #self.motor_ids = [10, 11, 20, 21, 30, 31, 40, 41]
        self.motor_ids = [10, 11]
        
        ## Create connection to 8 motors
        self.motors = []
        for id in self.motor_ids:
            self.motors.append( EX106(port_handler, id) )

    
    def enable(self):
        for motor in self.motors:
            print("Starting motor {}".format(motor.id))
            motor.enable(mode="joint", min_angle=-120, max_angle=120)
            motor.setTorqueEnable(True)
            
    def setPositions(self, positions):
        if len(positions) != len(self.motors):
            raise ValueError("Must have exactly {} position values".format(len(self.motors)))
        
        
        params = []
        for i in range(len(self.motors)):
            motor = self.motors[i]
            pos = positions[i]
            pos = int(pos/motor.UNIT_ANGLE + motor.CENTER_POSITION)
            
            params.append(motor.id)
            params.append(pos & 0x00FF)
            params.append((pos & 0xFF00) >> 8)
        
        self.packet_handler.syncWriteTxOnly(self.port_handler, motor.CTRL_GOAL_POSITION[0], motor.CTRL_GOAL_POSITION[1], params, len(params))
        
    def getPositions(self):
        positions = []
        for motor in self.motors:
            positions.append( motor.getPresentPosition() )
            
        
        return positions
        
        



# Default setting
BAUDRATE                    = 1000000             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 0           # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 100            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 0.3                # Dynamixel moving status threshold




if platform.system() == "Linux":
    FNULL = open(os.devnull, 'w')
    
    try:
        ret = subprocess.call(['setserial','/dev/ttyUSB0', 'low_latency'])
    
        if ret != 0:
            print("Error setting latency timer for USB-to-serial device\n\n")
            quit()
        
    except OSError:
        # executable not found
        print("\n\n")
        print("In order to obtain maximum communication speed with USB2DYNAMIXEL, install 'setserial' as follows:")
        print("   sudo apt-get install setserial")
        print("\n\n")
        quit()
    
else:
    print("Non-Linux platforms not currently supported")






# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()




"""
motor = EX106(portHandler, id=11)
motor.setBaudRate(1000000)
quit()
"""


controller = LegController(portHandler)
controller.enable()

t_prev = time.time()
while 1:
    t = time.time()
    
    positions = controller.getPositions()
    print(positions)
    
    theta = 2*pi*t*1
    controller.setPositions([sin(theta)*30, cos(theta)*30+90])
    print("Elapsed: {} ms".format((t - t_prev)*1000))
    t_prev = t
    
    #time.sleep(0.1)








"""
motor = EX106(portHandler, id=20)
motor.enable(mode="joint", min_angle=-120, max_angle=120)



# Enable Dynamixel Torque
motor.setTorqueEnable(True)
print("Dynamixel has been successfully connected")


###
led_state = 1
print("Time to blink")
count = 0

start = time.time()
while 1:
    led_state = 1-led_state
    motor.setLED(led_state)
    
    count+=1
    if count == 1000:
        break
    
interval = (time.time()-start)/1000
print("{}ms per request ({:3.1f} Hz)".format(interval*1000, 1/interval))
quit()
###

index = 0
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position

while 1:
    print("Press any key to continue! (or press ESC to quit!)")
    if getch() == chr(0x1b):
        break

    # Write goal position
    motor.setGoalPosition(dxl_goal_position[index])

    tick=0
    prev_pos = None
    prev_time = time.time()
    while 1:
        # Read present position
        pos = motor.getPresentPosition()
        #goal = motor.getGoalPosition()
        speed = motor.getPresentSpeed()
        
        now = time.time()
        t_elapsed = now - prev_time
        prev_time = now
        
        est_speed = 0
        if prev_pos is not None:
            est_speed = (pos - prev_pos)/t_elapsed
        prev_pos = pos
        
        print("Tick {}".format(tick))
        print("Position Limits: {} to {} deg".format(motor.cw_angle_limit, motor.ccw_angle_limit))
        #print("Goal: {} deg".format(goal))
        print("Position: {} deg".format(pos))
        print("Speed    : {} deg/sec".format(speed))
        print("Est Speed: {} deg/sec".format(est_speed))
        print("Time elapsed: {:4.1f} ms".format(t_elapsed*1000))
        print("")
        
        tick +=1
        
        #led_state = 1-led_state
        #motor.setLED(led_state)
        if not abs(dxl_goal_position[index] - pos) > DXL_MOVING_STATUS_THRESHOLD:
            break
        
        #time.sleep(0.01)

    # Change goal position
    if index == 0:
        index = 1
    else:
        index = 0


# Disable Dynamixel Torque
motor.setTorqueEnable(False)
"""

# Close port
portHandler.closePort()
