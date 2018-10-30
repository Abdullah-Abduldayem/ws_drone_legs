import ssc32
import time
import sys
import yaml

import keyboard #Using module keyboard

path_config = "../yaml/servo_calibration.yaml"




servo_names = []
for leg in ["fl", "fr", "bl", "br"]:
    servo_names.append(leg + "_base")
    servo_names.append(leg + "_thigh")
    servo_names.append(leg + "_toe")



if path_config:
    ssc = ssc32.SSC32(config=path_config)
    print('Version', ssc.get_firmware_version())
    ssc.is_done()
    
    
else:
    ssc = ssc32.SSC32("/dev/ttyUSB0", 115200, count=32)
    
    ssc.description = "Calibrated settings for the drone legs"
    
    print('Version', ssc.get_firmware_version())
    
    ssc[0].name = "fl_base"
    ssc[1].name = "fl_thigh"
    ssc[2].name = "fl_toe"
    
    ssc[8].name = "bl_base"
    ssc[9].name = "bl_thigh"
    ssc[10].name = "bl_toe"
    
    ssc[16].name = "fr_base"
    ssc[17].name = "fr_thigh"
    ssc[18].name = "fr_toe"
    
    ssc[24].name = "br_base"
    ssc[25].name = "br_thigh"
    ssc[26].name = "br_toe"
    
    ## Remove unnamed servos from controller
    count = 0
    while count < len(ssc._servos):
        s = ssc._servos[count]
        if s.name is None:
            del( ssc._servos[count] )
        else:
            count += 1

def MoveServoWithKeyboard(servo):
    pos = 1500 #servo.pwm_center
    servo.position = pos
    ssc.commit(1000)
    ssc.wait_for_movement_completion()
    
    while True:
        changed = False
        
        if keyboard.is_pressed('enter'):
            break#finishing the loop
        
        if keyboard.is_pressed('q'):
            pos += 20
            changed = True
        elif keyboard.is_pressed('w'):
            pos -= 20
            changed = True
        elif keyboard.is_pressed('a'):
            pos += 1
            changed = True
        elif keyboard.is_pressed('s'):
            pos -= 1
            changed = True
            
        if (changed):
            servo.position = pos
            ssc.commit(0)
            ssc.wait_for_movement_completion()
            
            #print(pos, ssc.query_pulse_width(servo), servo.min, servo.max )
            
        time.sleep(0.10)
        
    return pos



base = 0
thigh = 45
toe = 90

ssc["fl_base"].degrees = base
ssc["fr_base"].degrees = base
ssc["bl_base"].degrees = base
ssc["br_base"].degrees = base

ssc["fl_thigh"].degrees = thigh
ssc["fr_thigh"].degrees = thigh
ssc["bl_thigh"].degrees = thigh
ssc["br_thigh"].degrees = thigh

ssc["fl_toe"].degrees = toe
ssc["fr_toe"].degrees = toe
ssc["bl_toe"].degrees = toe
ssc["br_toe"].degrees = toe

ssc.commit(3000)
ssc.wait_for_movement_completion()

print(ssc)


print("[Q][W] for course movements, [A][S] for fine movements. [ENTER] when done.")
for i in range(0, len(servo_names)):
    servo_name = servo_names[i]
    servo = ssc[servo_name]
    
    print(servo.pwm_center, servo.pwm_per_degree)
    
    print("Adjusting {}. Moving to neutral position".format(servo_name))
    print("Set to zero degrees.")
    pos1 = MoveServoWithKeyboard(servo)
    
    print(pos1)
    
    """
    ## Python2
    #angle = raw_input('Enter your input:')
    
    time.sleep(0.5)
    a1 = input('What is the angle that this makes in degrees?')
    a1 = input('What is the angle that this makes in degrees?')
    a1 = float(a1)
    
    time.sleep(0.5)
    print("Set to some known angle")
    pos2 = MoveServoWithKeyboard(servo)
    
    time.sleep(0.5)
    a2 = input('What is the angle that this makes in degrees?')
    a2 = input('What is the angle that this makes in degrees?')
    a2 = float(a2)
    
    pwm_per_degree = (pos1 - pos2)/(a1 - a2)
    pwm_center = int(pos1 - a1*pwm_per_degree)
    
    print(pwm_center, pwm_per_degree)
    
    time.sleep(0.5)
    print("Move to max angle")
    pos_max = MoveServoWithKeyboard(servo)
    
    time.sleep(0.5)
    print("Move to min angle")
    pos_min = MoveServoWithKeyboard(servo)
    
    
    print(pos1, a1, pos2, a2, pos_max, pos_min)
    """
    
    time.sleep(0.5)