#!/usr/bin/env python
"""
authors: C. Casarez

This script will run an open loop PWM setting keyboard control of a robot with drive motors and (left, right) on motor channels A,B 
of the imageProc and jumper motor on motor channel C. There 

"""
from lib import command
import msvcrt, sys
import time,sys,os,traceback
import serial

# Path to imageproc-settings repo must be added
sys.path.append(os.path.dirname("../../imageproc-settings/"))
sys.path.append(os.path.dirname("../imageproc-settings/"))  
import shared_multi as shared

from velociroach import *

####### Wait at exit? #######
EXIT_WAIT   = False

def menu():
    print "---------------------------------------------------------------------------"
    print "Keyboard control June 26 2015"
    print " w:+speed    s:-speed   a:+left  d:+right"
    print " z:wind jumper    x:unwind jumper  c:jumper off  v: all motors off q: quit"
    print " r: start data save"
    print "---------------------------------------------------------------------------"


def main():    
    ### Change these parameters to adjust how the robot is controlled and how long you want to save data ###
    # Set the save time of the run
    EXPERIMENT_SAVE_TIME_MS     = 10000

    # Set constants for setting various PWMs
    WIND_PWM = 3000
    UNWIND_PWM = -3000
    DRIVE_PWM_INTERVAL = 200
    ########################################################################################################

    xb = setupSerial(shared.BS_COMPORT, shared.BS_BAUDRATE)
    

    R1 = Velociroach('\x21\x66', xb)

    ################################ Toggle data saving flag here ##########################################
    #R1.SAVE_DATA = True
    R1.SAVE_DATA = False
    ########################################################################################################
                            
    #R1.RESET = False       #current roach code does not support software reset
    
    shared.ROBOTS.append(R1) #This is necessary so callbackfunc can reference robots
    shared.xb = xb           #This is necessary so callbackfunc can halt before exit

    # Send resets
    for r in shared.ROBOTS:
        if r.RESET:
            r.reset()
            time.sleep(0.35)
    # Repeat this for other robots
    # TODO: move reset / telem flags inside robot class? (pullin)
    
    # Send robot a WHO_AM_I command, verify communications
    for r in shared.ROBOTS:
        r.query(retries = 3)
    
    #Verify all robots can be queried
    verifyAllQueried()  # exits on failure

    # Initialize variable states 
    left_PWM = 0
    right_PWM = 0
    jump_PWM = 0
    
    # Some preparation is needed to cleanly save telemetry data
    for r in shared.ROBOTS:
        if r.SAVE_DATA:
            #This needs to be done to prepare the .telemtryData variables in each robot object
            r.setupTelemetryDataTime(EXPERIMENT_SAVE_TIME_MS)
            r.eraseFlashMem()
    
        print ""

    print "  ***************************"
    print "  *******    READY    *******"
    print "  ***************************"
    raw_input("  Press ENTER to start run ...")
    print ""

    exit_flag = False

    while exit_flag == False:
        menu()
        keypress = msvcrt.getch()
        if keypress == 'w':
            if (left_PWM == 0)&(right_PWM == 0):
                R1.startDriveMotors()
            left_PWM+=DRIVE_PWM_INTERVAL
            right_PWM+=DRIVE_PWM_INTERVAL
        elif keypress == 's':
            left_PWM-=DRIVE_PWM_INTERVAL
            right_PWM-=DRIVE_PWM_INTERVAL
            if (left_PWM <= 0)&(right_PWM <= 0):
                R1.stopDriveMotors()
                left_PWM = 0
                right_PWM = 0
        elif keypress == 'a':
            if (left_PWM == 0)&(right_PWM == 0):
                R1.startDriveMotors()
            left_PWM-=DRIVE_PWM_INTERVAL
            right_PWM+=DRIVE_PWM_INTERVAL
        elif keypress == 'd':
            if (left_PWM == 0)&(right_PWM == 0):
                R1.startDriveMotors()
            left_PWM+=DRIVE_PWM_INTERVAL
            right_PWM-=DRIVE_PWM_INTERVAL
        elif keypress == 'z':
            if jump_PWM == 0:
                R1.startJumperMotor()
            jump_PWM = WIND_PWM
        elif keypress == 'x':
            if jump_PWM == 0:
                R1.startJumperMotor()
            jump_PWM = UNWIND_PWM
        elif keypress == 'c':
            R1.stopJumperMotor()
            jump_PWM = 0
        elif keypress == 'v':
            R1.stopJumperMotor()
            R1.stopDriveMotors()
            jump_PWM = 0
            left_PWM = 0
            right_PWM = 0
        elif keypress == 'r':
            # Initiate telemetry recording; the robot will begin recording immediately when cmd is received.
            for r in shared.ROBOTS:
                if r.SAVE_DATA:
                    r.startTelemetrySave()
                else:
                    print "Data saving disabled"
        elif (keypress == 'q') or (ord(keypress) == 26):
            R1.stopJumperMotor()
            R1.stopDriveMotors()
            print "Exit."
            exit_flag = True

        if jump_PWM > 4000:
            jump_PWM  = 4000
        if jump_PWM < -4000:
            jump_PWM  = -4000
        if left_PWM > 4000:
            left_PWM  = 4000
        if left_PWM < -4000:
            left_PWM  = -4000
        if right_PWM > 4000:
            right_PWM  = 4000
        if right_PWM < -4000:
            right_PWM  = -4000
        R1.setJumperPWM(jump_PWM)
        R1.setDrivePWM(left_PWM, right_PWM)

        time.sleep(0.2)

    time.sleep(0.1)

    ## Save data after runs
    for r in shared.ROBOTS:
        if r.SAVE_DATA:
            raw_input("Press Enter to start telemetry read-back ...")
            r.downloadTelemetry()
    
    if EXIT_WAIT:  #Pause for a Ctrl + C , if desired
        while True:
            time.sleep(0.1)

    print "Done"
    
#Provide a try-except over the whole main function
# for clean exit. The Xbee module should have better
# provisions for handling a clean exit, but it doesn't.
#TODO: provide a more informative exit here; stack trace, exception type, etc
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print "\nRecieved Ctrl+C, exiting."
    except Exception as args:
        print "\nGeneral exception from main:\n",args,'\n'
        print "\n    ******    TRACEBACK    ******    "
        traceback.print_exc()
        print "    *****************************    \n"
        print "Attempting to exit cleanly..."
    finally:
        xb_safe_exit(shared.xb)
