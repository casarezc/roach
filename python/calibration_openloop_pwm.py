#!/usr/bin/env python
"""
authors: apullin

This script will run an experiment with one or several Velociroach robots.

The main function will send all the setup parameters to the robots, execute defined manoeuvres, and record telemetry.

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
    print "-------------------------------------"
    print "Open loop PWM control August 30 2017"
    print " w:inc left    e:inc right "
    print " s:dec left    d:dec right "
    print " x:motors off  q: quit"

def main():    
    xb = setupSerial(shared.BS_COMPORT, shared.BS_BAUDRATE)
    
    R1 = Velociroach('\x21\x62', xb)
    # R1.SAVE_DATA = True
    R1.SAVE_DATA = False

    # PWM increment constant
    PWM_INC = 100
    LEFT_SIGN = 1
    RIGHT_SIGN = -1

    # Initialize pwm
    # left_pwm = 3000
    # right_pwm = 0
    left_pwm = 0
    right_pwm = -3000

    # Wait time between commands
    T_WAIT = 500

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

    R1.setOpenLoopThrust(left_pwm, right_pwm)
    R1.startDriveMotors()

    # Initiate telemetry recording; the robot will begin recording immediately when cmd is received.
    for r in shared.ROBOTS:
        if r.SAVE_DATA:
            r.startTelemetrySave()

    exit_flag = False

    while exit_flag == False:
        menu()
        keypress = msvcrt.getch()
        if keypress == 'w':
            left_pwm+=(LEFT_SIGN*PWM_INC)
        elif keypress == 'e':
            right_pwm+=(RIGHT_SIGN*PWM_INC)
        elif keypress == 's':
            left_pwm-=(LEFT_SIGN*PWM_INC)
        elif keypress == 'd':
            right_pwm-=(RIGHT_SIGN*PWM_INC)
        elif keypress == 'x':
            left_pwm = 0
            right_pwm = 0
        elif (keypress == 'q') or (ord(keypress) == 26):
            left_pwm = 0
            right_pwm = 0
            print "Exit."
            exit_flag = True

        # Print current PWM and set it
        R1.setOpenLoopThrust(left_pwm, right_pwm)

        time.sleep(T_WAIT/1000.0)


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
