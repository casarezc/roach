#!/usr/bin/env python
"""
authors: C. Casarez
This script will run an CL keyboard control of a robot with drive motors (left, right) on motor channels A,B 
of the imageProc and tail motor on motor channel C, all with encoders.
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
    print "Keyboard control tail position August 24 2016"
    print " w:+speed      s:-speed      a:+left       d:+right  "
    print " z:zero tail   x:tail ccw+   c:tail cw+    v:tail off"  
    print " f:drive motors off   g:all motors off     q:quit"
    print " r:start data save"
    print "---------------------------------------------------------------------------"


def main():    
    ### Change these parameters to adjust how the robot is controlled and how long you want to save data ###
    # Set the save time of the run
    EXPERIMENT_SAVE_TIME_MS     = 10000

    # Set constants for setting control
    TAIL_ANGLE_INC = 180 #degrees
    DRIVE_FREQ_INC = 1  #Hz
    TAIL_POS_MAX = 359 #degrees
    ########################################################################################################

    xb = setupSerial(shared.BS_COMPORT, shared.BS_BAUDRATE)
    

    R1 = Velociroach('\x21\x62', xb)

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

    # Motor gains format:
    #  [ Kp , Ki , Kd , Kaw , Kff     ,  Kp , Ki , Kd , Kaw , Kff ]
    #    ----------LEFT----------        ---------_RIGHT----------
    motorgains = [3000,300,200,30000,200, 3000,300,200,30000,200]

    # Tail gains format:
    #  [ Kp , Ki , Kd , Kaw , Kff]
    tailgains = [500,200,10,500,0]

    # Zero motors and set gains, fixed gait parameters
    R1.zeroPosition()
    R1.setMotorGains(motorgains)
    R1.setPhase(0)

    R1.zeroTailPosition()
    R1.setTailGains(tailgains)

    # Initialize variable states 
    left_freq = 0
    right_freq = 0
    tail_pos = 0
    tail_on = False
    
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
            if (left_freq == 0)&(right_freq == 0):
                R1.startDriveMotors()
            left_freq+=DRIVE_FREQ_INC
            right_freq+=DRIVE_FREQ_INC
        elif keypress == 's':
            left_freq-=DRIVE_FREQ_INC
            right_freq-=DRIVE_FREQ_INC
            if (left_freq <= 0)&(right_freq <= 0):
                R1.stopDriveMotors()
                left_freq = 0
                right_freq = 0
        elif keypress == 'a':
            if (left_freq == 0)&(right_freq == 0):
                R1.startDriveMotors()
            left_freq-=DRIVE_FREQ_INC
            right_freq+=DRIVE_FREQ_INC
        elif keypress == 'd':
            if (left_freq == 0)&(right_freq == 0):
                R1.startDriveMotors()
            left_freq+=DRIVE_FREQ_INC
            right_freq-=DRIVE_FREQ_INC
        elif keypress == 'z':
            R1.zeroTailPosition()
            tail_pos = 0
        elif keypress == 'x':
            if tail_on is False:
                R1.startTail()
                tail_on = True
            tail_pos+=TAIL_ANGLE_INC
            if tail_pos>=TAIL_POS_MAX:
                tail_pos = TAIL_POS_MAX
        elif keypress == 'c':
            if tail_on is False:
                R1.startTail()
                tail_on = True
            tail_pos-=TAIL_ANGLE_INC
            if tail_pos<=-TAIL_POS_MAX:
                tail_pos = -TAIL_POS_MAX
        elif keypress == 'v':
            tail_on = False
            R1.stopTail()
        elif keypress == 'f':
            left_freq = 0
            right_freq = 0
            R1.stopDriveMotors()
        elif keypress == 'g':
            tail_on = False
            R1.stopTail()
            left_freq = 0
            right_freq = 0
            R1.stopDriveMotors()
        elif keypress == 'r':
            tail_on = False
            R1.stopTail()
            left_freq = 0
            right_freq = 0
            R1.stopDriveMotors()
            # Initiate telemetry recording; the robot will begin recording immediately when cmd is received.
            for r in shared.ROBOTS:
                if r.SAVE_DATA:
                    r.startTelemetrySave()
                else:
                    print "Data saving disabled"
        elif (keypress == 'q') or (ord(keypress) == 26):
            R1.stopTail()
            R1.stopDriveMotors()
            print "Exit."
            exit_flag = True

        if (keypress == 'w') or (keypress == 's') or (keypress == 'a') or (keypress == 'd') or (keypress == 'f') or (keypress == 'g'):
            tempGait = GaitConfig(motorgains, rightFreq=right_freq, leftFreq=left_freq)
            tempGait.deltasLeft = [0.25, 0.25, 0.25]
            tempGait.deltasRight = [0.25, 0.25, 0.25]
            R1.setPhase(0)
            R1.setVelProfile(tempGait)

        R1.setTailPos(tail_pos)

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