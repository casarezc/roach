#!/usr/bin/env python
"""
authors: apullin

This script will run an experiment with one or several Velociroach robots.

The main function will send all the setup parameters to the robots, execute defined manoeuvres, and record telemetry.

"""
from lib import command
import time,sys,os,traceback
import serial

# Path to imageproc-settings repo must be added
sys.path.append(os.path.dirname("../../imageproc-settings/"))
sys.path.append(os.path.dirname("../imageproc-settings/"))  
import shared_multi as shared

from velociroach import *

####### Wait at exit? #######
EXIT_WAIT   = False

def main():    
    xb = setupSerial(shared.BS_COMPORT, shared.BS_BAUDRATE)
    
    R1 = Velociroach('\x21\x62', xb)
    R2 = Velociroach('\x21\x63', xb)
    R1.SAVE_DATA = False
    R2.SAVE_DATA = False
    # R1.SAVE_DATA = True
    # R2.SAVE_DATA = True
                            
    #R1.RESET = False       #current roach code does not support software reset
    
    shared.ROBOTS.append(R1) #This is necessary so callbackfunc can reference robots
    shared.ROBOTS.append(R2)
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
    motorgains = [5000,400,200,0,200, 5000,400,200,0,200]

    # Winch gains format:
    #  [ Kp , Ki , Kaw , Kff ]
    winchgains = [60, 30, 10, 0] 

    ## Set up different gaits to be used in the trials

    holdCenterConnect = GaitConfig(motorgains, rightFreq=1, leftFreq=1)
    holdCenterConnect.winchgains = winchgains
    holdCenterConnect.phase = 0                          
    holdCenterConnect.deltasLeft = [1, 0, 0]
    holdCenterConnect.deltasRight = [1, 0, 0]

    holdCenterConnect.winchSetpoint = 12500
    holdCenterConnect.winchMode = 0

    slowBoundRelease = GaitConfig(motorgains, rightFreq=8, leftFreq=8)
    slowBoundRelease.winchgains = winchgains
    slowBoundRelease.phase = 0                          
    slowBoundRelease.deltasLeft = [0.25, 0.25, 0.25]
    slowBoundRelease.deltasRight = [0.25, 0.25, 0.25]

    slowBoundRelease.winchSetpoint = 5000
    slowBoundRelease.winchMode = 1

    holdCenterRelease = GaitConfig(motorgains, rightFreq=1, leftFreq=1)
    holdCenterRelease.winchgains = winchgains
    holdCenterRelease.phase = 0                          
    holdCenterRelease.deltasLeft = [0.25, 0, 0]
    holdCenterRelease.deltasRight = [0.25, 0, 0]

    holdCenterRelease.winchSetpoint = 220
    holdCenterRelease.winchMode = 1

    r1Bound = GaitConfig(motorgains, rightFreq=4, leftFreq=4)
    r1Bound.phase = 0
    r1Bound.deltasLeft = [0.25, 0.25, 0.25]
    r1Bound.deltasRight = [0.25, 0.25, 0.25]

    holdBack = GaitConfig(motorgains, rightFreq=1, leftFreq=1)
    holdBack.phase = 0                          
    holdBack.deltasLeft = [0.25, 0, 0]
    holdBack.deltasRight = [0.25, 0, 0]

    
    # Set the timings of each segment of the run
    TPREP = 2000
    TLEADOUT = 2000
    T1 = 2000
    T2 = 3000

    # Set angle trigger parameters
    STOP_ANGLE_1 = -20
    ANGLE_TRIGGER_1 = 2

    STOP_ANGLE_2 = 0
    ANGLE_TRIGGER_2 = 0

    R1.setPitchThresh(STOP_ANGLE_1, ANGLE_TRIGGER_1)
    R2.setPitchThresh(STOP_ANGLE_2, ANGLE_TRIGGER_2)

    # example , 0.1s lead in + 2s run + 0.1s lead out
    EXPERIMENT_SAVE_TIME_MS     = TLEADOUT + 2*T1
    
    # Some preparation is needed to cleanly save telemetry data
    for r in shared.ROBOTS:
        if r.SAVE_DATA:
            #This needs to be done to prepare the .telemtryData variables in each robot object
            r.setupTelemetryDataTime(EXPERIMENT_SAVE_TIME_MS)
            r.eraseFlashMem()
    
        print ""


    # nextFlag = 0

    # R2.zeroLoadCell()

    # while(nextFlag == 0):
    #     print "  ***************************"
    #     print "  *********   PREP   ********"
    #     print "  ***************************"
    #     R2.setGait(holdCenterConnect)
    #     R2.startTimedRunWinch( TPREP )

    #     nextFlag = int(raw_input(" Move on to stage 1 (1 or 0)?: "))

    nextFlag = 0

    print "  ***************************"
    print "  *******    READY    *******"
    print "  ***************************"
    raw_input("  Press ENTER to start run ...")
    print ""

    # Initiate telemetry recording; the robot will begin recording immediately when cmd is received.
    for r in shared.ROBOTS:
        if r.SAVE_DATA:
            r.startTelemetrySave()

    time.sleep(0.1)


    while(nextFlag == 0):
        print "  ***************************"
        print "  *******   STAGE 1   *******"
        print "  ***************************"
        # R1.setGait(r1Bound)
        R2.setGait(slowBoundRelease)
        R2.startTimedRunWinch( T1 )
        # R1.startTimedRun( T1 )

        nextFlag = int(raw_input(" Move to stage 2 (1 or 0)?: "))

    # nextFlag  = 0
    # # R2.zeroLoadCell()
    # while(nextFlag == 0):
    #     print "  ***************************"
    #     print "  *******   STAGE 2   *******"
    #     print "  ***************************"
    #     R1.setGait(r1Bound)
    #     # R2.setGait(slowBoundRelease)
    #     R2.setGait(holdCenterRelease)
    #     R2.startTimedRunWinch( T2 )
    #     R1.startTimedRun( T2 )

    #     nextFlag = int(raw_input(" Exit (1 or 0)?: "))


    ## Save data after runs
    for r in shared.ROBOTS:
        retry_flag = 1
        if r.SAVE_DATA:
            while(retry_flag == 1):
                raw_input("Press Enter to start telemetry read-back ...")
                r.downloadTelemetry()
                retry_flag = int(raw_input(" Retry (1 or 0)?: "))
    
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
