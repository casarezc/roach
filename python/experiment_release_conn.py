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
    # R1.SAVE_DATA = False
    # R2.SAVE_DATA = False
    R1.SAVE_DATA = True
    R2.SAVE_DATA = True
                            
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
    motorgains = [5000,300,200,0,200, 5000,300,200,0,200]

    # Winch gains format:
    #  [ Kp , Ki , Kaw , Kff ]
    winchgains = [50, 40, 40, 0] 
    #motorgains = [0,0,0,0,0 , 0,0,0,0,0]

    #simpleAltTripod = GaitConfig(motorgains, rightFreq=0, leftFreq=0) # Parameters can be passed into object upon construction, as done here.
    #simpleBound = GaitConfig(motorgains, rightFreq=5, leftFreq=5)
    #winchPWM = 0

    ## Set up different gaits to be used in the trials
    slowBound = GaitConfig(motorgains, rightFreq=2, leftFreq=2)
    slowBound.winchgains = winchgains
    slowBound.phase = 0
    slowBound.deltasLeft = [0.25, 0.25, 0.25]
    slowBound.deltasRight = [0.25, 0.25, 0.25]

    slowBound.winchSetpoint = 7000
    slowBound.winchMode = 0

    fastBound = GaitConfig(motorgains, rightFreq=5, leftFreq=5)
    fastBound.winchgains = winchgains
    fastBound.phase = 0
    fastBound.deltasLeft = [0.25, 0.25, 0.25]
    fastBound.deltasRight = [0.25, 0.25, 0.25]


    fastBackwardBound = GaitConfig(motorgains, rightFreq=-5, leftFreq=-5)
    fastBackwardBound.phase = 0
    fastBackwardBound.deltasLeft = [0.25, 0.25, 0.25]
    fastBackwardBound.deltasRight = [0.25, 0.25, 0.25]

    slowAltTripod = GaitConfig(motorgains, rightFreq=2, leftFreq=2)
    slowAltTripod.phase = PHASE_180_DEG                          
    slowAltTripod.deltasLeft = [0.25, 0.25, 0.25]
    slowAltTripod.deltasRight = [0.25, 0.25, 0.25]

    fastAltTripod = GaitConfig(motorgains, rightFreq=4, leftFreq=4)
    fastAltTripod.phase = PHASE_180_DEG                           
    fastAltTripod.deltasLeft = [0.25, 0.25, 0.25]
    fastAltTripod.deltasRight = [0.25, 0.25, 0.25]

    holdCenter = GaitConfig(motorgains, rightFreq=2, leftFreq=2)
    holdCenter.winchgains = winchgains
    holdCenter.phase = 0                          
    holdCenter.deltasLeft = [0, 0, 0]
    holdCenter.deltasRight = [0, 0, 0]

    holdCenter.winchSetpoint = 3000
    holdCenter.winchMode = 1

    holdBack = GaitConfig(motorgains, rightFreq=2, leftFreq=2)
    holdBack.winchgains = winchgains
    holdBack.phase = 0                          
    holdBack.deltasLeft = [0.25, 0, 0]
    holdBack.deltasRight = [0.25, 0, 0]

    holdBackLong = GaitConfig(motorgains, rightFreq=1, leftFreq=1)
    holdBackLong.winchgains = winchgains
    holdBackLong.phase = 0                          
    holdBackLong.deltasLeft = [0.25, 0, 0]
    holdBackLong.deltasRight = [0.25, 0, 0]




    
    # Set the timings of each segment of the run
    TLEADOUT = 100
    T1 = 100
    T2 = 100

    STOP_ANGLE_2 = -20
    ANGLE_TRIGGER_2 = 2

    # example , 0.1s lead in + 2s run + 0.1s lead out
    EXPERIMENT_SAVE_TIME_MS     = 2*TLEADOUT + T1 + T2
    
    # Some preparation is needed to cleanly save telemetry data
    for r in shared.ROBOTS:
        if r.SAVE_DATA:
            #This needs to be done to prepare the .telemtryData variables in each robot object
            r.setupTelemetryDataTime(EXPERIMENT_SAVE_TIME_MS)
            r.eraseFlashMem()
    
        print ""


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


    nextFlag = 0

    R2.zeroLoadCell()

    while(nextFlag == 0):
        print "  ***************************"
        print "  *******   STAGE 1   *******"
        print "  ***************************"
        # R2.setPitchThresh(STOP_ANGLE, ANGLE_TRIGGER);
        R2.setGait(slowBound)
        R1.setGait(holdBackLong)
        R2.startTimedRunWinch( T1 )
        R1.startTimedWun( T1 )

        nextFlag = int(raw_input(" Move on to stage 2 (1 or 0)?: "))

    nextFlag = 0

    while(nextFlag == 0):
        print "  ***************************"
        print "  *******   STAGE 2  *******"
        print "  ***************************"
        R2.setGait(holdCenter)
        R1.setGait(fastBound)
        R1.setPitchThresh(STOP_ANGLE_2, ANGLE_TRIGGER_2)
        R1.startTimedRun( T2 )
        R2.startTimedRunWinchTorque( T2)

        nextFlag = int(raw_input(" Exit (1 or 0)?: "))

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
