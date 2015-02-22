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
    
    R1 = Velociroach('\x21\x63', xb)
    R2 = Velociroach('\x21\x62', xb)
    R1.SAVE_DATA = False
                            
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
    motorgains = [4000,100,200,0,200, 4000,100,200,0,200]
    #motorgains = [0,0,0,0,0 , 0,0,0,0,0]

    #simpleAltTripod = GaitConfig(motorgains, rightFreq=0, leftFreq=0) # Parameters can be passed into object upon construction, as done here.
    #simpleBound = GaitConfig(motorgains, rightFreq=5, leftFreq=5)
    #winchPWM = 0

    ## Set up different gaits to be used in the trials
    slowBound = GaitConfig(motorgains, rightFreq=2, leftFreq=2)
    slowBound.phase = 0
    slowBound.deltasLeft = [0.25, 0.25, 0.25]
    slowBound.deltasRight = [0.25, 0.25, 0.25]

    fastBound = GaitConfig(motorgains, rightFreq=5, leftFreq=5)
    fastBound.phase = 0
    fastBound.deltasLeft = [0.125, 0.125, 0.25]
    fastBound.deltasRight = [0.125, 0.125, 0.25]

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
    holdCenter.phase = 0                          
    holdCenter.deltasLeft = [0, 0, 0]
    holdCenter.deltasRight = [0, 0, 0]

    holdBack = GaitConfig(motorgains, rightFreq=2, leftFreq=2)
    holdBack.phase = 0                          
    holdBack.deltasLeft = [0.25, 0, 0]
    holdBack.deltasRight = [0.25, 0, 0]

    holdBackLong = GaitConfig(motorgains, rightFreq=1, leftFreq=1)
    holdBackLong.phase = 0                          
    holdBackLong.deltasLeft = [0.25, 0, 0]
    holdBackLong.deltasRight = [0.25, 0, 0]


    
    # Set the timings of each segment of the run
    T1 = 500
    T2 = 500
    T3 = 400
    T4 = 2000
    T5 = 1000
    T6 = 1000
    T7 = 1000

    # Set the winch PWM of each segment of the run
    winchPWM2 = 1500
    winchPWM3 = 3000
    winchPWM5 = -2000
    winchPWM6 = -2000
    winchPWM7 = 2500

    # example , 0.1s lead in + 2s run + 0.1s lead out
    EXPERIMENT_SAVE_TIME_MS     = (T1 + T2 + T3 + T4 + T5 + T6 + T7) + 3000
    
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

    while(nextFlag == 0):
        print "  ***************************"
        print "  *******   STAGE 1   *******"
        print "  ***************************"
        R1.setGait(fastBound)
        R1.startTimedRun( T1 )

        nextFlag = int(raw_input(" Move on to stage 2 (1 or 0)?: "))

    nextFlag = 0

    while(nextFlag == 0):
        print "  ***************************"
        print "  *******   STAGE 2   *******"
        print "  ***************************"
        R2.setGait(slowBound)
        R2.startTimedRunWinch( T2 , winchPWM2)

        nextFlag = int(raw_input(" Move on to stage 3 (1 or 0)?: "))

    nextFlag = 0

    while(nextFlag == 0):
        print "  ***************************"
        print "  *******   STAGE 3   *******"
        print "  ***************************"
        R1.setGait(holdBack)
        R2.setGait(holdCenter)
        R1.startTimedRun( T3 )
        R2.startTimedRunWinch( T3 , winchPWM3)

        nextFlag = int(raw_input(" Move on to stage 4 (1 or 0)?: "))

    nextFlag = 0

    while(nextFlag == 0):
        print "  ***************************"
        print "  *******   STAGE 4   *******"
        print "  ***************************"
        R1.setGait(fastBound)
        R2.setGait(fastBound)
        R1.startTimedRun( T4 )
        R2.startTimedRun( T4 )

        nextFlag = int(raw_input(" Move on to stage 5 (1 or 0)?: "))

    nextFlag = 0

    while(nextFlag == 0):
        print "  ***************************"
        print "  *******   STAGE 5   *******"
        print "  ***************************"
        R1.setGait(fastBound)
        R2.setGait(holdBackLong)
        R1.startTimedRun( T5 )
        R2.startTimedRunWinch( T5 , winchPWM5 )

        nextFlag = int(raw_input(" Move on to stage 6 (1 or 0)?: "))

    nextFlag = 0

    while(nextFlag == 0):
        print "  ***************************"
        print "  *******   STAGE 6   *******"
        print "  ***************************"
        R1.setGait(slowAltTripod)
        R2.setGait(holdBackLong)
        R1.startTimedRun( T6 )
        R2.startTimedRunWinch( T6 , winchPWM6)

        nextFlag = int(raw_input(" Move on to stage 7 (1 or 0)?: "))

    nextFlag = 0

    while(nextFlag == 0):
        print "  ***************************"
        print "  *******   STAGE 7   *******"
        print "  ***************************"
        R1.setGait(holdCenter)
        R2.setGait(fastBound)
        R1.startTimedRun( T7 )
        R2.startTimedRunWinch( T7 , winchPWM7)

        nextFlag = int(raw_input(" End experiment (1 or 0)?: ")) 

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
