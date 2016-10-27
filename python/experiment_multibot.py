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
    # R1.SAVE_DATA = False
    R1.SAVE_DATA = True
                            
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
    motorgains = [5000,1000,100,100,200, 5000,1000,100,100,200]
    # motorgains = [0,300,0,30000,0, 0,0,0,0,0] Note: with full error, unsaturates in 10-15 ms

    ## Set up different gaits to be used in the trials
    slowBound = GaitConfig(motorgains, rightFreq=2, leftFreq=2)
    slowBound.phase = 0
    slowBound.deltasLeft = [0.25, 0.25, 0.25]
    slowBound.deltasRight = [0.25, 0.25, 0.25]

    fastBound = GaitConfig(motorgains, rightFreq=5, leftFreq=5)
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

    fastAltTripod = GaitConfig(motorgains, rightFreq=8, leftFreq=8)
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


    
    # Configure intra-stride control
    R1.setGait(slowAltTripod)
    # R1.setGait(fastAltTripod)

    # example , 0.1s lead in + 2s run + 0.1s lead out
    EXPERIMENT_RUN_TIME_MS     = 3100 #ms
    EXPERIMENT_LEADIN_TIME_MS  = 500  #ms
    EXPERIMENT_LEADOUT_TIME_MS = 200  #ms
    
    # Some preparation is needed to cleanly save telemetry data
    for r in shared.ROBOTS:
        if r.SAVE_DATA:
            #This needs to be done to prepare the .telemtryData variables in each robot object
            r.setupTelemetryDataTime(EXPERIMENT_LEADIN_TIME_MS + EXPERIMENT_RUN_TIME_MS + EXPERIMENT_LEADOUT_TIME_MS)
            r.eraseFlashMem()
        
    # Pause and wait to start run, including lead-in time
    print ""
    print "  ***************************"
    print "  *******    READY    *******"
    print "  ***************************"
    raw_input("  Press ENTER to start run ...")
    print ""

    # Initiate telemetry recording; the robot will begin recording immediately when cmd is received.
    for r in shared.ROBOTS:
        if r.SAVE_DATA:
            r.startTelemetrySave()
    
    # Sleep for a lead-in time before any motion commands
    time.sleep(EXPERIMENT_LEADIN_TIME_MS / 1000.0)
    
    ######## Motion is initiated here! ########
    # R1.startTimedRun( EXPERIMENT_RUN_TIME_MS ) #Faked for now, since pullin doesn't have a working VR+AMS to test with
    time.sleep(EXPERIMENT_RUN_TIME_MS / 1000.0)  #argument to time.sleep is in SECONDS
    ######## End of motion commands   ########
    
    # Sleep for a lead-out time after any motion
    time.sleep(EXPERIMENT_LEADOUT_TIME_MS / 1000.0) 
    
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
