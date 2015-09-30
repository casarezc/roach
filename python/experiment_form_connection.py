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
    winchgains = [140, 40, 20, 0] 

    ## Set up different gaits to be used in the trials
    slowBoundTaut = GaitConfig(motorgains, rightFreq=2, leftFreq=2)
    slowBoundTaut.winchgains = winchgains
    slowBoundTaut.phase = 0
    slowBoundTaut.deltasLeft = [0.25, 0.25, 0.25]
    slowBoundTaut.deltasRight = [0.25, 0.25, 0.25]

    slowBoundTaut.winchSetpoint = 1000
    slowBoundTaut.winchMode = 0

    slowBoundConnect = GaitConfig(motorgains, rightFreq=4, leftFreq=4)
    slowBoundConnect.winchgains = winchgains
    slowBoundConnect.phase = 0                          
    slowBoundConnect.deltasLeft = [0.25, 0.25, 0.25]
    slowBoundConnect.deltasRight = [0.25, 0.25, 0.25]

    slowBoundConnect.winchSetpoint = 12000
    slowBoundConnect.winchMode = 0

    holdBack = GaitConfig(motorgains, rightFreq=1, leftFreq=1)
    holdBack.phase = 0                          
    holdBack.deltasLeft = [0.25, 0, 0]
    holdBack.deltasRight = [0.25, 0, 0]

    medBound = GaitConfig(motorgains, rightFreq=8, leftFreq=8)
    medBound.phase = 0                          
    medBound.deltasLeft = [0.25, 0.25, 0.25]
    medBound.deltasRight = [0.25, 0.25, 0.25]

    
    # Set the timings of each segment of the run
    TLEADOUT = 1500
    T1 = 500
    T2 = 500

    # example , 0.1s lead in + 2s run + 0.1s lead out
    EXPERIMENT_SAVE_TIME_MS     = 4*TLEADOUT + T1 + 4*T2
    
    # Some preparation is needed to cleanly save telemetry data
    for r in shared.ROBOTS:
        if r.SAVE_DATA:
            #This needs to be done to prepare the .telemtryData variables in each robot object
            r.setupTelemetryDataTime(EXPERIMENT_SAVE_TIME_MS)
            r.eraseFlashMem()
    
        print ""

    R2.zeroLoadCell()

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


    # nextFlag = 0


    # while(nextFlag == 0):
    #     print "  ***************************"
    #     print "  *******   STAGE 1   *******"
    #     print "  ***************************"
    #     R2.setGait(slowBoundTaut)
    #     R2.startTimedRunWinch( T1 )

    #     nextFlag = int(raw_input(" Move on to stage 2 (1 or 0)?: "))

    nextFlag = 0

    while(nextFlag == 0):
        print "  ***************************"
        print "  *******   STAGE 2  *******"
        print "  ***************************"
        R2.setGait(slowBoundConnect)
        R1.setGait(medBound)
        R2.startTimedRunWinch( T2 )
        R1.startTimedRun( T2 )

        nextFlag = int(raw_input(" Exit (1 or 0)?: "))

    
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
