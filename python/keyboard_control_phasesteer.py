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
    print "Keyboard control October 22 2015"
    print " a:slow left    s:slow straight   d:slow right    w:slow alt tripod   t:fast alt tripod"
    print " f:fast left    g:fast straight   h:fast right    q: quit"

def main():    
    xb = setupSerial(shared.BS_COMPORT, shared.BS_BAUDRATE)
    
    R1 = Velociroach('\x21\x62', xb)
    # R1.SAVE_DATA = True
    R1.SAVE_DATA = False
    PHASE_LEFT = 23666
    PHASE_RIGHT = 41870
    STRIDE_FREQ = 5
                            
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
    motorgains = [3000,200,100,0,200, 3000,200,100,0,200]
    steergains = [1500, 200, 0]

    #simpleAltTripod = GaitConfig(motorgains, rightFreq=0, leftFreq=0) # Parameters can be passed into object upon construction, as done here.
    #simpleBound = GaitConfig(motorgains, rightFreq=5, leftFreq=5)
    #winchPWM = 0

    ## Set up different gaits to be used in the trials

    frequency = 2
    slowAltTripod = GaitConfig(motorgains, rightFreq=frequency, leftFreq=frequency)
    slowAltTripod.phase = PHASE_180_DEG
    slowAltTripod.deltasLeft = [0.25, 0.25, 0.25]
    slowAltTripod.deltasRight = [0.25, 0.25, 0.25]

    steerLeftSlow = SteerConfig(motorgains, steergains, rightFreq = frequency, leftFreq = frequency)
    steerLeftSlow.steerangle = 90

    steerStraightSlow = SteerConfig(motorgains, steergains, rightFreq = frequency, leftFreq = frequency)
    steerStraightSlow.steerangle = 0

    steerRightSlow = SteerConfig(motorgains, steergains, rightFreq = frequency, leftFreq = frequency)
    steerRightSlow.steerangle = -90

    frequency = 5

    fastAltTripod = GaitConfig(motorgains, rightFreq=frequency, leftFreq=frequency)
    fastAltTripod.phase = PHASE_180_DEG
    fastAltTripod.deltasLeft = [0.25, 0.25, 0.25]
    fastAltTripod.deltasRight = [0.25, 0.25, 0.25]

    steerLeftFast = SteerConfig(motorgains, steergains, rightFreq = frequency, leftFreq = frequency)
    steerLeftFast.steerangle = 90

    steerStraightFast = SteerConfig(motorgains, steergains, rightFreq = frequency, leftFreq = frequency)
    steerStraightFast.steerangle = 0

    steerRightFast = SteerConfig(motorgains, steergains, rightFreq = frequency, leftFreq = frequency)
    steerRightFast.steerangle = -90
    
    # Set the timings of each segment of the run
    T = 4000


    # example , 0.1s lead in + 2s run + 0.1s lead out
    EXPERIMENT_SAVE_TIME_MS     = T
    
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

    # Initiate telemetry recording; the robot will begin recording immediately when cmd is received.
    for r in shared.ROBOTS:
        if r.SAVE_DATA:
            r.startTelemetrySave()

    exit_flag = False

    while exit_flag == False:
        menu()
        keypress = msvcrt.getch()
        if keypress == 'a':
            R1.setSteerGait(steerLeftSlow)
            R1.startTimedRun( T )
        elif keypress == 's':
            R1.setSteerGait(steerStraightSlow)
            R1.startTimedRun( T )
        elif keypress == 'd':
            R1.setSteerGait(steerRightSlow)
            R1.startTimedRun( T )
        elif keypress == 'w':
            R1.setGait(slowAltTripod)
            R1.startTimedRun( T )
        elif keypress == 'f':
            R1.setSteerGait(steerLeftFast)
            R1.startTimedRun( T )
        elif keypress == 'g':
            R1.setSteerGait(steerStraightFast)
            R1.startTimedRun( T )
        elif keypress == 'h':
            R1.setSteerGait(steerRightFast)
            R1.startTimedRun( T )
        elif keypress == 't':
            R1.setGait(fastAltTripod)
            R1.startTimedRun( T )
        elif (keypress == 'q') or (ord(keypress) == 26):
            print "Exit."
            exit_flag = True

        time.sleep(T/1000.0)
        R1.stopSteering()

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