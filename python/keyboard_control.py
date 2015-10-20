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
    print "Keyboard control April 28 2015"
    print " a:slow left    s:slow tripod   d:slow right    w:slow bound   t:fast bound"
    print " f:fast left    g:fast tripod   h:fast right    q: quit"

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

    # Winch gains format:
    #  [ Kp , Ki , Kaw , Kff ]
    winchgains = [30, 20, 20, 0] 
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

    fastBound = GaitConfig(motorgains, rightFreq=6, leftFreq=6)
    fastBound.winchgains = winchgains
    fastBound.phase = 0
    fastBound.deltasLeft = [0.25, 0.25, 0.25]
    fastBound.deltasRight = [0.25, 0.25, 0.25]

    slowAltTripod = GaitConfig(motorgains, rightFreq=STRIDE_FREQ, leftFreq=STRIDE_FREQ)
    slowAltTripod.phase = PHASE_180_DEG                          
    slowAltTripod.deltasLeft = [0.25, 0.25, 0.25]
    slowAltTripod.deltasRight = [0.25, 0.25, 0.25]

    fastAltTripod = GaitConfig(motorgains, rightFreq=6, leftFreq=6)
    fastAltTripod.phase = PHASE_180_DEG                           
    fastAltTripod.deltasLeft = [0.25, 0.25, 0.25]
    fastAltTripod.deltasRight = [0.25, 0.25, 0.25]

    slowRightTurn = GaitConfig(motorgains, rightFreq=STRIDE_FREQ, leftFreq=STRIDE_FREQ)
    slowRightTurn.phase = PHASE_RIGHT                        
    slowRightTurn.deltasLeft = [0.25, 0.25, 0.25]
    slowRightTurn.deltasRight = [0.25, 0.25, 0.25]

    slowLeftTurn = GaitConfig(motorgains, rightFreq=STRIDE_FREQ, leftFreq=STRIDE_FREQ)
    slowLeftTurn.phase = PHASE_LEFT                         
    slowLeftTurn.deltasLeft = [0.25, 0.25, 0.25]
    slowLeftTurn.deltasRight = [0.25, 0.25, 0.25]

    fastRightTurn = GaitConfig(motorgains, rightFreq=3, leftFreq=6)
    fastRightTurn.phase = PHASE_180_DEG                          
    fastRightTurn.deltasLeft = [0.25, 0.25, 0.25]
    fastRightTurn.deltasRight = [0.25, 0.25, 0.25]

    fastLeftTurn = GaitConfig(motorgains, rightFreq=6, leftFreq=3)
    fastLeftTurn.phase = PHASE_180_DEG                          
    fastLeftTurn.deltasLeft = [0.25, 0.25, 0.25]
    fastLeftTurn.deltasRight = [0.25, 0.25, 0.25]
    
    # Set the timings of each segment of the run
    T = 1000


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
            R1.setGait(slowLeftTurn)
            R1.startTimedRun( T )
        elif keypress == 's':
            R1.setGait(slowAltTripod)
            R1.startTimedRun( T )
        elif keypress == 'd':
            R1.setGait(slowRightTurn)
            R1.startTimedRun( T )
        elif keypress == 'w':
            R1.setGait(slowBound)
            R1.startTimedRun( T )
        elif keypress == 'f':
            R1.setGait(fastLeftTurn)
            R1.startTimedRun( T )
        elif keypress == 'g':
            R1.setGait(fastAltTripod)
            R1.startTimedRun( T )
        elif keypress == 'h':
            R1.setGait(fastRightTurn)
            R1.startTimedRun( T )
        elif keypress == 't':
            R1.setGait(fastBound)
            R1.startTimedRun( T )
        elif (keypress == 'q') or (ord(keypress) == 26):
            print "Exit."
            exit_flag = True

        time.sleep(T/1000.0)

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
