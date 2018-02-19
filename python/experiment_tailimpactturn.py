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
    #  [ Kp , Ki , Kd , Kaw , Kff]
    tailgains = [500,750,30,2000,0]
    # tailgains = [500,0,0,0,0]

    # Motor gains format:
    #  [ Kp , Ki , Kd , Kaw , Kff     ,  Kp , Ki , Kd , Kaw , Kff ]
    #    ----------LEFT----------        ---------_RIGHT----------
    motorgains = [10000,3000,100,0,500, 10000,3000,100,0,500]
    # motorgains = [0,0,0,0,3800, 0,0,0,0,3800] 

    # Set up tail swing parameters for zeroing
    pzero = 90

    # Set tail impact velocity (Hz)
    v_impact = 4

    # Set stride frequency for straight running
    freq = 10

    # Swing ccw to find zero
    zeroCCW = TailConfig(tailgains)
    zeroCCW.pInput = pzero

    # Swing cw to find zero
    zeroCW = TailConfig(tailgains)
    zeroCW.pInput = -pzero

    # Swing tail to drag against ground
    tailImpact = TailConfig(tailgains)
    tailImpact.vInput = v_impact

    # Hold tail upright
    tailUp = TailConfig(tailgains)
    tailUp.pInput = 0

    # Set alternating tripod gait
    altTripod = GaitConfig(motorgains, rightFreq=freq, leftFreq=freq)
    altTripod.phase = PHASE_180_DEG            
    # Constant vel              
    # altTripod.deltasLeft = [0.25, 0.25, 0.25]
    # altTripod.deltasRight = [0.25, 0.25, 0.25]
    # Faster push
    # altTripod.deltasLeft = [0.175, 0.325, 0.325]
    # altTripod.deltasRight = [0.325, 0.175, 0.175]
    # Faster recirculation
    altTripod.deltasLeft = [0.325, 0.175, 0.175]
    altTripod.deltasRight = [0.175, 0.325, 0.325]

    # Set initial tail control to perform zeroing swing CW
    R1.zeroTailPosition()
    R1.setTailControl(zeroCW)

    # Set leg gait
    R1.setGait(altTripod)

    # Set experiment run times
    T1 = 1000
    T2 = 1000
    T3 = 500
    T4 = 100
    T5 = 500
    T6 = 3500
    EXPERIMENT_WAIT_TIME_MS  = 200  #ms
    EXPERIMENT_SAVEBUFFER_TIME_MS = 500  #ms
    # EXPERIMENT_SAVEBUFFER_TIME_MS = 50  #ms
    EXPERIMENT_RUN_TIME_MS = T4 + T5 + T6
    
    # Some preparation is needed to cleanly save telemetry data
    for r in shared.ROBOTS:
        if r.SAVE_DATA:
            #This needs to be done to preparne the .telemtryData variables in each robot object
            r.setupTelemetryDataTime(EXPERIMENT_SAVEBUFFER_TIME_MS + EXPERIMENT_RUN_TIME_MS)
            r.eraseFlashMem()
        
    # Pause and wait to start run, including lead-in time
    print ""
    print "  ***************************"
    print "  *******    READY    *******"
    print "  ***************************"
    raw_input("  Press ENTER to start run ...")
    print ""

    # Zeroing swing CW
    R1.startTailTimedRun( T1 )
    time.sleep((T1 + EXPERIMENT_WAIT_TIME_MS) / 1000.0)  #argument to time.sleep is in SECONDS

    # Zeroing swing CCW
    R1.setTailControl(zeroCCW)
    R1.startTailTimedRun( T2 )
    time.sleep((T2 + EXPERIMENT_WAIT_TIME_MS) / 1000.0)  

    # Put tail upright
    R1.setTailControl(tailUp)
    R1.startTailTimedRun( T3 )
    time.sleep((T3 + 5*EXPERIMENT_WAIT_TIME_MS) / 1000.0) 
    R1.stopTail()
    time.sleep((4*EXPERIMENT_WAIT_TIME_MS) / 1000.0)
    R1.setTailControl(tailImpact)


    # Initiate telemetry recording; the robot will begin recording immediately when cmd is received.
    for r in shared.ROBOTS:
        if r.SAVE_DATA:
            r.startTelemetrySave()

    time.sleep((T4) / 1000.0)
    
    # Start leg motion
    R1.startTimedRun( T5 + T6 )
    time.sleep((T5) / 1000.0)  #argument to time.sleep is in SECONDS
    
    # Touch tail down
    R1.setTailControl(tailImpact)
    R1.startTailTimedRun( T6 )
    time.sleep((T6 + EXPERIMENT_WAIT_TIME_MS) / 1000.0)
    
    for r in shared.ROBOTS:
        if r.SAVE_DATA:
            raw_input("Press Enter to start telemetry read-back ...")
            r.downloadTelemetry(filename = 'TailImpactCarpet01192018')
    
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
