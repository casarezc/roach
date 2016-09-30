#!/usr/bin/env python
"""
authors: apullin

This script will run an experiment with a connected velociroach robot.

The main function will send all the setup parameters to the robots, execute defined manoeuvres, and record telemetry.

"""
from lib import command
import time,sys,os,traceback
import serial

# Path to imageproc-settings repo must be added
sys.path.append(os.path.dirname("../../imageproc-settings/"))
sys.path.append(os.path.dirname("../imageproc-settings/"))  
import shared_multi as shared

from velociroach_connected import *

####### Wait at exit? #######
EXIT_WAIT   = False

def main():    
    xb = setupSerial(shared.BS_COMPORT, shared.BS_BAUDRATE)
    
    R1 = Velociroach_Connected('\x21\x64', xb)
    R1.SAVE_DATA = True
    # R1.SAVE_DATA = False
                            
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
    #           [ Kp , Ki , Kd , Kaw , Kff,       Kp , Ki , Kd , Kaw , Kff ]
    #            ----------LEFT----------        ---------_RIGHT----------
    motorgains_1 = [5000,1000,100,100,200,         5000,1000,100,100,200]
    # motorgains_1 = [0,0,0,0,1500,         5000,1000,100,100,200]
    motorgains_2 = [5000,1000,100,100,200,         5000,1000,100,100,200]

    # Stride frequency
    freq_1 = 15
    freq_2 = 1

    # Configure gait for front robot
    straightGait = GaitConfig()
    straightGait.motorgains_1 = motorgains_1
    straightGait.rightFreq_1 = freq_1
    straightGait.leftFreq_1 = freq_1
    straightGait.phase_1 = PHASE_180_DEG 
    straightGait.deltasLeft_1 = [0.25, 0.25, 0.25]
    straightGait.deltasRight_1 = [0.25, 0.25, 0.25]

    # Configure gait for rear robot
    straightGait.motorgains_2 = motorgains_2
    straightGait.rightFreq_2 = freq_2
    straightGait.leftFreq_2 = freq_2
    straightGait.phase_2 = PHASE_180_DEG 
    straightGait.deltasLeft_2 = [0.25, 0.25, 0.25]
    straightGait.deltasRight_2 = [0.25, 0.25, 0.25]

    # Set gait
    R1.setGait(straightGait)

    # Steering gains format:
    #           [ Kp , Ki , Kff, thrust_nom]
    steergains = [300, 100, 1800, 1200]
    # steergains = [0, 0, 0, 0]

    # Yaw rate in deg/s
    yaw_rate = 60

    # Set steering gains
    R1.setSteerGains(steergains)

    # Set steering setpoint
    R1.setSteerRate(yaw_rate)
    # R1.setSteerRate(-yaw_rate)


    # Set run, lead in, lead out time
    EXPERIMENT_RUN_TIME_MS     = 4000 #ms
    EXPERIMENT_LEADIN_TIME_MS  = 500  #ms
    EXPERIMENT_LEADOUT_TIME_MS = 500  #ms
    
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
    R1.startTimedRun( EXPERIMENT_RUN_TIME_MS ) 
    time.sleep(EXPERIMENT_RUN_TIME_MS / 1000.0)  #argument to time.sleep is in SECONDS
    ######## End of motion commands   ########)
    
    # Shut off steering control to prevent windup of controller
    R1.stopSteering()

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
