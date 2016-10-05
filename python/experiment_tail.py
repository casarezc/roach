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
    tailgains = [1500,750,20,2000,0]
    # tailgains = [0,0,0,0,0]
    # Set up different tail commands
    plim = 180

    posFwd = TailConfig(tailgains)
    posFwd.pInput = plim

    posBwd = TailConfig(tailgains)
    posBwd.pInput = -plim

    tailFreq = 3.3

    spinFwd = TailConfig(tailgains)
    spinFwd.vInput = tailFreq

    spinBwd = TailConfig(tailgains)
    spinBwd.vInput = -tailFreq

    # Zero tail position
    R1.zeroTailPosition()

    # Set tail control
    R1.setTailControl(posFwd)
    # R1.setTailControl(posBwd)
    # R1.setTailControl(spinFwd)
    # R1.setTailControl(spinBwd)

    # example , 0.1s lead in + 2s run + 0.1s lead out
    EXPERIMENT_RUN_TIME_MS     = 3000 #ms
    EXPERIMENT_LEADIN_TIME_MS  = 500  #ms
    EXPERIMENT_LEADOUT_TIME_MS = 200  #ms
    
    # Some preparation is needed to cleanly save telemetry data
    for r in shared.ROBOTS:
        if r.SAVE_DATA:
            #This needs to be done to preparne the .telemtryData variables in each robot object
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
    R1.startTailTimedRun( EXPERIMENT_RUN_TIME_MS ) #Faked for now, since pullin doesn't have a working VR+AMS to test with
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
