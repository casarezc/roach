#!/usr/bin/env python
"""
authors: apullin

Contents of this file are copyright Andrew Pullin, 2013

"""
from lib import command
import time,sys,os,traceback
import serial

# Path to imageproc-settings repo must be added
sys.path.append(os.path.dirname("../../imageproc-settings/"))
sys.path.append(os.path.dirname("../imageproc-settings/"))  
import shared_multi as shared

from velociroach import *


###### Operation Flags ####
RESET_R1 = True  
EXIT_WAIT   = False

def main():    
    xb = setupSerial(shared.BS_COMPORT, shared.BS_BAUDRATE)
    
    R1 = Velociroach('\x20\x52', xb)
    
    shared.ROBOTS = [R1] #This is neccesary so callbackfunc can reference robots
    shared.xb = xb           #This is neccesary so callbackfunc can halt before exit
    
    if RESET_R1:
        R1.reset()
        time.sleep(0.35)
    
    # Query
    R1.query( retries = 8 )
    
    #Verify all robots can be queried
    verifyAllQueried()  #exits on failure

    numToDL = raw_input("How many samples to download? ")
    
    if numToDL > 0:
        
        #allocate an array to write the downloaded telemetry data into
        R1.numSamples = int(numToDL)
        R1.telemtryData = [ [] ] * R1.numSamples
        R1.clAnnounce()
        print "Telemetry samples to save: ",R1.numSamples

        R1.runtime = 'UNKNOWN'
        R1.moveq = 'UNKNOWN'

        blank_gait = GaitConfig()
        R1.currentGait = blank_gait
        
        # Pause and wait to start run, including leadin time
        print ""
        print "  ***************************"
        print "  *******    READY    *******"
        print "  ***************************"
        raw_input("  Press ENTER to start download ...")
        print ""

        R1.downloadTelemetry(retry = False)

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
