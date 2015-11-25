import glob
import time
import sys
from lib import command
from callbackFunc_connected import xbee_received
import datetime
import serial
import shared_multi as shared
from struct import pack,unpack
from xbee import XBee
from math import ceil,floor
import numpy as np

# TODO: check with firmware if this value is actually correct
PHASE_180_DEG = 0x8000

class GaitConfig:
    motorgains_1 = None
    rightFreq_1 = None
    leftFreq_1 = None
    phase_1 = None
    repeat_1 = None
    deltasLeft_1 = None
    deltasRight_1 = None

    motorgains_2 = None
    rightFreq_2 = None
    leftFreq_2 = None
    phase_2 = None
    repeat_2 = None
    deltasLeft_2 = None
    deltasRight_2 = None

    duration = None

    def __init__(self, motorgains_1 = None, rightFreq_1 = None, leftFreq_1 = None, phase_1 = None, repeat_1 = None, motorgains_2 = None, rightFreq_2 = None, leftFreq_2 = None, phase_2 = None, repeat_2 = None, duration = None):
        if motorgains_1 == None:
            self.motorgains_1 = [0,0,0,0,0 , 0,0,0,0,0]
        else:
            self.motorgains_1 = motorgains_1

        if motorgains_2 == None:
            self.motorgains_2 = [0,0,0,0,0 , 0,0,0,0,0]
        else:
            self.motorgains_2 = motorgains_2
        
        self.duration = duration

        self.rightFreq_1 = rightFreq_1
        self.leftFreq_1 = leftFreq_1
        self.phase_1 = phase_1
        self.repeat_1 = repeat_1

        self.rightFreq_2 = rightFreq_2
        self.leftFreq_2 = leftFreq_2
        self.phase_2 = phase_2
        self.repeat_2 = repeat_2
        
        
class Velociroach_Connected:
    motor_gains_set_1 = False
    motor_gains_set_2 = False
    encoders_zeroed_1 = False
    encoders_zeroed_2 = False
    robot_queried = False
    flash_erased = False
    
    currentGait = GaitConfig()

    dataFileName = ''
    telemtryData = [ [] ]
    numSamples = 0
    telemSampleFreq = 1000
    VERBOSE = True
    telemFormatString = '%d' # single type forces all data to be saved in this type
    SAVE_DATA = False
    RESET = False

    def __init__(self, address, xb):
            self.DEST_ADDR = address
            self.DEST_ADDR_int = unpack('>h',self.DEST_ADDR)[0] #address as integer
            self.xb = xb
            print "Robot with DEST_ADDR = 0x%04X " % self.DEST_ADDR_int

    ################## Methods with no radio command ###################

    def clAnnounce(self):
        print "DST: 0x%02X | " % self.DEST_ADDR_int,

    def findFileName(self):   
        # Construct filename
        path     = 'Data/'
        name     = 'trial'
        datetime = time.localtime()
        dt_str   = time.strftime('%Y.%m.%d_%H.%M.%S', datetime)
        root     = path + dt_str + '_' + name
        self.dataFileName = root + '_imudata.txt'
        #self.clAnnounce()
        #print "Data file:  ", shared.dataFileName

    def setupTelemetryDataTime(self, runtime):
        ''' This is NOT current for Velociroach! '''
        #TODO : update for Velociroach
        
        # Take the longer number, between numSamples and runTime
        nrun = int(self.telemSampleFreq * runtime / 1000.0)
        self.numSamples = nrun
        
        #allocate an array to write the downloaded telemetry data into
        self.telemtryData = [ [] ] * self.numSamples
        self.clAnnounce()
        print "Telemetry samples to save: ",self.numSamples
        
    def setupTelemetryDataNum(self, numSamples):
        ''' This is NOT current for Velociroach! '''
        #TODO : update for Velociroach
     
        self.numSamples = numSamples
        
        #allocate an array to write the downloaded telemetry data into
        self.telemtryData = [ [] ] * self.numSamples
        self.clAnnounce()
        print "Telemetry samples to save: ",self.numSamples

    def writeFileHeader(self):
        fileout = open(self.dataFileName,'w')
        #write out parameters in format which can be imported to Excel
        today = time.localtime()
        date = str(today.tm_year)+'/'+str(today.tm_mon)+'/'+str(today.tm_mday)+'  '
        date = date + str(today.tm_hour) +':' + str(today.tm_min)+':'+str(today.tm_sec)
        fileout.write('%  Data file recorded ' + date + '\n')

        fileout.write('%  Stride Frequency 1 (L,R)      = ' +repr( [ self.currentGait.leftFreq_1, self.currentGait.rightFreq_1]) + '\n')
        fileout.write('%  Deltas (Fractional) 1 (L,R)   = ' + repr(self.currentGait.deltasLeft_1) + ',' + repr(self.currentGait.deltasRight_1) + '\n')
        fileout.write('%  Phase 1                       = ' + repr(self.currentGait.phase_1) + '\n')
        fileout.write('%  Motor Gains 1 (L,R)           = ' + repr(self.currentGait.motorgains_1) + '\n')

        fileout.write('%  Stride Frequency 2 (L,R)       = ' +repr( [ self.currentGait.leftFreq_2, self.currentGait.rightFreq_2]) + '\n')
        fileout.write('%  Deltas (Fractional) 2 (L,R)    = ' + repr(self.currentGait.deltasLeft_2) + ',' + repr(self.currentGait.deltasRight_2) + '\n')
        fileout.write('%  Phase 2                        = ' + repr(self.currentGait.phase_2) + '\n')
        fileout.write('%  Motor Gains 2 (L,R)            = ' + repr(self.currentGait.motorgains_2) + '\n')
            
        fileout.write('%  experiment_connected.py \n')
        fileout.write('% Columns: \n')
        # order for wiring on RF Turner
        fileout.write('% time | Leg Pos L1 | Leg Pos R1 | Com. Leg Pos L1 | Com. Leg Pos R1 | Leg Pos L2 | Leg Pos R2 | Com. Leg Pos L2 | Com. Leg Pos R2 | DC_L1 | DC_R1 | DC_L2 | DC_R2 | GyroX | GyroY | GyroZ | AX | AY | AZ | BEMF_L1 | BEMF_R1 | BEMF_L2 | BEMF_R2 | VBatt\n')
        fileout.close()

    #########################################################################
    
    ########################## Transmit data packet ##########################
    def tx(self, status, type, data):
        payload = chr(status) + chr(type) + ''.join(data)
        self.xb.tx(dest_addr = self.DEST_ADDR, data = payload)

    ##########################################################################
        
    ################# Methods reset, echo message, who am I query ############
    def reset(self):
        self.clAnnounce()
        print "Resetting robot..."
        self.tx( 0, command.SOFTWARE_RESET, pack('h',1))
        
    def sendEcho(self, msg):
        self.tx( 0, command.ECHO, msg)
        
    def query(self, retries = 8):
        self.robot_queried = False
        tries = 1
        while not(self.robot_queried) and (tries <= retries):
            self.clAnnounce()
            print "Querying robot , ",tries,"/",retries
            self.tx( 0,  command.WHO_AM_I, "Robot Echo") #sent text is unimportant
            tries = tries + 1
            time.sleep(0.1)   

    ##############################################################################
    
    ################### Save and readback telemetry commands #####################
    def eraseFlashMem(self, timeout = 8):
        eraseStartTime = time.time()
        self.tx( 0, command.ERASE_SECTORS, pack('L',self.numSamples))
        self.clAnnounce()
        print "Started flash erase ..."
        while not (self.flash_erased):
            #sys.stdout.write('.')
            time.sleep(0.25)
            if (time.time() - eraseStartTime) > timeout:
                print"Flash erase timeout, retrying;"
                self.tx( 0, command.ERASE_SECTORS, pack('L',self.numSamples))
                eraseStartTime = time.time()   

    def startTelemetrySave(self):
        self.clAnnounce()
        print "Started telemetry save of", self.numSamples," samples."
        self.tx(0, command.START_TELEMETRY, pack('L',self.numSamples))

    ######TODO : sort out this function and flashReadback below
    def downloadTelemetry(self, timeout = 5, retry = True):
        #suppress callback output messages for the duration of download
        self.VERBOSE = False
        self.clAnnounce()
        print "Started telemetry download"
        self.tx( 0, command.FLASH_READBACK, pack('L',self.numSamples))
                
        dlStart = time.time()
        shared.last_packet_time = dlStart
        #bytesIn = 0
        while self.telemtryData.count([]) > 0:
            time.sleep(0.02)
            dlProgress(self.numSamples - self.telemtryData.count([]) , self.numSamples)
            if (time.time() - shared.last_packet_time) > timeout:
                print ""
                #Terminal message about missed packets
                self.clAnnounce()
                print "Readback timeout exceeded"
                print "Missed", self.telemtryData.count([]), "packets."
                #print "Didn't get packets:"
                #for index,item in enumerate(self.telemtryData):
                #    if item == []:
                #        print "#",index+1,
                print "" 
                break
                # Retry telem download            
                if retry == True:
                    raw_input("Press Enter to restart telemetry readback ...")
                    self.telemtryData = [ [] ] * self.numSamples
                    self.clAnnounce()
                    print "Started telemetry download"
                    dlStart = time.time()
                    shared.last_packet_time = dlStart
                    self.tx( 0, command.FLASH_READBACK, pack('L',self.numSamples))
                else: #retry == false
                    print "Not trying telemetry download."          

        dlEnd = time.time()
        dlTime = dlEnd - dlStart
        #Final update to download progress bar to make it show 100%
        dlProgress(self.numSamples-self.telemtryData.count([]) , self.numSamples)
        #totBytes = 52*self.numSamples
        totBytes = 70*(self.numSamples - self.telemtryData.count([]))
        datarate = totBytes / dlTime / 1000.0
        print '\n'
        #self.clAnnounce()
        #print "Got ",self.numSamples,"samples in ",dlTime,"seconds"
        self.clAnnounce()
        print "DL rate: {0:.2f} KB/s".format(datarate)
        
        #enable callback output messages
        self.VERBOSE = True

        print ""
        self.saveTelemetryData()
        #Done with flash download and save

    def saveTelemetryData(self):
        self.findFileName()
        self.writeFileHeader()
        fileout = open(self.dataFileName, 'a')
        
        sanitized = [item for item in self.telemtryData if item!= []];
        
        np.savetxt(fileout , np.array(sanitized), self.telemFormatString, delimiter = ',')
        fileout.close()
        self.clAnnounce()
        print "Telemetry data saved to", self.dataFileName
        
    ##########################################################################################
        
    ########################## Gait configuration methods ####################################

    def setMotorGains(self, gains_1, gains_2, retries = 8):

        rnum = 1
        tries = 1
        self.motorgains_1 = gains_1
        while not(self.motor_gains_set_1) and (tries <= retries):
            self.clAnnounce()
            print "Setting front motor gains...   ",tries,"/8"
            temp = [rnum] + gains_1
            self.tx( 0, command.SET_PID_GAINS, pack('11h',*temp))
            tries = tries + 1
            time.sleep(0.3)

        rnum = 2
        tries = 1
        self.motorgains_2 = gains_2
        while not(self.motor_gains_set_2) and (tries <= retries):
            self.clAnnounce()
            print "Setting rear motor gains...   ",tries,"/8"
            temp = [rnum] + gains_2 
            self.tx( 0, command.SET_PID_GAINS, pack('11h',*temp))
            tries = tries + 1
            time.sleep(0.3)

    def zeroPosition(self, retries = 8):
        tries = 1
        while not(self.encoders_zeroed_1) and (tries <= retries):
            self.clAnnounce()
            print "Zeroing front encoders...",tries,"/8"
            self.tx( 0, command.ZERO_POS, pack('h',1))
            tries = tries + 1
            time.sleep(0.3) #built-in holdoff, since reset apparently takes > 50ms

        tries = 1
        while not(self.encoders_zeroed_2) and (tries <= retries):
            self.clAnnounce()
            print "Zeroing rear encoders...",tries,"/8"
            self.tx( 0, command.ZERO_POS, pack('h',2))
            tries = tries + 1
            time.sleep(0.3) #built-in holdoff, since reset apparently takes > 50ms

    def setPhase(self, phase_1, phase_2):
        
        rnum = 1
        self.clAnnounce()
        print "Setting front phase to 0x%04X " % phase_1

        self.tx( 0, command.SET_PHASE, pack('=hl', rnum, phase_1))
        time.sleep(0.05)     

        rnum = 2
        self.clAnnounce()
        print "Setting rear phase to 0x%04X " % phase_2

        self.tx( 0, command.SET_PHASE, pack('=hl', rnum, phase_2))
        time.sleep(0.05)    

    def setVelProfile(self, gaitConfig):
        self.clAnnounce()
        print "Setting front robot stride velocity profile to: "

        rnum = 1
        
        periodLeft = 1000.0 / gaitConfig.leftFreq_1
        periodRight = 1000.0 / gaitConfig.rightFreq_1
        
        deltaConv = 0x4000 # TODO: this needs to be clarified (ronf, dhaldane, pullin)
        
        lastLeftDelta = 1-sum(gaitConfig.deltasLeft_1) #TODO: change this to explicit entry, with a normalization here
        lastRightDelta = 1-sum(gaitConfig.deltasRight_1)
        
        temp = [int(rnum), int(periodLeft), int(gaitConfig.deltasLeft_1[0]*deltaConv), int(gaitConfig.deltasLeft_1[1]*deltaConv),
                int(gaitConfig.deltasLeft_1[2]*deltaConv), int(lastLeftDelta*deltaConv) , 0, \
                int(periodRight), int(gaitConfig.deltasRight_1[0]*deltaConv), int(gaitConfig.deltasRight_1[1]*deltaConv),
                int(gaitConfig.deltasRight_1[2]*deltaConv), int(lastRightDelta*deltaConv), 0]
        
        self.clAnnounce()
        print "     ",temp
        
        self.tx( 0, command.SET_VEL_PROFILE, pack('13h', *temp))
        time.sleep(0.1)

        self.clAnnounce()
        print "Setting rear robot stride velocity profile to: "

        rnum = 2
        
        periodLeft = 1000.0 / gaitConfig.leftFreq_2
        periodRight = 1000.0 / gaitConfig.rightFreq_2
        
        deltaConv = 0x4000 # TODO: this needs to be clarified (ronf, dhaldane, pullin)
        
        lastLeftDelta = 1-sum(gaitConfig.deltasLeft_2) #TODO: change this to explicit entry, with a normalization here
        lastRightDelta = 1-sum(gaitConfig.deltasRight_2)
        
        temp = [int(rnum), int(periodLeft), int(gaitConfig.deltasLeft_2[0]*deltaConv), int(gaitConfig.deltasLeft_2[1]*deltaConv),
                int(gaitConfig.deltasLeft_2[2]*deltaConv), int(lastLeftDelta*deltaConv) , 0, \
                int(periodRight), int(gaitConfig.deltasRight_2[0]*deltaConv), int(gaitConfig.deltasRight_2[1]*deltaConv),
                int(gaitConfig.deltasRight_2[2]*deltaConv), int(lastRightDelta*deltaConv), 0]
        
        self.clAnnounce()
        print "     ",temp
        
        self.tx( 0, command.SET_VEL_PROFILE, pack('13h', *temp))
        time.sleep(0.1)

            
    def setGait(self, gaitConfig):
        self.currentGait = gaitConfig
        
        self.clAnnounce()
        print " --- Setting complete gait config --- "
        self.setMotorGains(gaitConfig.motorgains_1, gaitConfig.motorgains_2)
        self.zeroPosition()
        self.setPhase(gaitConfig.phase_1, gaitConfig.phase_2)
        self.setVelProfile(gaitConfig)

        self.clAnnounce()
        print " ------------------------------------ "
    
    def startTimedRun(self, duration):
        self.clAnnounce()
        print "Starting timed run of",duration," ms"
        self.tx( 0, command.START_TIMED_RUN, pack('h', duration))
        time.sleep(0.05)       

        
########## Helper functions #################
#TODO: find a home for these? Possibly in BaseStation class (pullin, abuchan)

def setupSerial(COMPORT , BAUDRATE , timeout = 3, rtscts = 0):
    print "Setting up serial ..."
    try:
        ser = serial.Serial(port = COMPORT, baudrate = BAUDRATE, \
                    timeout=timeout, rtscts=rtscts)
    except serial.serialutil.SerialException:
        print "Could not open serial port:",shared.BS_COMPORT
        sys.exit(1)
    
    shared.ser = ser
    ser.flushInput()
    ser.flushOutput()
    return XBee(ser, callback = xbee_received)
    
    
def xb_safe_exit(xb):
    print "Halting xb"
    if xb is not None:
        xb.halt()
        
    print "Closing serial"
    if xb.serial is not None:
        xb.serial.close()
        
    print "Exiting..."
    sys.exit(1)
    

   
def verifyAllMotorGainsSet():
    #Verify all robots have motor gains set
    for r in shared.ROBOTS:
        if not(r.motor_gains_set):
            print "CRITICAL : Could not SET MOTOR GAINS on robot 0x%02X" % r.DEST_ADDR_int
            xb_safe_exit(shared.xb)
            
def verifyAllTailGainsSet():
    #Verify all robots have motor gains set
    for r in shared.ROBOTS:
        if not(r.tail_gains_set):
            print "CRITICAL : Could not SET TAIL GAINS on robot 0x%02X" % r.DEST_ADDR_int
            xb_safe_exit(shared.xb)
            
def verifyAllQueried():            
    for r in shared.ROBOTS:
        if not(r.robot_queried):
            print "CRITICAL : Could not query robot 0x%02X" % r.DEST_ADDR_int
            xb_safe_exit(shared.xb)

def dlProgress(current, total):
    percent = int(100.0*current/total)
    dashes = int(floor(percent/100.0 * 45))
    stars = 45 - dashes - 1
    barstring = '|' + '-'*dashes + '>' + '*'*stars + '|'
    #sys.stdout.write("\r" + "Downloading ...%d%%   " % percent)
    sys.stdout.write("\r" + str(current).rjust(5) +"/"+ str(total).ljust(5) + "   ")
    sys.stdout.write(barstring)
    sys.stdout.flush()

#########################################################################