import glob
import time
import sys
from lib import command
from callbackFunc_multi import xbee_received
import datetime
import serial
import shared_multi as shared
from struct import pack,unpack
from xbee import XBee
from math import ceil,floor
import numpy as np

PHASE_0_DEG   = 0x0000
PHASE_180_DEG = 0x8000

class GaitConfig:
    motorgains = None
    duration = None
    rightFreq = None
    leftFreq = None
    phase = None
    repeat = None
    deltasLeft = None
    deltasRight = None
    def __init__(self, motorgains = None, duration = None, rightFreq = None, leftFreq = None, phase = None, repeat = None, deltasLeft = None, deltasRight = None):
        if motorgains == None:
            self.motorgains = [0,0,0,0,0 , 0,0,0,0,0]
        else:
            self.motorgains = motorgains 
        
        self.duration = duration

        if rightFreq == None:
            self.rightFreq = 0
        else:
            self.rightFreq = rightFreq

        if leftFreq == None:
            self.leftFreq = 0
        else:
            self.leftFreq = leftFreq

        if phase == None:
            self.phase = 0
        else:
            self.phase = phase

        self.repeat = repeat

        if deltasLeft == None:
            self.deltasLeft = [0,0,0]
        else:
            self.deltasLeft = deltasLeft

        if deltasRight == None:
            self.deltasRight = [0,0,0]
        else:
            self.deltasRight = deltasRight
        
class TailConfig:
    motorgains = None
    swing_duration = None
    right_flag = None
    pInput = None
    pBias = None
    vInput = None
    def __init__(self, motorgains = None, swing_duration = None, right_flag = None, pInput = None, pBias = None, vInput = None):
        if motorgains == None:
            self.motorgains = [0,0,0,0,0]
        else:
            self.motorgains = motorgains
        
        self.swing_duration = swing_duration
        self.right_flag = right_flag
        self.pInput = pInput
        self.pBias = pBias
        self.vInput = vInput
        
class Velociroach:
    motor_gains_set = False
    tail_gains_set = False
    robot_queried = False
    flash_erased = False
    
    currentGait = GaitConfig()
    cuurentTail = TailConfig()

    dataFileName = ''
    telemtryData = [ [] ]
    numSamples = 0
    telemSampleFreq = 1000
    VERBOSE = True
    telemFormatString = '%d' # single type forces all data to be saved in this type
    SAVE_DATA = False
    RESET = False

    def __init__(self, address, xb):
            self.currentGait = GaitConfig()
            self.currentTail = TailConfig()
            self.DEST_ADDR = address
            self.DEST_ADDR_int = unpack('>h',self.DEST_ADDR)[0] #address as integer
            self.xb = xb
            print "Robot with DEST_ADDR = 0x%04X " % self.DEST_ADDR_int

    def clAnnounce(self):
        print "DST: 0x%02X | " % self.DEST_ADDR_int,
    
    def tx(self, status, type, data):
        payload = chr(status) + chr(type) + ''.join(data)
        self.xb.tx(dest_addr = self.DEST_ADDR, data = payload)
        
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
    
    #TODO: getting flash erase to work is critical to function testing (pullin)    
    #existing VR firmware does not send a packet when the erase is done, so this will hang and retry.
    def eraseFlashMem(self, timeout = 30):
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
        
    def setPhase(self, phase):
        self.clAnnounce()
        print "Setting phase to 0x%04X " % phase
        self.tx( 0, command.SET_PHASE, pack('l', phase))
        time.sleep(0.05)        
    
    def startTimedRun(self, duration):
        self.clAnnounce()
        print "Starting timed run of",duration," ms"
        self.tx( 0, command.START_TIMED_RUN, pack('h', duration))
        time.sleep(0.05)
        
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
        
    def setVelProfile(self, gaitConfig):
        self.clAnnounce()
        print "Setting stride velocity profile to: "


        if gaitConfig.leftFreq == 0:
            gaitConfig.leftFreq =  0.1;
        if gaitConfig.rightFreq == 0:
            gaitConfig.rightFreq = 0.1;
        periodLeft = 1000.0 / gaitConfig.leftFreq
        periodRight = 1000.0 / gaitConfig.rightFreq
        
        deltaConv = 0x4000 # TODO: this needs to be clarified (ronf, dhaldane, pullin)
        
        lastLeftDelta = 1-sum(gaitConfig.deltasLeft) #TODO: change this to explicit entry, with a normalization here
        lastRightDelta = 1-sum(gaitConfig.deltasRight)
        
        temp = [int(periodLeft), int(gaitConfig.deltasLeft[0]*deltaConv), int(gaitConfig.deltasLeft[1]*deltaConv),
                int(gaitConfig.deltasLeft[2]*deltaConv), int(lastLeftDelta*deltaConv) , 0, \
                int(periodRight), int(gaitConfig.deltasRight[0]*deltaConv), int(gaitConfig.deltasRight[1]*deltaConv),
                int(gaitConfig.deltasRight[2]*deltaConv), int(lastRightDelta*deltaConv), 0]
        
        self.clAnnounce()
        print "     ",temp
        
        self.tx( 0, command.SET_VEL_PROFILE, pack('12h', *temp))
        time.sleep(0.1)

    ######TODO : sort out this function and flashReadback below
    def downloadTelemetry(self, timeout = 5, retry = True):
        #suppress callback output messages for the duration of download
        self.VERBOSE = False
        self.clAnnounce()
        print "Started telemetry download"
        self.tx( 0, command.FLASH_READBACK, pack('=L',self.numSamples))
                
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
                    self.tx( 0, command.FLASH_READBACK, pack('=L',self.numSamples))
                else: #retry == false
                    print "Not trying telemetry download."          

        dlEnd = time.time()
        dlTime = dlEnd - dlStart
        #Final update to download progress bar to make it show 100%
        dlProgress(self.numSamples-self.telemtryData.count([]) , self.numSamples)
        #totBytes = 46*self.numSamples
        totBytes = 58*(self.numSamples - self.telemtryData.count([]))
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
        
    def writeFileHeader(self):
        fileout = open(self.dataFileName,'w')
        #write out parameters in format which can be imported to Excel
        today = time.localtime()
        date = str(today.tm_year)+'/'+str(today.tm_mon)+'/'+str(today.tm_mday)+'  '
        date = date + str(today.tm_hour) +':' + str(today.tm_min)+':'+str(today.tm_sec)
        fileout.write('%  casarezc/roach tail_devel branch Data file recorded ' + date + '\n')

        fileout.write('%  Stride Frequency         = ' +repr( [ self.currentGait.leftFreq, self.currentGait.rightFreq]) + '\n')
        fileout.write('%  Deltas (Fractional)      = ' + repr(self.currentGait.deltasLeft) + ',' + repr(self.currentGait.deltasRight) + '\n')
        fileout.write('%  Phase                    = ' + repr(self.currentGait.phase) + '\n')    
        fileout.write('%  Motor Gains    = ' + repr(self.currentGait.motorgains) + '\n')

        fileout.write('%  Tail Gains    = ' + repr(self.currentTail.motorgains) + '\n')

        fileout.write('% Columns: \n')
        # order for wiring on RF Turner
        fileout.write('% time | Left Leg Pos | Right Leg Pos | Tail Pos | Commanded Left Leg Pos | Commanded Right Leg Pos | Commanded Tail Pos | DCL | DCR | DCT | GyroX | GyroY | GyroZ | AX | AY | AZ | LBEMF | RBEMF | TBEMF | VBatt\n')
        fileout.close()

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
    
    def startTelemetrySave(self):
        self.clAnnounce()
        print "Started telemetry save of", self.numSamples," samples."
        self.tx(0, command.START_TELEMETRY, pack('L',self.numSamples))

    def setMotorGains(self, gains, retries = 8):
        tries = 1
        self.motorGains = gains
        while not(self.motor_gains_set) and (tries <= retries):
            self.clAnnounce()
            print "Setting motor gains...   ",tries,"/8"
            self.tx( 0, command.SET_PID_GAINS, pack('10h',*gains))
            tries = tries + 1
            time.sleep(0.3)
            
    def setGait(self, gaitConfig, zero_position = False):
        self.currentGait = gaitConfig
        
        self.clAnnounce()
        print " --- Setting complete gait config --- "
        self.zeroPosition()
        self.setMotorGains(gaitConfig.motorgains)
        self.setPhase(gaitConfig.phase)
        self.setVelProfile(gaitConfig) #whole object is passed in, due to several references
        
        self.clAnnounce()
        print " ------------------------------------ "
        
    def zeroPosition(self):
        self.tx( 0, command.ZERO_POS, 'zero') #actual data sent in packet is not relevant
        time.sleep(0.1) #built-in holdoff, since reset apparently takes > 50ms

    def startDriveMotors(self):
        self.clAnnounce()
        print "Starting drive motors"
        self.tx( 0, command.PID_START_MOTORS, 'start')
        time.sleep(0.1)

    def stopDriveMotors(self):
        self.clAnnounce()
        print "Stopping drive motors"
        self.tx( 0, command.PID_STOP_MOTORS, 'stop')
        time.sleep(0.1)
            
    def setTailControl(self, tailConfig):
        self.currentTail = tailConfig
        
        self.clAnnounce()
        print " --- Setting complete tail config --- "
        self.setTailGains(tailConfig.motorgains)
        if tailConfig.swing_duration is not None:
            if tailConfig.right_flag is not None:
                self.setRightingInput(tailConfig.pInput, tailConfig.swing_duration)
            else:
                self.setPeriodicInput(tailConfig.pInput, tailConfig.pBias, tailConfig.swing_duration)
        else:
            if tailConfig.vInput is not None:
                self.setTailVel(tailConfig.vInput)
            elif tailConfig.pInput is not None:
                self.setTailPos(tailConfig.pInput)
            else:
                print "WARNING: no tail velocity or tail position set"
        
        self.clAnnounce()
        print " ------------------------------------ "

    def setTailGains(self, gains, retries = 8):
        tries = 1
        self.tailGains = gains
        while not(self.tail_gains_set) and (tries <= retries):
            self.clAnnounce()
            print "Setting tail gains...   ",tries,"/8"
            self.tx( 0, command.SET_TAIL_GAINS, pack('5h',*gains))
            tries = tries + 1
            time.sleep(0.3)

    def setTailVel(self, vInput):
        self.clAnnounce()
        print "Setting tail velocity to",vInput,"Hz"

        temp = vInput*32768/500
        
        self.tx( 0, command.SET_TAIL_VINPUT, pack('h', temp))
        time.sleep(0.1)

    def setTailPos(self, pInput):
        self.clAnnounce()
        print "Setting tail position to",pInput,"degrees"

        temp = pInput*32768/360
        
        self.tx( 0, command.SET_TAIL_PINPUT, pack('h', temp))
        time.sleep(0.1)

    def setRightingInput(self, pInput, period):
        self.clAnnounce()
        print "Autonomous self righting--swing amplitude",pInput,"degrees; swing duration",period,"ms"

        temp = [pInput*32768/360, period]
        
        self.tx( 0, command.SET_TAIL_RINPUT, pack('2h', *temp))
        time.sleep(0.1)

    def setPeriodicInput(self, pAmp, pBias, period):
        self.clAnnounce()
        print "Periodic tail motion--swing amplitude",pAmp,"degrees; position bias",pBias," degrees; swing duration",period,"ms"

        temp = [pAmp*32768/360, pBias*32768/360, period]
        
        self.tx( 0, command.SET_TAIL_PERINPUT, pack('3h', *temp))
        time.sleep(0.1)

    def startTail(self):
        self.clAnnounce()
        print "Starting tail motor"
        self.tx( 0, command.START_TAIL_MOTOR, 'start')
        time.sleep(0.1)

    def stopTail(self):
        self.clAnnounce()
        print "Stopping tail motor"
        self.tx( 0, command.STOP_TAIL_MOTOR, 'stop')
        time.sleep(0.1)
        
    def zeroTailPosition(self):
        self.tx( 0, command.ZERO_TAIL_POS, 'zero') #actual data sent in packet is not relevant
        time.sleep(0.1)

    def startTailTimedRun(self, duration):
        self.clAnnounce()
        print "Starting timed tail run of",duration," ms"
        self.tx( 0, command.START_TAIL_TIMED_RUN, pack('h', duration))
        time.sleep(0.05)
    def setOpenLoopThrust(self, left_pwm, right_pwm):
        print "Setting PWM Left:", left_pwm, " Right:", right_pwm
        temp = [left_pwm, right_pwm]
        self.tx(0, command.SET_MOTOR_MODE, pack('2h', *temp))

        
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