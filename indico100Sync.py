#!/usr/bin/env python
from pcaspy import Driver, SimpleServer, cas
import time, threading, winsound, os, os.path, pickle, socket, platform, sys
from PyDAQmx import *
from epics import *
import numpy as np
from datetime import datetime, timedelta
import psutil, gc
from multiprocessing import Process
sys.path.append(os.path.realpath('../utils'))
import epicsApps

"""
11/18/2015 (AAG)
modifications to support continuous fluoro
add fluoro on support
add fluoro status support
add photospot capabilities 

12/4/2015 (AP)
- added following ioc-stats record:HOSTNAME, ENGINEER, LOCATION, RECORD_CNT, EPICS_VERS, KERNEL_VERS
  These are processed only once when the server is initialized.
- added new batch file to start this server with procServ.

12/13/2015 (AAG)
-auto makes python process high priority when starting
-various timing diagnostic changes
-saves a timing report at the end of scan
-code clean up and addition of some comments

12/17/2015 (AAG)
-started work on live sync mode 

12/21/2015 (AP)
- added following ioc-stats record:STARTTOD, UPTIME and APP_DIR1 for start date of ioc, its uptime and start up directory.
  The uptime record is scanned every second in pcaspys native read method.

1/6/2016 (AAG)
- more work with live sync mode, currently not yet ready
- determined there is a 0.005- 0.01 sec delay from photospot in to cpi outputing expose signal
- we may need faster SSR for the photospot module since the ones we are using only guarantee 5 millisecond max

1/8/2016 (AP)
- changed: DAQ_NAME   = 'FcpiSyncDAQ, USB 6525 is not used to read cpi inputs anymore, eric's custom box converts
  24V signals to ttl which are read in by usb 6501. 

1/9/2016 (AAG)
- only one thread reads the daq now that we only have one module, all input lines are read in one read operation
- removed redundant code 
- modified main program thread(at bottom) so we can cleanly exit with ctrl-c at command prompt
- more improvements/ testing of livesync, now seems to work with 80ms CPI/ 100ms qi2 live mode

Online reference for NI-DAQ
http://zone.ni.com/reference/en-XX/help/370471W-01/


Note "input" and "output" for variables are named from the perspective of the CPI generator, not the ni-daq. (Unless otherwise specified)
This was so the terminology in the documentation for the cpi matches with the names in the code.

"""
#gc.disable()

"""
grab last exposure time so we know when warmup is required
"""
try:
    if os.path.isfile('objs.pickle'):
        with open('objs.pickle') as f:
            lastCpiExposure = pickle.load(f)
    else:
        print 'lastcpiexposure load fail'
        lastCpiExposure = time.time()
except:
        print 'lastcpiexposure load fail'
        lastCpiExposure = time.time()


# make this python process a high priority in Windows
try:
    p = psutil.Process(os.getpid())
    p.nice(psutil.HIGH_PRIORITY_CLASS)
    #p.nice(psuitl.REALTIME_PRIORITY_CLASS) #currently doesn't work 
except:
    print 'failed to make process high priority'


EXPERIMENT          = 'HPFI:'
DAQ_NAME            = 'cpiSync'
SCAN_IOC            =  EXPERIMENT + 'SCAN:'
XRAY_IOC            =  EXPERIMENT + 'CPI:xray:'
DET_IOC             =  EXPERIMENT + 'Qi2:'
# These are the output lines from DAQ (input to CPI etc.) 
CPI_FLUORO_IN       = DAQ_NAME + '/port0/line0' # setting high enables fluoro exposure 
CPI_PHOTOSPOT_IN    = DAQ_NAME + '/port0/line1' # setting high enables photospot (rad enable)
CPI_EXPOSE_IN       = DAQ_NAME + '/port0/line2' # setting high allows exposure
CPI_POWER_OFF_LINE  = DAQ_NAME + '/port0/line4' # pulse turns CPI off
CPI_POWER_ON_LINE   = DAQ_NAME + '/port0/line5' # pulse turns CPI on
CPI_RAD_PREP_IN     = DAQ_NAME + '/port0/line6' # setting high makes enables rad prep
QI2_EXPOSE          = DAQ_NAME + '/port2/line0' # Output from DAQ to Qi2 requests exposure
# These are the DAQ inputs (output from CPI etc.)
QI2_TRIGGERREADY    = DAQ_NAME  + '/port2/line1' # High when Qi2 ready to expose
QI2_EXPOSEOUT       = DAQ_NAME  + '/port2/line2' # QI2_EXPOSEOUT is high when it is exposing
CPI_RAD_PREP_OUT    = DAQ_NAME  + '/port1/line0'
CPI_RAD_READY_OUT   = DAQ_NAME  + '/port1/line1'
CPI_EXPOSE_OUT      = DAQ_NAME  + '/port1/line2'
CPI_FLUORO_OUT      = DAQ_NAME  + '/port1/line3'

# These are the lines that will be polled for change detection
INPUT_POLL= CPI_RAD_PREP_OUT + "," + CPI_RAD_READY_OUT + "," + CPI_EXPOSE_OUT  + "," + CPI_FLUORO_OUT + "," + QI2_TRIGGERREADY + "," + QI2_EXPOSEOUT 
# EPICS scan variables to keep track of scan so we know when to start/stop rad prep 
SCAN_DETECTOR_1     = PV(SCAN_IOC + 'scan1.T1PV', callback = False)
# very important to use scanProgress record instead of scan, so we can keep track of multi-dimensional scans
SCANPROGRESSIOC     = SCAN_IOC + 'scanProgress:' 
# So we know to immediately shut off ranode if scan is canceled, and initiate abort() function
SCAN_CANCEL_IOC     = PV(SCAN_IOC + 'AbortScans.PROC', callback = True)
# Keep track of scan status 
NFINISHED_PV        = PV(SCANPROGRESSIOC +'Nfinished')
NTOTAL_PV           = PV(SCANPROGRESSIOC +'Ntotal')
CPT_PV              = PV(SCAN_IOC + 'scan1.CPT')
NPTS_PV             = PV(SCAN_IOC + 'scan1.NPTS') 
# Functions to allow changes of CPI settings for automatic warm-up procedures 
CPI_KVP_PV          = PV(XRAY_IOC + 'SetKVP')
CPI_MA_PV           = PV(XRAY_IOC + 'SetMA')
CPI_MS_PV           = PV(XRAY_IOC + 'SetMS')
CPI_SETFOCUS_PV     = PV(XRAY_IOC + 'SetFocus')
# What PV's to grab the filepath and filename from for text doc writing
FILEPATH_PV         = PV(DET_IOC + 'TIFF1:FilePath')
FILENAME_PV         = PV(DET_IOC + 'TIFF1:FileName', callback = True) #reset Filenum to zero when called
#Constant numpy arrays for setting digital outputs high or low on NI-DAQs
LOW  = numpy.zeros((1,), dtype=numpy.uint8)
HIGH = numpy.ones((1,), dtype=numpy.uint8)

#How often to pause in "while" loops
POLL_TIME = 0.001

# List of PV's to save to text file if DOC is ON (1).
HPFI_PV_LIST = ['HPFI:KOHZU:m1.RBV',  'HPFI:KOHZU:m2.RBV',  'HPFI:KOHZU:m3.RBV',  \
                'HPFI:KOHZU:m4.RBV',  'HPFI:KOHZU:m5.RBV',  'HPFI:KOHZU:m6.RBV',  \
                'HPFI:KOHZU:m7.RBV',  'HPFI:KOHZU:m8.RBV',  'HPFI:KOHZU:m9.RBV',  \
                'HPFI:KOHZU:m10.RBV', 'HPFI:KOHZU:m11.RBV', 'HPFI:KOHZU:m12.RBV', \
                'HPFI:KOHZU:m13.RBV', 'HPFI:KOHZU:m14.RBV', 'HPFI:KOHZU:m15.RBV', \
                'HPFI:KOHZU:m16.RBV', 'HPFI:KOHZU:m17.RBV', 'HPFI:KOHZU:m18.RBV', \
               ]
PV_LIST      = HPFI_PV_LIST
prefix = EXPERIMENT + 'cpiSync:'

pvdb = {
    'ON'                    : { 'asyn' : True },
    'OFF'                   : { 'asyn' : True },
    'SEND_TRIGGER'          : { 'asyn' : True },
    'RAD_PREP'              : { 'asyn' : True },
    'EXPOSE'                : { 'asyn' : True },
    'FLUORO'                : { 'asyn' : True },
    'PHOTOSPOT'             : { 'asyn' : True },
    'RAD_PREP_RBV'          : { 'asyn' : True },
    'RAD_READY_RBV'         : { 'asyn' : True },
    'EXPOSE_RBV'            : { 'asyn' : True },
    'FLUORO_RBV'            : { 'asyn' : True },
    'ABORT'                 : { 'asyn' : True },
    'NikonSingleExposeSeq'  : { 'asyn' : True },
    'NikonScanExposeSeq'    : { 'asyn' : True },
    'WARM'                  : { 'asyn' : True },
    'FULL_WARM'             : { 'asyn' : True },
    'STATUS_RBV'            : { 'type': 'char',
                                'count': 300,
                                'value': 'Initialization'},
    'LAST_EXPOSE_TIME_RBV'  : { 'type': 'string',
                                'scan': 1},
    'TRIGGER_READY_RBV'     : { 'asyn' : True},
    'EXPOSING_RBV'          : { 'asyn' : True},
    'DOC'                   : { },
    'GEN_DELAY'             : {  'value': 0.02,# 20 milliseconds
                                 'prec': 3} ,
    'HighSpeedCpiTest'      : { },
    'SeqStarttoQi2'         : { },
    'TimeBetweenReq'        : { },
    'Qi2ReqToStart'         : { },
    'CpiReqToStart'         : { },
    'CpiStarttoQi2Start'    : { },
    'Qi2Duration'           : { },
    'CpiDuration'           : { },#'count': 1000 
    'DutyCycle'             : { }, 
    'ShotDuration'          : { },                    
}

pvdb.update(epicsApps.pvdb)


class myDriver(Driver):
    def  __init__(self):
        super(myDriver, self).__init__()
        self.start_time = datetime.now()
        self.iocStats()
        SCAN_CANCEL_IOC.add_callback( callback=self.ScanMonitor)

        self.lastCpiExposure = lastCpiExposure # keep track of most recent exposure
        self.cancel = 0 # cancel flag for abort procedure 
        self.currentFunction = '' # initialize current function string used in status messages
        self.seqInProgress=0 # flag to determine if we are inside a "sequence" or not
        # high speed scans
        self.firstImageFlag=1
        self.qi2EndFlag = False

        # the following variable is needed for pydaqmx
        self.written = int32()
        
        #set up the warning sounds which will play out of computer speakers (needs to be unmuted)
        #self.PrepSound=False
        #sid=threading.Thread(target=self.warningsound)
        #sid.start()
        
        #Setup DO lines
        self.photospotInTask = TaskHandle()
        self.setupOutput(self.photospotInTask, CPI_PHOTOSPOT_IN, True)
        self.Qi2TriggerTask = TaskHandle()
        self.setupOutput(self.Qi2TriggerTask, QI2_EXPOSE, True)
        self.powerOffTask = TaskHandle()
        self.setupOutput(self.powerOffTask, CPI_POWER_OFF_LINE, True)
        self.powerOnTask = TaskHandle()
        self.setupOutput(self.powerOnTask, CPI_POWER_ON_LINE, True)
        self.radPrepInTask = TaskHandle()
        self.setupOutput(self.radPrepInTask, CPI_RAD_PREP_IN, True)
        self.exposeInTask = TaskHandle()
        self.setupOutput(self.exposeInTask, CPI_EXPOSE_IN, True)
        self.fluoroInTask = TaskHandle()
        self.setupOutput(self.fluoroInTask, CPI_FLUORO_IN, True)

        #task to read DI lines combined into one (significantly improves performance to do one read)
        self.combinedTask = TaskHandle()
        DAQmxCreateTask("",byref(self.combinedTask))
        DAQmxCreateDIChan(self.combinedTask, INPUT_POLL, "", DAQmx_Val_ChanForAllLines)
        DAQmxStartTask(self.combinedTask)      

        ##start the polling thread
        self.cid = threading.Thread(target=self.pollInputs,args=())
        self.cid.start()

        #timing
        self.prepTime=0
        self.scanStartTime=0
        self.scanExposeRequestTime=0
        self.qi2ExposeReqeustTime=0
        self.qi2ExposeStartTime=0
        self.qi2ExposeEndTime=0
        self.cpiExposeRequestTime=0
        self.cpiExposeStartTime=0
        self.cpiExposeEndTime=0
        self.lastqi2ExposeEndTime=0
        self.scanExpEndTime=0
        self.qi2DurationList = []
        self.cpiDurationList = []
        self.dutyCycleList = []
        self.cpiRequestList = []
        self.qi2RequestList = []
        self.scanExpSeqList = []
        # Set scan detector PV to NikonSync hard trigger PV on init.
        SCAN_DETECTOR_1.put(EXPERIMENT + 'cpiSync:NikonScanExposeSeq')
    #    epicsApps.buildRequestFiles(prefix, pvdb.keys(), os.getcwd())
    #    epicsApps.makeAutosaveFiles()
        #Now ready to do exposures, change from Initilization to Idle 
        self.write('STATUS_RBV', 'Idle')
        print str(datetime.now())[:-3], 'cpiSync succesfully initialized'

    def iocStats(self):
        """
        Sets the iocAdmin related records
        """
        self.start_time = datetime.now()
        self.setParam('ENGINEER', 'Andrew Gomella')
        self.setParam('LOCATION', 'B1D521D SVR-SMWIN122')
        self.setParam('RECORD_CNT', len(pvdb.keys()))
        self.setParam('APP_DIR1', str(os.getcwd()))
        self.setParam('UPTIME', str(self.start_time))
        self.setParam('PARENT_ID', os.getpid())
        self.setParam('HEARTBEAT', 0)

    #Generic function for setting up daq output
    def setupOutput(self, taskName, lineLocation, setLow):
        DAQmxCreateTask("",byref(taskName))
        DAQmxCreateDOChan(taskName, lineLocation, "", DAQmx_Val_ChanForAllLines)
        DAQmxSetDOOutputDriveType(taskName, lineLocation, DAQmx_Val_ActiveDrive)
        DAQmxStartTask(taskName)   
        if setLow == True:
            self.setDigiOut(taskName, LOW)

    #since the ni daq 6501 doesnt support change detection, we have to poll and do change detection ourselves
    def pollInputs(self):
        oldval = np.array([0,0,0,0,0,0], dtype=np.uint8)
        newval = np.array([0,0,0,0,0,0], dtype=np.uint8)
        DAQmxReadDigitalLines(self.combinedTask, 1, 1, 0,  oldval, 6, None, None, None)
        while True:
            startPollTime = time.clock()
            DAQmxReadDigitalLines(self.combinedTask, 1, 1, 0, newval, 6, None, None, None) 
            #print newval
            #print 'daqreadtime', time.clock() - startPollTime    
            #setParamTime = time.clock()       
            self.setParam("RAD_PREP_RBV",      newval[0])
            self.setParam("RAD_READY_RBV",     newval[1])
            self.setParam("EXPOSE_RBV",        newval[2])
            self.setParam("FLUORO_RBV",        newval[3])
            self.setParam("TRIGGER_READY_RBV", newval[4])
            self.setParam("EXPOSING_RBV",      newval[5])
            self.updatePVs()
            #print 'updatePvTime', time.clock() - setParamTime          
            #if np.all(np.sort(oldval)!=np.sort(newval)):
            if newval[4] != oldval[4]:
                if newval[4] == 1:
                    print str(datetime.now())[:-3], 'Qi2 END EXPOSURE', '%.4f'%(time.clock()- self.scanExposeRequestTime)
                    self.qi2ExposeEndTime=time.clock()
                    #post exposure report
                    #time from last exposure
                    if self.lastqi2ExposeEndTime != 0:
                        #print 'Time between qi2 expose end signal (exposure duty cycle)', self.qi2ExposeEndTime-self.lastqi2ExposeEndTime
                        self.setParam('DutyCycle', self.qi2ExposeEndTime-self.lastqi2ExposeEndTime)
                        self.dutyCycleList.append(self.qi2ExposeEndTime-self.lastqi2ExposeEndTime)
                    #time from scan expose request to qi2 exp request
                    #print 'Expose Request to Qi2 request', self.qi2ExposeReqeustTime-self.scanExposeRequestTime
                    #time from qi2 exp request to cpi exp request
                    #print 'Expose Qi2 Request to Cpi Request', self.cpiExposeRequestTime-self.qi2ExposeReqeustTime
                    self.setParam('TimeBetweenReq',self.cpiExposeRequestTime-self.qi2ExposeReqeustTime)
                    #time from qi2 request to qi2 start
                    #print 'Qi2 Request to Qi2 Start', self.qi2ExposeStartTime -self.qi2ExposeReqeustTime
                    self.setParam('Qi2ReqToStart',self.qi2ExposeStartTime -self.qi2ExposeReqeustTime)
                    self.qi2RequestList.append(self.qi2ExposeStartTime -self.qi2ExposeReqeustTime)
                    #time from cpi request to cpi start
                    #print 'Cpi Request to Cpi Start', self.cpiExposeStartTime-self.cpiExposeRequestTime
                    self.setParam('CpiReqToStart',self.cpiExposeStartTime-self.cpiExposeRequestTime)
                    self.cpiRequestList.append(self.cpiExposeStartTime-self.cpiExposeRequestTime)
                    #time from qi2 start to cpi start
                    #print 'Cpi Start to Qi2 Start', self.cpiExposeStartTime-self.qi2ExposeStartTime
                    self.setParam('CpiStarttoQi2Start',self.cpiExposeStartTime-self.qi2ExposeStartTime)
                    #time from qi2 start to qi2 end
                    #print 'Qi2 Start to Qi2 End', self.qi2ExposeEndTime-self.qi2ExposeStartTime
                    self.setParam('Qi2Duration', self.qi2ExposeEndTime-self.qi2ExposeStartTime)
                    self.qi2DurationList.append(self.qi2ExposeEndTime-self.qi2ExposeStartTime)
                    #time from cpi start to cpi end
                    #print 'Cpi Start to Cpi End', self.cpiExposeEndTime-self.cpiExposeStartTime
                    #self.listTest.append((self.cpiExposeEndTime-self.cpiExposeStartTime))
                    #self.setParam('CpiDuration', self.listTest)                   
                    self.setParam('CpiDuration', self.cpiExposeEndTime-self.cpiExposeStartTime)
                    self.cpiDurationList.append(self.cpiExposeEndTime-self.cpiExposeStartTime)
                    self.lastqi2ExposeEndTime=self.qi2ExposeEndTime
            if newval[5] != oldval[5]:
                if newval[5] == 1:
                    #print 'qi2 zero time ', time.clock()-self.qi2ExposeEndTime
                    print str(datetime.now())[:-3], 'Qi2 START EXPOSURE', '%.4f'%(time.clock()- self.scanExposeRequestTime)
                    self.qi2ExposeStartTime=time.clock()
                else:
                    self.write('SEND_TRIGGER',0)
                    self.qi2EndFlag=True
                    self.qi2ExposeEndTime=time.clock()
                    print str(datetime.now())[:-3], 'Qi2 END EXPOSURE (LIVE)', '%.4f'%(time.clock()- self.scanExposeRequestTime)
            #check cpi expose out
            if newval[2] != oldval[2]:
                if newval[2] == 1:
                    print str(datetime.now())[:-3], 'GENERATOR RAD ENABLE', '%.4f'%(time.clock()- self.scanExposeRequestTime)
                    self.cpiExposeStartTime=time.clock()
                    print 'time between cpi exposures, ', self.cpiExposeStartTime - self.cpiExposeEndTime
                else:
                    self.write("PHOTOSPOT", 0)
                    #if self.getParam("PHOTOSPOT")==1:
                    #    self.write("PHOTOSPOT", 0)
                    print str(datetime.now())[:-3], 'GENERATOR RAD END', '%.4f'%(time.clock()- self.scanExposeRequestTime)
                    self.cpiExposeEndTime=time.clock()
                    #only update lastcpiexposure if it was a real exposure (not toggle during cpi boot)
                    #if RadReadyOut (newval[1]) is 1, then it probably was a real exposure
                    if newval[1] == 1:
                        self.lastCpiExposure = time.time()
            self.updatePVs()
            #Note: the below line is for storing the old value of array (before new read)
            # it must use the copy method, otherwise it will access the location of the array
            oldval = newval.copy()
            endPollTime = time.clock()
            if endPollTime - startPollTime > 0.002:
                print str(datetime.now())[:-3], 'DAQ Poll > 2 ms !', endPollTime - startPollTime

    def read(self, reason):
        format_time = ""
        if reason == 'LAST_EXPOSE_TIME_RBV' :
            secsSinceLastExposure = time.time() - self.lastCpiExposure
            value = self.printNiceTimeDelta(secsSinceLastExposure)
            if secsSinceLastExposure > 28800:
                self.setParam('STATUS_RBV', 'More than 8 hours since last exposure- run warm-up')
                self.updatePVs()
        elif reason == 'UPTIME':
            format_time = datetime.now() - self.start_time
            value  = str(format_time).split(".")[0] 
        elif reason == 'TOD':
            value = str(datetime.now().strftime("%m/%d/%Y %H:%M:%S"))
        elif reason == 'HEARTBEAT':
            value = self.getParam('HEARTBEAT') + 1
            self.setParam('HEARTBEAT', value)
        else:
            value = self.getParam(reason)
        self.updatePVs()
        return value

    def write(self, reason, value):
        self.setParam(reason, value)
        self.updatePVs()
        if  reason == 'RAD_PREP':
            if value == 1:
                self.did = threading.Thread(target = self.RadPrepOn, args = ())
                self.did.start()
            else:
                self.did = threading.Thread(target = self.RadPrepOff, args = ())
                self.did.start()
            if self.seqInProgress == 0 and self.cancel == 0:
                self.currentFunction = '(Manual)'    
        elif reason == 'EXPOSE':
            if (self.getParam("RAD_PREP_RBV") == 1 and value == 1 ) or value == 0:
                self.did = threading.Thread(target = self.setDigiOut, args = (self.exposeInTask, value))
                self.did.start()
                if value == 0:
                    with open('objs.pickle', 'w') as f:
                        pickle.dump(self.lastCpiExposure, f)
                if self.seqInProgress == 0 and self.cancel == 0:
                    self.currentFunction = '(Manual)'
        elif reason == 'FLUORO':
            self.zid = threading.Thread(target = self.setDigiOut, args = (self.fluoroInTask, value))
            self.zid.start()
            if self.seqInProgress == 0 and self.cancel == 0:
                self.currentFunction = '(Manual)'
        elif reason == 'PHOTOSPOT':
            self.mid = threading.Thread(target = self.setDigiOut, args = (self.photospotInTask, value))
            self.mid.start()
        elif reason == 'SEND_TRIGGER':
            self.niTriggerTime=time.clock()
            self.fid = threading.Thread(target = self.setDigiOut, args = (self.Qi2TriggerTask, value))
            self.fid.start()
        elif reason == 'ON' and value == 1:
            self.eid = threading.Thread(target = self.On, args = ())
            self.eid.start()
        elif reason == 'OFF' and value == 1:
            self.fid = threading.Thread(target = self.Off, args = ())
            self.fid.start()
        elif reason == 'NikonSingleExposeSeq' and value == 1:
            print str(datetime.now())[:-3],'SingleExpose requested 0.0000'
            self.scanExposeRequestTime=time.clock()
            #if Qi2 trigger isn't ready the user likely forgot to enable collection on Qi2
            # otherwise may be a problem with the Qi2 
            if self.getParam('TRIGGER_READY_RBV')==1:
                #self.NikonSingleExposeSeq()
                self.fid = threading.Thread(target = self.NikonSingleExposeSeq, args = ())
                self.fid.start()
            else:               
                self.write('STATUS_RBV',  'Error, Qi2 not ready to expose. ')
        elif reason == 'NikonScanExposeSeq' and value == 1:
            print str(datetime.now())[:-3],'ScanExpose requested 0.0000'
            self.scanExposeRequestTime=time.clock()
            #self.NikonScanExposeSeq()
            self.fid = threading.Thread(target = self.NikonScanExposeSeq, args = ())
            self.fid.start()
            #if self.getParam('TRIGGER_READY_RBV')!=1:
            #    self.write('Status',  'Error, Qi2 not ready to expose. ')
        elif reason == 'WARM' and value == 1:
            self.fid = threading.Thread(target = self.GenWarmUpSeq, args = ())
            self.fid.start()
        elif reason == 'FULL_WARM' and value == 1:
            self.fid = threading.Thread(target = self.GenWarmUpSeqFull, args = ())
            self.fid.start()
        elif reason == 'HighSpeedCpiTest' and value == 1:
            self.fid = threading.Thread(target = self.liveSync, args = ()) 
            self.fid.start()
        elif reason == 'ABORT' and value == 1:
            self.fid = threading.Thread(target = self.abort, args = ())
            self.fid.start()
        elif reason == 'STATUS_RBV':
            print str(datetime.now())[:-3], value
        self.updatePVs()

    def document(self):
        print str(datetime.now())[:-3],'Generate Doc String Start'
        pathname = self.filepath + '\\' + self.filename + '_' + str(self.filenum).zfill(3)
        try:
            f = open(pathname + '.txt', 'w')
            for pvName in PV_LIST:
                f.write(pvName + ' - ' + str(caget(pvName)) + '\n')
            print str(datetime.now())[:-3],'Generate Doc String Finish'
        except:
            print str(datetime.now())[:-3], "Error writing doc string", Exception

    def GenWarmUpSeq(self):
        self.currentFunction = '(Gen. Warm-Up)'
        print str(datetime.now())[:-3], '!!!!! Short GenWarmUp Sequence Start !!!!!'
        self.seqInProgress=1
        self.GenExposeOnly(80,  25, 100, 3, 1)
        time.sleep(1)
        self.GenExposeOnly(100, 25, 100, 3, 1)
        time.sleep(1)
        self.GenExposeOnly(125, 25, 100, 3, 1)
        self.seqInProgress=0
        if self.cancel == 0 :
            self.write('STATUS_RBV', self.currentFunction + 'Complete')

    def GenWarmUpSeqFull(self):
        self.currentFunction = '(Full Gen. Warm-Up)'
        print str(datetime.now())[:-3], '!!!!! Full GenWarmUp Sequence Start !!!!!'
        self.seqInProgress=1
        CPI_SETFOCUS_PV.put(1, wait=True) #set focus to large    
        self.GenExposeOnly(80,  200, 2000, 6, 5)
        if self.cancel == 0:
            time.sleep(1)
        self.GenExposeOnly(90,  320, 100, 3, 1)
        if self.cancel == 0:
            time.sleep(1)
        self.GenExposeOnly(100,  320, 100, 3, 1)
        if self.cancel == 0:
            time.sleep(1)
        self.GenExposeOnly(110,  320, 100, 3, 1)
        if self.cancel == 0:
            time.sleep(1)
        self.GenExposeOnly(120,  320, 100, 3, 1)
        CPI_SETFOCUS_PV.put(0, wait=True) #set focus back to small
        self.seqInProgress=0
        if self.cancel == 0:
            self.write('STATUS_RBV', self.currentFunction + 'Complete')

    def GenExposeOnly(self, kvp, current, millisec, numberOfExposures, dutyCycle):
        if self.cancel == 0:
            self.write('STATUS_RBV', self.currentFunction + ' Changing Settings')
            CPI_KVP_PV.put(kvp, wait=True)
            time.sleep(.1)
            CPI_MA_PV.put(current, wait=True)
            time.sleep(.1)
            CPI_MS_PV.put(millisec, wait=True) 
            time.sleep(.1)
            self.prepExpose() # returns when ready to expose
            for count in range(0, numberOfExposures):
                if self.cancel == 1:
                    break
                # send trigger signal to CPI expose
                self.write('STATUS_RBV', self.currentFunction + ' EXPOSING!')
                self.write('PHOTOSPOT', 1)
                # wait for exposure to finish
                while self.getParam('PHOTOSPOT') == 1 and self.cancel == 0:
                    time.sleep(POLL_TIME)
                #self.write('PHOTOSPOT', 0)
                self.write('STATUS_RBV', self.currentFunction + ' wait duty cycle')
                if self.cancel == 1:
                    break
                time.sleep(dutyCycle)
            if self.cancel == 0:
                self.write('EXPOSE',0)
                self.write('RAD_PREP',0)
                # wait for rad prep to turn off
                while self.getParam('RAD_READY_RBV') == 1:
                    time.sleep(POLL_TIME)

    #Take single isolated Nikon-CPI sync shot
    def NikonSingleExposeSeq(self):
        self.currentFunction = '(Nikon Single Shot)'
        print str(datetime.now())[:-3], '!!!!! Nikon - CPI Single Shot Sequence Start !!!!!'
        self.seqInProgress=1 #let abort sequence know this function is in progress
        self.prepExpose()         #do chores and prepare gen (during this time camera has more time to prepare)
        self.checkIfCameraReady() # see if camera is actually ready to acquire
        self.exposeNow()          #send trigger signals, returns when exposure is over
        self.exposeEnd()          #resets nikon trigger, turns off cpi expose signal
        if self.cancel == 0 :   #if canceled rad prep is already off and we don't want status to be idle
            self.write('RAD_PREP',0)   #single expose, so turn RadPrep off
            self.write("EXPOSE",0)
            self.write('STATUS_RBV', 'Idle')
        if caget(SCAN_IOC + "scan1.P1PV") == XRAY_IOC + "SetKVP":
            caput(SCAN_IOC + '.WAIT', 0) #tell scan we're finished acquiring the image and it can progress
        self.seqInProgress=0 #let abort sequence know this function is over
        print str(datetime.now())[:-3], '!!!!! Nikon - CPI Single Shot Sequence Over !!!!!'

    #Take Nikon-CPI sync shot during a scan
    # ideally we will have the scan performing its next move while various nikon-cpi sync chores are also taking place
    def NikonScanExposeSeq(self):
        self.currentFunction = '(Nikon Scan Shot)'
        print str(datetime.now())[:-3], '!!!!! Nikon - CPI Single Scan Shot Sequence Start !!!!!'
        self.seqInProgress=1 #let abort sequence know this function is in progress
        self.prepExpose() #do chores and prepare gen (do nothing if gen is already ready)
        self.checkIfCameraReady() #chores finished, see if camera is actually ready
        self.exposeNow() #Returns when exposure is over
        self.exposeEnd()
        self.scanExpEndTime=time.clock()
        if self.prepTime == 0:
            self.scanExpSeqList.append(time.clock()-self.scanExposeRequestTime)
            self.setParam('ShotDuration', self.scanExpEndTime -self.scanExposeRequestTime)
        else:
            self.setParam('ShotDuration', (self.scanExpEndTime-self.scanExposeRequestTime)-self.prepTime) 
            self.scanExpSeqList.append(time.clock()-self.scanExposeRequestTime-self.prepTime)
            self.prepTime=0
        if self.cancel == 0:
            #check if this was the last shot of the scan
            #if caget(SCANPROGRESSIOC+'Nfinished') + 1 == caget(SCANPROGRESSIOC +'Ntotal') or caget(SCAN_IOC + '.CPT') + 1 == caget(SCAN_IOC + '.NPTS'): #just took last image, so turn off radprep
            if NFINISHED_PV.get() + 1 == NTOTAL_PV.get() or CPT_PV.get() + 1 == NPTS_PV.get():             
                print str(datetime.now())[:-3], "RadPrep turning off!"
                self.write('RAD_PREP',0)
                self.write("EXPOSE",0)
                self.write('STATUS_RBV', 'Idle')
                #print post scan timing report
                reportfilename= self.filepath + '\\' + self.filename + '_Timing_'+time.strftime("%Y%m%d-%H%M%S")+ '.txt'
                f1=open(reportfilename, 'w+')
                #print 'qi2 duration num mean stddev min max',len(self.qi2DurationList), np.mean(self.qi2DurationList), np.nanstd(self.qi2DurationList), np.amin(self.qi2DurationList), np.amax(self.qi2DurationList)
                #print 'cpi duration num mean stddev min max',len(self.cpiDurationList), np.mean(self.cpiDurationList), np.nanstd(self.cpiDurationList), np.amin(self.cpiDurationList), np.amax(self.cpiDurationList)
                #print 'dutycycle num mean stddev min max',len(self.dutyCycleList), np.mean(self.dutyCycleList), np.nanstd(self.dutyCycleList), np.amin(self.dutyCycleList), np.amax(self.dutyCycleList)
                f1.write('CPI-Qi2 Scan Time Report\n')
                f1.write('Scan duration ' + str((time.clock()-self.scanStartTime)) + '\n')
                f1.write('Prep Time ' + str(self.prepTime) + '\n')
                f1.write('Number of points ' + str(NPTS_PV.get()) + '\n')
                f1.write('Points/Duration ("FPS") '+ str(((time.clock()-self.scanStartTime)-self.prepTime)/NPTS_PV.get()) + '\n')
                #f1.write('Time lost to scan motors/scan processing ', str(  )  )
                f1.write('qi2 duration num mean stddev min max ' + str(len(self.qi2DurationList)) + ' ' + str(np.mean(self.qi2DurationList)) + ' ' + str(np.nanstd(self.qi2DurationList)) + ' ' + str(np.amin(self.qi2DurationList)) + ' ' + str(np.amax(self.qi2DurationList)) + '\n')
                f1.write('cpi duration num mean stddev min max  '+str(len(self.cpiDurationList)) + ' ' + str(np.mean(self.cpiDurationList)) + ' ' + str(np.nanstd(self.cpiDurationList))+ ' ' + str(np.amin(self.cpiDurationList))+ ' ' +  str(np.amax(self.cpiDurationList))+ '\n')
                f1.write('dutycycle num mean stddev min max  '+ str(len(self.dutyCycleList))+ ' ' + str(np.mean(self.dutyCycleList))+ ' ' + str( np.nanstd(self.dutyCycleList)) + ' ' + str(np.amin(self.dutyCycleList))+ ' ' + str(np.amax(self.dutyCycleList))+ '\n')
                f1.write('single shot length num mean stddev min max  '+ str(len(self.scanExpSeqList))+ ' ' + str(np.mean(self.scanExpSeqList))+ ' ' + str( np.nanstd(self.scanExpSeqList)) + ' ' + str(np.amin(self.scanExpSeqList))+ ' ' + str(np.amax(self.scanExpSeqList))+ '\n')
                f1.close()
                self.scanExpSeqList = []
                self.qi2DurationList = []
                self.cpiDurationList = []
                self.dutyCycleList = []
                self.cpiRequestList = []
                self.qi2RequestList = []
            else:
                self.write('STATUS_RBV', self.currentFunction + ' Wait sscan ')
        if self.cancel == 0:
            caput(SCAN_IOC + 'scan1.WAIT', 0) # tell scan we're finished acquiring the image and it can progress
        self.seqInProgress=0 # let abort sequence know this function is over

        print str(datetime.now())[:-3], '!!!!! Nikon - CPI Single Scan Shot Sequence Over !!!!!'

    #Checks if gen is ready to expose, if not prepares generator for an exposure, otherwise return
    #   function should only ever run at the beginning of a single shot OR a scan, never in the middle
    #   so we can do house keeping tasks here as well while waiting for rad gen ready
    #Returns when generator is ready to expose
    def prepExpose(self):
        if self.getParam('RAD_READY_RBV') == 0 and self.cancel == 0:
            self.lastqi2ExposeEndTime=0
            self.scanStartTime=time.clock()
            #ENABLE RAD PREP          
            self.write('RAD_PREP', 1) #sets rad prep in to CPI to 1
            self.write('STATUS_RBV', self.currentFunction + ' Wait RadReady')
            #Signal is sent to radprep, will take a few seconds, so do housekeeping-
            #   -grab info for doc string, filenum will change during scan so we will increment it later
            self.filepath=FILEPATH_PV.char_value
            self.filename=FILENAME_PV.char_value
            self.filenum=caget(DET_IOC + 'TIFF1:FileNumber')
            self.docmode=self.getParam('DOC')
            while self.getParam('RAD_READY_RBV') == 0 and self.cancel == 0:
                time.sleep(POLL_TIME)
            #ENABLE PHOTOSPOT 
            self.write("EXPOSE", 1) #request for photospot
            print str(datetime.now())[:-3], 'Wait for generator serial confirmation of Exposure Status'
            while caget(XRAY_IOC + "GeneratorStatus") != 4:
                time.sleep(POLL_TIME)
                if self.cancel == 1:
                    break
            if self.cancel != 1:
                time.sleep(.002)
            self.prepTime=time.clock()-self.scanStartTime

    #Check to see if camera is ready to take an image (to be run immediately before exposeNow)
    # -indifferent to scan status
    def checkIfCameraReady(self):
        if self.cancel == 0:
            #For Qi2 this is extremely simple. If TriggerReady is high, it is ready to expose
            startTriggerWait = time.clock()
            if self.getParam('TRIGGER_READY_RBV') != 1:
                print str(datetime.now())[:-3], 'Wait Qi2 TriggerReady', '%.4f'%(time.clock()- self.scanExposeRequestTime)
                self.write('STATUS_RBV', self.currentFunction + ' Wait Qi2 Trigger Ready')
                while self.getParam('TRIGGER_READY_RBV') != 1:
                    time.sleep(POLL_TIME)
            print str(datetime.now())[:-3], 'Qi2 TriggerReady', '%.4f'%(time.clock()- startTriggerWait)

    #Call when immediately ready to send sync signals camera + x-ray
    #   -Sends trigger signal to  Qi2 and waits a specified GenDelay then sends CPI trigger
    #   -indifferent to scan status
    def exposeNow(self):
        if self.cancel == 0:
            self.write('STATUS_RBV', self.currentFunction + ' EXPOSING!')
            self.write('SEND_TRIGGER', 1) # send trigger release signal to nikon
            self.qi2ExposeReqeustTime=time.clock()
            self.setParam('SeqStarttoQi2', self.qi2ExposeReqeustTime-self.scanExposeRequestTime)
            print str(datetime.now())[:-3], 'Qi2 Expose request sent', '%.4f'%(time.clock()- self.scanExposeRequestTime)
            #while True:
            #    if self.getParam("EXPOSING_RBV") == 1:
            #        break
            time.sleep(self.getParam('GEN_DELAY'))
            print str(datetime.now())[:-3], 'Generator Expose request sent', '%.4f'%(time.clock()- self.scanExposeRequestTime)
            self.write('PHOTOSPOT', 1)
            self.cpiExposeRequestTime=time.clock()
            # Wait for CPI to be exposing
            while self.getParam("EXPOSE_RBV")==0:
                time.sleep(POLL_TIME)
                if self.cancel == 1:
                    break
            if self.docmode == 1: # to save time generate doc string during exposure, usually takes less than 100ms
                self.document()
                self.filenum=self.filenum + 1 #increment local count of filenum, might be overwritten later
           # Wait for end of exposure AND Qi2 exposure, unless something sets cancel flag to 1, then return
            print 'wait for expose to stop or qi2expose to stop'
            while self.getParam("EXPOSE_RBV") == 1 or self.getParam("EXPOSING_RBV") == 1:
                 time.sleep(POLL_TIME)
                 if self.cancel == 1:
                     break
            #W ait for CPI to finish exposing 
            while self.getParam("EXPOSE_RBV") == 1:
                time.sleep(POLL_TIME)
                if self.cancel == 1:
                    break
            #while self.getParam("EXPOSING_RBV") == 1:
            #    time.sleep(POLL_TIME)
            #    if self.cancel == 1:
            #        break
            self.write('PHOTOSPOT',0)
            self.write('SEND_TRIGGER',0)
            time.sleep(0.025)
    
    #experimental alternative to sleep
    def busy_wait(self, dt):   
        current_time = time.time()
        while (time.time() < current_time+dt):
            pass

    #Call after nikon exposure has finished to set up for the next exposure, calculate image rate, update buffer
    #   -indifferent to scan status
    def exposeEnd(self):
        if self.cancel == 0:
            #print str(datetime.now())[:-3], 'resetting qi2 exposure output'
            with open('objs.pickle', 'w') as f:
                pickle.dump(self.lastCpiExposure, f)
            self.updatePVs()

    #sync cpi to qi2 when in live mode
    # we have no control over qi2 frame start and end while live mode is active
    #   (though the qi2 is very consistent in its signal output)
    # therefore we attempt to sync cpi exposures to the signals we recieve from the qi2
    #  in this sense the qi2 acts as the master clock 
    #  ideally we will start a timer after every qi2 exposeout transition to zero
    #       then exactly X seconds later we will trigger photospotin to cpi high
    #      times must be chose such that they 
    def liveSync2(self):
        self.prepExpose()
        delayPhotospotToExposeOut=.015
        delayQi2ExposeOfftoOn=.043
        self.liveRecoveryPeriod=delayQi2ExposeOfftoOn - delayPhotospotToExposeOut
        while self.getParam('HighSpeedCpiTest') == 1 and self.cancel == 0:
            if self.getParam('EXPOSING_RBV') == 0:
                if self.firstImageFlag == 1:
                    caput(DET_IOC + 'cam1:Acquire', 1)
                    offset = 0 
                    ##
                    #wait for one frame to finish, and on transition to 0 continue
                    while(self.getParam('EXPOSING_RBV')==0) and self.cancel == 0:
                        time.sleep(POLL_TIME)
                    while(self.getParam('EXPOSING_RBV')==1) and self.cancel == 0:
                        time.sleep(POLL_TIME)
                else: #not the first image, but qi2 exposure already over so we need to account for lateness
                    #offset is time since qi2 exposure ended 
                    print 'THREAD LATE: qi2 already finished exposing attempting to compensate'
                    offset = time.clock() - self.qi2ExposeEndTime
            else: # qi2 is exposing and we need to wait for it to transition to 0 before sleeping
                offset = 0 
                print '!!!wait for qi2 to go to zero!!!!'
                while(self.getParam('EXPOSING_RBV')==1) and self.cancel == 0:
                    time.sleep(POLL_TIME)
            #Ideally starting from here code executes right on transition to zero 
            # qi2 exposing just went from 1 to 0 indicating a frame just ended 
            # sigexposureoutput set to "OUTPUT" results in 42.5 - 44.5 milliseconds gap between end and start signals 
            # for 100 millisecond exposures
            #print "sleeping for ", self.liveRecoveryPeriod-offset , " seconds"
            time.sleep(self.liveRecoveryPeriod-offset)
            #request CPI exposure 
            self.write('PHOTOSPOT', 1)
            #wait for start of cpi exposure
            #print "wait for cpi exposeout start"
            while self.getParam('EXPOSE_RBV') == 0 and self.cancel == 0:
                time.sleep(POLL_TIME)
            #print "wait for cpi exposeout end"
            #wait for end of cpi exposure
            while self.getParam('EXPOSE_RBV') == 1 and self.cancel == 0:
                time.sleep(POLL_TIME)
            #print "SINGLE SEQUENCE FINISH"
            #self.write('PHOTOSPOT', 0)
        caput(DET_IOC + 'cam1:Acquire', 0)
        #sequence is over reset flag to 1
        self.firstImageFlag = 1
    
    def liveSync(self):
        oldheartbeattime=0
        self.prepExpose()
        caput(DET_IOC + 'cam1:Acquire', 1)
        while True and self.cancel == 0:
            while self.qi2EndFlag != True:
                time.sleep(POLL_TIME)
            self.qi2EndFlag = False
            heartbeattime= time.clock()
            time.sleep(0.035)
            self.write('PHOTOSPOT', 1)
            print str(datetime.now())[:-3],'PHOTSPOT NOW', heartbeattime - oldheartbeattime
            oldheartbeattime=heartbeattime
        self.write('PHOTOSPOT', 0)
        caput(DET_IOC + 'cam1:Acquire', 0)
        caput(DET_IOC + 'cam1:Acquire', 0)

    def liveSync3(self):
        self.firstImageFlag=1
        #self.prepExpose()
        delayPhotospotToExposeOut=0#.015
        delayQi2ExposeOfftoOn=0#.043
        self.liveRecoveryPeriod=delayQi2ExposeOfftoOn - delayPhotospotToExposeOut
        oldheartbeattime=0
        while self.getParam('HighSpeedCpiTest') == 1 and self.cancel == 0:
            if self.getParam('EXPOSING_RBV') == 0:
                if self.firstImageFlag == 1:
                    offset = 0 
                    caput(DET_IOC + 'cam1:Acquire', 1)
                    ##
                    #wait for one frame to finish, and on transition to 0 continue
                    print 'wait for first exposure to start'
                    while(self.getParam('EXPOSING_RBV')==0) and self.cancel == 0:
                        time.sleep(POLL_TIME)
                    print 'wait for first exposure to finish'
                    while(self.getParam('EXPOSING_RBV')==1) and self.cancel == 0:
                        time.sleep(POLL_TIME)
                    self.firstImageFlag=0 #no longer the first image
                else: #not the first image, but qi2 exposure already over so we need to account for lateness
                    #offset is time since qi2 exposure ended 
                    print 'THREAD LATE: qi2 already finished exposing attempting to compensate'
                    offset = time.clock() - self.qi2ExposeEndTime
            else: # qi2 is exposing and we need to wait for it to transition to 0 before sleeping
                offset = 0 
                print '!!!wait for qi2 to go to zero!!!!'
                while(self.getParam('EXPOSING_RBV')==1) and self.cancel == 0:
                    time.sleep(POLL_TIME)
            #Ideally starting from here code executes right on transition to zero 
            # qi2 exposing just went from 1 to 0 indicating a frame just ended 
            # sigexposureoutput set to "OUTPUT" results in 42.5 - 44.5 milliseconds gap between end and start signals 
            # for 100 millisecond exposures
            #print "sleeping for ", self.liveRecoveryPeriod-offset , " seconds"
            
            time.sleep(self.liveRecoveryPeriod-offset)
            #request CPI exposure 
            heartbeattime= time.clock()
            print str(datetime.now())[:-3],'heartbeat', heartbeattime - oldheartbeattime
            oldheartbeattime=heartbeattime

            #self.write('PHOTOSPOT', 1)
            #wait for start of cpi exposure
            #print "wait for cpi exposeout start"
            #while self.getParam('EXPOSE_RBV') == 0 and self.cancel == 0:
            #    time.sleep(POLL_TIME)
            #print "wait for cpi exposeout end"
            #wait for end of cpi exposure
            #while self.getParam('EXPOSE_RBV') == 1 and self.cancel == 0:
            #    time.sleep(POLL_TIME)
            #print "SINGLE SEQUENCE FINISH"
            #self.write('PHOTOSPOT', 0)
        caput(DET_IOC + 'cam1:Acquire', 0)
        caput(DET_IOC + 'cam1:Acquire', 0)
        #sequence is over reset flag to 1
        self.firstImageFlag = 1

    #Testing how quickly we can toggle Photospot on and off
    # seems to be 5-10millisecond delay from triggering photospot until cpi is exposing
    def highSpeedCpi(self):
        self.prepExpose()
        time.sleep(.01)
        cpiOldEnd = 0
        while self.cancel==0:
            sendPSin = time.clock()
            self.write('PHOTOSPOT', 1)
            #wait for start of cpi exposure
            print 'wait for start of exposure'
            while self.getParam('EXPOSE_RBV')==0:
                time.sleep(POLL_TIME)
                if self.cancel == 1:
                    break
            # cpi exposure started
            cpiStart = time.clock()
            print 'wait for end of exposure'
            # wait for end of cpi exposure
            while self.getParam('EXPOSE_RBV') == 1:
                time.sleep(POLL_TIME)
                if self.cancel == 1:
                    break
            # cpi exposure ended
            cpiEnd = time.clock()
            print '!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! cpi expose on for', cpiEnd - cpiStart
            print '!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! photospot in lag', cpiStart - sendPSin
            print '!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! duty cycle ', cpiEnd - cpiOldEnd
            cpiOldEnd= cpiEnd
            # cpi needs some time to prepare for next shot
            time.sleep(0.01)
            print 'endloop'
        self.exposeEnd()

    """
    Call when user hits abort button (through write function) or hits abort on scan (through callback function)
     -set self.cancel = 1 - this lets any sequences in progress know to cancel 
     -Turn expose off
     -Turn radprep off
     -Reset nikon trigger
     -cancel scan if in progress 
     -Wait for all sequences to finish (self.seqInProgress=0)
    """
    def abort(self):
        self.write('STATUS_RBV', self.currentFunction + ' ABORTING!')
        self.cancel=1 # flag to tell exposures to cancel
        self.write('PHOTOSPOT', 0)
        self.write('EXPOSE', 0)
        self.write('FLUORO',0)
        self.write('RAD_PREP',0)
        self.write('SEND_TRIGGER', 0)
        if caget(SCANPROGRESSIOC +'running') == 1:
            SCAN_CANCEL_IOC.put(1)
        # wait for any sequence in progress to finish
        while self.seqInProgress == 1:
            time.sleep(POLL_TIME)
        self.cancel=0
        self.write('STATUS_RBV', self.currentFunction + ' Abort complete')

    """
    Need to monitor scan abort PV so we can properly disable exposure/RadPrep
    """
    def ScanMonitor(self, **kw):
        print str(datetime.now())[:-3], 'Scan Abort Callback'
        #This if statement is true if the scan was canceled, and Radprep is true
        if self.getParam('RAD_PREP')== 1 and self.cancel == 0 : 
            self.aid = threading.Thread(target = self.abort, args = ())
            self.aid.start() 

    def warningsound(self):
        """
        Play warning sound from Windows
        """
        while(self.PrepSound):
            winsound.Beep(1318,500)
            winsound.Beep(1108,750)
    
    def printNiceTimeDelta(self, seconds):
        """
        Print time delta in easily readable format
        """
        # modified from stackoverflow example
        # for displaying how long it has been since last CPI exposure
        delay = timedelta(seconds=(seconds))
        if (delay.days == 1):
            out = str(delay).replace(" day, ", ":")
        elif (delay.days > 1):
            out = str(delay).replace(" days, ", ":")
        else:
            out = "0:" + str(delay)
        outAr = out.split(':')
        outAr = ["%02d" % (int(float(x))) for x in outAr]
        out   = ":".join(outAr)
        return out

    def On(self):
        """
        Sequence to turn cpi generator on
        """
        self.write('STATUS_RBV', 'Powering Generator On')
        self.setDigiOut(self.powerOnTask, HIGH)
        time.sleep(.6) # simulating push button
        self.setDigiOut(self.powerOnTask, LOW)
        time.sleep(4)
        self.write('STATUS_RBV', 'Idle')
        caput(XRAY_IOC + 'GetKVP.PROC', 1) # try to update values since they will be out of sync
        
    def Off(self):
        """
        Sequence to turn cpi generator off
        """
        self.write('STATUS_RBV', 'Powering Generator Off')
        self.setDigiOut(self.powerOffTask, HIGH)
        time.sleep(.6) #simulating push button
        self.setDigiOut(self.powerOffTask, LOW)
        self.write('STATUS_RBV', 'Idle')

    def RadPrepOn(self):
        time.sleep(.1)
        self.setDigiOut(self.radPrepInTask, HIGH)

    def RadPrepOff(self):
        self.setDigiOut(self.radPrepInTask, LOW)
        time.sleep(.1)
    
    def setDigiOut(self, DAQtaskName, value ):
        #print 'setdigiout', value, str(DAQtaskName)
        #startDO = time.clock()
        if type(value) != numpy.ndarray:
            if value == 0:
                value = LOW
            else:
                value = HIGH
        self.processDAQstatus(DAQmxWriteDigitalLines(DAQtaskName,1,1,10.0,DAQmx_Val_GroupByChannel, value ,self.written, None))
        #print 'write took', time.clock() - startDO

    def processDAQstatus(self, errorcode):
        if errorcode != 0:
            print str(datetime.now())[:-3], "NI-DAQ error! Code:", errorcode

if __name__ == '__main__':
    server = SimpleServer()
    server.createPV(prefix, pvdb)
    driver = myDriver()
    # process CA transactions
    while True:
        try:
            server.process(0.0001)
        except KeyboardInterrupt:
            try:
                sys.exit(0)
            except SystemExit:
                os._exit(0)
