#!/usr/bin/env python
from pcaspy import Driver, SimpleServer, cas
import time, threading, winsound, os, os.path, pickle, socket, platform
from PyDAQmx import *
from epics import *
import numpy as np
from datetime import datetime, timedelta
import psutil

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
auto makes python process high priority when starting
various timing diagnostic changes
code clean up and addition of some comments
Online reference for NI-DAQ
http://zone.ni.com/reference/en-XX/help/370471W-01/


Note "input" and "output" for variables are named from the perspective of the CPI generator, not the ni-daq. (Unless otherwise specified)
This was so the terminology in the documentation for the cpi matches with the names in the code.

"""

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

"""
make this python process a high priority in Windows
"""
try:
    p = psutil.Process(os.getpid())
    p.nice(psutil.HIGH_PRIORITY_CLASS)
    #p.nice(psuitl.REALTIME_PRIORITY_CLASS) currently doesn't work 
except:
    print 'failed to make process high priority'

"""
pv database for pcaspy
"""
prefix = 'cpiSync:'
pvdb = {
    'On'            : { 'asyn' : True },
    'Off'           : { 'asyn' : True },
    'NikonTrigger'  : { 'asyn' : True },
    'RadPrep'       : { 'asyn' : True },
    'Expose'        : { 'asyn' : True },
    'FluoroIn'      : { 'asyn' : True },
    'PhotospotIn'   : { 'asyn' : True },
    'RadPrepOut'    : { 'asyn' : True },
    'RadReadyOut'   : { 'asyn' : True },
    'ExposeOut'     : { 'asyn' : True },
    'FluoroOut'     : { 'asyn' : True },
    'SyncCam'       : { 'asyn' : True},
    'Abort'         : { 'asyn' : True },
    'NikonExpose'   : { 'asyn' : True },
    'NikonSingleExposeSeq'
                    : { 'asyn' : True },
    'NikonScanExposeSeq'
                    : { 'asyn' : True },
    'GenWarmUpSeq'
                    : { 'asyn' : True },
    'GenWarmUpSeqFull'
                    : { 'asyn' : True },
    'CpiErrorString': {  'type': 'string'},
    'Status'        : {  'type': 'char',
                        'count': 300,
                        'value': 'Initialization'},
    'TimeSinceLastExposure'
                    : {   'type': 'string',
                          'scan': 1},
    'Qi2TriggerReady'
                    : { },
    'Qi2Exposing'   : { },
    'Document'      : { },
    'GenDelay'      : {  'value': 0.02,#20 milliseconds
                          'prec': 3} ,
    'SeqStarttoQi2'      : { },
    'TimeBetweenReq'     : { },
    'Qi2ReqToStart'      : { },
    'CpiReqToStart'      : { },
    'CpiStarttoQi2Start' : { },
    'Qi2Duration'        : { },
    'CpiDuration'        : {},#'count': 1000 
    'DutyCycle'          : { }, 
    'trigger'             : {},      
    'ShotDuration'         : {},                    
    'HOSTNAME'      : {'type': 'string'},
    'ENGINEER'      : {'type': 'string'},
    'LOCATION'      : {'type': 'string'},
    'RECORD_CNT'    : {'type': 'int'},
    'EPICS_VERS'    : {'type': 'string'},
    'KERNEL_VERS'   : {'type': 'string'},
}

#Implementation Specific Definitions
#Currently using 2 DAQ's due to the CPI outputs requiring special digital input type
#cpiSyncDAQ only handles input from CPI (CPI output)
CPI_INPUT_DAQ   = 'cpiSyncDAQ'
#FcpiSyncDAQ controls CPI via eric's SSR module as well as Qi2 inputs
NI_DAQ_OUTPUTS  = 'FcpiSyncDAQ'

#These are the output lines from DAQ to CPI  (input to CPI etc.) 
CPI_FLUORO_IN       = NI_DAQ_OUTPUTS + '/port0/line0' #setting high enables fluoro exposure 
CPI_PHOTOSPOT_IN    = NI_DAQ_OUTPUTS + '/port0/line1' #setting high enables photospot (rad enable)
CPI_EXPOSE_IN       = NI_DAQ_OUTPUTS + '/port0/line2' #setting high allows exposure
CPI_POWER_OFF_LINE  = NI_DAQ_OUTPUTS + '/port0/line4' #pulse turns CPI off
CPI_POWER_ON_LINE   = NI_DAQ_OUTPUTS + '/port0/line5' #pulse turns CPI on
CPI_RAD_PREP_IN     = NI_DAQ_OUTPUTS + '/port0/line6' #setting high makes enables rad prep

################QI2 Defines###################
#QI2_EXPOSE when set to high requests exposure
QI2_EXPOSE       = NI_DAQ_OUTPUTS + '/port2/line0' #Output from DAQ to Qi2 requests exposure
QI2_TRIGGERREADY = NI_DAQ_OUTPUTS + '/port2/line1' #High when Qi2 ready to expose
QI2_EXPOSEOUT    = NI_DAQ_OUTPUTS + '/port2/line2' #QI2_EXPOSEOUT is high when it is exposing
#qi2 lines to poll:
QI2_INPUTS       = QI2_TRIGGERREADY + "," + QI2_EXPOSEOUT #QI2 lines to poll

#These are the DAQ inputs (output from CPI etc.)
CPI_RAD_PREP_OUT    = CPI_INPUT_DAQ  + '/port1/line0'
CPI_RAD_READY_OUT   = CPI_INPUT_DAQ  + '/port1/line1'
CPI_EXPOSE_OUT      = CPI_INPUT_DAQ  + '/port1/line2'
CPI_FLUORO_OUT      = CPI_INPUT_DAQ  + '/port1/line3'
#CPI lines to poll:
NI_DAQ_CHANGEDETECT = CPI_RAD_PREP_OUT + "," + CPI_RAD_READY_OUT + "," + CPI_EXPOSE_OUT  + "," + CPI_FLUORO_OUT

#EPICS scan variables to keep track of scan so we know when to start/stop rad prep 
#very important to use scanProgress record instead of scan, so we can keep track of multi-dimensional scans
SCANIOC1       ='IOC:scan1'
SCANPROGRESSIOC='IOC:scanProgress:' 
SCANPROGRESSIOC_PV=PV('IOC:scanProgress')

#So we know to immediately shut off ranode if scan is canceled, and initiate abort() function
SCAN_CANCEL_IOC = PV('IOC:AbortScans.PROC', callback = True) 

#Functions to allow changes of CPI settings for automatic warm-up procedures 
CPI_KVP_PV      = PV('cpi:SetKVP')
CPI_MA_PV       = PV('cpi:SetMA')
CPI_MS_PV       = PV('cpi:SetMS')
CPI_SETFOCUS_PV = PV('cpi:SetFocus')

#What PV's to grab the filepath and filename from for text doc writing
FILEPATH_PV=PV('Qi2:TIFF1:FilePath')
FILENAME_PV=PV('Qi2:TIFF1:FileName', callback = True) #reset Filenum to zero when called

#Variable to prevent CPI polling during exposure sequence
CPI_GENERATOR_SCAN='cpi:GeneratorStatus.SCAN'

#Keep track of scan status 
NFINISHED_PV=PV(SCANPROGRESSIOC+'Nfinished')
NTOTAL_PV=PV(SCANPROGRESSIOC+'Ntotal')
CPT_PV=PV(SCANIOC1 + '.CPT')
NPTS_PV=PV(SCANIOC1 + '.NPTS')

#Constant numpy arrays for setting digital outputs high or low on NI-DAQs
LOW  = numpy.zeros((1,), dtype=numpy.uint8)
HIGH = numpy.ones((1,), dtype=numpy.uint8)

#How often to pause in "while" loops
POLL_TIME = 0.0001

#PV_LIST for .txt documentation files to be generated
PV_LIST = [
           'cpi:GetKVP', 'cpi:GetMA', 'cpi:GetMS', 'cpi:GetFocus',\
           'Qi2:cam1:Gain_RBV', 'Qi2:cam1:AcquireTime_RBV', \
           'IOC:scan1.P1PV', 'IOC:scan1.P1SI', 'IOC:scan1.P1DV', \
           'KOZ:m1.RBV', 'KOZ:m2.RBV', 'KOZ:m3.RBV', 'KOZ:m4.RBV',\
           'KOZ:m5.RBV', 'KOZ:m6.RBV', 'KOZ:m7.RBV', 'KOZ:m8.RBV',\
           'KOZ:m9.RBV', 'KOZ:m10.RBV', 'KOZ:m11.RBV', 'KOZ:m12.RBV',\
           'NEW:m18.RBV',\
           'KOZ:m13.RBV', 'KOZ:m14.RBV', 'KOZ:m15.RBV', 'KOZ:m16.RBV',\
           'KOZ:m17.RBV', 'KOZ:m18.RBV', 'KOZ:m19.RBV',\
           'NEW:m1.RBV', 'NEW:m2.RBV', 'NEW:m3.RBV', 'NEW:m6.RBV',\
           'NEW:m7.RBV', 'NEW:m8.RBV', 'NEW:m9.RBV',  'NEW:m10.RBV',\
           'NEW:m11.RBV', 'NEW:m12.RBV', 'NEW:m13.RBV', 'NEW:m14.RBV', \
           'NEW:m15.RBV', 'NEW:m17.RBV',\
           ]

#Dictionary of error codes from CPI generator, obtained from SDK doc file PS-VJW2931.doc
#will be deleted soon, this is handled by cpp driver 
cpiErrorDictionary =\
    {
    '0':'No Error',
    '1':' Generator CPU EPROM checksum error ',
    '2':'Generator CPU EEPROM data checksum error ',
    '3':'Generator CPU NVRAM error',
    '4':'Generator CPU Real Time Clock error',
    '5':'Main Contactor error',
    '6':'Rotor Fault',
    '7':'Filament Fault',
    '8':'kV / mA Fault (previously known as Beam Fault)',
    '9':'Power Supply Not Ready',
    '10':'No KV during exposure',
    '11':'mA during exposure too high',
    '12':'mA during exposure too low',
    '13':'Manually Terminated Exposure',
    '14':'AEC Back-up Timer Exposure Terminated',
    '15':'AEC MAS Exceeded Exposure Terminated',
    '16':'Tomo Back-up Timer Exposure Terminated',
    '17':'Uncalibrated Exposure Parameter',
    '18':'Preparation Time-out Error',
    '19':'Anode Heat Limit ',
    '20':'Thermal Switch Interlock #1 Error',
    '21':'Thermal Switch Interlock #2 Error',
    '22':'Door Interlock Error',
    '23':'Collimator Interlock Error',
    '24':'Cassette Interlock Error',
    '25':'II Safety Interlock Error',
    '26':'Spare Input Interlock Error',
    '27':'Receptor Time-out Error Receptor did not respond within time-out',
    '28':'Prep Input active during Initialization Phase ',
    '29':'X-ray Input active during Initialization Phase ',
    '30':'Fluoro Input active during Initialization Phase ',
    '31':'Communication Error Remote Fluoro',
    '32':'Communication Error Console',
    '33':'Lithium Battery Low Voltage Error',
    '34':'+12VDC Error',
    '35':'-12VDC Error',
    '36':'+15VDC Error',
    '37':'-15VDC Error',
    '38':'Calibration Data Corrupt Error1',
    '39':'AEC Data Corrupt Error1',
    '40':'Fluoro Data Corrupt Error1',
    '41':'Receptor Data Corrupt Error1',
    '42':'Tube Data Corrupt Error1',
    '43':'High Voltage Error KV detected in non x-ray state',
    '44':'Invalid Communication Message',
    '45':'Communication Message Not Supported',
    '46':'Communication Message Not Allowed',
    '47':'Fluoro Timer Limit Error',
    '48':'Focus Mismatch Error ',
    '49':'Not Enabled Error',
    '50':'Generator Limit Data Corrupt Error1',
    '51':'AEC Feedback Error (No Feedback Signal Detected)',
    '52':'High Small Focus Filament Current Error in Standby',
    '53':'High Large Focus Filament Current Error in Standby',
    '54':'AEC Reference out of range',
    '55':'No Fields Selected in AEC mode',
    '56':'No Tube Programmed',
    '57':'AEC Stop signal in wrong state',
    '58':'Console Back-Up Timer',
    '59':'Housing Heat Limit Exceeded2',
    '60':'High KV Error',
    '61':'Low KV Error',
    '62':'EXP_SW signal active in standby state',
    '63':'Factory Defaults Enabled',
    '64':'No Exposure Release2 ',
    '65':'Tomo Device Error2',
    '66':'No Sync Pulse Input',
    '67':'Power Supply Duty Cycle Limit',
    '70':'Software Key Error',
    '71':'DAP Dose Overflow',
    '72':'DAP Device Error',
    '73':'DAP Data Error',
    '74':'Table Communication Error',
    '75':'Table Emergency Stop',
    '76':'High Micro Focus Filament Current',
    '77':'ADR Cable INterlock',
    '78':'ADR Error',
    '79':'Mask Abort Error',
    '80':'Field Orientation Error',
    '81':'No Cine Tube Selected',
    '82':'Cine Data Error',
    '83':'Air Kerma Dose Overflow',
    '84':'Air Kerma Data Error',
    '85':'Table Error',
    '86':'Digtial System Comms Error',
    '100':'Calibration Error Maximum mA Exceeded',
    '101':'Calibration Error Calibration Data Table Exceeded',
    '102':'Calibration Error Maximum Filament Current Exceeded',
    '103':'Calibration Error Manually Terminated',
    '104':'Calibration Error No mA',
    '105':'Calibration Error Minimum mA not calibrated',
    '200':'Anode Warning Level Exceeded',
    '201':'Fluoro Timer Warning Level Exceeded',
    '202':'Generator KW Limit',
    '203':'Generator KV Limit',
    '204':'Generator MA Limit',
    '205':'Generator MS Limit',
    '206':'Generator MAS Limit',
    '207':'Tube KW Limit',
    '208':'Tube KV Limit',
    '209':'Tube MA Limit',
    '210':'Tube MAS Limit',
    '211':'Calibration Limit, Selected Parameter not Calibrated',
    '212':'Generator AEC Density Limit',
    '213':'Invalid Communication Parameter',
    '214':'Housing Heat Warning2',
    '215':'CT Termination Input Wrong State2',
    '216':'Deselect Tomo Table2',
    '217':'Select Tomo Angle2',
    '218':'Invalid Tomo Angle2',
    '219':'Generator PPS Limit',
    '220':'Generator Power Supply Duty Cycle Warning',
    '221':'DAP Device Not Ready',
    '222':'DAP Rate Warning Level Exceeded',
    '223':'DAP Accumulated Warning Level Exceeded',
    '224':'Parameter Limit',
    '225':'Fluoro Focus Auto Changeover (default focus damaged)',
    '226':'Air Kerma Rate Warning',
    '227':'Air Kerma Accumulated Warning'
}

class myDriver(Driver):
    def  __init__(self):
        super(myDriver, self).__init__()
        SCAN_CANCEL_IOC.add_callback( callback=self.ScanMonitor)
        self.setParam('HOSTNAME', str(socket.gethostname()))
        self.setParam('ENGINEER', 'Andrew Gomella')
        self.setParam('LOCATION', 'B1D521D Windows Server')
        self.setParam('RECORD_CNT', len(pvdb.keys()))
        self.setParam('EPICS_VERS', str(cas.EPICS_VERSION_STRING))
        self.setParam('KERNEL_VERS', str(platform.system() + " " + platform.release()))

        self.lastCpiExposure = lastCpiExposure #keep track of most recent exposure
        self.cancel = 0 #cancel flag for abort procedure 
        self.currentFunction = '' #initialize current function string used in status messages
        self.seqInProgress=0 #flag to determine if we are inside a "sequence" or not

        #the following variable is needed for pydaqmx
        self.written = int32()
        
        #set up the warning sounds which will play out of computer speakers (needs to be unmuted)
        self.PrepSound=False
        sid=threading.Thread(target=self.warningsound)
        sid.start()

        #Initialize DAQ- don't reset with custom DO-SSR module because it will cause the SSR's to toggle on startup
        #DAQmxResetDevice(NI_DAQ_OUTPUTS)
        #DAQmxResetDevice(NI_DAQ_INPUTS) CPI_PHOTOSPOT_IN  
        
        #Set up DO task for Photospot
        self.photospotInTask = TaskHandle()
        DAQmxCreateTask("",byref(self.photospotInTask))
        DAQmxCreateDOChan(self.photospotInTask, CPI_PHOTOSPOT_IN, "", DAQmx_Val_ChanForAllLines)
        DAQmxSetDOOutputDriveType(self.photospotInTask, CPI_PHOTOSPOT_IN , DAQmx_Val_ActiveDrive)
        DAQmxStartTask(self.photospotInTask)   
        self.setDigiOut(self.photospotInTask, LOW)

        #Set up DO task to trigger Qi2 expose
        self.Qi2TriggerTask = TaskHandle()
        DAQmxCreateTask("",byref(self.Qi2TriggerTask))
        DAQmxCreateDOChan(self.Qi2TriggerTask, QI2_EXPOSE, "", DAQmx_Val_ChanForAllLines)
        DAQmxSetDOOutputDriveType(self.Qi2TriggerTask, QI2_EXPOSE, DAQmx_Val_ActiveDrive)
        DAQmxStartTask(self.Qi2TriggerTask)      

        #Set up the DO task to turn off the CPI generator
        self.powerOffTask = TaskHandle()
        DAQmxCreateTask("",byref(self.powerOffTask))
        DAQmxCreateDOChan(self.powerOffTask, CPI_POWER_OFF_LINE, "", DAQmx_Val_ChanForAllLines)
        DAQmxSetDOOutputDriveType(self.powerOffTask, CPI_POWER_OFF_LINE, DAQmx_Val_ActiveDrive)      
        DAQmxStartTask(self.powerOffTask)
        self.setDigiOut(self.powerOffTask, LOW)

        #Set up the DO task to turn on the CPI generator
        self.powerOnTask = TaskHandle()
        DAQmxCreateTask("",byref(self.powerOnTask))
        DAQmxCreateDOChan(self.powerOnTask, CPI_POWER_ON_LINE, "", DAQmx_Val_ChanForAllLines)
        DAQmxSetDOOutputDriveType(self.powerOnTask, CPI_POWER_ON_LINE, DAQmx_Val_ActiveDrive)      
        DAQmxStartTask(self.powerOnTask)
        self.setDigiOut(self.powerOnTask, LOW)

        #Set up the DO task to send the Rad Prep signal to the generator.
        self.radPrepInTask = TaskHandle()
        DAQmxCreateTask("",byref(self.radPrepInTask))
        DAQmxCreateDOChan(self.radPrepInTask, CPI_RAD_PREP_IN, "", DAQmx_Val_ChanForAllLines)
        DAQmxSetDOOutputDriveType(self.radPrepInTask, CPI_RAD_PREP_IN, DAQmx_Val_ActiveDrive)      
        DAQmxStartTask(self.radPrepInTask)
        self.RadPrepOff()  

        #Set up the DO task to send the Expose signal to the generator. 
        self.exposeInTask = TaskHandle()
        DAQmxCreateTask("",byref(self.exposeInTask))
        DAQmxCreateDOChan(self.exposeInTask, CPI_EXPOSE_IN, "", DAQmx_Val_ChanForAllLines)
        DAQmxSetDOOutputDriveType(self.exposeInTask, CPI_EXPOSE_IN, DAQmx_Val_ActiveDrive)      
        DAQmxStartTask(self.exposeInTask)
        self.setDigiOut(self.exposeInTask, LOW)

        #Set up the DO task to send the Fluoro signal to the generator. 
        self.fluoroInTask = TaskHandle()
        DAQmxCreateTask("",byref(self.fluoroInTask))
        DAQmxCreateDOChan(self.fluoroInTask, CPI_FLUORO_IN , "", DAQmx_Val_ChanForAllLines)
        DAQmxSetDOOutputDriveType(self.fluoroInTask, CPI_FLUORO_IN, DAQmx_Val_ActiveDrive)      
        DAQmxStartTask(self.fluoroInTask)
        self.setDigiOut(self.fluoroInTask, LOW)

        #task to read DI lines combined into one (significantly improves performance to do one read instead of 4)
        self.combinedTask = TaskHandle()
        DAQmxCreateTask("",byref(self.combinedTask))
        DAQmxCreateDIChan(self.combinedTask, NI_DAQ_CHANGEDETECT, "", DAQmx_Val_ChanForAllLines)
        DAQmxStartTask(self.combinedTask)      

        #Declare variable for change detection callback comparison
        self.multipleOld=numpy.array([0,0,0,0], dtype=numpy.uint8)

        #Set up polling for the 2 Qi2 Inputs
        self.qi2PollTask = TaskHandle()
        DAQmxCreateTask("",byref(self.qi2PollTask))
        DAQmxCreateDIChan(self.qi2PollTask, QI2_INPUTS , "", DAQmx_Val_ChanForAllLines)
        DAQmxStartTask(self.qi2PollTask)
        
        #Start the Qi2 polling thread
        self.xid = threading.Thread(target=self.pollQi2,args=())
        self.xid.start()

        ##start the CPI polling thread
        self.cid = threading.Thread(target=self.pollCPI,args=())
        self.cid.start()

        #Now ready to do exposures, change from Initilization to Idle 
        self.write('Status', 'Idle')
        print str(datetime.now())[:-3], 'cpiSync succesfully initialized'

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

    #since the ni daq 6501 doesnt support change detection, we have to poll and do change detection ourselves
    def pollQi2(self):
        oldval = np.array([0,0], dtype=np.uint8)
        DAQmxReadDigitalLines(self.qi2PollTask, 1, 1, 0,  oldval, 2, None, None, None)
        while True:
            #startqi2Polltime = time.clock()
            newval = np.array([0,0], dtype=np.uint8)
            DAQmxReadDigitalLines(self.qi2PollTask, 1, 1, 0,  newval, 2, None, None, None) 
            self.setParam("Qi2TriggerReady", newval[0])
            self.setParam("Qi2Exposing", newval[1])
            if newval[0] != oldval[0]:
                if newval[0] == 1:
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
                    self.setParam('SeqStarttoQi2', self.qi2ExposeReqeustTime-self.scanExposeRequestTime)
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
                    if self.prepTime == 0:
                        self.setParam('ShotDuration', self.scanExpEndTime -self.scanExposeRequestTime)
                    else:
                        self.setParam('ShotDuration', (self.scanExpEndTime-self.scanExposeRequestTime)-self.prepTime) 
                    self.lastqi2ExposeEndTime=self.qi2ExposeEndTime
                    self.prepTime=0
            if newval[1] != oldval[1]:
                if newval[1] == 1:
                    print str(datetime.now())[:-3], 'Qi2 START EXPOSURE', '%.4f'%(time.clock()- self.scanExposeRequestTime)
                    self.qi2ExposeStartTime=time.clock()
                else:
                    self.write('NikonTrigger',0)
            oldval = newval
            self.updatePVs()
            #endqi2PollT= time.clock()
            #if endqi2PollT - startqi2Polltime > 0.002:
            #    print endqi2PollT - startqi2Polltime, 'time to poll qi2'

    #since the ni daq 6501 doesnt support change detection, we have to poll and do change detection ourselves
    def pollCPI(self):
        oldval = np.array([0,0,0,0], dtype=np.uint8)
        DAQmxReadDigitalLines(self.combinedTask, 1, 1, 0,  oldval, 4, None, None, None) 
        while True:
            #startcpipollT= time.clock()
            newval = np.array([0,0,0,0], dtype=np.uint8)
            DAQmxReadDigitalLines(self.combinedTask, 1, 1, 0,  newval, 4, None, None, None) 
            self.setParam("RadPrepOut", newval[0])
            self.setParam("RadReadyOut", newval[1])
            self.setParam("ExposeOut", newval[2])
            self.setParam("FluoroOut", newval[3])
            if newval[2] != oldval[2]:
                if newval[2] == 1:
                    print str(datetime.now())[:-3], 'GENERATOR RAD ENABLE', '%.4f'%(time.clock()- self.scanExposeRequestTime)
                    self.cpiExposeStartTime=time.clock()
                else:
                    if self.getParam("PhotospotIn")==1:
                        self.write("PhotospotIn", 0)
                    print str(datetime.now())[:-3], 'GENERATOR RAD END', '%.4f'%(time.clock()- self.scanExposeRequestTime)
                    self.cpiExposeEndTime=time.clock()
                    #only update lastcpiexposure if it was a real exposure (not toggle during cpi boot)
                    #if RadReadyOut (newval[1]) is 1, then it probably was a real exposure
                    if newval[1] == 1:
                        self.lastCpiExposure = time.time()
                oldval[2] = newval[2]
            #if np.not_equal(newval, oldval).any:
            #    oldval = newval
            #oldval = newval
            self.updatePVs()
            #endcpiPollT= time.clock()
            #if endcpiPollT - startcpipollT > 0.002:
            #    print endcpiPollT - startcpipollT, 'time to poll cpi'

    """
    Override read function from pcaspy
    """
    def read(self, reason):
        if reason == 'TimeSinceLastExposure' :
            secsSinceLastExposure = time.time() - self.lastCpiExposure
            value = self.printNiceTimeDelta(secsSinceLastExposure)
            if secsSinceLastExposure > 28800:
                self.setParam('Status', 'More than 8 hours since last exposure- run warm-up')
                self.updatePVs()
        else:
            value = self.getParam(reason)
        self.updatePVs()
        return value

    """
    Override write function from pcaspy
    """
    def write(self, reason, value):
        self.setParam(reason, value)
        self.updatePVs()
        if  reason == 'RadPrep':
            if value == 1:
                self.did = threading.Thread(target = self.RadPrepOn, args = ())
                self.did.start()
            else:
                self.did = threading.Thread(target = self.RadPrepOff, args = ())
                self.did.start()
            if self.seqInProgress == 0 and self.cancel == 0:
                self.currentFunction = '(Manual)'    
        elif reason == 'Expose':
            if (self.getParam("RadPrepOut") == 1 and value == 1 ) or value == 0:
                self.did = threading.Thread(target = self.setDigiOut, args = (self.exposeInTask, value))
                self.did.start()
                if value == 0:
                    with open('objs.pickle', 'w') as f:
                        pickle.dump(self.lastCpiExposure, f)
                if self.seqInProgress == 0 and self.cancel == 0:
                    self.currentFunction = '(Manual)'
        elif reason == 'FluoroIn':
            self.zid = threading.Thread(target = self.setDigiOut, args = (self.fluoroInTask, value))
            self.zid.start()
            if self.seqInProgress == 0 and self.cancel == 0:
                self.currentFunction = '(Manual)'
        elif reason == 'PhotospotIn':
            self.mid = threading.Thread(target = self.setDigiOut, args = (self.photospotInTask, value))
            self.mid.start()
        elif reason == 'NikonTrigger':
            self.niTriggerTime=time.clock()
            self.fid = threading.Thread(target = self.setDigiOut, args = (self.Qi2TriggerTask, value))
            self.fid.start()
        elif reason == 'On' and value == 1:
            self.eid = threading.Thread(target = self.On, args = ())
            self.eid.start()
        elif reason == 'Off' and value == 1:
            self.fid = threading.Thread(target = self.Off, args = ())
            self.fid.start()
        elif reason == 'NikonSingleExposeSeq' and value == 1:
            print str(datetime.now())[:-3],'SingleExpose requested 0.0000'
            self.scanExposeRequestTime=time.clock()
            #if Qi2 trigger isn't ready the user likely forgot to enable collection on Qi2
            # otherwise may be a problem with the Qi2 
            if self.getParam('Qi2TriggerReady')==1:
                self.NikonSingleExposeSeq()
                #self.fid = threading.Thread(target = self.NikonSingleExposeSeq, args = ())
                #self.fid.start()
            else:               
                self.write('Status',  'Error, Qi2 not ready to expose. ')
        elif reason == 'NikonScanExposeSeq' and value == 1:
            print str(datetime.now())[:-3],'ScanExpose requested 0.0000'
            self.scanExposeRequestTime=time.clock()
            self.NikonScanExposeSeq()
            #self.fid = threading.Thread(target = self.NikonScanExposeSeq, args = ())
            #self.fid.start()
            #if self.getParam('Qi2TriggerReady')!=1:
            #    self.write('Status',  'Error, Qi2 not ready to expose. ')
        elif reason == 'GenWarmUpSeq' and value == 1:
            self.fid = threading.Thread(target = self.GenWarmUpSeq, args = ())
            self.fid.start()
        elif reason == 'GenWarmUpSeqFull' and value == 1:
            self.fid = threading.Thread(target = self.GenWarmUpSeqFull, args = ())
            self.fid.start()
        elif reason == 'Abort' and value == 1:
            self.fid = threading.Thread(target = self.abort, args = ())
            self.fid.start()
        elif reason == 'Status':
            print str(datetime.now())[:-3], value
        self.updatePVs()

    def document(self):
        print str(datetime.now())[:-3],'Generate Doc String Start'
        pathname = self.filepath + '\\' + self.filename + '_' + str(self.filenum).zfill(3)
        try:
            f = open(pathname + '.txt', 'w')
            documentString = ''
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
            self.write('Status', self.currentFunction + 'Complete')

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
            self.write('Status', self.currentFunction + 'Complete')

    def GenExposeOnly(self, kvp, current, millisec, numberOfExposures, dutyCycle):
        if self.cancel == 0:
            self.write('Status', self.currentFunction + ' Changing Settings')
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
                #send trigger signal to CPI expose
                self.write('Status', self.currentFunction + ' EXPOSING!')
                self.write('PhotospotIn', 1)
                #wait for exposure to finish
                while self.getParam('PhotospotIn') == 1 and self.cancel == 0:
                    time.sleep(POLL_TIME)
                #self.write('PhotospotIn', 0)
                self.write('Status', self.currentFunction + ' wait duty cycle')
                if self.cancel == 1:
                    break
                time.sleep(dutyCycle)
            if self.cancel == 0:
                self.write('Expose',0)
                self.write('RadPrep',0)
                #wait for rad prep to turn off
                while self.getParam('RadReadyOut') == 1:
                    time.sleep(POLL_TIME)

    #Take single isolated Nikon-CPI sync shot
    def NikonSingleExposeSeq(self):
        self.currentFunction = '(Nikon Single Shot)'
        print str(datetime.now())[:-3], '!!!!! Nikon - CPI Single Shot Sequence Start !!!!!'
        self.seqInProgress=1 #let abort sequence know this function is in progress
        self.prepExpose()         #do chores and prepare gen (during this time camera has more time to prepare)
        self.checkIfCameraReady() #see if camera is actually ready to acquire
        self.exposeNow()          #send trigger signals, returns when exposure is over
        self.exposeEnd()          #resets nikon trigger, turns off cpi expose signal
        if self.cancel == 0 :   #if canceled rad prep is already off and we don't want status to be idle
            self.write("RadPrep",0)   #single expose, so turn RadPrep off
            self.write("Expose",0)
            self.write('Status', 'Idle')
        if caget("IOC:scan1.P1PV") == "cpi:SetKVP":
            caput(SCANIOC1 + '.WAIT', 0) #tell scan we're finished acquiring the image and it can progress
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
        else:
            self.scanExpSeqList.append(time.clock()-self.scanExposeRequestTime-self.prepTime)
        if self.cancel == 0:
            #check if this was the last shot of the scan
            #if caget(SCANPROGRESSIOC+'Nfinished') + 1 == caget(SCANPROGRESSIOC +'Ntotal') or caget(SCANIOC1 + '.CPT') + 1 == caget(SCANIOC1 + '.NPTS'): #just took last image, so turn off radprep
            if NFINISHED_PV.get() + 1 == NTOTAL_PV.get() or CPT_PV.get() + 1 == NPTS_PV.get():             
                print str(datetime.now())[:-3], "RadPrep turning off!"
                self.write("RadPrep",0)
                self.write("Expose",0)
                self.write('Status', 'Idle')
                #print post scan timing report
                reportfilename= self.filepath + '\\' + self.filename + '_Timing_'+time.strftime("%Y%m%d-%H%M%S")+ '.txt'
                f1=open(reportfilename, 'w+')
                #print 'qi2 duration num mean stddev min max',len(self.qi2DurationList), np.mean(self.qi2DurationList), np.nanstd(self.qi2DurationList), np.amin(self.qi2DurationList), np.amax(self.qi2DurationList)
                #print 'cpi duration num mean stddev min max',len(self.cpiDurationList), np.mean(self.cpiDurationList), np.nanstd(self.cpiDurationList), np.amin(self.cpiDurationList), np.amax(self.cpiDurationList)
                #print 'dutycycle num mean stddev min max',len(self.dutyCycleList), np.mean(self.dutyCycleList), np.nanstd(self.dutyCycleList), np.amin(self.dutyCycleList), np.amax(self.dutyCycleList)
                f1.write('CPI-Qi2 Scan Time Report\n')
                f1.write('Scan duration ' + str((time.clock()-self.scanStartTime)) + '\n')
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
                self.write('Status', self.currentFunction + ' Wait sscan ')
        if self.cancel == 0:
            caput(SCANIOC1 + '.WAIT', 0) #tell scan we're finished acquiring the image and it can progress
        self.seqInProgress=0 #let abort sequence know this function is over

        print str(datetime.now())[:-3], '!!!!! Nikon - CPI Single Scan Shot Sequence Over !!!!!'

    #Checks if gen is ready to expose, if not prepares generator for an exposure, otherwise return
    #   function should only ever run at the beginning of a single shot OR a scan, never in the middle
    #   so we can do house keeping tasks here as well while waiting for rad gen ready
    #Returns when generator is ready to expose
    def prepExpose(self):
        if self.getParam('RadReadyOut') == 0 and self.cancel == 0:
            self.lastqi2ExposeEndTime=0
            self.scanStartTime=time.clock()
            #ENABLE RAD PREP          
            self.write("RadPrep", 1) #sets rad prep in to CPI to 1
            self.write('Status', self.currentFunction + ' Wait RadReady')
            #Signal is sent to radprep, will take a few seconds, so do housekeeping-
            #   -grab info for doc string, filenum will change during scan so we will increment it later
            self.filepath=FILEPATH_PV.char_value
            self.filename=FILENAME_PV.char_value
            self.filenum=caget('Qi2:TIFF1:FileNumber')
            self.docmode=self.getParam('Document')
            while self.getParam('RadReadyOut') == 0 and self.cancel == 0:
                time.sleep(POLL_TIME)
            #ENABLE PHOTOSPOT 
            self.write("Expose", 1) #request for photospot
            print str(datetime.now())[:-3], 'Wait for generator serial confirmation of Exposure Status'
            while caget("cpi:GeneratorStatus") != 4:
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
            if self.getParam('Qi2TriggerReady') != 1:
                print str(datetime.now())[:-3], 'Wait Qi2 TriggerReady', '%.4f'%(time.clock()- self.scanExposeRequestTime)
                self.write('Status', self.currentFunction + ' Wait Qi2 Trigger Ready')
                while self.getParam('Qi2TriggerReady') != 1:
                    time.sleep(POLL_TIME)

    #Call when immediately ready to send sync signals camera + x-ray
    #   -Sends trigger signal to  Qi2 and waits a specified GenDelay then sends CPI trigger
    #   -indifferent to scan status
    def exposeNow(self):
        if self.cancel == 0:
            self.write('Status', self.currentFunction + ' EXPOSING!')
            self.write('NikonTrigger',1) # send trigger release signal to nikon
            self.qi2ExposeReqeustTime=time.clock()
            print str(datetime.now())[:-3], 'Qi2 Expose request sent', '%.4f'%(time.clock()- self.scanExposeRequestTime)
            #while True:
            #    if self.getParam("Qi2Exposing") == 1:
            #        break
            time.sleep(self.getParam('GenDelay'))
            print str(datetime.now())[:-3], 'Generator Expose request sent', '%.4f'%(time.clock()- self.scanExposeRequestTime)
            self.write('PhotospotIn', 1)
            self.cpiExposeRequestTime=time.clock()
            #Wait for CPI to be exposing
            while self.getParam("ExposeOut")==0:
                time.sleep(POLL_TIME)
                if self.cancel == 1:
                    break
            #if self.docmode == 1: #to save time generate doc string during exposure, usually takes less than 100ms
            #    self.document()
            #    self.filenum=self.filenum+1 #increment local count of filenum, might be overwritten later
            #Wait for end of exposure AND Qi2 exposure, unless something sets cancel flag to 1, then return
            #print 'wait for expose to stop or qi2expose to stop'
            # while self.getParam("ExposeOut") == 1 or self.getParam("Qi2Exposing") == 1:
            #     time.sleep(POLL_TIME)
            #     if self.cancel == 1:
            #         break
            #Wait for CPI to finish exposing 
            while self.getParam("ExposeOut") == 1:
                time.sleep(POLL_TIME)
                if self.cancel == 1:
                    break
            self.write('PhotospotIn',0)
    
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

    #Call when user hits abort button (through write function) or hits abort on scan (through callback function)
    # -set self.cancel = 1 - this lets any sequences in progress know to cancel 
    # -Turn expose off
    # -Turn radprep off
    # -Reset nikon trigger
    # -cancel scan if in progress 
    # -Wait for all sequences to finish (self.seqInProgress=0)
    def abort(self):
        self.write('Status', self.currentFunction + ' ABORTING!')
        self.cancel=1 # flag to tell exposures to cancel
        self.write('Expose', 0)
        self.write('FluoroIn',0)
        self.write('RadPrep',0)
        self.write('NikonTrigger', 0)
        if caget(SCANPROGRESSIOC +'running') == 1:
            caput('IOC:AbortScans.PROC', 1) #cancel scan if in progress
        # wait for any sequence in progress to finish
        while self.seqInProgress == 1:
            time.sleep(POLL_TIME)
        self.cancel=0
        self.write('Status', self.currentFunction + ' Abort complete')

    def ScanMonitor(self, **kw):
        print str(datetime.now())[:-3], 'Scan Abort Callback'
        #This if statement is true if the scan was canceled, and Radprep is true
        if self.getParam('RadPrep')== 1 and self.cancel == 0 : 
            self.aid = threading.Thread(target = self.abort, args = ())
            self.aid.start() 

    """
    play warning sound from Windows
    """
    def warningsound(self):
        while(self.PrepSound):
            winsound.Beep(1318,500)
            winsound.Beep(1108,750)
    
    """
    Print time delta in easily readable format
    """
    def printNiceTimeDelta(self, seconds):
        #modified from stackoverflow example
        #for displaying how long it has been since last CPI exposure
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

    # sequence to turn cpi generator on
    def On(self):
        self.write('Status', 'Powering Generator On')
        self.setDigiOut(self.powerOnTask, HIGH)
        time.sleep(.6) #simulating push button
        self.setDigiOut(self.powerOnTask, LOW)
        time.sleep(4)
        self.write('Status', 'Idle')
        caput('cpi:GetKVP.PROC', 1) #try to update values since they will be out of sync
    
    # sequence to turn cpi generator off
    def Off(self):
        self.write('Status', 'Powering Generator Off')
        self.setDigiOut(self.powerOffTask, HIGH)
        time.sleep(.6) #simulating push button
        self.setDigiOut(self.powerOffTask, LOW)
        self.write('Status', 'Idle')

    def RadPrepOn(self):
        time.sleep(.1)
        self.setDigiOut(self.radPrepInTask, HIGH)

    def RadPrepOff(self):
        self.setDigiOut(self.radPrepInTask, LOW)
        time.sleep(.1)
    
    def setDigiOut(self, DAQtaskName, value ):
        #print 'setdigiout', value, str(DAQtaskName)
        if type(value) != numpy.ndarray:
            if value == 0:
                value = LOW
            else:
                value = HIGH
        self.processDAQstatus(DAQmxWriteDigitalLines(DAQtaskName,1,1,10.0,DAQmx_Val_GroupByChannel, value ,self.written, None))

    def processDAQstatus(self, errorcode):
        if errorcode != 0:
            print str(datetime.now())[:-3], "NI-DAQ error! Code:", errorcode

if __name__ == '__main__':
    server = SimpleServer()
    server.createPV(prefix, pvdb)
    driver = myDriver()
    # process CA transactions
    while True:
        server.process(0.0001)
