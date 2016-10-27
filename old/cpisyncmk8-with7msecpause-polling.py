#!/usr/bin/env python
from pcaspy import Driver, SimpleServer
import time, threading, winsound, os, os.path, pickle
from PyDAQmx import *
from epics import *
import numpy as np
from datetime import datetime, timedelta
#Revised version using Digital outputs with Eric's custom SSR module. Use same digital inputs on USB-6501

#Online reference for NI-DAQ
#http://zone.ni.com/reference/en-XX/help/370471W-01/

#To-do
# - add cpi string translation for latching errors as well
# - re-do docstring code
# - change scanSeq code so it doesn't require cagets to scan IOC, (rely on callback alone?)

#Note "input" and "output" for variables are named from the perspective of the CPI generator, not the ni-daq. (Unless otherwise specified)
#This was so the terminology in the documentation for the cpi matches with the names in the code.

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

prefix = 'cpiSync:'
pvdb = {
    'On'            : { 'asyn' : True },
    'Off'           : { 'asyn' : True },
    'NikonTrigger'  : { 'asyn' : True },
    'RadPrep'       : { 'asyn' : True },
    'Expose'        : { 'asyn' : True },
    'RadPrepOut'    : { 'asyn' : True },
    'RadReadyOut'   : { 'asyn' : True },
    'ExposeOut'     : { 'asyn' : True },
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
    'Document'      : { }
}

#Implementation Specific Definitions
#Currently using 2 DAQ's due to the CPI outputs requiring special digital input type
CPI_INPUT_DAQ   = 'cpiSyncDAQ'
NI_DAQ_OUTPUTS  = 'FcpiSyncDAQ'

#These are the DAQ outputs (input to CPI etc.) Currently all control Eric's custom SSR module
#NIKON_TRIGGER_LINE  = NI_DAQ_OUTPUTS + '/port0/line3' #no longer used, was previously for D800
CPI_POWER_OFF_LINE  = NI_DAQ_OUTPUTS + '/port0/line4' #pulse turns CPI off
CPI_POWER_ON_LINE   = NI_DAQ_OUTPUTS + '/port0/line5' #pulse turns CPI on
CPI_RAD_PREP_IN     = NI_DAQ_OUTPUTS + '/port0/line6' #setting high makes enables rad prep
CPI_EXPOSE_IN       = NI_DAQ_OUTPUTS + '/port0/line2' #setting high allows exposure

################QI2 Defines###################
#QI2_EXPOSE when set to high requests exposure
QI2_EXPOSE       = NI_DAQ_OUTPUTS + '/port2/line0'


#QI2_TRIGGERREADY is high when ready to expose
QI2_TRIGGERREADY = NI_DAQ_OUTPUTS + '/port2/line1'
#QI2_EXPOSEOUT is high when it is exposing
QI2_EXPOSEOUT    = NI_DAQ_OUTPUTS + '/port2/line2'
#QI2 lines to poll
QI2_INPUTS       = QI2_TRIGGERREADY + "," + QI2_EXPOSEOUT

#These are the DAQ inputs (output from CPI etc.)

CPI_RAD_PREP_OUT    = CPI_INPUT_DAQ + '/port1/line0'
CPI_RAD_READY_OUT   = CPI_INPUT_DAQ + '/port1/line1'
CPI_EXPOSE_OUT      = CPI_INPUT_DAQ + '/port1/line2'

#Change Detection Lines- which lines to monitor for changes and do callbacks, so basically all CPI DAQ inputs
NI_DAQ_CHANGEDETECT = CPI_RAD_PREP_OUT + "," + CPI_RAD_READY_OUT + "," + CPI_EXPOSE_OUT 

#EPICS scan variables to keep track of scan so we know when to start/stop rad prep 
#very important to use scanProgress record instead of scan, so we can keep track of multi-dimensional scans
SCANIOC1       ='IOC:scan1'
SCANPROGRESSIOC='IOC:scanProgress:' 
SCANPROGRESSIOC_PV=PV('IOC:scanProgress')

#So we know to immediately shut off ranode if scan is canceled, and initiate abort() function
SCAN_CANCEL_IOC = PV('IOC:AbortScans.PROC', callback = True) 

#Function to convert error code to text string whenever an error appears
#CPI_ERROR_PV    = PV('cpi:Error',  callback = True)
#CPI_LATCHING_PV = PV('cpi:ErrorL', callback = True)
#Functions to allow changes of CPI settings for automatic warm-up procedures 
CPI_KVP_PV      = PV('cpi:SetKVP')
CPI_MA_PV       = PV('cpi:SetMA')
CPI_MS_PV       = PV('cpi:SetMS')
CPI_SETFOCUS_PV = PV('cpi:SetFocus')

#What PV's to grab the filepath and filename from for text doc writing
FILEPATH_PV=PV('Qi2:TIFF1:FilePath')
FILENAME_PV=PV('Qi2:TIFF1:FileName', callback = True) #reset Filenum to zero when called
#FILENUM_PV =PV('Qi2:TIFF1:FileNumber')

#Variable to prevent CPI polling during exposure sequence
CPI_GENERATOR_SCAN='cpi:GeneratorStatus.SCAN'

#Constant numpy arrays for setting digital outputs high or low on NI-DAQs
LOW  = numpy.zeros((1,), dtype=numpy.uint8)
HIGH = numpy.ones((1,), dtype=numpy.uint8)

#How often to pause in "while" loops
POLL_TIME = 0.001

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
#Ideally this would be part of stream device driver, but enums can only contain 16 variables 
#  and we need many more than that. Eventually will move CPI code from streamdevice to a 
#  custom C++ driver, and we will be able to move this list into that. 
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
        #CPI_ERROR_PV.add_callback(    callback=self.cpiErrorHandler)
        #CPI_LATCHING_PV.add_callback( callback=self.cpiErrorHandler)
        FILENAME_PV.add_callback(     callback=self.resetFileNum)
        self.lastCpiExposure = lastCpiExposure
        self.cancel = 0 #cancel flag for abort procedure 
        self.currentFunction = ''
        self.seqInProgress=0 #flag to determine if we are inside a "sequence" or not

        #the following variable is needed for pydaqmx
        self.written = int32()
        
        #set up the warning sounds which will play out of computer speakers (needs to be unmuted)
        self.PrepSound=False
        sid=threading.Thread(target=self.warningsound)
        sid.start()

        #Initialize DAQ- don't reset with custom DO-SSR module because it will cause the SSR's to toggle on startup
        #DAQmxResetDevice(NI_DAQ_OUTPUTS)
        #DAQmxResetDevice(NI_DAQ_INPUTS)

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
        #self.ExposeOff() #make sure it is off initially
        self.setDigiOut(self.exposeInTask, LOW)

        #task to read DI lines combined into one (significantly improves performance to do one read instead of 3)
        self.combinedTask = TaskHandle()
        DAQmxCreateTask("",byref(self.combinedTask))
        DAQmxCreateDIChan(self.combinedTask, NI_DAQ_CHANGEDETECT, "", DAQmx_Val_ChanForAllLines)

        #This part is deleted in this verison due to this version using polling (faster than change detection)
        #set up change detection as part of DI line task
        ##self._DIChangeCallback = DAQmxSignalEventCallbackPtr(self.DIChangeCallback)
        ##DAQmxCfgChangeDetectionTiming(self.combinedTask, NI_DAQ_CHANGEDETECT, NI_DAQ_CHANGEDETECT ,DAQmx_Val_ContSamps,8)
        #DAQmxRegisterSignalEvent(self.combinedTask,DAQmx_Val_ChangeDetectionEvent,0,self._DIChangeCallback,None)
        DAQmxStartTask(self.combinedTask)      

        #Declare variable for change detection callback comparison
        self.multipleOld=numpy.array([0,0,0], dtype=numpy.uint8)

        #Set up polling for the 2 Qi2 Inputs
        self.qi2PollTask = TaskHandle()
        DAQmxCreateTask("",byref(self.qi2PollTask))
        DAQmxCreateDIChan(self.qi2PollTask, QI2_INPUTS , "", DAQmx_Val_ChanForAllLines)
        DAQmxStartTask(self.qi2PollTask)
        #Start the polling thread
        self.xid = threading.Thread(target=self.pollQi2,args=())
        self.xid.start()

        ##start the cpi polling thread
        self.cid = threading.Thread(target=self.pollCPI,args=())
        self.cid.start()

        #Now ready to do exposures, change from Initilization to Idle 
        self.write('Status', 'Idle')
        print str(datetime.now())[:-3], 'cpiSync succesfully initialized'

    #since the ni daq 6501 doesnt support change detection, we have to poll and do change detection ourselves
    def pollQi2(self):
        f = open('test','a+')
        oldval = np.array([0,0], dtype=np.uint8)
        DAQmxReadDigitalLines(self.qi2PollTask, 1, 1, 0,  oldval, 2, None, None, None) 
        while True:
            temptest = time.clock()
            newval = np.array([0,0], dtype=np.uint8)
            DAQmxReadDigitalLines(self.qi2PollTask, 1, 1, 0,  newval, 2, None, None, None) 
            self.setParam("Qi2TriggerReady", newval[0])
            self.setParam("Qi2Exposing", newval[1])
            if newval[1] != oldval[1]:
                # change occured
                if newval[1] == 1:
                    print str(datetime.now())[:-3], 'Qi2 START EXPOSURE'
                else:
                    print str(datetime.now())[:-3], 'Qi2 STOP EXPOSURE'
                oldval = newval
            self.updatePVs()
            f.write(str(time.clock()-temptest)+'\n')
            time.sleep(.001)

    #since the ni daq 6501 doesnt support change detection, we have to poll and do change detection ourselves
    def pollCPI(self):
        oldval = np.array([0,0,0], dtype=np.uint8)
        DAQmxReadDigitalLines(self.combinedTask, 1, 1, 0,  oldval, 3, None, None, None) 
        while True:
            newval = np.array([0,0,0], dtype=np.uint8)
            DAQmxReadDigitalLines(self.combinedTask, 1, 1, 0,  newval, 3, None, None, None) 
            self.setParam("ExposeOut", newval[2])
            self.setParam("RadPrepOut", newval[0])
            self.setParam("RadReadyOut", newval[1])
            if newval[2] != oldval[2]:
                # change occured
                if newval[2] == 1:
                    print str(datetime.now())[:-3], 'GENERATOR START EXPOSURE'
                else:
                    print str(datetime.now())[:-3], 'GENERATOR STOP EXPOSURE'
                    if self.getParam("Expose") == 1:
                        self.lastCpiExposure = time.time()
                        self.write("Expose", 0)
                #oldval = newval
            if np.not_equal(newval, oldval).any:
                oldval = newval
            self.updatePVs()
            time.sleep(.001)

    """
    def DIChangeCallback(self, Task, status, callbackData): 
        temptest = time.time()
        multipleNew=numpy.array([0,0,0], dtype=numpy.uint8) #need to redeclare every time function runs
        DAQmxReadDigitalLines(self.combinedTask, 1, 1, 0,  multipleNew, 3, None, None, None)
        if  multipleNew[2] != self.multipleOld[2]:
            #print 'expose out changed'
            if  multipleNew[2] == 1:
                #print  "Expose out is now high"
                self.setParam("ExposeOut", 1)
                print str(datetime.now())[:-3], 'GENERATOR START EXPOSURE'
            else:
                print str(datetime.now())[:-3], 'GENERATOR STOP EXPOSURE'
                if self.getParam("Expose") == 1:
                    self.lastCpiExposure = time.time()
                    self.write("Expose", 0)
                self.setParam("ExposeOut", 0)
        if multipleNew[0] != self.multipleOld[0]:
            #print 'radprepout changed'
            if multipleNew[0] == 1:
                self.setParam("RadPrepOut", 1)
                self.PrepSound = True
            else:
                self.setParam("RadPrepOut", 0)
                self.PrepSound = False
        if multipleNew[1] != self.multipleOld[1]:
            #print 'radreadyout changed'
            if multipleNew[1] == 1:
                self.setParam("RadReadyOut", 1)
            else:
                self.setParam("RadReadyOut", 0)  
        self.updatePVs()
        self.multipleOld = multipleNew
        print "time for callback", time.time()-temptest
        return 0 # DAQMX requires callback function to return an integer
    """

    def warningsound(self):
        while(self.PrepSound):
            winsound.Beep(1318,500)
            winsound.Beep(1108,750)
    
    def printNiceTimeDelta(self, seconds):
        #modified from stackoverflow example
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
        elif reason == 'NikonTrigger':
            self.fid = threading.Thread(target = self.setDigiOut, args = (self.Qi2TriggerTask, value))
            self.fid.start()
        elif reason == 'On' and value == 1:
            self.eid = threading.Thread(target = self.On, args = ())
            self.eid.start()
        elif reason == 'Off' and value == 1:
            self.fid = threading.Thread(target = self.Off, args = ())
            self.fid.start()
        elif reason == 'NikonSingleExposeSeq' and value == 1:
            #if Qi2 trigger isn't ready the user likely forgot to enable collection on Qi2
            # otherwise may be a problem with the Qi2 
            if self.getParam('Qi2TriggerReady')==1:
                self.fid = threading.Thread(target = self.NikonSingleExposeSeq, args = ())
                self.fid.start()
            else:               
                self.write('Status',  'Error, Qi2 not ready to expose. ')
        elif reason == 'NikonScanExposeSeq' and value == 1:
            self.fid = threading.Thread(target = self.NikonScanExposeSeq, args = ())
            self.fid.start()
            if self.getParam('Qi2TriggerReady')!=1:
                self.write('Status',  'Error, Qi2 not ready to expose. ')
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
                self.write('Expose', 1) 
                #wait for exposure to finish
                while self.getParam('Expose') == 1 and self.cancel == 0:
                    time.sleep(POLL_TIME)
                self.write('Status', self.currentFunction + ' wait duty cycle')
                if self.cancel == 1:
                    break
                time.sleep(dutyCycle)
            if self.cancel == 0:
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
        if self.cancel == 0:
            if caget(SCANPROGRESSIOC+'Nfinished') + 1 == caget(SCANPROGRESSIOC +'Ntotal') or caget(SCANIOC1 + '.CPT') + 1 == caget(SCANIOC1 + '.NPTS'): #just took last image, so turn off radprep
                print "RadPrep turning off!"
                self.write("RadPrep",0)
                self.write('Status', 'Idle')
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

   
    #Check to see if camera is ready to take an image (to be run immediately before exposeNow)
    # -indifferent to scan status
    def checkIfCameraReady(self):
        if self.cancel == 0:
            #For Qi2 this is extremely simple. If TriggerReady is high, it is ready to expose
            if self.getParam('Qi2TriggerReady') != 1:
                self.write('Status', self.currentFunction + ' Wait Qi2 Trigger Ready')
                while self.getParam('Qi2TriggerReady') != 1:
                    time.sleep(POLL_TIME)

    #Call when immediately ready to send sync signals camera + x-ray
    #   -Sends trigger signal to both Qi2 and CPI then calls self.document() if document mode is enabled
    #   -indifferent to scan status
    def exposeNow(self):
        if self.cancel == 0:
            self.write('Status', self.currentFunction + ' EXPOSING!')
            self.write('NikonTrigger',1) # send trigger release signal to nikon
            time.sleep(0.007)
            #self.busy_wait(0.007)
            print str(datetime.now())[:-3], 'Qi2 Expose request sent'
            self.write('Expose', 1) #send trigger signal to CPI expose
            print str(datetime.now())[:-3], 'Generator Expose request sent'
            if self.docmode == 1: #to save time generate doc string during exposure, usually takes less than 100ms
                self.document()
                self.filenum=self.filenum+1 #increment local count of filenum, might be overwritten later
            #Wait for end of exposure AND Qi2 exposure, unless something sets cancel flag to 1, then return
            while self.getParam("Expose") == 1 or self.getParam("Qi2Exposing") == 1:
                time.sleep(POLL_TIME)
                if self.cancel == 1:
                    break
    
    def busy_wait(self, dt):   
        current_time = time.time()
        while (time.time() < current_time+dt):
            pass

    #Call after nikon exposure has finished to set up for the next exposure, calculate image rate, update buffer
    #   -indifferent to scan status
    def exposeEnd(self):
        if self.cancel == 0:
            print str(datetime.now())[:-3], 'resetting qi2 exposure output'
            self.write('NikonTrigger', 0) # set shutter cable to 0
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

    #def cpiErrorHandler(self, **kw):
    #    print str(datetime.now())[:-3], 'Error Callback', cpiErrorDictionary[kw['char_value']]
    #    self.setParam('CpiErrorString', cpiErrorDictionary[kw['char_value']])

    # sequence to turn cpi generator on
    def On(self):
        self.write('Status', 'Powering Generator On')
        self.setDigiOut(self.powerOnTask, HIGH)
        time.sleep(.6) #simulating push button
        self.setDigiOut(self.powerOnTask, LOW)
        self.write('Status', 'Idle')
        time.sleep(4)
        caput('cpi:GetKVP.PROC', 1) #try to update values since they will be out of sync
    
    # sequence to turn cpi generator off
    def Off(self):
        self.write('Status', 'Powering Generator Off')
        self.setDigiOut(self.powerOffTask, HIGH)
        time.sleep(.6) #simulating push button
        self.setDigiOut(self.powerOffTask, LOW)
        self.write('Status', 'Idle')

    def RadPrepOn(self):
        #caput(CPI_GENERATOR_SCAN, "Passive") #have trouble with the unsolicited commands unless we stop polling
        time.sleep(.1)
        self.setDigiOut(self.radPrepInTask, HIGH)

    def RadPrepOff(self):
        self.setDigiOut(self.radPrepInTask, LOW)
        time.sleep(.1)
        #caput(CPI_GENERATOR_SCAN, "2 second")#now that exposure is over resume polling
    
    def setDigiOut(self, DAQtaskName, value ):
        if type(value) != numpy.ndarray:
            if value == 0:
                value = LOW
            else:
                value = HIGH
        self.processDAQstatus(DAQmxWriteDigitalLines(DAQtaskName,1,1,10.0,DAQmx_Val_GroupByChannel, value ,self.written, None))

    def processDAQstatus(self, errorcode):
        if errorcode != 0:
            print str(datetime.now())[:-3], "NI-DAQ error! Code:", errorcode

    def resetFileNum(self, **kw):
        self.rid = threading.Thread(target = self.reset, args=())
        self.rid.start()

    def reset(self):
        caput('Qi2:TIFF1:FileNumber', 0)


if __name__ == '__main__':
    server = SimpleServer()
    server.createPV(prefix, pvdb)
    driver = myDriver()
    # process CA transactions
    while True:
        server.process(0.001)
