#!/usr/bin/env python
from pcaspy import Driver, SimpleServer
import time, threading, winsound, os, os.path, pickle
from PyDAQmx import *
from epics import *
import numpy as np
from datetime import datetime, timedelta
#Revised version using Digital outputs with Eric's custom SSR module. Use same digital inputs on USB-6501
# - code for digital inputs should largely remain the same as the original version
# - for output portion, this version (mk4) controls digital outputs, rather than 
#   SSR's directly(mk1)
# - for input portion this uses a separate daq module, so this requires 2 modules total 

#Online reference for NI-DAQ
#http://zone.ni.com/reference/en-XX/help/370471W-01/

#To-do
# - add cpi string translation for latching errors as well
# - re-do docstring code
# - generalize code so it will work with any detector- not just D800
# - change scanSeq code so it doesn't require cagets to scan IOC, (rely on callback alone?)

#Note "input" and "output" for variables are named from the perspective of the CPI generator, not the ni-daq. (Unless otherwise specified)
#This was so the terminology in the documentation for the cpi matches with the names in the code.

try:
    if os.path.isfile('objs.pickle'):
        with open('objs.pickle') as f:
            lastCpiExposure = pickle.load(f)
    else:
        lastCpiExposure = time.time()
except:
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
    'NikonShutterDelay'     
                    : { 'value': 0.048, #value determined experimentally with D800
                         'prec': 3},
    'ImageRate'     : {  'prec': 2},
    'NikonBuffer'   : {  'prec': 0},
    'DocFlag'       : { 'value': 1 },#enable documentation by default
    'CpiErrorString': {  'type': 'string'},
    'Status'        : {  'type': 'char',
                        'count': 300,
                        'value': 'Initialization'},
    'TimeSinceLastExposure'
                    : {   'type': 'string',
                          'scan': 1}
}

#Implementation Specific Definitions
NI_DAQ_INPUTS   = 'cpiSyncDAQ'
NI_DAQ_OUTPUTS  = 'FcpiSyncDAQ'

#These are the DAQ outputs (input to CPI etc.)
NIKON_TRIGGER_LINE  = NI_DAQ_OUTPUTS + '/port0/line3'
CPI_POWER_OFF_LINE  = NI_DAQ_OUTPUTS + '/port0/line4'
CPI_POWER_ON_LINE   = NI_DAQ_OUTPUTS + '/port0/line5'
CPI_RAD_PREP_IN     = NI_DAQ_OUTPUTS + '/port0/line6'
CPI_EXPOSE_IN       = NI_DAQ_OUTPUTS + '/port0/line7'

#These are the DAQ inputs (output from CPI etc.)
CPI_RAD_PREP_OUT    = NI_DAQ_INPUTS + '/port1/line0'
CPI_RAD_READY_OUT   = NI_DAQ_INPUTS + '/port1/line1'
CPI_EXPOSE_OUT      = NI_DAQ_INPUTS + '/port1/line2'

#Change Detection Lines- which lines to monitor for changes and do callbacks, so basically all DAQ inputs
NI_DAQ_CHANGEDETECT = NI_DAQ_INPUTS+ "/port1/line0:2"

#EPICS scan variables to keep track of scan so we know when to start/stop rad prep 
#very important to use scanProgress record instead of scan, so we can keep track of multi-dimensional scans
SCANIOC1       ='IOC:scan1'
SCANPROGRESSIOC='IOC:scanProgress:' 

#So we know to immediately shut off ranode if scan is canceled, and initiate abort() function
SCAN_CANCEL_IOC = PV('IOC:AbortScans.PROC', callback = True) 

#Function to convert error code to text string whenever an error appears
CPI_ERROR_PV    = PV('cpi:Error', callback = True)
CPI_KVP_PV      = PV('cpi:SetKVP')
CPI_MA_PV       = PV('cpi:SetMA')
CPI_MS_PV       = PV('cpi:SetMS')
CPI_SETFOCUS_PV = PV('cpi:SetFocus')
#Nikon driver handles whether we document
#D800_DOC is a process variable which tells us whether documentation is enabled or not in camera driver
D800_DOC=PV('D800:DOC') 

#Need to allow Nikon some time between the end of an exposure and the start of the next
#this is until we determine a way to monitor exposure status of DSLRs
#If in crop mode we may be able to decrease this to 0.2
NIKON_RELOAD_TIME=0.25

#NIKON_BUFFER_MAX is max number of images to allowed to be stored in camera. for d800 this should be around 14
#NIKON_BUFFER_RESUME is what number of images in buffer to allow scan to resume at
NIKON_BUFFER_MAX=13
NIKON_BUFFER_RESUME=11

#Variable to prevent CPI polling during exposure sequence
CPI_GENERATOR_SCAN='cpi:GeneratorStatus.SCAN'

#How often to pause in while loops
POLL_TIME = 0.0001

#PV_LIST for .txt documentation files to be generated
PV_LIST = [
           'cpi:GetKVP', 'cpi:GetMA', 'cpi:GetMS', 'cpi:GetFocus',\
           'srxr:GKVP', 'srxr:GUA', \
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
#Ideally this would be part of stream device driver, but enums can only obtain 16 variables 
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
        SCAN_CANCEL_IOC.add_callback(callback=self.ScanMonitor)
        CPI_ERROR_PV.add_callback(callback=self.cpiErrorHandler)
        self.lastCpiExposure = lastCpiExposure
        self.NikonReloadTimer=time.time()
        self.timeToReload = 0
        self.ExposeTimer = 0
        self.imageRateTimer = 0
        self.cancel = 0
        self.currentFunction = ''
        self.seqInProgress=0 #flag to determine if we are inside a "sequence" or not

        #the following variables are needed for pydaqmx
        self.zero = numpy.zeros((1,), dtype=numpy.uint8)
        self.one = numpy.ones((1,), dtype=numpy.uint8)
        self.written = int32()

        #set up the warning sounds which will play out of computer speaks (needs to be unmuted)
        self.PrepSound=False
        #self.ExposeSound=False
        sid=threading.Thread(target=self.warningsound)
        sid.start()

        #Initialize DAQ- don't reset with custom DO-SSR module because it will cause the SSR's to toggle on startup
        #DAQmxResetDevice(NI_DAQ_OUTPUTS)
        #DAQmxResetDevice(NI_DAQ_INPUTS)      

        #Set up DO task to trigger shutter sync cable to nikon
        self.NikonTriggerTask = TaskHandle()
        DAQmxCreateTask("",byref(self.NikonTriggerTask))
        DAQmxCreateDOChan(self.NikonTriggerTask, NIKON_TRIGGER_LINE, "", DAQmx_Val_ChanForAllLines)
        DAQmxSetDOOutputDriveType(self.NikonTriggerTask, NIKON_TRIGGER_LINE, DAQmx_Val_ActiveDrive)
        DAQmxStartTask(self.NikonTriggerTask)

        #Set up the DO task to turn off the CPI generator
        self.powerOffTask = TaskHandle()
        DAQmxCreateTask("",byref(self.powerOffTask))
        DAQmxCreateDOChan(self.powerOffTask, CPI_POWER_OFF_LINE, "", DAQmx_Val_ChanForAllLines)
        DAQmxSetDOOutputDriveType(self.powerOffTask, CPI_POWER_OFF_LINE, DAQmx_Val_ActiveDrive)      
        DAQmxStartTask(self.powerOffTask)

        #Set up the DO task to turn on the CPI generator
        self.powerOnTask = TaskHandle()
        DAQmxCreateTask("",byref(self.powerOnTask))
        DAQmxCreateDOChan(self.powerOnTask, CPI_POWER_ON_LINE, "", DAQmx_Val_ChanForAllLines)
        DAQmxSetDOOutputDriveType(self.powerOnTask, CPI_POWER_ON_LINE, DAQmx_Val_ActiveDrive)      
        DAQmxStartTask(self.powerOnTask)

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
        self.ExposeOff() #make sure it is off initially

        #task to read DI lines combined into one (significantly improves performance to do one write instead of 3)
        self.combinedTask = TaskHandle()
        DAQmxCreateTask("",byref(self.combinedTask))
        DAQmxCreateDIChan(self.combinedTask, NI_DAQ_CHANGEDETECT, "", DAQmx_Val_ChanForAllLines)

        #set up change detection as part of DI line task
        self._DIChangeCallback = DAQmxSignalEventCallbackPtr(self.DIChangeCallback)
        DAQmxCfgChangeDetectionTiming(self.combinedTask, NI_DAQ_CHANGEDETECT, NI_DAQ_CHANGEDETECT ,DAQmx_Val_ContSamps,8)
        DAQmxRegisterSignalEvent(self.combinedTask,DAQmx_Val_ChangeDetectionEvent,0,self._DIChangeCallback,None)
        DAQmxStartTask(self.combinedTask)      

        #Declare variable for change detection
        self.multipleOld=numpy.array([0,0,0], dtype=numpy.uint8)
        print str(datetime.now())[:-3], 'cpiSync succesfully initialized'
        
        #Now ready to do exposures, change from Initilization to Idle 
        self.write('Status', 'Idle')

    def DIChangeCallback(self, Task, status, callbackData): 
        multipleNew=numpy.array([0,0,0], dtype=numpy.uint8) #need to redeclare every time function runs
        DAQmxReadDigitalLines(self.combinedTask, 1, 1, 0,  multipleNew, 3, None, None, None)
        #print 'old array:', self.multipleOld[0], self.multipleOld[1], self.multipleOld[2]
        #print 'new array:', multipleNew[0], multipleNew[1], multipleNew[2]
        if  multipleNew[2] != self.multipleOld[2]:
            #print 'expose out changed'
            if  multipleNew[2] == 1:
                self.setParam("ExposeOut", 1)
                #self.ExposeTimer=time.time() 
            else:
                self.write("Expose", 0)
                self.setParam("ExposeOut", 0)
                #print str(datetime.now())[:-3], 'EXPOSE OFF, time between on/off signal:', time.time()-self.ExposeTimer
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
        return 0 # DAQMX requires callback function to return an integer

    def warningsound(self):
        while(self.PrepSound):
            winsound.Beep(1318,500)
            winsound.Beep(1108,750)
    
    def printNiceTimeDelta(self, seconds):
        #taken from stackoverflow example
        delay = timedelta(seconds=(seconds))
        if (delay.days > 0):
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
        if  reason == 'RadPrep' and value == 1:
            self.tid = threading.Thread(target = self.RadPrepOn, args = ())
            self.tid.start()
            if self.seqInProgress == 0 and self.cancel == 0:
                self.currentFunction = '(Manual)'
        elif reason == 'RadPrep' and value == 0:
            self.bid = threading.Thread(target = self.RadPrepOff, args = ())
            self.bid.start()
        elif reason == 'Expose' and value == 1:
            if self.getParam("RadPrepOut")==1:#only allow exposure if radprepout has already been enabled
                self.did = threading.Thread(target = self.ExposeOn, args = ())
                self.did.start()
                if self.seqInProgress == 0 and self.cancel == 0:
                    self.currentFunction = '(Manual)'
        elif reason == 'Expose' and value == 0:
            self.cid = threading.Thread(target = self.ExposeOff, args = ())
            self.cid.start()
        elif reason == 'NikonTrigger' and value == 1:
            self.fid = threading.Thread(target = self.TriggerNikon, args = ())
            self.fid.start()
        elif reason == 'NikonTrigger' and value == 0:
            self.fid = threading.Thread(target = self.TriggerNikonReset, args = ())
            self.fid.start()
        elif reason == 'On' and value == 1:
            self.eid = threading.Thread(target = self.On, args = ())
            self.eid.start()
        elif reason == 'Off' and value == 1:
            self.fid = threading.Thread(target = self.Off, args = ())
            self.fid.start()
        elif reason == 'NikonExpose' and value == 1:
            self.fid = threading.Thread(target = self.NikonExpose, args = ())
            self.fid.start()
        elif reason == 'NikonSingleExposeSeq' and value == 1:
            self.fid = threading.Thread(target = self.NikonSingleExposeSeq, args = ())
            self.fid.start()
        elif reason == 'NikonScanExposeSeq' and value == 1:
            self.fid = threading.Thread(target = self.NikonScanExposeSeq, args = ())
            self.fid.start()
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
        pathname = self.filepath + self.filename + str(self.filenum)
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
            self.write('Status', currentFunction + 'Complete')

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
            self.write('Status', currentFunction + 'Complete')


    def GenExposeOnly(self, kvp, current, millisec, numberOfExposures, dutyCycle):
        if self.cancel == 0:
            self.write('Status', self.currentFunction + ' Changing Settings')
            CPI_KVP_PV.put(kvp, wait=True)
            CPI_MA_PV.put(current, wait=True)
            CPI_MS_PV.put(millisec, wait=True) 
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
        self.seqInProgress=1
        self.prepExpose()         #do chores and prepare gen (during this time camera has more time to prepare)
        self.checkIfCameraReady() #see if camera is actually ready
        self.exposeNow()          #send trigger signals, returns when exposure is over
        self.exposeEnd()          #resets nikon trigger
        if self.cancel == 0 :   
            self.write("RadPrep",0)   #single expose, so turn RadPrep off
            self.write('Status', 'Idle')
        self.seqInProgress=0
        print str(datetime.now())[:-3], '!!!!! Nikon - CPI Single Shot Sequence Over !!!!!'

    #Take Nikon-CPI sync shot during a scan
    # ideally we will have the scan performing its next move while various nikon-cpi sync chores are also taking place
    def NikonScanExposeSeq(self):
        #scanseqtime= time.time()
        self.currentFunction = '(Nikon Scan Shot)'
        print str(datetime.now())[:-3], '!!!!! Nikon - CPI Single Scan Shot Sequence Start !!!!!'
        self.seqInProgress=1
        self.prepExpose() #do chores and prepare gen (do nothing if gen is already ready)
        self.checkIfCameraReady() #chores finished, see if camera is actually ready
        self.exposeNow() #Returns when exposure is over
        self.exposeEnd()
        if self.cancel == 0:
            if caget(SCANPROGRESSIOC+'Nfinished') + 1== caget(SCANPROGRESSIOC +'Ntotal'): #just took last image, so turn off radprep
                self.write("RadPrep",0)
                self.write('Status', 'Idle')
            else:
                self.write('Status', self.currentFunction + ' Wait sscan ')
        if self.cancel == 0:
            caput(SCANIOC1 + '.WAIT', 0) #tell scan we're finished acquiring the image and it can progress
        self.seqInProgress=0
        print str(datetime.now())[:-3], '!!!!! Nikon - CPI Single Scan Shot Sequence Over !!!!!'
        #print str(datetime.now())[:-3], 'time waste', time.time() - scanseqtime - 1 - self.ShutterDelay - self.timeToReload

    #Checks if gen is ready to expose, if not prepares generator for an exposure
    #   function should only ever run at the beginning of a single shot OR a scan, never in the middle
    #   so we can do house keeping tasks here as well while waiting for rad gen ready
    #Returns when generator is ready to expose
    def prepExpose(self):
        if self.getParam('RadReadyOut') == 0 and self.cancel == 0:          
            self.write("RadPrep", 1) #sets rad prep in to CPI to 1
            self.write('Status', self.currentFunction + ' Wait RadReady')
            #Signal is sent to radprep, will take a few seconds, so do housekeeping-
            #   -grab info for doc string, filenum will change during scan so we will increment it later
            self.filepath=caget('D800:FILEPATH')
            self.filename=caget('D800:FILENAME')
            self.filenum=caget('D800:FILENUM')
            self.docmode=D800_DOC.get()
            #   -reset buffer to zero
            self.write('NikonBuffer',0) 
            #   -make sure we have the latest NikonShutterDelay value
            self.ShutterDelay=self.getParam("NikonShutterDelay")
            #   -reset imageRateTimer so we can keep track of exposures/sec
            self.imageRateTimer=0 
            while self.getParam('RadReadyOut') == 0 and self.cancel == 0:
                time.sleep(POLL_TIME)
   
    #Check to see if camera is ready to take an image, to be run immediately before exposeNow
    # -first check if buffer is full, if it is wait until it is equal NIKON_BUFFER_RESUME
    # -next, check if Nikon has had enough time since the last shot to prepare for next shot (NIKON_RELOAD_TIME)
    # -indifferent to scan status
    def checkIfCameraReady(self):
        if self.cancel == 0:
            #Check NikonBuffer:
            if self.getParam('NikonBuffer') > NIKON_BUFFER_MAX: #buffer approaching max capacity, need to let it unload
                self.write('Status', self.currentFunction + ' Nikon buffer full-emptying')
                while self.getParam('NikonBuffer') > NIKON_BUFFER_RESUME and self.cancel == 0: #wait for buffer to empty images               
                    time.sleep(POLL_TIME)
            #Check Nikon reload timer:
            #   for sample scans, this reload time may already be reached due to waiting for the motor to complete move
            #   however, if that happens very quickly, we may need to wait to ensure that the nikon is ready for the next shot
            if time.time()-self.NikonReloadTimer < NIKON_RELOAD_TIME:
                self.timeToReload= (NIKON_RELOAD_TIME - (time.time()-self.NikonReloadTimer))
                self.write('Status', '%s Waiting %.3f sec for Nikon reload' % (self.currentFunction, self.timeToReload))
                while time.time()-self.NikonReloadTimer < NIKON_RELOAD_TIME:
                    time.sleep(POLL_TIME)

    #Call when immediately ready to send sync signals camera + x-ray
    #   -ends trigger signal to Nikon, waits ShutterDelay, then sends expose signal to CPI, then calls self.document()
    #   -indifferent to scan status
    def exposeNow(self):
        if self.cancel == 0:
            self.write('Status', self.currentFunction + ' EXPOSING!')
            self.write('NikonTrigger',1) # send trigger release signal to nikon
            shutterstart=time.time()
            self.setParam('NikonBuffer', self.getParam('NikonBuffer')+1 ) #image incoming to add to buffer count
            while time.time() -shutterstart < self.ShutterDelay:
                time.sleep(POLL_TIME)
            #time.sleep(self.ShutterDelay) #Nikon shutter reportedly opens 43ms after triggering the shutter 
            self.write('Expose', 1) #send trigger signal to CPI expose
            if self.docmode == 1: #to save time generate doc string during exposure
                self.document()
                self.filenum=self.filenum+1 #increment local count of filenum, might be overwritten later if corrected
            #Wait for end of exposure, unless something sets ca ncel flag to 1, then return
            # -here is where we need some sort of signal that the camera has finished exposing
            # -we can use a shutter speed timer until we have a physical signal coming from the camera 
            while self.getParam("Expose") == 1 and self.cancel == 0:
                time.sleep(POLL_TIME)

    #Call after nikon exposure has finished to set up for the next exposure, calculate image rate, update buffer
    #   -indifferent to scan status
    #   -no need to check for self.cancel status at this point
    def exposeEnd(self):
        if self.cancel == 0:
            print str(datetime.now())[:-3], 'stopping exposure outputs'
            self.write('NikonTrigger', 0) # set shutter cable to 0
            self.NikonReloadTimer=time.time() #reset timer so we don't trigger nikon again too soon
            self.setParam('ImageRate', 1/(time.clock()-self.imageRateTimer))
            self.imageRateTimer=time.clock() #reset imagerate clock
            self.lastCpiExposure = time.time()
            with open('objs.pickle', 'w') as f:
                pickle.dump(self.lastCpiExposure, f)
            self.updatePVs()

    #Call when user hits abort button (through write function) or hits abort on scan (through callback function)
    # -set self.cancel = 1 - this lets any sequences in progress know to cancel 
    # -Turn expose off
    # -Turn radprep off
    # -Reset nikon trigger
    # -cancel scan if in progress 
    # -Wait for sequences to all finish (self.seqInProgress=0)
    def abort(self):
        self.write('Status', self.currentFunction + ' ABORTING!')
        self.cancel=1 # flag to tell exposures to cancel
        self.write('Expose', 0)
        self.write('RadPrep',0)
        self.write('NikonTrigger', 0)
        self.NikonReloadTimer=time.time() #reset timers
        self.imageRateTimer=time.clock() 
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

    def cpiErrorHandler(self, **kw):
        print str(datetime.now())[:-3], 'Error Callback', cpiErrorDictionary[kw['char_value']]
        self.setParam('CpiErrorString', cpiErrorDictionary[kw['char_value']])

    def On(self):
        self.write('Status', 'Powering Generator On')
        x=DAQmxWriteDigitalLines(self.powerOnTask,1,1,10.0,DAQmx_Val_GroupByChannel,self.one,self.written, None)
        self.processDAQstatus(x)
        time.sleep(.6) #simulating push button
        x=DAQmxWriteDigitalLines(self.powerOnTask,1,1,10.0,DAQmx_Val_GroupByChannel,self.zero,self.written, None)
        self.processDAQstatus(x)
        self.write('Status', 'Idle')


    def Off(self):
        self.write('Status', 'Powering Generator Off')
        x=DAQmxWriteDigitalLines(self.powerOffTask,1,1,10.0,DAQmx_Val_GroupByChannel,self.one,self.written, None)
        self.processDAQstatus(x)
        time.sleep(.6) #simulating push button
        x=DAQmxWriteDigitalLines(self.powerOffTask,1,1,10.0,DAQmx_Val_GroupByChannel,self.zero,self.written, None)
        self.processDAQstatus(x)
        self.write('Status', 'Idle')

    def RadPrepOn(self):
        caput(CPI_GENERATOR_SCAN, "Passive") #have trouble with the unsolicited commands unless we stop polling
        time.sleep(.1)
        x = DAQmxWriteDigitalLines(self.radPrepInTask,1,1,10.0,DAQmx_Val_GroupByChannel,self.one,self.written, None)
        self.processDAQstatus(x)

    def RadPrepOff(self):
        x=DAQmxWriteDigitalLines(self.radPrepInTask,1,1,10.0,DAQmx_Val_GroupByChannel,self.zero,self.written, None)
        self.processDAQstatus(x)
        time.sleep(.1)
        caput(CPI_GENERATOR_SCAN, "2 second")#now that exposure is over resume polling

    def ExposeOn(self):      
        x=DAQmxWriteDigitalLines(self.exposeInTask,1,1,10.0,DAQmx_Val_GroupByChannel,self.one,self.written, None)
        self.processDAQstatus(x)

    def ExposeOff(self):
        x=DAQmxWriteDigitalLines(self.exposeInTask,1,1,10.0,DAQmx_Val_GroupByChannel,self.zero,self.written, None)
        self.processDAQstatus(x)

    def TriggerNikon(self):
        x=DAQmxWriteDigitalLines(self.NikonTriggerTask,1,1,10.0,DAQmx_Val_GroupByChannel,self.one,self.written, None)
        self.processDAQstatus(x)

    def TriggerNikonReset(self):
        x=DAQmxWriteDigitalLines(self.NikonTriggerTask,1,1,10.0,DAQmx_Val_GroupByChannel,self.zero,self.written, None)
        self.processDAQstatus(x)

    def processDAQstatus(self, errorcode):
        if errorcode != 0:
            print str(datetime.now())[:-3], "NI-DAQ error! Code:", errorcode

if __name__ == '__main__':
    server = SimpleServer()
    server.createPV(prefix, pvdb)
    driver = myDriver()
    # process CA transactions
    while True:
        server.process(0.001)
