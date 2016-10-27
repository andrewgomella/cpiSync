#!/usr/bin/env python
from pcaspy import Driver, SimpleServer
import time
from PyDAQmx import *
import numpy as np
from epics import *
import threading
import winsound
import os, os.path
from datetime import datetime

#Revised version using Digital outputs with Eric's custom SSR module. Use same digital inputs on USB-6501
# - code for digital inputs should largely remain the same as the original version
# - for output portion, this version (mk4) controls digital outputs, rather than 
#   SSR's directly(mk1)
# - for input portion this uses a separate daq module, so this requires 2 modules total 

#Online reference for NI-DAQ
#http://zone.ni.com/reference/en-XX/help/370471W-01/

#Note "input" and "output" for variables are named from the perspective of the CPI generator, not the ni-daq. (Unless otherwise specified)
#This was so the terminology in the documentation for the cpi matches with the names in the code.

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
    'NikonExpose'   : { 'asyn' : True },
    'NikonShutterDelay'     
                    : {'value': 0.048, #value determined experimentally with D800
                        'prec': 3},
    'ImageRate'     : { 'prec': 2},
    'NikonBuffer'   : { 'prec': 0},
    'DocFlag'       : { 'value' : 1 },#enable documentation by default
    'CpiErrorString': { 'type': 'string'},
}

#Implemntation Specific Definitions
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
SCANIOC1='IOC:scan1'
SCANIOC2='IOC:scan2'
SCANIOC3='IOC:scan3'
SCANPROGRESSIOC="IOC:scanProgress:"

#So we know to immediately shut off ranode if scan is canceled
SCAN_CANCEL_IOC = PV('IOC:scan1.EXSC', callback = True) 

#Function to convert error code to text string whenever an error appears
CPI_ERROR_PV = PV('cpi:Error', callback = True)

#Nikon driver handles documentation, so we need to coordinate with it using teh following PV's:
#UPDATEDOCSTRING_PV is a process variable which signals camera driver to generate a doc string 
# (this is so it happens during an exposure and not before or after scan has progressed)
#D800_DOC is a process variable which tells us whether documentation is enabled or not in camera driver
UPDATEDOCSTRING_PV = PV('D800:UPDATEDOCSTRING') 
D800_DOC=PV('D800:DOC') 

#Need to allow Nikon some time between the end of an exposure and the start of the next
NIKON_RELOAD_TIME=0.04

#NIKON_BUFFER_MAX is max number of images to allowed to be stored in camera. for d800 this should be around 14
#NIKON_BUFFER_RESUME is what number of images in buffer to allow scan to resume at
NIKON_BUFFER_MAX=13
NIKON_BUFFER_RESUME=11

#Variable to prevent CPI polling during exposure sequence
CPI_GENERATOR_SCAN="cpi:GeneratorStatus.SCAN"
CPI_GENERATOR_PROC="cpi:GeneratorStatus.PROC"

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
        self.ExposeTimer = 0
        self.imageRateTimer = 0
        self.imagecount = 0
        self.cancel = 0
        self.nikonReloadTimer=time.time()

        #the following variables are needed for pydaqmx
        self.zero = numpy.zeros((1,), dtype=numpy.uint8)
        self.one = numpy.ones((1,), dtype=numpy.uint8)
        self.written = int32()
        self.x=np.array([0], dtype=np.uint8)

        #set up the warning sounds which will play out of computer speaks (needs to be unmuted)
        self.PrepSound=False
        self.ExposeSound=False
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
        self.radPrepOutTask = TaskHandle()
        DAQmxCreateTask("",byref(self.radPrepOutTask))
        DAQmxCreateDIChan(self.radPrepOutTask, NI_DAQ_CHANGEDETECT, "", DAQmx_Val_ChanForAllLines)

        #set up change detection as part of DI line task
        self._DIChangeCallback = DAQmxSignalEventCallbackPtr(self.DIChangeCallback)
        DAQmxCfgChangeDetectionTiming(self.radPrepOutTask, NI_DAQ_CHANGEDETECT, NI_DAQ_CHANGEDETECT ,DAQmx_Val_ContSamps,8)
        DAQmxRegisterSignalEvent(self.radPrepOutTask,DAQmx_Val_ChangeDetectionEvent,0,self._DIChangeCallback,None)
        DAQmxStartTask(self.radPrepOutTask)      

        #Declare variable for change detection
        self.multipleOld=numpy.array([0,0,0], dtype=numpy.uint8)
        print str(datetime.now()), 'cpiSync succesfully initialized'


    def DIChangeCallback(self, Task, status, callbackData): 
        startcallback=time.clock()
        multipleNew=numpy.array([0,0,0], dtype=numpy.uint8)
        DAQmxReadDigitalLines(self.radPrepOutTask, 1, 1, 0,  multipleNew, 3, None, None, None)
        #print 'old array:', self.multipleOld[0], self.multipleOld[1], self.multipleOld[2]
        #print 'new array:', multipleNew[0], multipleNew[1], multipleNew[2]
        if  multipleNew[2] != self.multipleOld[2]:
            #print 'expose out changed'
            if  multipleNew[2] == 1:
                self.ExposeSound = True
                self.setParam("ExposeOut", 1)
                self.ExposeTimer=time.clock() 
            else:
                self.setParam("ExposeOut", 0)
                #print str(datetime.now()), 'EXPOSE OFF, time between on/off signal:', time.clock()-self.ExposeTimer
                self.ExposeSound = False
                if self.getParam("Expose")==1:
                    self.write("Expose", 0)
                    #caput("cpi:GeneratorStatus.PROC",1)
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
        return 0 # The function should return an integer

    def warningsound(self):
        while(True):
            if self.ExposeSound == True:
                winsound.Beep(1500,500)            
            elif self.PrepSound == True:
                winsound.Beep(1318,500)
                winsound.Beep(1108,750)
            
    def write(self, reason, value):
        if reason == 'RadPrep' and value == 1:
            self.tid = threading.Thread(target = self.RadPrepOn, args = ())
            self.tid.start()
        elif reason == 'RadPrep' and value == 0:
            self.bid = threading.Thread(target = self.RadPrepOff, args = ())
            self.bid.start()
        elif reason == 'Expose' and value == 1:
            #only allow exposure if radprepout has already been enabled
            if self.getParam("RadPrepOut")==1:
                self.did = threading.Thread(target = self.ExposeOn, args = ())
                self.did.start()
        elif reason == 'Expose' and value == 0:
            self.cid = threading.Thread(target = self.ExposeOff, args = ())
            self.cid.start()
        elif reason == 'On' and value == 1:
            self.eid = threading.Thread(target = self.On, args = ())
            self.eid.start()
        elif reason == 'Off' and value == 1:
            self.fid = threading.Thread(target = self.Off, args = ())
            self.fid.start()
        elif reason == 'NikonExpose' and value == 1:
            self.fid = threading.Thread(target = self.NikonExpose, args = ())
            self.fid.start()
        self.setParam(reason, value)
        self.updatePVs()

    def NikonExpose(self):
        print str(datetime.now()), '!!!!! NikonExpose Sequence Starting !!!!!'
        ScanStatus=caget(SCANPROGRESSIOC +'running') #grab scan info once, we don't want to be waiting for excess cagets
        ShutterDelay=self.getParam("NikonShutterDelay") #grab param here to prevent slowing down sequence later
        if D800_DOC.get() == 1:
            print str(datetime.now()), 'update doc string request'
            UPDATEDOCSTRING_PV.put(1) #tell nikepics to update doc string for logging purposes
            while self.getParam('DocFlag') ==1:
                print str(datetime.now()), 'wait for doc string'
                time.sleep(.001)
        if ScanStatus == 1: #scan is in progress
            Nfinished=caget(SCANPROGRESSIOC+'Nfinished')
            NfinishedIOC= caget(SCANIOC1+ '.CPT')
            Ntotal=caget(SCANPROGRESSIOC+'Ntotal')
            print str(datetime.now()), "Scanstatus", ScanStatus, "Nfinished", Nfinished, "NfinishedIOC", NfinishedIOC, "Ntotal", Ntotal
            if self.getParam("RadReadyOut")==0: #the scan just started if this is true
                print str(datetime.now()), '!!!!! SCAN STARTING !!!!!'
                self.cancel = 0 #reset cancel flag
                self.setParam('NikonBuffer',0) # reset buffer count
                self.write("RadPrep", 1) #sets rad prep in to 1
                print str(datetime.now()), 'wait for scan start rad ready'
                while self.getParam("RadReadyOut") == 0: #wait for radready out from cpi to be 1
                    time.sleep(.001)
                self.imageRateTimer=0 #reset image rate timer
            while self.getParam('NikonBuffer') > NIKON_BUFFER_MAX: #buffer approaching max capacity, need to let it unload
                while self.getParam('NikonBuffer') > NIKON_BUFFER_RESUME: #wait for buffer to empty images               
                    print str(datetime.now()),'waiting for buffer to empty'
                    time.sleep(.001)
            if caget('IOC:scan1.EXSC') == 0: #this means the scan was aborted by user
                print str(datetime.now()),'SCAN ABORTED before shot'
                self.cancel = 1
        elif ScanStatus == 0: #not running a scan
            self.write("RadPrep", 1) #sets rad prep in to 1
            print str(datetime.now()), 'wait single shot for rad ready'
            while self.getParam("RadReadyOut") == 0:
                time.sleep(.0001)
        if self.cancel == 0: #go ahead with exposure
            print str(datetime.now()), 'starting exposure outputs'
            self.TriggerNikon() # send trigger release signal to nikon
            time.sleep(ShutterDelay) #Nikon shutter reportedly opens 43ms after triggering the shutter 
            self.ExposeOn() # send expose on signal to CPI
            time.sleep(.1)
            while self.getParam("ExposeOut") == 1 and self.cancel == 0:
                #print "wait for expose off"
                if ScanStatus !=0  and caget('IOC:scan1.EXSC') == 0: # scan canceled in middle of an exposure, shut expose in off
                    print 'SCAN ABORTED during exposure'
                    print caget(SCANIOC1 +'.FAZE')
                    self.cancel = 1
                    self.ExposeOff()
                    self.TriggerNikonReset()
                    self.setParam('NikonBuffer', self.getParam('NikonBuffer')+1 ) #image incoming to add to buffer count
        #exposure not canceled, end exposure per normal operation
        if self.cancel == 0:
            print str(datetime.now()), 'stopping exposure outputs'
            self.TriggerNikonReset() # set shutter cable to 0
            self.ExposeOff()   #set expose in to 0
            self.setParam('NikonBuffer', self.getParam('NikonBuffer')+1 ) #image incoming to add to buffer count
        if ScanStatus == 0: #scan not in progress its a single exposure
            self.write("RadPrep", 0) #turn off rad prep
        elif self.cancel == 0:
            time.sleep(NIKON_RELOAD_TIME)
            caput(SCANIOC1 + '.WAIT', 0) #tell scan we're done acquiring the image and it can move the positionermove on
            if caget(SCANPROGRESSIOC+'Nfinished')==Ntotal: #just took last image, so turn off radprep
                print str(datetime.now()), 'Took final scan image, turn rad prep off'
                self.write("RadPrep",0)
        elif self.cancel == 1:
            self.write("RadPrep",0)
        self.setParam('ImageRate', 1/(time.clock()-self.imageRateTimer))
        self.imageRateTimer=time.clock() #reset imagerate clock
        #self.cancel = 0 #reset cancel status
        self.setParam('DocFlag', 1) #reset doc flag 
        self.updatePVs()
        print str(datetime.now()), 'single NikonExpose sequence over'


    def ScanMonitor(self, **kw):
        print str(datetime.now()), 'SCAN canceled or finished'
        if self.getParam('RadPrep')==1 and SCAN_CANCEL_IOC.get() == 0: 
            self.write('RadPrep',0)

    def cpiErrorHandler(self, **kw):
        print str(datetime.now()), cpiErrorDictionary[kw['char_value']]
        self.setParam('CpiErrorString', cpiErrorDictionary[kw['char_value']])

    def On(self):
        x=DAQmxWriteDigitalLines(self.powerOnTask,1,1,10.0,DAQmx_Val_GroupByChannel,self.one,self.written, None)
        self.processDAQstatus(x)
        time.sleep(.6) #simulating push button
        x=DAQmxWriteDigitalLines(self.powerOnTask,1,1,10.0,DAQmx_Val_GroupByChannel,self.zero,self.written, None)
        self.processDAQstatus(x)

    def Off(self):
        x=DAQmxWriteDigitalLines(self.powerOffTask,1,1,10.0,DAQmx_Val_GroupByChannel,self.one,self.written, None)
        self.processDAQstatus(x)
        time.sleep(.6) #simulating push button
        x=DAQmxWriteDigitalLines(self.powerOffTask,1,1,10.0,DAQmx_Val_GroupByChannel,self.zero,self.written, None)
        self.processDAQstatus(x)

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
        self.setParam("NikonTrigger", 1)
        self.updatePVs()
        x=DAQmxWriteDigitalLines(self.NikonTriggerTask,1,1,10.0,DAQmx_Val_GroupByChannel,self.one,self.written, None)
        self.processDAQstatus(x)

    def TriggerNikonReset(self):
        self.setParam("NikonTrigger", 0)
        self.updatePVs()
        x=DAQmxWriteDigitalLines(self.NikonTriggerTask,1,1,10.0,DAQmx_Val_GroupByChannel,self.zero,self.written, None)
        self.processDAQstatus(x)

    def processDAQstatus(self, errorcode):
        if errorcode != 0:
            print str(datetime.now()), "NI-DAQ error! Code:", errorcode

if __name__ == '__main__':
    server = SimpleServer()
    server.createPV(prefix, pvdb)
    driver = myDriver()
    # process CA transactions
    while True:
        server.process(0.01)
