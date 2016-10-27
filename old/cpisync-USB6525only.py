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

#Note "input" and "output" for variables are named from the perspective of the CPI generator, not the ni-daq.
#This was so the terminology in the documentation for the cpi matches with the names in the code. 
prefix = 'cpiSync:'
pvdb = {
    'On'     : { 'asyn' : True },
    'Off'     : { 'asyn' : True },
    'NikonTrigger'     : { 'asyn' : True },
    'RadPrep'     : { 'asyn' : True },
    'Expose'      : { 'asyn' : True },
    'RadPrepOut'      : { 'asyn' : True },
    'RadReadyOut'      : { 'asyn' : True },
    'ExposeOut'      : { 'asyn' : True },
    'SyncCam'      : { 'asyn' : True},
    'NikonExpose'     : { 'asyn' : True },
    'NikonShutterDelay'     : {'value': 0.048,
                                'prec':3},
    'ImageRate'     : { 'prec': 2},
    'NikonBuffer'     : { 'prec': 0},
        'DocFlag'     : { 'value' : 1 },


}

SCANIOC1='IOC:scan1'
SCANIOC2='IOC:scan2'
SCANIOC3='IOC:scan3'

SCANPROGRESSIOC="IOC:scanProgress:"

SCAN_CANCEL_IOC = PV('IOC:scan1.EXSC', callback = True)
UPDATEDOCSTRING_PV = PV('D800:UPDATEDOCSTRING')
D800_DOC=PV('D800:DOC')

class myDriver(Driver):
    def  __init__(self):
        super(myDriver, self).__init__()
        SCAN_CANCEL_IOC.add_callback(callback=self.ScanMonitor)
        self.ExposeTimer = 0
        self.imageRateTimer = 0
        self.imagecount = 0
        self.cancel = 0
        #the following variables are needed for pydaqmx
        self.zero = numpy.zeros((1,), dtype=numpy.uint8)
        self.one = numpy.ones((1,), dtype=numpy.uint8)
        self.SyncNikonHigh = numpy.ones((2,), dtype=numpy.uint8)
        self.SyncNikonLow = numpy.zeros((2,), dtype=numpy.uint8)

        self.ExposeOnly = numpy.array([0,1], dtype=numpy.uint8)
        self.NikonOnly = numpy.array([1,0], dtype=numpy.uint8)

        self.written = int32()
        self.x=np.array([0], dtype=np.uint8)
        #set up the warning sounds which will play out of computer speaks (needs to be unmuted)
        self.PrepSound=False
        self.ExposeSound=False
        sid=threading.Thread(target=self.warningsound)
        sid.start()

        DAQmxResetDevice('cpiSyncDAQ')

        
        #Set up DO task which will handle both camera triggering and the expose in signal to the generator. 
        #We could set up them in different tasks, but then would have to trigger them sequentially
        #having them in the same task should allow for simulatenous triggering of both devices 
        #self.SyncTask = TaskHandle()
        #DAQmxCreateTask("",byref(self.SyncTask))
        #DAQmxCreateDOChan(self.SyncTask,"cpiSyncDAQ/port0/line3,cpiSyncDAQ/port0/line7", "", DAQmx_Val_ChanForAllLines)
        #DAQmxStartTask(self.SyncTask)

        #Set up DO task to trigger shutter sync cable to nikon
        self.NikonTriggerTask = TaskHandle()
        DAQmxCreateTask("",byref(self.NikonTriggerTask))
        DAQmxCreateDOChan(self.NikonTriggerTask,"cpiSyncDAQ/port0/line3", "", DAQmx_Val_ChanForAllLines)
        DAQmxStartTask(self.NikonTriggerTask)

        #Set up the DO task to turn off the CPI generator
        self.powerOffTask = TaskHandle()
        DAQmxCreateTask("",byref(self.powerOffTask))
        DAQmxCreateDOChan(self.powerOffTask,"cpiSyncDAQ/port0/line4", "", DAQmx_Val_ChanForAllLines)
        DAQmxStartTask(self.powerOffTask)

        #Set up the DO task to turn on the CPI generator
        self.powerOnTask = TaskHandle()
        DAQmxCreateTask("",byref(self.powerOnTask))
        DAQmxCreateDOChan(self.powerOnTask,"cpiSyncDAQ/port0/line5", "", DAQmx_Val_ChanForAllLines)
        DAQmxStartTask(self.powerOnTask)

        #Set up the DO task to send the Rad Prep signal to the generator.
        self.radPrepInTask = TaskHandle()
        DAQmxCreateTask("",byref(self.radPrepInTask))
        DAQmxCreateDOChan(self.radPrepInTask,"cpiSyncDAQ/port0/line6", "", DAQmx_Val_ChanForAllLines)
        DAQmxStartTask(self.radPrepInTask)

        #Set up the DO task to send the Expose signal to the generator. 
        self.exposeInTask = TaskHandle()
        DAQmxCreateTask("",byref(self.exposeInTask))
        DAQmxCreateDOChan(self.exposeInTask,"cpiSyncDAQ/port0/line7", "", DAQmx_Val_ChanForAllLines)
        DAQmxStartTask(self.exposeInTask)

        #set up change detection
        self._DIChangeCallback = DAQmxSignalEventCallbackPtr(self.DIChangeCallback)
        #self.changeDetectTask = TaskHandle()
        #DAQmxCreateTask("",byref(self.changeDetectTask))

        #task to read DI line for rad prep port1/line0, use radPrepOutTask as the task
        self.radPrepOutTask = TaskHandle()
        DAQmxCreateTask("",byref(self.radPrepOutTask))
        DAQmxCreateDIChan(self.radPrepOutTask,"cpiSyncDAQ/port1/line0", "", DAQmx_Val_ChanForAllLines)
        #DAQmxCreateDIChan(self.changeDetectTask,"cpiSyncDAQ/port1/line0:3", "", DAQmx_Val_ChanForAllLines)
        DAQmxCfgChangeDetectionTiming(self.radPrepOutTask,"cpiSyncDAQ/port1/line0:2","cpiSyncDAQ/port1/line0:2",DAQmx_Val_ContSamps,8)
        DAQmxRegisterSignalEvent(self.radPrepOutTask,DAQmx_Val_ChangeDetectionEvent,0,self._DIChangeCallback,None)
        DAQmxStartTask(self.radPrepOutTask)

        #task to read DI line for rad ready port1/line1
        self.radReadyOutTask = TaskHandle()
        DAQmxCreateTask("",byref(self.radReadyOutTask))
        DAQmxCreateDIChan(self.radReadyOutTask,"cpiSyncDAQ/port1/line1", "", DAQmx_Val_ChanForAllLines)

        #task to read DI line for expose port1/line2
        self.exposeOutTask = TaskHandle()
        DAQmxCreateTask("",byref(self.exposeOutTask))
        DAQmxCreateDIChan(self.exposeOutTask,"cpiSyncDAQ/port1/line2", "", DAQmx_Val_ChanForAllLines)

    def DIChangeCallback(self, Task, status, callbackData): #this callback currently takes ~6ms to process
        DAQmxReadDigitalLines(self.exposeOutTask, 1, 1, 0,  self.x, 1, None, None, None)
        if self.x == 1:
            self.ExposeSound = True
            self.setParam("ExposeOut", 1)
        elif self.x == 0:
            self.setParam("ExposeOut", 0)
            self.ExposeSound = False
            if self.getParam("Expose")==1:
                time.sleep(.4)
                self.setParam("ExposeOut", 0)
                self.write("Expose", 0)
                caput("cpi:GeneratorStatus.PROC",1)
        DAQmxReadDigitalLines(self.radPrepOutTask, 1, 1, 0,  self.x, 1, None, None, None)
        if self.x == 1:
            self.setParam("RadPrepOut", 1)
            self.PrepSound = True
        elif self.x == 0:
            self.setParam("RadPrepOut", 0)
            self.PrepSound = False
        DAQmxReadDigitalLines(self.radReadyOutTask, 1, 1, 0,  self.x, 1, None, None, None)
        if self.x == 1:
            self.setParam("RadReadyOut", 1)
        elif self.x == 0:
            self.setParam("RadReadyOut", 0)
        self.updatePVs()
        return 0 # The function should return an integer

    def RadReadyOutCallback(self, Task, status, callbackData):
        print "CALLBACK"
        DAQmxReadDigitalLines(Task, 1, 1, 0,  self.x, 1, None, None, None)
        if self.x == 1:
            print "RAD Ready ON"
        elif self.x == 0:
            print "RAD Ready OFF"
        return 0 # The function should return an integer
    
    def warningsound(self):
        while(True):
            if self.ExposeSound == True:
                winsound.Beep(1500,500)            
            elif self.PrepSound == True:
                winsound.Beep(1318,500)
                winsound.Beep(1108,750)
            
    def write(self, reason, value):
        #take proper actions
        if reason == 'RadPrep' and value == 1:
            self.tid = threading.Thread(target = self.RadPrepOn, args = ())
            self.tid.start()
        elif reason == 'RadPrep' and value == 0:
            self.bid = threading.Thread(target = self.RadPrepOff, args = ())
            self.bid.start()
        elif reason == 'Expose' and value == 1:
            #if self.getParam("RadPrep")==1: # for now only allow setting Expose 1, if Rad Prep == 1 
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
        self.NikonExposureInProgress=1
        ScanStatus=caget(SCANPROGRESSIOC +'running') #grab scan info once, we don't want to be waiting for excess cagets
        if D800_DOC.get() == 1:
            print str(datetime.now()), 'update doc string request'
            UPDATEDOCSTRING_PV.put(1) #tell nikepics to update doc string for logging purposes
            while self.getParam('DocFlag') ==1:
                print str(datetime.now()), 'wait for doc string'
                time.sleep(.01)
        if ScanStatus == 1: #scan is in progress
            Nfinished=caget(SCANPROGRESSIOC+'Nfinished')
            NfinishedIOC= caget(SCANIOC1+ '.CPT')
            Ntotal=caget(SCANPROGRESSIOC+'Ntotal')
            print "Scanstatus", ScanStatus, "Nfinished", Nfinished, "NfinishedIOC", NfinishedIOC, "Ntotal", Ntotal
            if self.getParam("RadReadyOut")==0: #the scan just started if this is true
                print str(datetime.now()), '!!!!! SCAN STARTING !!!!!'
                self.cancel = 0 #reset cancel flag
                self.setParam('NikonBuffer',0) # reset buffer count
                self.write("RadPrep", 1) #sets rad prep in to 1
                print str(datetime.now()), 'wait for scan start rad ready'
                while self.getParam("RadReadyOut") == 0: #wait for radready out from cpi to be 1
                    time.sleep(.01)
                self.imageRateTimer=0 #reset image rate timer
            while self.getParam('NikonBuffer') > 13: #buffer approaching max capacity, need to let it unload
                while self.getParam('NikonBuffer') > 11: #wait for buffer to empty 2 images                   
                    print str(datetime.now()),'waiting for buffer to empty'
                    time.sleep(.01)
            #if Nfinished !=0: #not the first image, may need to wait
            #    print 'we need to wait for ', .225 - (time.clock()-self.ExposeTimer), ' due to current daq limitations'
            while time.clock()-self.ExposeTimer <.225: #make sure we're not following the previous exposure daq trigger too closely
                    #print str(datetime.now()),'waiting for mandatory .2sec delay betwen writes'
                    time.sleep(.01)
            if caget('IOC:scan1.EXSC') == 0: #this means the scan was aborted by user
                print str(datetime.now()),'SCAN ABORTED before shot'
                self.cancel = 1
        elif ScanStatus == 0: #not running a scan
            self.write("RadPrep", 1) #sets rad prep in to 1
            print str(datetime.now()), 'wait single shot for rad ready'
            while self.getParam("RadReadyOut") == 0:
                time.sleep(.001)
        if self.cancel == 0: #go ahead with exposure
            print str(datetime.now()), 'starting exposure outputs'
            self.TriggerNikon() # send trigger release signal to nikon
            time.sleep(ShutterDelay) #Nikon shutter reportedly opens 43ms after triggering the shutter 
            self.ExposeOn() # send expose on signal to CPI
            start=time.clock() #there is a limit to how quickly we can switch a digital out on the DAQ on and off, (5 operations/sec) so wait at least .2 seconds
            while self.getParam("ExposeOut") == 1 or (time.clock()-start)<.225  and self.cancel == 0:
                if ScanStatus !=0  and caget('IOC:scan1.EXSC') == 0: # scan canceled in middle of an exposure, shut expose in off
                    print 'Scan canceled in middle of exposure'
                    if  (time.clock()-start)<.2:
                        print 'error: must wait .2 seconds between daq operations'
                    else:
                        print 'SCAN ABORTED during shot'
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
        self.ExposeTimer=time.clock()   #reset timer that makes sure we wait at least .225 seconds before next exposure due to DAQ limitations
        if ScanStatus == 0: #scan not in progress its a single exposure
            self.write("RadPrep", 0) #turn off rad prep
        elif self.cancel == 0:
            caput(SCANIOC1 + '.WAIT', 0) #tell scan we're done acquiring the image and it can move the positionermove on
            if Nfinished==(Ntotal-1): #just took last image, so turn off radprep
                print str(datetime.now()), 'Took final scan image, turn rad prep off'
                self.write("RadPrep",0)
        elif self.cancel == 1:
            self.write("RadPrep",0)
        self.setParam('ImageRate', 1/(time.clock()-self.imageRateTimer))
        self.imageRateTimer=time.clock() #reset imagerate clock
        #self.cancel = 0 #reset cancel status
        self.NikonExposureInProgress=0
        self.setParam('DocFlag', 1) #reset doc flag 
        self.updatePVs()
        print str(datetime.now()), 'single NikonExpose sequence over'


    def ScanMonitor(self, **kw):
        print str(datetime.now()), 'SCAN canceled or finished'
        if self.getParam('RadPrep')==1 and SCAN_CANCEL_IOC.get() == 0: #and self.NikonExposureInProgress == 0:
            self.write('RadPrep',0)


    def On(self):
        x=DAQmxWriteDigitalLines(self.powerOnTask,1,1,10.0,DAQmx_Val_GroupByChannel,self.one,self.written, None)
        self.processDAQstatus(x)
        time.sleep(.5) #simulating push button
        x=DAQmxWriteDigitalLines(self.powerOnTask,1,1,10.0,DAQmx_Val_GroupByChannel,self.zero,self.written, None)
        self.processDAQstatus(x)

    def Off(self):
        x=DAQmxWriteDigitalLines(self.powerOffTask,1,1,10.0,DAQmx_Val_GroupByChannel,self.one,self.written, None)
        self.processDAQstatus(x)
        time.sleep(.5) #simulating push button
        x=DAQmxWriteDigitalLines(self.powerOffTask,1,1,10.0,DAQmx_Val_GroupByChannel,self.zero,self.written, None)
        self.processDAQstatus(x)

    def RadPrepOn(self):
        caput("cpi:GeneratorStatus.SCAN", "Passive") #have trouble with the unsolicited commands unless we stop polling
        time.sleep(.5)
        x = DAQmxWriteDigitalLines(self.radPrepInTask,1,1,10.0,DAQmx_Val_GroupByChannel,self.one,self.written, None)
        self.processDAQstatus(x)

    def RadPrepOff(self):
        x=DAQmxWriteDigitalLines(self.radPrepInTask,1,1,10.0,DAQmx_Val_GroupByChannel,self.zero,self.written, None)
        self.processDAQstatus(x)
        time.sleep(.5)
        caput("cpi:GeneratorStatus.SCAN", "2 second")#now that exposure is over resume polling

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
            print "NI-DAQ error! Code:", errorcode


if __name__ == '__main__':
    server = SimpleServer()
    server.createPV(prefix, pvdb)
    driver = myDriver()
    # process CA transactions
    while True:
        server.process(0.1)
