#!/usr/bin/env python
"""
PCAS driver to trigger the CPI-CMP200 generator to fire the x-ray tube.
Adapted from indico100Sync.py (previously cpisync.py). Requires sscan, scanProgress,
IOC's to be running.

__author__   =   Alireza Panna
__status__   =   Stable
__version__  =   1-0
__to-do__    =   
__date__     =   07/23/2016
"""
"""
CHANGELOG:
07/23/2016       (AP) 1st version created


Online reference for NI-DAQ
http://zone.ni.com/reference/en-XX/help/370471W-01/
Note "input" and "output" for variables are named from the perspective of the CPI generator, not the ni-daq. (Unless otherwise specified)
This was so the terminology in the documentation for the cpi matches with the names in the code.

"""
from pcaspy import Driver, SimpleServer, cas
import time, threading, os, os.path, pickle, socket, platform, sys
from PyDAQmx import *
from epics import *
import numpy as np
from datetime import datetime, timedelta
import psutil, gc
from multiprocessing import Process
sys.path.append(os.path.realpath('../utils'))
import epicsApps

EXPERIMENT          = 'CEL:'
DAQ_NAME            = 'cmp200Sync'
XRAY_IOC            =  EXPERIMENT + 'CPI:xray:'
SCAN_IOC            =  EXPERIMENT + 'SCAN:'
# These are the output lines from DAQ (input to CPI) 
CPI_LED_IN          = DAQ_NAME + '/port0/line0'             # turns the led in the box on/off
CPI_EXPOSE_OK_IN    = DAQ_NAME + '/port0/line1'             # this line will always be set high for exposure
CPI_EXPOSE_IN       = DAQ_NAME + '/port0/line2'             # setting high allows exposure
CPI_RAD_PREP_IN     = DAQ_NAME + '/port0/line6'             # setting high enables rad prep
# These are the DAQ inputs (output from CPI)
CPI_RAD_PREP_OUT    = DAQ_NAME  + '/port1/line0'
CPI_RAD_READY_OUT   = DAQ_NAME  + '/port1/line1'
CPI_EXPOSE_OUT      = DAQ_NAME  + '/port1/line2'
# These are the lines that will be polled for change detection
INPUT_POLL= CPI_RAD_PREP_OUT + "," + CPI_RAD_READY_OUT + "," + CPI_EXPOSE_OUT
SCAN_CANCEL_IOC     = PV(SCAN_IOC + 'AbortScans.PROC', callback = True)
# Constant numpy arrays for setting digital outputs high or low on NI-DAQs
LOW  = numpy.zeros((1,), dtype=numpy.uint8)
HIGH = numpy.ones((1,), dtype=numpy.uint8)
POLL_TIME = 0.001                                           # How often to pause in "while" loops
# pcas records
prefix = EXPERIMENT + 'cpiSync:'
pvdb = {
    'RAD_PREP'              : { 'asyn' : True },
    'EXPOSE'                : { 'asyn' : True },
    'RAD_PREP_RBV'          : { 'asyn' : True },
    'RAD_READY_RBV'         : { 'asyn' : True },
    'EXPOSE_RBV'            : { 'asyn' : True },
    'ABORT'                 : { 'asyn' : True },
    'LAST_EXPOSURE_RBV'     : { 'type': 'string',
                                'scan': 1},
    'STATUS_RBV'            : { 'type': 'char',
                                'count': 300,
                                'value': 'Initialization'},
    'GEN_DELAY'             : { 'value': 0.02, # 20 milliseconds
                                'prec': 3} ,             
}

pvdb.update(epicsApps.pvdb)
class myDriver(Driver):
    def  __init__(self):
        super(myDriver, self).__init__()
        self.start_time = datetime.now()
        self.iocStats()
        self.cancel = 0                                     # cancel flag for abort procedure 
        self.currentFunction = ''                           # initialize current function string used in status messages
        self.seqInProgress=0                                # flag to determine if we are inside a "sequence" or not
        # The following variable is needed for pydaqmx
        self.written = int32()
        SCAN_CANCEL_IOC.add_callback(callback=self.ScanMonitor)
        # Setup DO lines
        self.radPrepInTask = TaskHandle()
        self.setupOutput(self.radPrepInTask, CPI_RAD_PREP_IN, True)
        self.exposeInTask = TaskHandle()
        self.setupOutput(self.exposeInTask, CPI_EXPOSE_IN, True)   
        self.exposeOkInTask = TaskHandle()
        self.setupOutput(self.exposeOkInTask, CPI_EXPOSE_OK_IN, True)
        # Task to read DI lines combined into one (significantly improves performance to do one read)
        self.combinedTask = TaskHandle()
        DAQmxCreateTask("",byref(self.combinedTask))
        DAQmxCreateDIChan(self.combinedTask, INPUT_POLL, "", DAQmx_Val_ChanForAllLines)
        DAQmxStartTask(self.combinedTask)      
        # start the polling thread
        self.cid = threading.Thread(target=self.pollInputs,args=())
        self.cid.start()
        # set this line on init
        self.ExposeOkOn()
        epicsApps.buildRequestFiles(prefix, pvdb.keys(), os.getcwd())
        epicsApps.makeAutosaveFiles()
        # Now ready to do exposures, change from Initilization to Idle 
        self.write('STATUS_RBV', 'Idle')
        print '############################################################################'
        print '## CMP-200 SYNC PCAS IOC Online $Date:' + str(datetime.now())[:-3]
        print '############################################################################'

    def iocStats(self):
        """
        Sets the iocAdmin related records
        """
        self.start_time = datetime.now()
        self.setParam('ENGINEER', 'Andrew Gomella')
        self.setParam('LOCATION', 'B1D521D SVR-SMWIN121')
        self.setParam('RECORD_CNT', len(pvdb.keys()))
        self.setParam('APP_DIR1', str(os.getcwd()))
        self.setParam('UPTIME', str(self.start_time))
        self.setParam('PARENT_ID', os.getpid())
        self.setParam('HEARTBEAT', 0)

    def setupOutput(self, taskName, lineLocation, setLow):
        """
         Generic function for setting up daq output
        """
        DAQmxCreateTask("",byref(taskName))
        DAQmxCreateDOChan(taskName, lineLocation, "", DAQmx_Val_ChanForAllLines)
        DAQmxSetDOOutputDriveType(taskName, lineLocation, DAQmx_Val_ActiveDrive)
        DAQmxStartTask(taskName)   
        if setLow == True:
            self.setDigiOut(taskName, LOW)

    def pollInputs(self):
        """
        For the usb-6501 we have to perform change detection via polling the input lines
        """
        oldval = np.array([0,0,0], dtype=np.uint8)
        newval = np.array([0,0,0], dtype=np.uint8)
        DAQmxReadDigitalLines(self.combinedTask, 1, 1, 0,  oldval, 3, None, None, None)
        while True:
            startPollTime = time.clock()
            DAQmxReadDigitalLines(self.combinedTask, 1, 1, 0, newval, 3, None, None, None)     
            self.setParam("RAD_PREP_RBV",      newval[0])
            self.setParam("RAD_READY_RBV",     newval[1])
            self.setParam("EXPOSE_RBV",        newval[2])
            self.updatePVs()
            if newval[2] != oldval[2]:
                if newval[2] == 1:
                    print str(datetime.now())[:-3], 'GENERATOR EXPOSE START'
                    self.cpiExposeStartTime = time.clock()
                else:
                    self.write("EXPOSE", 0)
                    print str(datetime.now())[:-3], 'GENERATOR EXPOSE END',
                    self.cpiExposeEndTime=time.clock()
                    print str(datetime.now())[:-3], 'TOTAL EXPOSURE TIME', '%2.6f'%(self.cpiExposeEndTime - self.cpiExposeStartTime)
                    if newval[1] == 1:
                        self.lastCpiExposure = time.time()
            self.updatePVs()
            # Note: the below line is for storing the old value of array (before new read)
            # it must use the copy method, otherwise it will access the location of the array
            oldval = newval.copy()
            endPollTime = time.clock()
#            if endPollTime - startPollTime > 0.002:
#                print str(datetime.now())[:-3], 'DAQ Poll > 2 ms !', endPollTime - startPollTime

    def read(self, reason):
        """
        pcaspy native read method
        """
        format_time = ""
        if reason == 'UPTIME':
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
        """
        pcaspy native write method
        """
        self.setParam(reason, value)
        self.updatePVs()
        if  reason == 'RAD_PREP':
            self.did = threading.Thread(target = self.setDigiOut, args = (self.radPrepInTask, value))
            self.did.start()
        elif reason == 'EXPOSE':
            if (self.getParam("RAD_PREP_RBV") == 1 and value == 1) or value == 0:
                self.did = threading.Thread(target = self.setDigiOut, args = (self.exposeInTask, value))
                self.did.start()
        elif reason == 'ABORT' and value == 1:
            self.fid = threading.Thread(target = self.abort, args = ())
            self.fid.start()
        elif reason == 'STATUS_RBV':
            print str(datetime.now())[:-3], value
        self.updatePVs()

    def abort(self):
        """
        Call when user hits abort button (through write function) or hits abort on scan (through callback function)
        -set self.cancel = 1 - this lets any sequences in progress know to cancel 
        -Turn expose off
        -Turn radprep off
        -cancel scan if in progress 
        -Wait for all sequences to finish (self.seqInProgress=0)
        """
        self.write('STATUS_RBV', self.currentFunction + ' ABORTING!')
        self.cancel=1 # flag to tell exposures to cancel
        self.write('EXPOSE', 0)
        self.write('RAD_PREP',0)
        # wait for any sequence in progress to finish
        while self.seqInProgress == 1:
            time.sleep(POLL_TIME)
        self.cancel=0
        self.write('STATUS_RBV', self.currentFunction + ' Abort complete')
        
    def ScanMonitor(self, **kw):
        """
        Need to monitor scan abort PV so we can properly disable exposure/RadPrep
        """
        print str(datetime.now())[:-3], 'Scan Abort Callback'
        # This if statement is true if the scan was canceled, and Radprep is true
        if self.getParam('RAD_PREP')== 1 and self.cancel == 0 : 
            self.aid = threading.Thread(target = self.abort, args = ())
            self.aid.start()
        
    def ExposeOkOn(self):
        self.setDigiOut(self.exposeOkInTask, HIGH)
    
    def ExposeOkOff(self):
        self.setDigiOut(self.exposeOkInTask, LOW)
    
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
