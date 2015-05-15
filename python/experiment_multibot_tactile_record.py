#!/usr/bin/env python
"""
authors: jgoldberg and apullin

This script will run an experiment with one or several Velociroach robots.

The main function will send all the setup parameters to the robots, execute defined manoeuvres, and record telemetry.

"""
from lib import command
import time,sys,os,traceback
import serial
import thread

# Path to imageproc-settings repo must be added
sys.path.append(os.path.dirname("../../imageproc-settings/"))
sys.path.append(os.path.dirname("../imageproc-settings/"))  
import shared_multi as shared

from velociroach import *
import skinvisualizer4

####### Wait at exit? #######
EXIT_WAIT   = False

def main():    
    xb = setupSerial(shared.BS_COMPORT, shared.BS_BAUDRATE)
    
    R1 = Velociroach('\x30\x02', xb)
    R1.SAVE_DATA = False
                            
    #R1.RESET = False       #current roach code does not support software reset
    
    shared.ROBOTS.append(R1) #This is necessary so callbackfunc can reference robots
    shared.xb = xb           #This is necessary so callbackfunc can halt before exit

    # Send resets
    for r in shared.ROBOTS:
        if r.RESET:
            r.reset()
            time.sleep(0.35)
    # Repeat this for other robots
    # TODO: move reset / telem flags inside robot class? (pullin)
    
    # Send robot a WHO_AM_I command, verify communications
    for r in shared.ROBOTS:
        r.query(retries = 3)
    
    #Verify all robots can be queried
    verifyAllQueried()  # exits on failure
    
    # Motor gains format:
    #  [ Kp , Ki , Kd , Kaw , Kff     ,  Kp , Ki , Kd , Kaw , Kff ]
    #    ----------LEFT----------        ---------_RIGHT----------
    motorgains = [1800,100,200,0,200, 1800,100,200,0,200]
    #motorgains = [0,0,0,0,0 , 0,0,0,0,0]

    simpleAltTripod = GaitConfig(motorgains, rightFreq=2, leftFreq=2) # Parameters can be passed into object upon construction, as done here.
    simpleAltTripod.phase = PHASE_180_DEG                             # Or set individually, as here
    simpleAltTripod.deltasLeft = [0.25, 0.25, 0.25]
    simpleAltTripod.deltasRight = [0.25, 0.25, 0.25]
    #simpleAltTripod.deltasTime  = [0.25, 0.25, 0.25] # Not current supported by firmware; time deltas are always exactly [0.25, 0.25, 0.25, 0.25]
    
    # Configure intra-stride control
    R1.setGait(simpleAltTripod)

    # example , 0.1s lead in + 2s run + 0.1s lead out
    EXPERIMENT_RUN_TIME_MS     = 3000 #ms
    EXPERIMENT_LEADIN_TIME_MS  = 100  #ms
    EXPERIMENT_LEADOUT_TIME_MS = 100  #ms
    
    # Some preparation is needed to cleanly save telemetry data
    for r in shared.ROBOTS:
        if r.SAVE_DATA:
            #This needs to be done to prepare the .telemtryData variables in each robot object
            r.setupTelemetryDataTime(EXPERIMENT_LEADIN_TIME_MS + EXPERIMENT_RUN_TIME_MS + EXPERIMENT_LEADOUT_TIME_MS)
            r.eraseFlashMem()
        
    # Pause and wait to start run, including lead-in time
    print ""
    print "  ***************************"
    print "  *******    READY    *******"
    print "  ***************************"
    raw_input("  Press ENTER to start run ...")
    print ""

    # Send tactile commands here
    R1.getSkinSize()
    time.sleep(.5)
    #R1.testFrame()
    #time.sleep(.5)

    # Initiate telemetry recording; the robot will begin recording immediately when cmd is received.
    for r in shared.ROBOTS:
        if r.SAVE_DATA:
            r.startTelemetrySave()
            #time.sleep(0.1)
            #r.startScan()
    
    # Sleep for a lead-in time before any motion commands
    
    time.sleep(EXPERIMENT_LEADIN_TIME_MS / 1000.0)
    
    ######## Motion is initiated here! ########
    #R1.startTimedRun( EXPERIMENT_RUN_TIME_MS ) #Faked for now, since pullin doesn't have a working VR+AMS to test with
    #time.sleep(.001)
    R1.startScan()
    raw_input()
    '''
    R1.RECORDSHELL = True
    time.sleep(10)
    R1.RECORDSHELL = False
    R1.stopScan()
    time.sleep(.5)
    R1.stopScan()
    time.sleep(.5)
    R1.stopScan()
    time.sleep(.5)
    #raw_input("")
    #thread.start_new_thread(skinvisualizer4.main, ())
    raw_input("stopped")
    return
    R1.RECORDSHELL = True'''
    #time.sleep(EXPERIMENT_RUN_TIME_MS / 1000.0)  #argument to time.sleep is in SECONDS
    #R1.startRun()
    #raw_input("hit enter")
    #R1.stopRun()

    ######## End of motion commands   ########

    #raw_input("Start scan?")
    #R1.VERBOSE = False
    #R1.startScan()
    #time.sleep(.5)
    #thread.start_new_thread(skinvisualizer4.main, ())
    #while raw_input("record for 5 seconds") == "y":
    #R1.RECORDSHELL = True
    #time.sleep(EXPERIMENT_RUN_TIME_MS)
    #R1.RECORDSHELL = False
    #R1.stopRun()
    #time.sleep(.1)
    #R1.stopRun()
    #time.sleep(.1)
    #raw_input()
    #R1.stopScan()
    #time.sleep(.1)
    #R1.stopScan()
    #time.sleep(.1)
    #R1.stopScan()

    #time.sleep(10)
    #R1.RECORDSHELL = False

    # Sleep for a lead-out time after any motion
    time.sleep(EXPERIMENT_LEADOUT_TIME_MS / 1000.0) 
    #raw_input()
    R1.stopScan()
    time.sleep(0.1)
    R1.stopScan()
    time.sleep(0.1)
    R1.stopScan()
    time.sleep(0.1)
    #R1.RECORDSHELL = False
    
    for r in shared.ROBOTS:
        if r.SAVE_DATA:
            raw_input("Press Enter to start telemetry read-back ...")
            r.downloadTelemetry()
    
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
