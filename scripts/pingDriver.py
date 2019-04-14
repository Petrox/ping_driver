#!/usr/bin/env python

# Adding certain modules to PYTHONPATH so that they can be correctly imported directly afterward
import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + "/packages")
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + "/cfg")
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + "/msg")

# Imports
from brping import Ping1D # Imports Blue Robotics's Python library for interfacing with the Ping 
import rospy # Python version of ROS 
from ping_driver.msg import pingMessage # Imports the custom message that we use

# Dynamic Reconfig Imports
from dynamic_reconfigure.server import Server # Required import for dynamic reconfigure's server to run correctly 
from ping_driver.cfg import PingDriverConfig

# Simulated Data Imports
import threading # Allows us to generate and listen for fake data natively in this class on different threads
import random # Allows us to generate values for fake data 
import time # Allows us to dictate interval between fake data publishing 
import pty # Allows us to set up a terminal that serves as a fake Ping, basically
from serial import Serial # Allows us to simulate fake data on a specific serial port

# I'd normally do this using currentCfg = PingDriverConfig() but python's yelling at me that it can't use a module like that
# So, this implements basically the same thing - A cache of the current value so dynamic reconfigure can tell if it needs to change something 
currentCfg = {
    "ping_enabled": False,
    "ping_frequency": 10, 
    "speed_of_sound": 1498, 
    "auto": True,
    "gain": 0,
    "scan_start": 0,
    "scan_length": 10
}

# Tracks whether we're reading from a fake stream or from the real ping 
# Ideally, you can change only this value and the entire class's behavior will responsively change 
readingFromFakeStream = True

# Used to communicate across threads
# # This is NOT thread safe, but doesn't need to be 
cachedFakeDistance = 0
cachedFakeConfidence = 0

# Setting default config parameters 
currentCfg['auto'] = True
currentCfg['scan_start'] = 0
currentCfg['scan_length'] = 10
currentCfg['gain'] = 0

# Callback for Dynamic Reconfigure
# Called whenever a value is changed in the dynamic reconfigure GUI - Basically just updates all of the Ping's inherent properties to whatever they are through Dynamic Reconfigure 
# TODO: Might need to validate requested parameter changes
def reconfigure_cb(config, level):   

    # Setting Ping Enabled State 
    if (currentCfg['ping_enabled'] != config.ping_enabled):
        if not myPing.set_ping_enable(config.ping_enabled) and not readingFromFakeStream:            
            print("Failed to set Ping Enable. Exiting...")
            exit(-1)
        currentCfg['ping_enabled'] = config.ping_enabled

    # Setting Ping polling frequency
    # Note - The config takes a passed-in frequency, but ping_interval is a TIME between iterations. We convert them inline as this part of the function.
    # Note - If this is set to a really low value, the Ping will just poll as fast as it can computationally support, which in our case was 14Hz or so
    if (currentCfg['ping_frequency'] != config.ping_frequency):
        interval = 1000.0 / config.ping_frequency # Convert to ms interval
        if not myPing.set_ping_interval(interval) and not readingFromFakeStream:
            print("Failed to set Ping Interval. Exiting...")
            exit(-1)
        currentCfg['ping_frequency'] = config.ping_frequency

    # Setting the speed of sound that the ping is currently using 
    if (currentCfg['speed_of_sound'] != config.speed_of_sound) and not readingFromFakeStream:
        sos = config.speed_of_sound * 1000 # Convert to mm/s
        if not myPing.set_speed_of_sound(sos):
            print("Failed to set Speed of Sound. Exiting...")
            exit(-1)
        currentCfg['speed_of_sound'] = config.speed_of_sound

    # Setting which gain setting is being used (manual or auto) 
    if (currentCfg['auto'] != config.auto):
        if not myPing.set_mode_auto(1 if config.auto else 0) and not readingFromFakeStream:
            print("Failed to set Mode. Exiting...")
            exit(-1)
        currentCfg['auto'] = config.auto

    # Can only change Scan Range and Gain if not in auto mode
    if not currentCfg['auto']:

        # Setting scan start range (where the Ping will start detecting objects - Is zero by default iirc)
        if (currentCfg['scan_start'] != config.scan_start or currentCfg.scan_length != config.scan_length):
            if not myPing.set_range(config.scan_start, config.scan_length) and not readingFromFakeStream:
                print("Failed to set Scan Range. Exiting...")
                exit(-1)
            currentCfg['ping_enabled'] = config.ping_enabled

        # Setting the gain used - Affects the accuracy and processing method of the ping, but we don't fully understand it as of yet 
        if (currentCfg['gain'] != config.gain):
            if not myPing.set_gain_index(config.gain) and not readingFromFakeStream:
                print("Failed to set Gain Index. Exiting...")
                exit(-1)
            currentCfg['gain'] = config.gain
        
    # Everything's been processed from dynamic reconfig, so we're good to return this 
    # Todo - Make sure this doesn't cause errors due to me running off of a custom dictionary here instead of using a variable instantiated directly from the config file 
    return currentCfg

# Initializing Ping if we're not working with fake data 
myPing = None # Avoids scope issues
if not readingFromFakeStream:

    # This specific serial port is usually correct but could theoretically need changed
    myPing = Ping1D("/dev/ttyUSB0", 115200)
    if myPing.initialize() is False:
        rospy.logwarn("Failed to initialize Ping! This probably means that it couldn't find the correct serial port or something similar. Fatal error.")

# Setting up the node and the publisher for the Ping's data with ROS
rospy.init_node('ping_viewer')
pub = rospy.Publisher('/ping/raw', pingMessage, queue_size=10)

# Ping is connected and readable, so we start the dynamic reconfig server 
srv = Server(PingDriverConfig, reconfigure_cb)

'''

    FUNCTION DEFINITIONS

'''
currentCfg = {
    "ping_enabled": False,
    "ping_frequency": 10, 
    "speed_of_sound": 1498, 
    "auto": True,
    "gain": 0,
    "scan_start": 0,
    "scan_length": 10
}

def initializePingDefaultValues():

    if not myPing.set_speed_of_sound(currentCfg['speed_of_sound']):
        rospy.logwarn("Was not able to set the ping's speed of sound.")

    if not myPing.set_ping_interval(currentCfg['ping_interval']):
        rospy.logwarn("Was not able to set the ping's sampling interval.")

    if not myPing.set_mode_auto(True):
        rospy.logwarn("Was not able to set the Ping's sampling mode.")

    if not myPing.set_range( currentCfg['scan_start'] * 1000, currentCfg[scan_length] * 1000):
        rospy.logwarn("Was not able to set the Ping's range.")

    if not myPing.set_gain_index(currentCfg['gain']):
        rospy.logwarn("Was not able to set the Ping's gain index.")

def outputStartupPingValues():

    # ROS Initialization Debug Information
    rospy.loginfo("Device ID: " + str(myPing.get_device_id()['device_id']))
    rospy.loginfo("Major Firmware Version: " + str(myPing.get_general_info()['firmware_version_major']))
    rospy.loginfo("Minor Firmware Version: " + str(myPing.get_general_info()['firmware_version_minor']))
    rospy.loginfo("Device Supply Voltage (in mV): " + str(myPing.get_general_info()['voltage_5']))
    rospy.loginfo("Ping Interval: " + str(myPing.get_general_info()['ping_interval']))
    rospy.loginfo("Gain Index: " + str(myPing.get_general_info()['gain_index']))
    rospy.loginfo("Operating Mode (0 = Manual, 1 = Auto): " + str(myPing.get_general_info()['mode_auto']))

def setupFakeData():

    # Instance of the message so its values can be continually changed then published
    ping_msg = pingMessage()
    
    # Makes references to these two variables in this function point towards the global variables
    global cachedFakeDistance
    global cachedFakeConfidence

    # Todo - Make both the server and listener only happen if the bool is set to the correct value (I would do this, but I'm having issues getting the variable scope right)
    # Sets up a server that's basically publishing fake data just incase we need it
    if readingFromFakeStream:

        master, slave = pty.openpty()
        s_name = os.ttyname(slave)
        ser = Serial(s_name, 2400, timeout=1)
        publisherThread = threading.Thread(target=publishFakeData, args=[ser])
        publisherThread.start()

        # Sets up another thread that's always listening to it just incase we need the values 
        listenerThread = threading.Thread(target=readFakeData, args=[master])
        listenerThread.start()

    distanceData = None
    while not rospy.is_shutdown():

        # If we're reading from the ping, get data from the actual library function 
        if not readingFromFakeStream:

            distanceData = Ping1D.get_distance_simple(myPing)
        
        # Otherwise, set up distanceData's values as the last cached fake values from the stream
        else: 

            distanceData = {
                "distance": None, 
                "confidence": None
            }

            distanceData['distance'] = cachedFakeDistance / 1000
            distanceData['confidence'] = cachedFakeConfidence 

        # If distance data this time around is valid 
        if distanceData is not None:

            print("distanceData: " + str(distanceData))

            # Raw Data is in millimeters, data published to our topic should be in meters
            ping_msg.distance = distanceData['distance']
            ping_msg.confidence = distanceData['confidence']

            pub.publish(ping_msg)

        # Otherwise, distance data somehow screwed up this iteration around 
        else:

            print("get_distance_simple() returned an invalid value. Skipping iteration.")

        rospy.sleep(.1)

def publishFakeData(serialPort):

    # Makes references to these two variables point towards the global variables
    global cachedFakeDistance
    global cachedFakeConfidence

    while True:

        # Refreshing distance ranged every cycle incase reconfig changed them 
        MIN_DISTANCE = currentCfg['scan_start'] * 1000
        MAX_DISTANCE = currentCfg['scan_length'] * 1000

        # Calculating values
        distance = MIN_DISTANCE + random.random() * (MAX_DISTANCE - MIN_DISTANCE)
        confidence = random.random()

        # Writing values to serial port 
        serialPort.write(str(distance) + "\r\n")
        serialPort.write(str(confidence) + "\r\n")

        time.sleep(.1)

# Updates a global variable cache with the most up-to-date fake data value for distance and confidence
def readFakeData(port):

    # Makes references to these two variables point towards the global variables
    global cachedFakeDistance
    global cachedFakeConfidence

    # Used to alternate which cached value is being read with this line
    currentlyReadingDistance = True

    while True:

        res=b""

        while not res.endswith(b"\r\n"):

            res += os.read(port, 1)

        if currentlyReadingDistance:

            cachedFakeDistance = float(res)
            currentlyReadingDistance = False

        else:

            cachedFakeConfidence = float(res)
            currentlyReadingDistance = True

# Runs what's necessary contingent upon if we're reading fake data or not 
if __name__ == "__main__":

    if readingFromFakeStream:        

        setupFakeData()        

    else:

        initializePingDefaultValues()
        outputStartupPingValues()

        

        




