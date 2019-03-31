#!/usr/bin/env python

# Imports
from brping import Ping1D # Imports Blue Robotics's Python library for interfacing with the Ping 
import rospy # Imports the Python version of ROS 
from ping_driver.msg import pingMessage # Imports the custom message that we use 

# Dynamic Reconfigure Imports
from dynamic_reconfigure.server import Server # Required import for dynamic reconfigure's server to run correctly 
from ping_driver.cfg import PingDriverConfig # Importing the config file 

# Establishing 
currentCfg = PingDriverConfig()

#TODO: Load default values from a config file or something
currentCfg.ping_enabled = False
currentCfg.ping_frequency = 10
currentCfg.speed_of_sound = 1498
currentCfg.auto = True
currentCfg.scan_start = 0
currentCfg.scan_length = 10
currentCfg.gain = 0

# Callback for Dynamic Reconfigure
# Called whenever a value is changed in the dynamic reconfigure GUI - Basically just updates all of the Ping's inherent properties to whatever they are through Dynamic Reconfigure 
#TODO: Might need to validate requested parameter changes
def reconfigure_cb(config, level):   

    # Setting Ping Enabled State 
    if (currentCfg.ping_enabled != config.ping_enabled):
        if not myPing.set_ping_enable(config.ping_enabled):            
            print("Failed to set Ping Enable. Exiting...")
            exit(-1)
        currentCfg.ping_enabled = config.ping_enabled

    # Setting Ping polling frequency
    # Note - The config takes a passed-in frequency, but ping_interval is a TIME between iterations. We convert them inline as this part of the function.
    # Note - If this is set to a really low value, the Ping will just poll as fast as it can computationally support, which in our case was 14Hz or so
    if (currentCfg.ping_frequency != config.ping_frequency):
        interval = 1000.0 / config.ping_frequency # Convert to ms interval
        if not myPing.set_ping_interval(interval):
            print("Failed to set Ping Interval. Exiting...")
            exit(-1)
        currentCfg.ping_frequency = config.ping_frequency

    # Setting the speed of sound that the ping is currently using 
    if (currentCfg.speed_of_sound != config.speed_of_sound):
        sos = config.speed_of_sound * 1000 # Convert to mm/s
        if not myPing.set_speed_of_sound(sos):
            print("Failed to set Speed of Sound. Exiting...")
            exit(-1)
        currentCfg.speed_of_sound = config.speed_of_sound

    # Setting which gain setting is being used (manual or auto) 
    if (currentCfg.auto != config.auto):
        if not myPing.set_mode_auto(1 if config.auto else 0):
            print("Failed to set Mode. Exiting...")
            exit(-1)
        currentCfg.auto = config.auto

    # Can only change Scan Range and Gain if not in auto mode
    if not currentCfg.auto:

        # Setting scan start range (where the Ping will start detecting objects - Is zero by default iirc)
        if (currentCfg.scan_start != config.scan_start or currentCfg.scan_length != config.scan_length):
            if not myPing.set_range(config.scan_start, config.scan_length):
                print("Failed to set Scan Range. Exiting...")
                exit(-1)
            currentCfg.ping_enabled = config.ping_enabled

        # Setting the gain used - Affects the accuracy and processing method of the ping, but we don't fully understand it as of yet 
        if (currentCfg.gain != config.gain):
            if not myPing.set_gain_index(config.gain):
                print("Failed to set Gain Index. Exiting...")
                exit(-1)
            currentCfg.gain = config.gain
        
    # Everything's been processed from dynamic reconfig, so we're good to return this 
    return currentCfg

# Initializing the Ping and checking that it correctly initialized 
myPing = Ping1D("/dev/ttyUSB0", 115200)
if myPing.initialize() is False:
    rospy.logwarn("Failed to initialize Ping! This probably means that it couldn't find the correct serial port or something similar. Fatal error.")

# Setting the Ping's speed of sound and checking that it was correctly set 
SPEED_IN_WATER = 1498000 # mm/s (What's required by the ping) 
SPEED_IN_AIR = 346000

# Setting Ping's speed of sound 
if not myPing.set_speed_of_sound(SPEED_IN_AIR):
    # TODO - Decide if logwarn is appropriate, or should errors be handled differently?
    rospy.logwarn("Was not able to set the ping's speed of sound.")

# Setting Ping's sampling interval 
# Note - If you set this to a really low value, the Ping will just sample as fast as its processor can handle
if not myPing.set_ping_interval(1):
    rospy.logwarn("Was not able to set the ping's sampling interval.")

# Setting Ping to Manual (0) or Auto (1) Sampling Mode
if not myPing.set_mode_auto(0):
    rospy.logwarn("Was not able to set the Ping's sampling mode.")

# TODO: Make both of these only run through if the sampling mode is manual; Otherwise, this will probably consistently error in that case 
# Setting Ping's Sampling Range
if not myPing.set_range(0, 10000):
    rospy.logwarn("Was not able to set the Ping's range.")

# Setting Ping's Gain Index
if not myPing.set_gain_index(5):
    rospy.logwarn("Was not able to set the Ping's gain index.")

# Setting up the node and the publisher for the Ping's data with ROS
rospy.init_node('ping_viewer')
pub = rospy.Publisher('/ping/raw', pingMessage, queue_size=10)

# ROS Initialization Debug Information
rospy.loginfo("Device ID: " + str(myPing.get_device_id()['device_id']))
rospy.loginfo("Major Firmware Version: " + str(myPing.get_general_info()['firmware_version_major']))
rospy.loginfo("Minor Firmware Version: " + str(myPing.get_general_info()['firmware_version_minor']))
rospy.loginfo("Device Supply VOltage (in mV): " + str(myPing.get_general_info()['voltage_5']))
rospy.loginfo("Ping Interval: " + str(myPing.get_general_info()['ping_interval']))
rospy.loginfo("Gain Index: " + str(myPing.get_general_info()['gain_index']))
rospy.loginfo("Operating Mode (0 = Manual, 1 = Auto): " + str(myPing.get_general_info()['mode_auto']))

# Ping is connected and readable, so we start the dynamic reconfig server 
srv = Server(PingDriverConfig, reconfigure_cb)

# Initializes an instance of the message so its values can be populated
ping_msg = pingMessage()

# Loops forever, publishing the Ping's data 
# ? If we're trying to make this an official package, should we put the other attributes other than just distance and confidence in too? Won't be hard, just have to use the other method and add a few more keys
while not rospy.is_shutdown():

    # Getting data using the library (get_distance_simple() returns a dict, which is basically python's equivalent of a JS object)
    distanceData = Ping1D.get_distance_simple(myPing)

    if distanceData is not None:

        # Raw Data is in millimeters, our message is in meters
        ping_msg.distance = distanceData['distance'] / 1000.0 
        ping_msg.confidence = distanceData['confidence']

        pub.publish(ping_msg)

    else:
        print("get_distance_simple() returned an invalid value. Skipping iteration.")

