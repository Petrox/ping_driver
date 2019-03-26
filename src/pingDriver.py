#!/usr/bin/env python

# Imports
from brping import Ping1D # Imports Blue Robotics's Python library for interfacing with the Ping 
import rospy # Imports the Python version of ROS 
from ping_driver.msg import pingMessage # Imports the custom message that we use 

# Initializing our ping device and checking to make sure it initialized correctly
myPing = Ping1D("/dev/ttyUSB0", 115200)
if myPing.initialize() is False:

    print("Failed to initialize Ping! This probably means that it couldn't find the correct serial port or something similar.")
    exit(1)

# Initializes the class with ROS
rospy.init_node('ping_viewer')

# Sets up the publisher that will be used to publish the Ping's data 
pub = rospy.Publisher('/ping/raw', pingMessage, queue_size=10)

# Constants used in next step (mm/s)
SPEED_IN_WATER = 1498000
SPEED_IN_AIR = 346000

# Setting speed of sound and making sure it was correctly set
# The method returns false if it wasn't able to set the speed of sound, basically
if not myPing.set_speed_of_sound(SPEED_IN_AIR):

    print("Was not able to set the ping's speed of sound.")
    print("Exiting program.")
    exit(1)

myPing.set_ping_interval(1)
myPing.set_mode_auto(0)
myPing.set_range(0, 10000)
myPing.set_gain_index(5)

print(myPing.get_profile()['scan_length'])
print(myPing.get_profile()['scan_start'])
print(myPing.get_gain_index()['gain_index'])

# Debug Information
rospy.loginfo("Device ID: " + str(myPing.get_device_id()['device_id']))
rospy.loginfo("Major Firmware Version: " + str(myPing.get_general_info()['firmware_version_major']))
rospy.loginfo("Minor Firmware Version: " + str(myPing.get_general_info()['firmware_version_minor']))
rospy.loginfo("Device Supply VOltage (in mV): " + str(myPing.get_general_info()['voltage_5']))
rospy.loginfo("Ping Interval: " + str(myPing.get_general_info()['ping_interval']))
rospy.loginfo("Gain Index: " + str(myPing.get_general_info()['gain_index']))
rospy.loginfo("Operating Mode (0 = Manual, 1 = Auto): " + str(myPing.get_general_info()['mode_auto']))

# Initializes an instance of the message so its values can be populated
ping_msg = pingMessage()

# Continuous loop that keeps getting data from the ping and publishing it onto a topic 
while not rospy.is_shutdown():

    # Getting data using the library (get_distance_simple() returns a dict, which is basically python's equivalent of a JS object)
    distanceData = Ping1D.get_distance_simple(myPing)

    if distanceData is not None:

        # Converting to meters
        ping_msg.distance = distanceData['distance'] / 1000.0
        ping_msg.confidence = distanceData['confidence']

        pub.publish(ping_msg)

    else:

        print("get_distance_simple() returned an invalid value. Skipping iteration.")

