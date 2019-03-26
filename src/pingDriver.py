#!/usr/bin/env python

# Imports
from brping import Ping1D # Imports Blue Robotics's Python library for interfacing with the Ping 
import rospy # Imports the Python version of ROS 
from ping_driver.msg import pingMessage # Imports the custom message that we use 

# Serial Port & Baud (data transfer rate / sampling rate)
myPing = Ping1D("/dev/ttyUSB0", 115200)

# Checking to make sure that it was set up correctly
if myPing.initialize() is False:
    print("Failed to initialize Ping! This probably means that it couldn't find the correct serial port or something similar.")
    exit(1)

# Initializes the class with ROS
rospy.init_node('ping_viewer')

# Sets up the publisher that will be used to publish the Ping's data 
pub = rospy.Publisher('/ping/raw', pingMessage, queue_size=10)

# Constants used in next step 
SPEED_IN_WATER = 1498
SPEED_IN_AIR = 346 

# Setting speed of sound and making sure it was correctly set
if not myPing.set_speed_of_sound(SPEED_IN_AIR):
    print("Was not able to set the ping's speed of sound.")
    print("Exiting program.")
    exit(1)

# Loops at a frequency specified by the rate until ROS is shut down
rate = rospy.Rate(10)
while not rospy.is_shutdown():

    # Getting data using the library (get_distance_simple() returns a dict, which is basically python's equivalent of a JS object)
    distanceData = Ping1D.get_distance_simple(myPing)
    print(distanceData)

    # Converting it into the format used by our message 
    # Python assumes that pingMessage is an object
    pingMessage['distance'] = distanceData['distance']
    pingMessage['confidence'] = distanceData['confidence']

    # Sleep until the next measurement is taken 
    rate.sleep()


