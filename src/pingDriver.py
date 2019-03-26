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

# Initializes an instance of the message so its values can be populated
ping_msg = pingMessage()

print(myPing.get_speed_of_sound())

# Loops at a frequency specified by the rate until ROS is shut down, refreshing and publishing the data 
rate = rospy.Rate(50)
while not rospy.is_shutdown():

    # Getting data using the library (get_distance_simple() returns a dict, which is basically python's equivalent of a JS object)
    distanceData = Ping1D.get_distance_simple(myPing)

    ping_msg.distance = distanceData['distance']
    ping_msg.confidence = distanceData['confidence']

    pub.publish(ping_msg)

    # Sleep until the next measurement is taken 
    rate.sleep()


