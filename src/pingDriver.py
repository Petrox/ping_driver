#!/usr/bin/env python

# Issues: 
# Have to run source devel/setup.bash at every terminal except for the roscore one for ROS to recognize the package 
# Had to chmod the file itself to modify its permissions - Kinda wonky, make sure it's OK 

# Imports ping stuff
from brping import Ping1D

# Imports other stuff
import rospy
from ping_driver.msg import pingMessage

# Setting up the ping and checking that it registered correctly
# Serial Port & Baud (data transfer rate / sampling rate)
myPing = Ping1D("/dev/ttyUSB0", 115200)

if myPing.initialize() is False:
    print("Failed to initialize Ping!")
    exit(1)

# Initializes the class with ROS and sets up a publisher
rospy.init_node('ping_viewer')
pub = rospy.Publisher('/ping/raw', pingMessage, queue_size=10)

# Sets the rate at which ROS publishes to the topic 
rate = rospy.Rate(10)

# pingMessage = {'distance': 0, 'confidence': 0}

# Loops until ROS is shut down
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


