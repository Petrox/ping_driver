#!/usr/bin/env python

# Issues: 
# Have to run source devel/setup.bash at every terminal for ROS to recognize the package 
# Had to chmod the file itself to modify its permissions - Should we have? 
# How to physically import the message? Any weird ROS stuff? 

# Imports ping packages 
from brping import Ping1D

# Serial Port & Baud (data transfer rate / sampling rate)
myPing = Ping1D("/dev/ttyUSB0", 115200)

# Imports ros packages and necessary messages 
import rospy
from ping_driver.msg import pingMsg

# Initializes the class with ROS and sets up a publisher
rospy.init_node('ping_viewer')
pub = rospy.Publisher('/ping/raw', String, queue_size=10)

# Initializes this as a node within ros 
rate = rospy.Rate(10)

outputString = "hello, world!"

while not rospy.is_shutdown():

    # Publishes it onto its constructed topic via ros 
    pub.publish(outputString)

    # Sleeps according to the Hz given to it 
    rate.sleep()


