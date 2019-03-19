from brping import Ping1Dpy
import serial
import rospy
from std_msgs.msg import String, Header

#make new msg?

myPing = Ping1D("/dev/ttyUSB0", 115200)

#corepro

import rospy; 

def talker():
    rospy.init_node('ping_viewer')
    dataRead = True
    #publisher
    pub = rospy.Publisher('/ping/raw'

    if __name__ == "__talker__": talker()


