from brping import Ping1Dpy
myPing = Ping1D("/dev/ttyUSB0", 115200)

import rospy; 

def talker():
    pub = rospy.Publisher('/ping/raw'


