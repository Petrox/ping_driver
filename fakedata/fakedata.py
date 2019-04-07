import os, pty
from serial import Serial
import threading
import random # Used for rng 
import time # Used to sleep during publishing 

# Configuration Constants (In meters unless otherwise stated)
MIN_DISTANCE = 0
MAX_DISTANCE = 30

# Used by the thread that listens in on the terminal 
def listenToTerminal(port):

    # Each iteration reads in a single line 
    while True:

        # Explicitly casting input stream to bytes so future stuff is read as bytes 
        res = b""

        # Read in a single line from the terminal
        # Assumes each one ends w/ both a carriage return and a newline character
        while not res.endswith(b"\r\n"):

            # Reads in a single byte 
            res += os.read(port, 1)

        print("Line: " + str(res))

# See this link for a great writeup on Python Pseudoterminals: 
# http://rachid.koucha.free.fr/tech_corner/pty_pdip.html
# Used by the thread that physically produces the data
def createTerminal():

    # Star"t up the master, slave terminal pair 
    master, slave = pty.openpty()

    # Get filename of slave 
    s_name = os.ttyname(slave) 
    print("Port: " + str(s_name))

    # Open a serial connection to the slave 
    ser = Serial(s_name, 2400, timeout=1)

    # Making another thread so we can both publish and listen in on it at the same time (uncomment next two lines to listen in on the data)
    thread = threading.Thread(target=listenToTerminal, args=[master])
    thread.start()

    # Continuously publish data 
    while True:

        # Generates a float from MIN_DISTANCE to MAX_DISTANCE
        # Todo - Check this equation - From brief checking, it seems to work, but I basically guessed at it 
        distance = MIN_DISTANCE + random.random() * (MAX_DISTANCE - MIN_DISTANCE)

        # This is always from 0 to 1 
        # Todo - Make this gradually go up and down like the actual ping tends to do
        confidence = random.random()

        ser.write(str(distance) + "\r\n")
        ser.write(str(confidence) + "\r\n")

        # Publishes at approx. 10 Hz
        time.sleep(.1)

if __name__=='__main__':
    createTerminal()

