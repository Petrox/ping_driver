import os, pty
from serial import Serial
import threading
import random # Used for rng 
import time # Used to sleep during publishing 

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

    # Open a serial connection to the slave 
    ser = Serial(s_name, 2400, timeout=1)

    # Making another thread so we can both publish and listen in on it at the same time
    thread = threading.Thread(target=listener, args=[master])
    thread.start()

    # Continuously publish data 
    while True:

        ser.write("Random Number: " + str(random.randrange(5)) + "\r\n")

        # Waits half a second before outputting the next message 
        time.sleep(.5)

if __name__=='__main__':
    listenToTerminal()

