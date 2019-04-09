#!/usr/bin/env python

import sys, os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/packages")

for path in sys.path:
    print(path)

from brping import Ping1D
myPing = Ping1D("/dev/pts/2", 115200)