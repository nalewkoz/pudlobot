#!/usr/bin/python

import RPIO
import time

rst=12 #Numeracja ogolna (pinow)

#RPIO.setmode(RPIO.BOARD)
#RPIO.setup(rst, RPIO.OUT)

RPIO.setmode(RPIO.BOARD)
RPIO.setup(rst, RPIO.OUT)


RPIO.output(rst,True)
time.sleep(1)
RPIO.output(rst,False)

RPIO.cleanup()

