#!/usr/bin/python
import RPIO
import time
import getch
import sys
import subprocess

rst=12 #Numeracja ogolna (pinow)

#RPIO.setmode(RPIO.BOARD)
#RPIO.setup(rst, RPIO.OUT)

bla=subprocess.call(["cp src/sketch.cpp src/sketch.ino"],shell=True)
output_build=subprocess.call(["ino build >log_build"],shell=True)
p = subprocess.Popen(['ino', 'upload'], stdout=subprocess.PIPE,stderr=subprocess.PIPE)

RPIO.setmode(RPIO.BOARD)
RPIO.setup(rst, RPIO.OUT)


RPIO.output(rst,True)
time.sleep(1)
RPIO.output(rst,False)
out, err = p.communicate()
#subprocess.call(["ino upload > log_upload"],shell=True)
# print output_build # Nie wiedziec czemu, nie wyswietla sie output z komend, wiec do pliku
#print out
print err
RPIO.cleanup()
