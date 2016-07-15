# Python program to run on PyBoard https://micropython.org/
# Tests ScaraOne class which controls the robot

import time
import math

# Handle test mode using a stub of hardware library
TEST_MODE = False
if TEST_MODE:
    import HardwareLibrary
else:
    import pyb as HardwareLibrary

# ScaraOne is the library supporting the ScaraOne Single Arm Scara robot
from ScaraOne import ScaraOne
from PyBoardDisplay import PyBoardDisplay

robotConfig = { "shoulderGearMismatchFactor": -1/30 }

# Create the robot
scaraOne = ScaraOne(HardwareLibrary, robotConfig)

scaraOne.pulseWidthUsecs = 20
scaraOne.betweenPulsesUsecs = 600

# Create display
pyBoardDisplay = PyBoardDisplay(HardwareLibrary, True)

# Show our readiness
print("SCARA Arm Test")
statusStr = "Running Test"
pyBoardDisplay.showStatus(statusStr)

# Move
scaraOne.enableMotorDrive(True, 60000)
#scaraOne.moveTo(0, 100)
#scaraOne.moveVertical(20)
#scaraOne.moveVertical(-100)
#scaraOne.moveVertical(500)
#scaraOne.moveVertical(-500)

#scaraOne.verticalEnableBar.value(0)
#scaraOne.verticalDirn.value(0)
#for i in range(500):
#    scaraOne.verticalStep.value(1)
#    HardwareLibrary.udelay(1)
#    scaraOne.verticalStep.value(0)
#    HardwareLibrary.udelay(300)

print(scaraOne.robotConfiguration["origin"])
print(scaraOne.robotConfiguration["shoulderGearMismatchFactor"])
print(scaraOne.robotConfiguration["upperArm"]["stepsPerDegree"])
print(scaraOne.robotConfiguration["lowerArm"]["stepsPerDegree"])

curX = 0
curY = 200

def line(x,y,steps):
    global curX, curY
    xDiff = (x-curX)/steps
    yDiff = (y-curY)/steps
    for i in range(steps):
        scaraOne.moveTo(curX+xDiff*i, curY+yDiff*i)
    curX = x
    curY = y

def circle(x,y,r,steps):
    global curX, curY
    angleDiff = math.pi * 2 / steps
    scaraOne.moveTo(x, y + r)
    for i in range(steps):
        curX = x + r*math.sin(angleDiff*(i+1))
        curY = y + r*math.cos(angleDiff*(i+1))
        scaraOne.moveTo(curX, curY)
    
#scaraOne.moveTo(xStart, yStart)

#line(0,80,20)
circle(0,120,40,100)

time.sleep(0.5)

scaraOne.moveTo(0,200)

time.sleep(0.5)

scaraOne.enableMotorDrive(False, 10000)

