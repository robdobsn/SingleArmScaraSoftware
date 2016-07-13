# Python program to run on PyBoard https://micropython.org/
# Program controls a single-arm SCARA robot http://robdobson.com/2016/03/single-arm-robot/
# Uses a serial ASCII connection to receive commands in a vaguely G-Code syntax

# Each command is on a separate line and is terminated with an LF (newline or linefeed \n) (CR \r is ignored)
# Commands include:
# G0 XXXXX YYYYY      ... go to X,Y position            ... XXXXX and YYYYY are floating point ascii numbers   
# S0 NNNNN            ... move upper arm NNNNN steps    ... NNNNN is an integer between -1000 and 1000         
# S1 NNNNN            ... move lower arm NNNNN steps    ... NNNNN is an integer between -1000 and 1000         
# C0                  ... calibrate, which means set the current position as the home (straight out) position
# P0                  ... pen up
# P1                  ... pen down
# E0 [TTTTT]          ... enable motor drive            ... TTTTT is an optional integer in milliseconds, 0 = indefinitely
# E1                  ... disable motor drive
# D0 TTTTT            ... set default motor on time     ... TTTTT is in milliseconds

# Handle test mode using a stub of hardware library
TEST_MODE = False
if TEST_MODE:
    import HardwareLibrary
else:
    import pyb as HardwareLibrary

# ScaraOne is the library supporting the ScaraOne Single Arm Scara robot
from ScaraOne import ScaraOne
from RobotCommandInterpreter import RobotCommandInterpreter
from PyBoardDisplay import PyBoardDisplay

# Serial Connection - this uses pins Y1 and Y2 (Tx and Rx)
uart = HardwareLibrary.UART(6, 115200)

# Create the robot
scaraOne = ScaraOne(HardwareLibrary)

# Create display
pyBoardDisplay = PyBoardDisplay(HardwareLibrary, True)

# Create the command interpreter
robotCommandInterpreter = RobotCommandInterpreter(scaraOne, pyBoardDisplay)

# Show our readiness
uart.write("SCARA Arm Awaiting Command\r\n")
print("SCARA Arm Awaiting Command")
statusStr = "Ready"
pyBoardDisplay.showStatus(statusStr)

def receiveAndExecuteCommands():
    # Check for any characters waiting to be handled
    if uart.any() > 0:
        for i in range(uart.any()):
            ch = uart.readchar()
            rslt = robotCommandInterpreter.handleChar(ch)
            for ch in rslt:
                uart.write(ch)

# Loop here indefinitely receiving serial commands and executing them
while(True):
    receiveAndExecuteCommands()
    scaraOne.motorOnTimeLimitCheck()
    HardwareLibrary.delay(10)
