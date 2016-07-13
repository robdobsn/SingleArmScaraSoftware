# Python program to run on PyBoard https://micropython.org/
# Tests ScaraOne class which controls the robot

# Handle test mode using a stub of hardware library
TEST_MODE = False
if TEST_MODE:
    import HardwareLibrary
else:
    import pyb as HardwareLibrary

# ScaraOne is the library supporting the ScaraOne Single Arm Scara robot
from ScaraOne import ScaraOne
from PyBoardDisplay import PyBoardDisplay

# Create the robot
scaraOne = ScaraOne(HardwareLibrary)

# Create display
pyBoardDisplay = PyBoardDisplay(HardwareLibrary, True)

# Show our readiness
print("SCARA Arm Test")
statusStr = "Running Test"
pyBoardDisplay.showStatus(statusStr)

# Move
scaraOne.enableMotorDrive(True, 60000)
scaraOne.moveTo(0, 100)
