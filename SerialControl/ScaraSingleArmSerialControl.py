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

# PYB is the main pyboard library
import pyb

# Serial Connection - this uses pins Y1 and Y2 (Tx and Rx)
uart = pyb.UART(6, 115200)

# LCD - only used for visual confirmation of activity
# Set lcdIsUsed to False if not required
lcdIsUsed = True
if lcdIsUsed:
    lcd = pyb.LCD('X')
    lcd.light(True)

# Hardware connections to stepper motor drivers
# The two stepper drivers are Pololu A4988 modules and each has a step and direction pin
lowerArmStep = pyb.Pin('Y9', pyb.Pin.OUT_PP)
lowerArmDirn = pyb.Pin('Y10', pyb.Pin.OUT_PP)
lowerArmEnableBar = pyb.Pin('Y3', pyb.Pin.OUT_PP)
upperArmStep = pyb.Pin('Y11', pyb.Pin.OUT_PP)
upperArmDirn = pyb.Pin('Y12', pyb.Pin.OUT_PP)
upperArmEnableBar = pyb.Pin('Y4', pyb.Pin.OUT_PP)

# Initially disable motor drivers
lowerArmEnableBar.value(1)
upperArmEnableBar.value(1)

# Leave motor drivers on for this amount of time after last move
motorOnTimeMillis = 60000

# Pen control is via an electromagnet - apply power (logic 1) to push pen down
penMag = pyb.Pin('Y8', pyb.Pin.OUT_PP)

# Pulse width and time between pulses for the stepper motors
# Setting betweenPulsesUsecs to 300 is medium speed
# Set betweenPulsesUsecs to a lower number to increase speed of arm movement
pulseWidthUsecs = 1
betweenPulsesUsecs = 300

# Upper arm degrees per step calculation
# Stepper motors move 1.8 degrees per full step
# In microstepping mode so 16 microsteps per step
# Motor shaft pulley has 20 teeth
# Upper arm pulley has 62 teeth
upperStepsPerDegree = 1/((1.8/16)*(20/62))
print("Upper steps per degree", upperStepsPerDegree)

# Lower arm degrees per step calculation
# Stepper motors move 1.8 degrees per full step
# In microstepping mode so 16 microsteps per step
# Motor shaft pulley has 20 teeth
# Upper arm pulley has 62 teeth
lowerStepsPerDegree = 1/((1.8/16)*(20/62))
print("Lower steps per degree", lowerStepsPerDegree)

# Max ranges of arms from straight forwards
upperArmMaxAngle = 90
lowerArmMaxAngle = 160

# Origin and arm lengths, L1 is the upper arm and L2 the lower arm
x0 = 0
y0 = 0
L1 = 100
L2 = 100

# Accumulated movement and elbow x,y 0position
curLowerStepsFromZero = 0
curUpperStepsFromZero = 0
curElbowX = 0
curElbowY = L1

# Maths required for calculations of geometry0
from math import cos, sin, pi, sqrt, atan2, asin, acos
d2r = pi/180

# Circle intersection algorithm from http://paulbourke.net/geometry/circlesphere/
# and https://gist.github.com/xaedes/974535e71009fa8f090e
def circle_intersection(circle1, circle2):
    '''
    @summary: calculates intersection points of two circles
    @param circle1: tuple(x,y,radius)
    @param circle2: tuple(x,y,radius)
    @result: tuple of intersection points (which are (x,y) tuple)
    '''
    # return self.circle_intersection_sympy(circle1,circle2)
    x1,y1,r1 = circle1
    x2,y2,r2 = circle2
    # http://stackoverflow.com/a/3349134/798588
    dx,dy = x2-x1,y2-y1
    d = sqrt(dx*dx+dy*dy)
    if d > r1+r2:
        print ("Circle intersection failed #1")
        return None # no solutions, the circles are separate
    if d < abs(r1-r2):
        print ("Circle intersection failed #2")
        return None # no solutions because one circle is contained within the other
    if d == 0 and r1 == r2:
        print ("Circle intersection failed #3")
        return None # circles are coincident and there are an infinite number of solutions

    a = (r1*r1-r2*r2+d*d)/(2*d)
    h = sqrt(r1*r1-a*a)
    xm = x1 + a*dx/d
    ym = y1 + a*dy/d
    xs1 = xm + h*dy/d
    xs2 = xm - h*dy/d
    ys1 = ym - h*dx/d
    ys2 = ym + h*dx/d

    return (xs1,ys1),(xs2,ys2)

# Perform a single step of the upper arm
# Direction must already be set
def stepUpper():
    upperArmStep.value(1)
    pyb.udelay(pulseWidthUsecs)
    upperArmStep.value(0)
    pyb.udelay(betweenPulsesUsecs)

# Perform a single step of the lower arm
# Direction must already be set
def stepLower():
    lowerArmStep.value(1)
    pyb.udelay(pulseWidthUsecs)
    lowerArmStep.value(0)
    pyb.udelay(betweenPulsesUsecs)

# Move to an x,y point
# Does not attempt to move in a completely straight line
# But does move upper and lower arm proportionately - so if upper needs to move
# 100 steps and lower to move 500 steps to reach destination then move the lower
# arm 5 steps for every one step of the upper
def moveTo(x,y):
    # Needs to know the current position of the arm
    global curLowerStepsFromZero, curUpperStepsFromZero
    global curElbowX, curElbowY

    # Find the intersection point of the circles centred on the "shoulder" and the pen
    p1, p2 = circle_intersection((x0,y0,L1), (x,y,L2))
    # print("MoveTo x,y ", x, y, " intersection points ", p1, p2)

    # Check the y values of each alternative geometrical solutions for the "elbow" position
    # If only one of the solutions has y value > 0 then choose that one
    targetElbowPt = p1
    if p1[1] >= 0 and p2[1] > 0:
        # Both have y > 0 so choose the point nearest to current position
        # This should avoid the arm its elbow back an forth unnecessarily as it moves small distances
        delta1 = atan2(p1[0]-curElbowX, p1[1]-curElbowY)
        delta2 = atan2(p2[0]-curElbowX, p2[1]-curElbowY)
        if delta2 < delta1:
            targetElbowPt = p2
    elif p1[1] < 0 and p2[1] < 0:
        # Can't reach this position
        print("XXXX Requested MoveTo x,y ", x, y, " intersection points ", p1, p2)
        print("XXXX Can't reach this point")
        return False
    elif p1[1] < 0:
        # Choose second point
        targetElbowPt = p2
    x1 = targetElbowPt[0]
    y1 = targetElbowPt[1]
    print("TargetElbowPt ", x1, y1)

    # Calculate rotation angles
    thetaUpper = atan2(x1-x0, y1-y0) / d2r
    thetaLower = -atan2(x-x1, y-y1) / d2r

    # Adjest lower rotation angle to compensate for mismatched gears in my build
    # Shoulder gear has 60 teeth and elbow gear has 62
    # The result of this is that a 90 degree rotation of the upper arm 
    # results in a ((90 * 62/60) - 90) = 1/30 degree turn of the lower arm
    # So we need to correct lower angle by 1/30th of upper angle
    uncorrectedThetaLower = thetaLower
    thetaLower -= thetaUpper / 30
    print("ThetaUpper", thetaUpper, "ThetaLower", thetaLower, "(uncorrected thetaLower)", uncorrectedThetaLower)
    lowerSteps = int(thetaLower*lowerStepsPerDegree - curLowerStepsFromZero)
    upperSteps = int(thetaUpper*upperStepsPerDegree - curUpperStepsFromZero)
    print("Moving lower(total) ", lowerSteps, "(", curLowerStepsFromZero, ") upper(total) ", upperSteps, "(", curUpperStepsFromZero, ")")

    # Check the angles calculated against the robot capabilities to ensure the arm can actually move to the required position
    if (curUpperStepsFromZero + upperSteps > upperArmMaxAngle * upperStepsPerDegree) \
                    or (curUpperStepsFromZero + upperSteps < -upperArmMaxAngle * upperStepsPerDegree):
        print("Upper arm movement out of bounds - angle would be ", curUpperStepsFromZero*upperSteps/upperStepsPerDegree)
        return False
    if (curLowerStepsFromZero + lowerSteps > lowerArmMaxAngle * lowerStepsPerDegree) \
                    or (curLowerStepsFromZero + lowerSteps < -lowerArmMaxAngle * lowerStepsPerDegree):
        print("Lower arm movement out of bounds - angle would be ", curLowerStepsFromZero*lowerSteps/lowerStepsPerDegree)
        return False

    # Set the direction pins on the stepper motor controllers
    lowerArmDirn.value(lowerSteps < 0)
    upperArmDirn.value(upperSteps < 0)

    # Check movement is required
    lowerAbsSteps = abs(lowerSteps)
    upperAbsSteps = abs(upperSteps)
    if lowerAbsSteps == 0 and upperAbsSteps == 0:
        print("Neither upper or lower arms need to move to reach destination")
        return False

    # Use an integer form of Bresenham's line algorithm https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
    # The good thing about this is that it involves only repeated addition to generate a smooth stepping motion
    stepAccumulator = 0
    lowerStepCount = 0
    upperStepCount = 0
    while(1):
        # This code could be simplified as each case is pretty much the same
        if lowerAbsSteps > upperAbsSteps:
            stepLower()
            lowerStepCount += 1
            stepAccumulator += upperAbsSteps
            if stepAccumulator >= lowerAbsSteps:
                stepUpper()
                upperStepCount += 1
                stepAccumulator -= lowerAbsSteps
            if lowerStepCount == lowerAbsSteps:
                if upperStepCount < upperAbsSteps:
                    print("Lower > Upper - upper catching up by ", upperAbsSteps, upperStepCount)
                    for i in range(upperAbsSteps-upperStepCount):
                        stepUpper()
                break
        else:
            stepUpper()
            upperStepCount += 1
            stepAccumulator+= lowerAbsSteps
            if stepAccumulator >= upperAbsSteps:
                stepLower()
                lowerStepCount += 1
                stepAccumulator -= upperAbsSteps
            if upperStepCount == upperAbsSteps:
                if lowerStepCount < lowerAbsSteps:
                    print("Upper > Lower - lower catching up by ", lowerAbsSteps, lowerStepCount)
                    for i in range(lowerAbsSteps-lowerStepCount):
                        stepLower()
                break

    # Update the current arm position
    curUpperStepsFromZero = curUpperStepsFromZero + upperSteps
    curLowerStepsFromZero = curLowerStepsFromZero + lowerSteps

# Show status on LCD screen if required
def showStatus(statusStr):
    if lcdIsUsed:
        lcd.fill(0)
        lcd.text("SCARA Serial", 0, 0, 1)
        lcd.text(statusStr, 0, 20, 1)
        lcd.show()

# Enable motor drive for a period of time / Disable motor drive
motorsEnabledFlag = False
motorsEnabledLastMillis = 0
motorsEnabledForMillis = 0
def enableMotorDrive(turnMotorsOn, timeLimitForDriveMillis):
    global motorsEnabledFlag, motorsEnabledLastMillis, motorsEnabledForMillis
    # Check if we are turning the motors off
    if not turnMotorsOn:
        lowerArmEnableBar.value(1)
        upperArmEnableBar.value(1)
        motorsEnabledFlag = False
        return
    # Turn the motors on and remember when we did it
    lowerArmEnableBar.value(0)
    upperArmEnableBar.value(0)
    motorsEnabledForMillis = timeLimitForDriveMillis
    motorsEnabledLastMillis = pyb.millis()
    motorsEnabledFlag = True

# Enable motor drive for a period of time
def motorOnTimeLimitCheck():
    global motorsEnabledFlag, motorsEnabledLastMillis, motorsEnabledForMillis
    # Check if motors are on
    if not motorsEnabledFlag:
        return
    # Check if motors enabled indefinitely
    if motorsEnabledForMillis == 0:
        return
    # Check if time limit for motors being on has elapsed and turn off if so
    if pyb.elapsed_millis(motorsEnabledLastMillis) > motorsEnabledForMillis:
        enableMotorDrive(False, 0)

# Extract a floating point number from a string ensuring no exceptions are thrown
def extractNum(inStrList, listIdx, minVal, maxVal):
    if listIdx < 0 or listIdx >= len(inStrList):
        return 0, False
    try:
        val = float(inStrList[listIdx])
    except:
        return 0, False
    if val < minVal or val > maxVal:
        return 0, False
    return val, True

# The absolute min and max possible values for the pen position
# These are based on the "bounding box" biggest rectangle around all possible positions
# Due to polar nature of arm some value combinations will be invalid
boundingBoxMinXValue = -(L1+L2)
boundingBoxMaxXValue = L1+L2
boundingBoxMinYValue = -L2
boundingBoxMaxYValue = L1+L2

# Interpret a command line
# More information on the command syntax at the top of this file
def interpCommand(cmdStr):

    global motorOnTimeMillis

    # Split the command line at the spaces
    splitStr = cmdStr.split()

    # G0 command - go to X,Y
    if splitStr[0] == 'G0':
        # Goto command
        x, xValidity = extractNum(splitStr, 1, boundingBoxMinXValue, boundingBoxMaxXValue)
        y, yValidity = extractNum(splitStr, 2, boundingBoxMinYValue, boundingBoxMaxYValue)
        if xValidity and yValidity:
            statusStr = "Go " + str(x) + ', ' + str(y)
            print(statusStr)
            showStatus(statusStr)
            enableMotorDrive(True, motorOnTimeMillis)
            moveTo(x,y)
        else:
            print("Move to cmd ", cmdStr, " failed")

    # S0 & S1 commands - move an arm a given number of steps
    elif splitStr[0] == 'S0' or splitStr[0] == 'S1':
        steps, stepsValidity = extractNum(splitStr, 1, -1000, 1000)
        if stepsValidity:
            enableMotorDrive(True, motorOnTimeMillis)
            if splitStr[0] == 'S1':
                lowerArmDirn.value(steps > 0)
                for i in range(abs(steps)):
                    stepLower()
            else:
                upperArmDirn.value(steps < 0)
                for i in range(abs(steps)):
                    stepUpper()
            statusStr = "Step " + "lower" if splitStr[0] == 0 else "upper" + str(steps)
            print(statusStr)
            showStatus(statusStr)
        else:
            print("Step cmd", cmdStr, "failed")

    # C0 command - set the current position to be the home position (calibrate)
    elif splitStr[0] == 'C0':
        curLowerStepsFromZero = 0
        curUpperStepsFromZero = 0
        curElbowX = 0
        curElbowY = L1
        statusStr = "Calibrated"
        showStatus(statusStr)
        print(statusStr)

    # P0 & P1 - pen up and pen down0
    elif splitStr[0] == 'P0' or splitStr[0] == 'P1':
        isPenDown = (splitStr[0] == 'P1')
        penMag.value(isPenDown)
        statusStr = "Pen Down" if isPenDown else "Pen Up"
        showStatus(statusStr)
        print(statusStr)

    # E0 & E1 - Disable and Enable motor drive
    elif splitStr[0] == 'E0' or splitStr[0] == 'E1':
        if splitStr[0] == 'E1':
            statusStr = "Enable Motors"
            timeLimit, timeLimitVaidity = extractNum(splitStr, 1, 0, 1000000000)
            if not timeLimitVaidity:
                timeLimit = motorOnTimeMillis
            enableMotorDrive(True, timeLimit)
        else:
            statusStr = "Disable Motors"
            enableMotorDrive(False, 0)
        showStatus(statusStr)
        print(statusStr)

    # D0 - Set default motor on time
    elif splitStr[0] == 'D0':
        statusStr = "Motor on Time"
        timeLimit, timeLimitVaidity = extractNum(splitStr, 1, 0, 1000000000)
        if not timeLimitVaidity:
            return
        motorOnTimeMillis = timeLimit
        showStatus(statusStr)
        print(statusStr)

    # Unknown command
    else:
        statusStr = "Unknown " + cmdStr
        showStatus(statusStr)
        print(statusStr)

# Build up a command string from characters received on the serial port
serialCommandStr = ""
def receiveAndExecuteCommands():
    global serialCommandStr
    # Check for any characters waiting to be handled
    if uart.any() > 0:
        for i in range(uart.any()):
            ch = uart.readchar()

            # Linefeed (\n or 0x0a) is used to signify end of command
            if ch == 0x0a:
                uart.write("LF\r\n")
                # print("Command is:", serialCommandStr)
                interpCommand(serialCommandStr)
                serialCommandStr = ""
                continue

            # Ignore CR   
            if ch == 0x0d:
                continue

            # Handle backspace to make it easier to control by typing directly into a
            # serial terminal program
            if ch == 127:
                uart.writechar(0x08)
                uart.writechar(0x20)
                uart.writechar(0x08)
                if len(serialCommandStr) > 0:
                    serialCommandStr = serialCommandStr[:-1]
                continue

            # Handle other chars by adding to the command string
            serialCommandStr += chr(ch)
            uart.write(chr(ch))

# Show our readiness
uart.write("SCARA Arm Awaiting Command\r\n")
print("SCARA Arm Awaiting Command")
statusStr = "Ready"
showStatus(statusStr)

# Loop here indefinitely receiving serial commands and executing them
while(True):
    receiveAndExecuteCommands()
    motorOnTimeLimitCheck()
    pyb.delay(10)
