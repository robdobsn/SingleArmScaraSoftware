import pyb

#Serial Connection
uart = pyb.UART(6, 115200)

# Touch keypad
import mpr121
keybd = mpr121.MPR121(pyb.I2C(1, pyb.I2C.MASTER))
keybd.debounce(3,3)
for electr in range(4):
    keybd.threshold(electr, 50, 30)

# LCD
lcd = pyb.LCD('X')
lcd.light(True)

# Maths required for calculations
from math import cos, sin, pi, sqrt, atan2, asin, acos
d2r = pi/180

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

lowerArmStep = pyb.Pin('Y9', pyb.Pin.OUT_PP)
lowerArmDirn = pyb.Pin('Y10', pyb.Pin.OUT_PP)
upperArmStep = pyb.Pin('Y11', pyb.Pin.OUT_PP)
upperArmDirn = pyb.Pin('Y12', pyb.Pin.OUT_PP)

penMag = pyb.Pin('Y8', pyb.Pin.OUT_PP)

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

# Origin and arm lengths
x0 = 0
y0 = 0
L1 = 100
L2 = 100

# Accumulated movement and elbow x,y 0position
curLowerStepsFromZero = 0
curUpperStepsFromZero = 0
curElbowX = 0
curElbowY = L1

def stepUpper():
    upperArmStep.value(1)
    pyb.udelay(pulseWidthUsecs)
    upperArmStep.value(0)
    pyb.udelay(betweenPulsesUsecs)

def stepLower():
    lowerArmStep.value(1)
    pyb.udelay(pulseWidthUsecs)
    lowerArmStep.value(0)
    pyb.udelay(betweenPulsesUsecs)

# Move to an x,y point
def moveTo(x,y):
    global curLowerStepsFromZero, curUpperStepsFromZero
    global curElbowX, curElbowY

    p1, p2 = circle_intersection((x0,y0,L1), (x,y,L2))
    # print("MoveTo x,y ", x, y, " intersection points ", p1, p2)

    # Check the y values of each point - if only one is > 0 then choose that one
    targetElbowPt = p1
    if p1[1] >= 0 and p2[1] > 0:
        # Both > 0 so choose point nearest to current position
        delta1 = atan2(p1[0]-curElbowX, p1[1]-curElbowY)
        delta2 = atan2(p2[0]-curElbowX, p2[1]-curElbowY)
        if delta2 < delta1:
            targetElbowPt = p2
    elif p1[1] < 0 and p2[1] < 0:
        print("XXXX Requested MoveTo x,y ", x, y, " intersection points ", p1, p2)
        print("XXXX Can't reach this point")
        return False
    elif p1[1] < 0:
        targetElbowPt = p2
    x1 = targetElbowPt[0]
    y1 = targetElbowPt[1]
    print("TargetElbowPt ", x1, y1)

    # Calculate rotation angles
    thetaUpper = atan2(x1-x0, y1-y0) / d2r
    thetaLower = -atan2(x-x1, y-y1) / d2r

    # Adjest lower rotation angle to compensate for mismatched gears - shoulder gear has 60 teeth and elbow gear has 62
    # The result of this is that a 90 degree rotation of the upper arm results in a ((90 * 62/60) - 90) = 1/30 degree turn of the lower arm
    # So need to correct lower angle by 1/30th of upper angle

    uncorrectedThetaLower = thetaLower
    thetaLower -= thetaUpper / 30
    print("ThetaUpper", thetaUpper, "ThetaLower", thetaLower, "(uncorrected thetaLower)", uncorrectedThetaLower)

    lowerSteps = int(thetaLower*lowerStepsPerDegree - curLowerStepsFromZero)
    upperSteps = int(thetaUpper*upperStepsPerDegree - curUpperStepsFromZero)

    print("Moving lower(total) ", lowerSteps, "(", curLowerStepsFromZero, ") upper(total) ", upperSteps, "(", curUpperStepsFromZero, ")")

    # Check bounds
    if (curUpperStepsFromZero + upperSteps > upperArmMaxAngle * upperStepsPerDegree) or (curUpperStepsFromZero + upperSteps < -upperArmMaxAngle * upperStepsPerDegree):
        print("Upper arm movement out of bounds - angle would be ", curUpperStepsFromZero*upperSteps/upperStepsPerDegree)
        return False
    if (curLowerStepsFromZero + lowerSteps > lowerArmMaxAngle * lowerStepsPerDegree) or (curLowerStepsFromZero + lowerSteps < -lowerArmMaxAngle * lowerStepsPerDegree):
        print("Lower arm movement out of bounds - angle would be ", curLowerStepsFromZero*lowerSteps/lowerStepsPerDegree)
        return False

    # lowerArmDirn.value(lowerSteps > 0)
    # for i in range(abs(lowerSteps)):
    #     stepLower()
    # upperArmDirn.value(upperSteps < 0)
    # for i in range(abs(upperSteps)):
    #     stepUpper()

    lowerArmDirn.value(lowerSteps < 0)
    upperArmDirn.value(upperSteps < 0)
    accum = 0
    lowerCount = 0
    upperCount = 0
    lowerAbsSteps = abs(lowerSteps)
    upperAbsSteps = abs(upperSteps)
    if lowerAbsSteps > 0 or upperAbsSteps > 0:
        while(1):
            if lowerAbsSteps > upperAbsSteps:
                stepLower()
                lowerCount += 1
                accum += upperAbsSteps
                if accum >= lowerAbsSteps:
                    stepUpper()
                    upperCount += 1
                    accum -= lowerAbsSteps
                if lowerCount == lowerAbsSteps:
                    if upperCount < upperAbsSteps:
                        print("Lower > Upper - upper catching up by ", upperAbsSteps, upperCount)
                        for i in range(upperAbsSteps-upperCount):
                            stepUpper()
                    break
            else:
                stepUpper()
                upperCount += 1
                accum+= lowerAbsSteps
                if accum >= upperAbsSteps:
                    stepLower()
                    lowerCount += 1
                    accum -= upperAbsSteps
                if upperCount == upperAbsSteps:
                    if lowerCount < lowerAbsSteps:
                        print("Upper > Lower - lower catching up by ", lowerAbsSteps, lowerCount)
                        for i in range(lowerAbsSteps-lowerCount):
                            stepLower()
                    break
    else:
        print("Neither upper of lower arm moving")

    curUpperStepsFromZero = curUpperStepsFromZero + upperSteps
    curLowerStepsFromZero = curLowerStepsFromZero + lowerSteps

# Extract a number from a string
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


# Min and max values for interpretation function 
# Due to polar nature of arm some value combinations will be invalid
minXValue = -200
maxXValue = 200
minYValue = 0
maxYValue = 200

# Interpret command string
def interpCommand(cmdStr):
	# Split command
	splitStr = cmdStr.split()
	if splitStr[0] == 'G0':
		# Goto command
		x, xValidity = extractNum(splitStr, 1, minXValue, maxXValue)
		y, yValidity = extractNum(splitStr, 2, minYValue, maxYValue)
		if xValidity and yValidity:
			moveTo(x,y)
			print("Moving to ", x, ", ", y)
		else:
			print("Move to cmd ", cmdStr, " failed")
	elif splitStr[0] == 'S0' or splitStr[0] == 'S1':
		steps, stepsValidity = extractNum(splitStr, 1, -1000, 1000)
		if stepsValidity:
			if splitStr[0] == 'S1':
				lowerArmDirn.value(steps > 0)
				for i in range(abs(steps)):
					stepLower()
			else:
				upperArmDirn.value(steps < 0)
				for i in range(abs(steps)):
					stepUpper()
			print("Step", "lower" if splitStr[0] == 0 else "upper", steps)
		else:
			print("Step cmd", cmdStr, "failed")
	elif splitStr[0] == 'C0':
		curLowerStepsFromZero = 0
		curUpperStepsFromZero = 0
		curElbowX = 0
		curElbowY = L1
		print("Calibrated")
	elif splitStr[0] == 'P0':
		penMag.value(0)
		print("Pen Up")
	elif splitStr[0] == 'P1':
		penMag.value(1)
		print("Pen Down")
	else:
		print("Unknown command", cmdStr)

curPressHysteresis = 0
cmdStr = ""

uart.write("SCARA Arm Awaiting Command\r\n")
print("SCARA Arm Awaiting Command")

while(True):
    # if (keybd.elec_voltage(0) < 220):  # Y key
    #     if curPressHysteresis == 0:
    #         curPressHysteresis = 10
    #         print("stepLower")
    #         stepLower()
    # elif (keybd.elec_voltage(1) < 220):  # X key
    #     if curPressHysteresis == 0:
    #         curPressHysteresis = 10
    #         print("stepUpper")
    #         stepUpper()
    # elif curPressHysteresis > 0:
    #     curPressHysteresis -= 1

    # Check for a command
	if uart.any() > 0:
		for i in range(uart.any()):
			ch = uart.readchar()
			if ch == 0x0d:
				uart.write("\r\n")
				print("Command is:", cmdStr)
				interpCommand(cmdStr)
				cmdStr = ""
				continue
			if ch == 0x0a:
				print("LF")
				continue
			if ch == 127:
				print("BS")
				uart.writechar(0x08)
				uart.writechar(0x20)
				uart.writechar(0x08)
				if len(cmdStr) > 0:
					cmdStr = cmdStr[:-1]
				continue
			cmdStr += chr(ch)
			print(ch)
			uart.write(chr(ch))
	pyb.delay(10)
