import pyb

# Touch keypad
import mpr121
keybd = mpr121.MPR121(pyb.I2C(1, pyb.I2C.MASTER))
keybd.debounce(3,3)
for electr in range(4):
    keybd.threshold(electr, 50, 30)

# LCD
lcd = pyb.LCD('X')
lcd.light(True)

# Test program
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

pulseWidthUsecs = 1
betweenPulsesUsecs = 500

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

x0 = 0
y0 = 0
L1 = 100
L2 = 100

curLowerStepsFromZero = 0
curUpperStepsFromZero = 0
curElbowX = 0
curElbowY = L1

activityText = ""

def updateScreen():
    lcd.fill(0) 
    lcd.text(str(curElbowX), 0,0,1)
    lcd.text(str(curElbowY), 40,0,1)
    lcd.text(str(curLowerStepsFromZero), 0,10,1)
    lcd.text(str(curUpperStepsFromZero), 40,10,1)
    lcd.text(activityText, 0,20,1)
    lcd.show()

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


def drawCircle(centreX, centreY, radius):
    # Draw in segments
    print("Circle x,y,radius ", centreX, centreY, radius)
    lineSegmentCount = 60
    for i in range(lineSegmentCount + 1):
        t = 360 / lineSegmentCount
        x = radius * cos(i * t * pi / 180) + centreX
        y = radius * sin(i * t * pi / 180) + centreY
        moveTo(x,y)

def drawLine(x1, y1, x2, y2):
    lineLen = sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))
    lineSegmentCount = int(lineLen)
    x = x1
    y = y1
    xinc = (x2-x1)/lineSegmentCount
    yinc = (y2-y1)/lineSegmentCount
    for i in range(lineSegmentCount):
        moveTo(x,y)
        x += xinc
        y += yinc
    moveTo(x2,y2)

curPressHysteresis = 0
curEffect = 0

while(True):
    updateScreen()
    # lcd.fill(0) 
    # lcd.text(str(keybd.elec_voltage(0)), 40,10,1)
    # lcd.text(str(keybd.elec_voltage(1)), 0,10,1)
    # lcd.text(str(keybd.elec_voltage(2)), 40,0,1)
    # lcd.text(str(keybd.elec_voltage(3)), 0,0,1)
    # lcd.show()
    if (keybd.elec_voltage(0) < 220):  # Y key
        if curPressHysteresis == 0:
            curPressHysteresis = 10
            drawCircle(0,100, 30)
            pyb.delay(3000)
            moveTo(0,200)
    elif (keybd.elec_voltage(1) < 220):  # X key
        if curPressHysteresis == 0:
            curPressHysteresis = 10
            moveTo(0, 100)
            pyb.delay(3000)
            moveTo(0,200)

    elif (keybd.elec_voltage(2) < 220):  # B key
        if curPressHysteresis == 0:
            curPressHysteresis = 10
            moveTo(100, 100)
            pyb.delay(3000)
            moveTo(0,200)
    elif (keybd.elec_voltage(3) < 220):  # A key
        if curPressHysteresis == 0:
            curPressHysteresis = 10
            x = 0
            y = 200
            if curEffect == 0:
                x=100
                y = 50
            elif curEffect == 1:
                x = -100
                y = 50
            elif curEffect == 2:
                x = 100
                y = 100
            elif curEffect == 3:
                x = -100
                y = 100
            elif curEffect == 4:
                moveTo(100,50)
                pyb.delay(1000)
                x = 100
                y = 100
            elif curEffect == 5:
                moveTo(-100,50)
                pyb.delay(1000)
                x = 100
                y = 100
            # drawLine(100,100,-100,100)
            # pyb.delay(1000)
            # drawLine(0,60,0,200)
            # pyb.delay(1000)
            # drawLine(100,150,-100,50)
            # pyb.delay(1000)
            # lcd.text("moveTo " + str(x) + "," + str(y), 0, 20, 1)
            # lcd.show()
            activityText = "moveTo " + str(x) + "," + str(y)
            updateScreen()

            moveTo(x,y)
            curEffect += 1
            if curEffect >= 9:
                curEffect = 0
            pyb.delay(3000)
            moveTo(0,200)
            activityText = ""
            updateScreen()
    elif curPressHysteresis > 0:
        curPressHysteresis -= 1
    pyb.delay(10)
