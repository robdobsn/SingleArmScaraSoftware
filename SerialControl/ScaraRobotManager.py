
# ScaraGeometry contains calculations used for arm position
import ScaraGeometry
import math

class ScaraRobotManager:

    def __init__(self, robotConfiguration, robotControl):

        # Save the robot config
        self.robotConfiguration = robotConfiguration
        self.robotControl = robotControl

        # Extract common values
        self.xOrigin = self.robotConfiguration["origin"][0]
        self.yOrigin = self.robotConfiguration["origin"][1]
        self.upperArmLen = self.robotConfiguration["upperArm"]["armLen"]
        self.lowerArmLen = self.robotConfiguration["lowerArm"]["armLen"]
        self.upperStepsPerDegree = self.robotConfiguration["upperArm"]["stepsPerDegree"]
        self.lowerStepsPerDegree = self.robotConfiguration["lowerArm"]["stepsPerDegree"]
        self.upperArmMaxAngle = self.robotConfiguration["upperArm"]["armMaxAngle"]
        self.lowerArmMaxAngle = self.robotConfiguration["lowerArm"]["armMaxAngle"]
        self.shoulderGearMismatchFactor = self.robotConfiguration["shoulderGearMismatchFactor"]
        self.verticalStepsPerMM = self.robotConfiguration["vertical"]["stepsPerMM"]
        self.verticalTravelMax = self.robotConfiguration["vertical"]["verticalTravelMax"]

        # Accumulated movement and elbow x,y position
        self.curLowerStepsFromZero = 0
        self.curUpperStepsFromZero = 0
        self.curElbowX = 0
        self.curElbowY = self.upperArmLen

        # Z (vertical) position
        self.curVerticalStepsFromZero = 0

    def setHomeAsCurrentPos(self):
        self.curLowerStepsFromZero = 0
        self.curUpperStepsFromZero = 0
        self.curElbowX = 0
        self.curElbowY = self.upperArmLen
        return True

    def setVerticalPosHome(self):
        self.curVerticalStepsFromZero = 0
        return True

    # Move to an x,y point
    # Does not attempt to move in a completely straight line
    # But does move upper and lower arm proportionately - so if upper needs to move
    # 100 steps and lower to move 500 steps to reach destination then move the lower
    # arm 5 steps for every one step of the upper
    def moveTo(self, x,y):

        # Find the intersection point of the circles centred on the "shoulder" and the pen
        p1, p2 = ScaraGeometry.circleIntersection((self.xOrigin,self.yOrigin,self.upperArmLen), (x,y,self.lowerArmLen))

        # Check the y values of each alternative geometrical solutions for the "elbow" position
        # If only one of the solutions has y value > 0 then choose that one
        targetElbowPt = p1
        otherIntersectPt = p2
        if p1[1] >= 0 and p2[1] > 0:
            # Both have y > 0 so choose the point while moves the elbow the least
            # This should avoid moving the elbow back an forth unnecessarily as it moves small distances
            delta1 = math.atan2(p1[0]-self.curElbowX, p1[1]-self.curElbowY)
            delta2 = math.atan2(p2[0]-self.curElbowX, p2[1]-self.curElbowY)
            if delta2 < delta1:
                targetElbowPt = p2
                otherIntersectPt = p1
        elif p1[1] < 0 and p2[1] < 0:
            # Can't reach this position
            print("XXXX Requested MoveTo x,y ", x, y, " intersection points ", p1, p2)
            print("XXXX Can't reach this point")
            return False
        elif p1[1] < 0:
            # Choose second point
            targetElbowPt = p2
            otherIntersectPt = p1
        print("MoveTo", x, y, "TargetElbow", targetElbowPt, "OtherIntersect", otherIntersectPt)
        x1 = targetElbowPt[0]
        y1 = targetElbowPt[1]

        # Calculate rotation angles
        d2r = math.pi/180
        thetaUpper = math.atan2(x1-self.xOrigin, y1-self.yOrigin) / d2r
        thetaLower = math.atan2(x-x1, y-y1) / d2r

        # The lower arm may be rotated when the upper arm rotates if the gears at the shoulder joint are
        # mismatched - this code corrects for this by applying an adjustment to the lower arm angle based
        # on the upper arm angle
        debugUncorrectedThetaLower = thetaLower
        thetaLower += thetaUpper * self.shoulderGearMismatchFactor
        uncorrStr = ""
        if self.shoulderGearMismatchFactor != 0:
            uncorrStr = "uncorrected thetaLower {0:0.2f}".format(debugUncorrectedThetaLower)
        print("Theta Upper/Lower", thetaUpper, thetaLower, uncorrStr)
        lowerSteps = int(round(thetaLower*self.lowerStepsPerDegree - self.curLowerStepsFromZero))
        upperSteps = int(round(thetaUpper*self.upperStepsPerDegree - self.curUpperStepsFromZero))
        print("Moving upper(total) ", upperSteps, "(", self.curUpperStepsFromZero, ") lower(total) ", lowerSteps, "(",
              self.curLowerStepsFromZero, ")")

        # Check the angles calculated against the robot capabilities to ensure the arm can actually move to the required position
        if (self.curUpperStepsFromZero + upperSteps > self.upperArmMaxAngle * self.upperStepsPerDegree) \
                        or (self.curUpperStepsFromZero + upperSteps < -self.upperArmMaxAngle * self.upperStepsPerDegree):
            print("Upper arm movement out of bounds - angle would be ", self.curUpperStepsFromZero*upperSteps/self.upperStepsPerDegree)
            return False
        if (self.curLowerStepsFromZero + lowerSteps > self.lowerArmMaxAngle * self.lowerStepsPerDegree) \
                        or (self.curLowerStepsFromZero + lowerSteps < -self.lowerArmMaxAngle * self.lowerStepsPerDegree):
            print("Lower arm movement out of bounds - angle would be ", self.curLowerStepsFromZero*lowerSteps/self.lowerStepsPerDegree)
            return False

        # Check movement is required
        lowerAbsSteps = abs(lowerSteps)
        upperAbsSteps = abs(upperSteps)
        if lowerAbsSteps == 0 and upperAbsSteps == 0:
            print("Neither upper or lower arms need to move to reach destination")
            return False

        # # Use an integer form of Bresenham's line algorithm https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
        # # The good thing about this is that it involves only repeated addition to generate a smooth stepping motion
        # stepAccumulator = 0
        # lowerStepCount = 0
        # upperStepCount = 0
        # while(1):
        #     # This code could be simplified as each case is pretty much the same
        #     if lowerAbsSteps > upperAbsSteps:
        #         self.robotControl.stepLowerArm(lowerSteps < 0)
        #         lowerStepCount += 1
        #         stepAccumulator += upperAbsSteps
        #         if stepAccumulator >= lowerAbsSteps:
        #             self.robotControl.stepUpperArm(upperSteps < 0)
        #             upperStepCount += 1
        #             stepAccumulator -= lowerAbsSteps
        #         if lowerStepCount == lowerAbsSteps:
        #             if upperStepCount < upperAbsSteps:
        #                 print("Lower > Upper - upper catching up by ", upperAbsSteps, upperStepCount)
        #                 for i in range(upperAbsSteps-upperStepCount):
        #                     self.robotControl.stepUpperArm(upperSteps < 0)
        #             break
        #     else:
        #         self.robotControl.stepUpperArm(upperSteps < 0)
        #         upperStepCount += 1
        #         stepAccumulator+= lowerAbsSteps
        #         if stepAccumulator >= upperAbsSteps:
        #             self.robotControl.stepLowerArm(lowerSteps < 0)
        #             lowerStepCount += 1
        #             stepAccumulator -= upperAbsSteps
        #         if upperStepCount == upperAbsSteps:
        #             if lowerStepCount < lowerAbsSteps:
        #                 print("Upper > Lower - lower catching up by ", lowerAbsSteps, lowerStepCount)
        #                 for i in range(lowerAbsSteps-lowerStepCount):
        #                     self.robotControl.stepLowerArm(lowerSteps < 0)
        #             break

        # Move each section independently
        for iStp in range(upperAbsSteps):
            self.robotControl.stepUpperArm(upperSteps > 0)
        # The motor on the lower arm is upside down so steps in opposite direction
        for iStp in range(lowerAbsSteps):
            self.robotControl.stepLowerArm(lowerSteps < 0)

        # Update the current arm position
        self.curUpperStepsFromZero += upperSteps
        self.curLowerStepsFromZero += lowerSteps
        return True

    def moveVertical(self, z):
        finalStepPos = z * self.verticalStepsPerMM
        if finalStepPos > self.verticalTravelMax * self.verticalStepsPerMM:
            finalStepPos = self.verticalTravelMax * self.verticalStepsPerMM
        elif finalStepPos < 0:
            finalStepPos = 0
        # Calculate steps to get there
        requiredSteps = round(finalStepPos - self.curVerticalStepsFromZero)
        print("MoveVertical z", z, "Steps", requiredSteps)
        # print("Req steps", requiredSteps)
        # Step in the required direction
        # print("Int req", int(abs(requiredSteps)))
        for i in range(int(abs(requiredSteps))):
            self.robotControl.stepVertical(requiredSteps > 0)
        self.curVerticalStepsFromZero += requiredSteps
        return True
