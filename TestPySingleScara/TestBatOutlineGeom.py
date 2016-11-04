
import math
import matplotlib.pyplot as plt
d2r = math.pi/180

# Circle intersection algorithm from http://paulbourke.net/geometry/circlesphere/
# and https://gist.github.com/xaedes/974535e71009fa8f090e
def circleIntersection(circle1, circle2):
    '''
    @summary: calculates intersection points of two circles
    @param circle1: tuple(x,y,radius)
    @param circle2: tuple(x,y,radius)
    @result: tuple of intersection points (which are (x,y) tuple)
    '''
    # return self.circle_intersection_sympy(circle1,circle2)
    x1, y1, r1 = circle1
    x2, y2, r2 = circle2
    # http://stackoverflow.com/a/3349134/798588
    dx, dy = x2 - x1, y2 - y1
    d = math.sqrt(dx * dx + dy * dy)
    if d > r1 + r2:
        print("Circle intersection failed #1")
        return None  # no solutions, the circles are separate
    if d < abs(r1 - r2):
        print("Circle intersection failed #2")
        return None  # no solutions because one circle is contained within the other
    if d == 0 and r1 == r2:
        print("Circle intersection failed #3")
        return None  # circles are coincident and there are an infinite number of solutions

    a = (r1 * r1 - r2 * r2 + d * d) / (2 * d)
    h = math.sqrt(r1 * r1 - a * a)
    xm = x1 + a * dx / d
    ym = y1 + a * dy / d
    xs1 = xm + h * dy / d
    xs2 = xm - h * dy / d
    ys1 = ym - h * dx / d
    ys2 = ym + h * dx / d

    return (xs1, ys1), (xs2, ys2)


class ScaraGeomCheck:

    def __init__(self):

        self.xOrigin = 0
        self.yOrigin = 0
        self.upperArmLen = 100
        self.lowerArmLen = 100
        self.upperStepsPerDegree = 1/((1.8/16)*(20/62))
        self.lowerStepsPerDegree = 1/((1.8/16)*(20/60))
        self.upperArmMaxAngle = 90
        self.lowerArmMaxAngle = 160
        self.shoulderGearMismatchFactor = 0
        self.verticalStepsPerMM = 400
        self.verticalTravelMax = 400

        # Accumulated movement and elbow x,y position
        self.curLowerStepsFromZero = 0
        self.curUpperStepsFromZero = 0
        self.curElbowX = 0
        self.curElbowY = self.upperArmLen

        # Z (vertical) position
        self.curVerticalStepsFromZero = 0

    def getSteps(self, x,y):

        # Find the intersection point of the circles centred on the "shoulder" and the pen
        p1, p2 = circleIntersection((self.xOrigin, self.yOrigin, self.upperArmLen), (x, y, self.lowerArmLen))
        print("MoveTo x,y ", x, y, " intersection points ", p1, p2)

        # Check the y values of each alternative geometrical solutions for the "elbow" position
        # If only one of the solutions has y value > 0 then choose that one
        targetElbowPt = p1
        if p1[1] >= 0 and p2[1] > 0:
            # Both have y > 0 so choose the point while moves the elbow the least
            # This should avoid moving the elbow back an forth unnecessarily as it moves small distances
            delta1 = math.atan2(p1[0] - self.curElbowX, p1[1] - self.curElbowY)
            delta2 = math.atan2(p2[0] - self.curElbowX, p2[1] - self.curElbowY)
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
        d2r = math.pi / 180
        thetaUpper = math.atan2(x1 - self.xOrigin, y1 - self.yOrigin) / d2r
        thetaLower = math.atan2(x - x1, y - y1) / d2r

        # The lower arm may be rotated when the upper arm rotates if the gears at the shoulder joint are
        # mismatched - this code corrects for this by applying an adjustment to the lower arm angle based
        # on the upper arm angle
        debugUncorrectedThetaLower = thetaLower
        thetaLower += thetaUpper * self.shoulderGearMismatchFactor
        print("ThetaUpper", thetaUpper, "ThetaLower", thetaLower, "(uncorrected thetaLower)", debugUncorrectedThetaLower)
        lowerSteps = int(round(thetaLower * self.lowerStepsPerDegree - self.curLowerStepsFromZero))
        upperSteps = int(round(thetaUpper * self.upperStepsPerDegree - self.curUpperStepsFromZero))
        print("Moving upper(total) ", upperSteps, "(", self.curUpperStepsFromZero, ") lower(total) ", lowerSteps, "(",
              self.curLowerStepsFromZero, ")")

        # Check the angles calculated against the robot capabilities to ensure the arm can actually move to the required position
        if (self.curUpperStepsFromZero + upperSteps > self.upperArmMaxAngle * self.upperStepsPerDegree) \
                or (self.curUpperStepsFromZero + upperSteps < -self.upperArmMaxAngle * self.upperStepsPerDegree):
            print("Upper arm movement out of bounds - angle would be ",
                  self.curUpperStepsFromZero * upperSteps / self.upperStepsPerDegree)
            return False
        if (self.curLowerStepsFromZero + lowerSteps > self.lowerArmMaxAngle * self.lowerStepsPerDegree) \
                or (self.curLowerStepsFromZero + lowerSteps < -self.lowerArmMaxAngle * self.lowerStepsPerDegree):
            print("Lower arm movement out of bounds - angle would be ",
                  self.curLowerStepsFromZero * lowerSteps / self.lowerStepsPerDegree)
            return False

        # Check movement is required
        lowerAbsSteps = abs(lowerSteps)
        upperAbsSteps = abs(upperSteps)
        if lowerAbsSteps == 0 and upperAbsSteps == 0:
            print("Neither upper or lower arms need to move to reach destination")
            return False

        # Update the current arm position
        self.curUpperStepsFromZero += upperSteps
        self.curLowerStepsFromZero += lowerSteps
        return (upperSteps, lowerSteps)

batOutlinePoints = [
[4.501693, 32.209405, 0.0],[6.554918, 31.946409, 0.0],[8.487224, 32.688767, 0.0],[10.076201, 34.015437, 0.0],[10.972057, 35.88154, 0.0],[10.761726, 37.940826, 0.0],[9.763341, 39.754146, 0.0],[8.485961, 41.383011, 0.0],[10.52521, 41.027535, 0.0],[12.267153, 39.909261, 0.0],[13.906055, 38.644785, 0.0],[15.37165, 37.18296, 0.0],[16.614563, 35.527646, 0.0],[17.661677, 33.742022, 0.0],[18.531526, 31.863655, 0.0],[19.207161, 29.907021, 0.0],[19.674219, 27.8904, 0.0],[19.922269, 25.835316, 0.0],[19.945873, 23.765451, 0.0],[19.74516, 21.705205, 0.0],[19.325782, 19.678132, 0.0],[18.698268, 17.705538, 0.0],[17.876922, 15.805462, 0.0],[16.854088, 14.00582, 0.0],[15.63849, 12.330344, 0.0],[14.255131, 10.790468, 0.0],[12.702171, 9.421813, 0.0],[10.962991, 8.299248, 0.0],[11.558234, 10.281818, 0.0],[12.540634, 12.103847, 0.0],[13.073751, 14.104019, 0.0],[13.074049, 16.174019, 0.0],[12.382265, 18.125002, 0.0],[10.319134, 18.293501, 0.0],[8.756659, 16.935719, 0.0],[7.723393, 15.142046, 0.0],[6.576421, 16.865228, 0.0],[5.995555, 18.852058, 0.0],[4.07789, 19.631457, 0.0],[2.812871, 17.992974, 0.0],[1.900939, 16.134674, 0.0],[1.237007, 14.174037, 0.0],[0.635916, 12.193232, 0.0],[0.098021, 10.19434, 0.0],[-0.721135, 12.095362, 0.0],[-1.310417, 14.079713, 0.0],[-1.994545, 16.033393, 0.0],[-2.871206, 17.908591, 0.0],[-4.101999, 19.572937, 0.0],[-6.057316, 18.8935, 0.0],[-6.705574, 16.927626, 0.0],[-7.048689, 14.886261, 0.0],[-8.494477, 16.367679, 0.0],[-9.841972, 17.939034, 0.0],[-11.854503, 18.423406, 0.0],[-13.14168, 16.802272, 0.0],[-13.305115, 14.738734, 0.0],[-12.898023, 12.709158, 0.0],[-12.008294, 10.840126, 0.0],[-10.912576, 9.083908, 0.0],[-12.93821, 9.510176, 0.0],[-14.492802, 10.876977, 0.0],[-15.863293, 12.428317, 0.0],[-17.046535, 14.126798, 0.0],[-18.042134, 15.941648, 0.0],[-18.850334, 17.847354, 0.0],[-19.470993, 19.822115, 0.0],[-19.90428, 21.84626, 0.0],[-20.07648, 23.909085, 0.0],[-19.993312, 25.977414, 0.0],[-19.751783, 28.033275, 0.0],[-19.295424, 30.052343, 0.0],[-18.60326, 32.003191, 0.0],[-17.705231, 33.86825, 0.0],[-16.646945, 35.647275, 0.0],[-15.361882, 37.270085, 0.0],[-13.881192, 38.716618, 0.0],[-12.254034, 39.996171, 0.0],[-10.515006, 41.118973, 0.0],[-8.680731, 42.078313, 0.0],[-9.559682, 40.204188, 0.0],[-10.657352, 38.449189, 0.0],[-11.162232, 36.441704, 0.0],[-10.530289, 34.470524, 0.0],[-9.070217, 33.003183, 0.0],[-7.217935, 32.079088, 0.0],[-5.172067, 31.763935, 0.0],[-4.299535, 33.641057, 0.0],[-3.992677, 35.688187, 0.0],[-3.743213, 37.7431, 0.0],[-3.52807, 39.801889, 0.0],[-3.321159, 41.861522, 0.0],[-3.044258, 43.912918, 0.0],[-2.020332, 42.113897, 0.0],[-1.198958, 40.213833, 0.0],[0.856909, 39.972353, 0.0],[1.780854, 41.82471, 0.0],[2.649713, 43.703535, 0.0],[3.219862, 41.713603, 0.0],[3.41985, 39.653286, 0.0],[3.636929, 37.5947, 0.0],[3.882326, 35.539298, 0.0],[4.188339, 33.492042, 0.0]

]

scaraCheck = ScaraGeomCheck()

th1 = 0
th2 = 0

finalPoints = []
finalSteps = []
routerYOrigin = 200
drillPoints = []
for point in batOutlinePoints:
    drillPoints.append([point[0], routerYOrigin - point[1]])
for drillPoint in drillPoints:
    cmdStr = "G0 {0:.0f} {1:.0f}".format(drillPoint[0], drillPoint[1])
    print(cmdStr)
    uppSteps, lowSteps = scaraCheck.getSteps(drillPoint[0],drillPoint[1])
    print("Move steps: ", uppSteps, lowSteps)
    finalSteps.append((uppSteps, lowSteps))
    th1 += uppSteps * ((1.8/16)*(20/62))
    th2 += lowSteps * ((1.8/16)*(20/60))
    xNew = 100 * math.sin(th1 * d2r) + 100 * math.sin(th2 * d2r)
    yNew = 100 * math.cos(th1 * d2r) + 100 * math.cos(th2 * d2r)
    finalPoints.append((xNew, yNew))
    print("New", xNew, yNew, "Error", xNew-drillPoint[0], yNew-drillPoint[1])

print("Final Steps")
for finStep in finalSteps:
    print("[{0:d},{1:d}],".format(finStep[0], finStep[1]))

print()

xPts = [x for [x, y] in finalPoints]
yPts = [y for [x, y] in finalPoints]
plt.scatter(xPts, yPts, s=1)
plt.show()

