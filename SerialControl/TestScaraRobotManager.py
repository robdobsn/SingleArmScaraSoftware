from ScaraRobotManager import ScaraRobotManager

class TestRobot:

    def __init__(self):
        # Adjest lower rotation angle to compensate for mismatched gears in my build
        # Shoulder gear has 60 teeth and elbow gear has 62
        # The result of this is that a 90 degree rotation of the upper arm
        # results in a ((90 * 62/60) - 90) = 1/30 degree turn of the lower arm
        # So we need to correct lower angle by 1/30th of upper angle
        shoulderGearMismatchFactor = 1/30

        # Robot configuration
        robotConfiguration = {
            "origin": [0, 0],
            "upperArm": {
                "armLen": 100,
                "stepsPerDegree": 1.8,
                "armMaxAngle": 90
            },
            "lowerArm": {
                "armLen": 100,
                "stepsPerDegree": 1.8,
                "armMaxAngle": 160
            },
            "shoulderGearMismatchFactor": shoulderGearMismatchFactor
        }

        # Create the ScaraRobotManager - which does movement calculations
        self.scaraRobotManager = ScaraRobotManager(robotConfiguration, self)

    # Perform a single step of the upper arm
    def stepUpperArm(self, dirn):
        #print("Upper Step", dirn)
        return

    # Perform a single step of the lower arm
    def stepLowerArm(self, dirn):
        #print("Lower Step", dirn)
        return

    # MoveTo
    def moveTo(self, x, y):
        self.scaraRobotManager.moveTo(x, y)


testRobot = TestRobot()
testRobot.moveTo(100,100)
testRobot.moveTo(0,200)
testRobot.moveTo(0,100)
testRobot.moveTo(0,200)
