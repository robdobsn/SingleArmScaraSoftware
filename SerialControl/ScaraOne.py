from ScaraRobotManager import ScaraRobotManager

class ScaraOne:

    def __init__(self, hardwareLibrary):

        # HW lib
        self.hardwareLibrary = hardwareLibrary

        # Hardware connections to stepper motor drivers
        # The two arm stepper drivers are Pololu A4988 modules and each has a step, direction and enable pin
        self.lowerArmStep = hardwareLibrary.Pin('Y9', hardwareLibrary.Pin.OUT_PP)
        self.lowerArmDirn = hardwareLibrary.Pin('Y10', hardwareLibrary.Pin.OUT_PP)
        self.lowerArmEnableBar = hardwareLibrary.Pin('Y3', hardwareLibrary.Pin.OUT_PP)
        self.upperArmStep = hardwareLibrary.Pin('Y11', hardwareLibrary.Pin.OUT_PP)
        self.upperArmDirn = hardwareLibrary.Pin('Y12', hardwareLibrary.Pin.OUT_PP)
        self.upperArmEnableBar = hardwareLibrary.Pin('Y4', hardwareLibrary.Pin.OUT_PP)

        # The vertical stepper drivers is also a Pololu A4988 module
        self.verticalStep = hardwareLibrary.Pin('Y6', hardwareLibrary.Pin.OUT_PP)
        self.verticalDirn = hardwareLibrary.Pin('Y5', hardwareLibrary.Pin.OUT_PP)
        self.verticalEnableBar = hardwareLibrary.Pin('Y7', hardwareLibrary.Pin.OUT_PP)

        # Pen control is via an electromagnet - apply power (logic 1) to push pen down
        self.penMag = hardwareLibrary.Pin('Y8', hardwareLibrary.Pin.OUT_PP)

        # Initially disable motor drivers
        self.lowerArmEnableBar.value(1)
        self.upperArmEnableBar.value(1)
        self.verticalEnableBar.value(1)

        # Enable motor drive for a period of time / Disable motor drive
        self.motorsEnabledFlag = False
        self.motorsEnabledLastMillis = 0
        self.motorsEnabledForMillis = 0

        # Pulse width and time between pulses for the stepper motors
        # Setting betweenPulsesUsecs to 300 is medium speed
        # Set betweenPulsesUsecs to a lower number to increase speed of arm movement
        self.pulseWidthUsecs = 1
        self.betweenPulsesUsecs = 300

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

        # Vertical steps per mm
        verticalStepsPerMM = 1

        # Adjest lower rotation angle to compensate for mismatched gears in my build
        # Shoulder gear has 60 teeth and elbow gear has 62
        # The result of this is that a 90 degree rotation of the upper arm
        # results in a ((90 * 62/60) - 90) = 1/30 degree turn of the lower arm
        # So we need to correct lower angle by 1/30th of upper angle
        shoulderGearMismatchFactor = 1 / 30

        # Leave motor drivers on for this amount of time after last move
        defaultMotorOnTimeMillis = 60000

        # Robot configuration
        self.robotConfiguration = {
            "origin": [0,0],
            "upperArm": {
                "armLen": 100,
                "stepsPerDegree": upperStepsPerDegree,
                "armMaxAngle": 90
                },
            "lowerArm": {
                "armLen": 100,
                "stepsPerDegree": lowerStepsPerDegree,
                "armMaxAngle": 160
                },
            "vertical": {
                "stepsPerMM": verticalStepsPerMM,
                "verticalTravelMax": 100,
            },
            "shoulderGearMismatchFactor": shoulderGearMismatchFactor,
            "defaultMotorOnTimeMillis": defaultMotorOnTimeMillis
        }

        # Create the ScaraRobotManager - which does movement calculations
        self.scaraRobotManager = ScaraRobotManager(self.robotConfiguration, self)

    # Set the current position of the arms as the home position
    def setHomeAsCurrentPos(self):
        return self.scaraRobotManager.setHomeAsCurrentPos()

    def setVerticalPosHome(self):
        return self.scaraRobotManager.setVerticalPosHome()

    # Move to an x,y point
    # Does not attempt to move in a completely straight line
    # But does move upper and lower arm proportionately - so if upper needs to move
    # 100 steps and lower to move 500 steps to reach destination then move the lower
    # arm 5 steps for every one step of the upper
    def moveTo(self, x, y):
        return self.scaraRobotManager.moveTo(x, y)

    # Move vertically
    def moveVertical(self, z):
        return self.scaraRobotManager.moveVertical(z)

    # Perform a single step of the upper arm
    def stepUpperArm(self, dirn):
        self.upperArmDirn.value(dirn)
        self.upperArmStep.value(1)
        self.hardwareLibrary.udelay(self.pulseWidthUsecs)
        self.upperArmStep.value(0)
        self.hardwareLibrary.udelay(self.betweenPulsesUsecs)

    # Perform a single step of the lower arm
    def stepLowerArm(self, dirn):
        self.lowerArmDirn.value(dirn)
        self.lowerArmStep.value(1)
        self.hardwareLibrary.udelay(self.pulseWidthUsecs)
        self.lowerArmStep.value(0)
        self.hardwareLibrary.udelay(self.betweenPulsesUsecs)

    # Perform a single step of the vertical motor
    def stepVertical(self, dirn):
        self.verticalDirn.value(dirn)
        self.verticalStep.value(1)
        self.hardwareLibrary.udelay(self.pulseWidthUsecs)
        self.verticalStep.value(0)
        self.hardwareLibrary.udelay(self.betweenPulsesUsecs)

    def enableMotorDrive(self, turnMotorsOn, timeLimitForDriveMillis):
        # Check if we are turning the motors off
        if not turnMotorsOn:
            self.lowerArmEnableBar.value(1)
            self.upperArmEnableBar.value(1)
            self.verticalEnableBar.value(1)
            self.motorsEnabledFlag = False
            return
        # Turn the motors on and remember when we did it
        self.lowerArmEnableBar.value(0)
        self.upperArmEnableBar.value(0)
        self.verticalEnableBar.value(0)
        self.motorsEnabledForMillis = timeLimitForDriveMillis
        self.motorsEnabledLastMillis = self.hardwareLibrary.millis()
        self.motorsEnabledFlag = True

    # Enable motor drive for a period of time
    def motorOnTimeLimitCheck(self):
        # Check if motors are on
        if not self.motorsEnabledFlag:
            return
        # Check if motors enabled indefinitely
        if self.motorsEnabledForMillis == 0:
            return
        # Check if time limit for motors being on has elapsed and turn off if so
        if self.hardwareLibrary.elapsed_millis(self.motorsEnabledLastMillis) > self.motorsEnabledForMillis:
            self.enableMotorDrive(False, 0)

    # Get robot configuration
    def getRobotConfig(self):
        return self.robotConfiguration