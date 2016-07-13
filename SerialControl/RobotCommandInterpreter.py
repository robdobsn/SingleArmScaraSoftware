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
# V0 ZZZZZ            ... move to Z position

class RobotCommandInterpreter:

    def __init__(self, robot, display):

        # Robot config
        self.robot = robot
        self.display = display
        robotConfig = robot.getRobotConfig()
        L1 = robotConfig["upperArm"]["armLen"]
        L2 = robotConfig["lowerArm"]["armLen"]
        ZMax = robotConfig["vertical"]["verticalTravelMax"]

        # The absolute min and max possible values for the pen position
        # These are based on the "bounding box" biggest rectangle around all possible positions
        # Due to polar nature of arm some value combinations will be invalid
        self.boundingBoxMinXValue = -(L1 + L2)
        self.boundingBoxMaxXValue = L1 + L2
        self.boundingBoxMinYValue = -L2
        self.boundingBoxMaxYValue = L1 + L2
        self.boundingBoxMinZValue = 0
        self.boundingBoxMaxZValue = ZMax
        self.motorOnTimeMillis = robotConfig["defaultMotorOnTimeMillis"]

        # Build up a command string from characters passed in
        self.serialCommandStr = ""

    def handleChar(self, ch):
        # Linefeed (\n or 0x0a) is used to signify end of command
        if ch == 0x0a:
            print("Command is:", self.serialCommandStr)
            self.interpCommand(self.serialCommandStr)
            self.serialCommandStr = ""
            return ""

        # Ignore CR
        if ch == 0x0d:
            return ""

        # Handle backspace to make it easier to control by typing directly into a
        # serial terminal program
        if ch == 127:
            if len(self.serialCommandStr) > 0:
                self.serialCommandStr = self.serialCommandStr[:-1]
            return "\x08\x20\x08"

        # Handle other chars by adding to the command string
        self.serialCommandStr += chr(ch)
        return(chr(ch))

    # Interpret a command line
    # More information on the command syntax at the top of this file
    def interpCommand(self, cmdStr):

        # Split the command line at the spaces
        splitStr = cmdStr.split()

        # G0 command - go to X,Y
        if splitStr[0] == 'G0':
            # Goto command
            x, xValidity = self.extractNum(splitStr, 1, self.boundingBoxMinXValue, self.boundingBoxMaxXValue)
            y, yValidity = self.extractNum(splitStr, 2, self.boundingBoxMinYValue, self.boundingBoxMaxYValue)
            if xValidity and yValidity:
                statusStr = "Go " + str(x) + ', ' + str(y)
                print(statusStr)
                self.display.showStatus(statusStr)
                self.robot.enableMotorDrive(True, self.motorOnTimeMillis)
                self.robot.moveTo(x,y)
            else:
                print("Move to cmd ", cmdStr, " failed")

        # V0 command - go to Z
        elif splitStr[0] == 'V0':
            # Goto Z command
            z, zValidity = self.extractNum(splitStr, 1, self.boundingBoxMinZValue, self.boundingBoxMaxZValue)
            if zValidity:
                statusStr = "Vertical " + str(z)
                print(statusStr)
                self.display.showStatus(statusStr)
                self.robot.enableMotorDrive(True, self.motorOnTimeMillis)
                self.robot.moveVertical(z)
            else:
                print("MoveVertical ", cmdStr, " failed")

        # S0 & S1 commands - move an arm a given number of steps
        elif splitStr[0] == 'S0' or splitStr[0] == 'S1':
            steps, stepsValidity = self.extractNum(splitStr, 1, -1000, 1000)
            if stepsValidity:
                self.robot.enableMotorDrive(True, self.motorOnTimeMillis)
                if splitStr[0] == 'S1':
                    for i in range(abs(steps)):
                        self.robot.stepLower(steps > 0)
                else:
                    for i in range(abs(steps)):
                        self.robot.stepUpper(steps < 0)
                statusStr = "Step " + "lower" if splitStr[0] == 0 else "upper" + str(steps)
                print(statusStr)
                self.display.showStatus(statusStr)
            else:
                print("Step cmd", cmdStr, "failed")

        # C0 command - set the current position to be the home position (calibrate)
        elif splitStr[0] == 'C0':
            self.robot.setHomeToCurrentPos()
            statusStr = "Calibrated"
            self.display.showStatus(statusStr)
            print(statusStr)

        # P0 & P1 - pen up and pen down0
        elif splitStr[0] == 'P0' or splitStr[0] == 'P1':
            isPenDown = (splitStr[0] == 'P1')
            self.robot.penMag.value(isPenDown)
            statusStr = "Pen Down" if isPenDown else "Pen Up"
            self.display.showStatus(statusStr)
            print(statusStr)

        # E0 & E1 - Disable and Enable motor drive
        elif splitStr[0] == 'E0' or splitStr[0] == 'E1':
            if splitStr[0] == 'E1':
                statusStr = "Enable Motors"
                timeLimit, timeLimitVaidity = self.extractNum(splitStr, 1, 0, 1000000000)
                if not timeLimitVaidity:
                    timeLimit = self.motorOnTimeMillis
                self.robot.enableMotorDrive(True, timeLimit)
            else:
                statusStr = "Disable Motors"
                self.robot.enableMotorDrive(False, 0)
            self.display.showStatus(statusStr)
            print(statusStr)

        # D0 - Set default motor on time
        elif splitStr[0] == 'D0':
            statusStr = "Motor on Time"
            timeLimit, timeLimitVaidity = self.extractNum(splitStr, 1, 0, 1000000000)
            if not timeLimitVaidity:
                return
            self.motorOnTimeMillis = timeLimit
            self.showStatus(statusStr)
            print(statusStr)

        # Unknown command
        else:
            statusStr = "Unknown " + cmdStr
            self.showStatus(statusStr)
            print(statusStr)

    # Extract a floating point number from a string ensuring no exceptions are thrown
    def extractNum(self, inStrList, listIdx, minVal, maxVal):
        if listIdx < 0 or listIdx >= len(inStrList):
            return 0, False
        try:
            val = float(inStrList[listIdx])
        except:
            return 0, False
        if val < minVal or val > maxVal:
            return 0, False
        return val, True

