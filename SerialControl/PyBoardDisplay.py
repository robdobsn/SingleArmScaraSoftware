
class PyBoardDisplay:

    def __init__(self, HardwareLibrary, enable):
        # LCD - only used for visual confirmation of activity
        # Set lcdIsUsed to False if not required
        self.lcdIsUsed = enable
        if self.lcdIsUsed:
            self.lcd = HardwareLibrary.LCD('X')
            self.lcd.light(True)

    def showStatus(self, statusStr):
        if self.lcdIsUsed:
            self.lcd.fill(0)
            self.lcd.text("SCARA Serial", 0, 0, 1)
            self.lcd.text(statusStr, 0, 20, 1)
            self.lcd.show()

