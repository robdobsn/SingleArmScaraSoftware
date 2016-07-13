
import time

class UART:

    testString = "G0 0 100\x0aV0 100\x0aV0 0\x0a"
    testStringPos = 0

    def __init__(self, port, baud):
        print("UART", port, "baud", baud)

    def write(self, ch):
        print(ch,end="")

    def any(self):
        return len(self.testString)-self.testStringPos

    def readchar(self):
        ch = ord(self.testString[self.testStringPos])
        self.testStringPos += 1
        return ch

class LCD:

    def __init__(self, code):
        print("LCD", code)

    def light(self, onOff):
        print("LCD light", onOff)

    def fill(self, val):
        print("LCD fill", val)

    def text(self, str1, x,y,z):
        print(str1, x, y, z)

    def show(self):
        return

def delay(time):
    return

def udelay(time):
    return

def millis():
    return time.time()

def elapsed_millis(lastTime):
    return int((time.time()-lastTime) * 1000)

class Pin:

    OUT_PP = 0

    def __init__(self, name, dirn):
        print("Pin", name, "dirn", dirn)

    def value(self, val):
        return

