import pyb
import mpr121

lowerArmStep = pyb.Pin('Y9', pyb.Pin.OUT_PP)
lowerArmDirn = pyb.Pin('Y10', pyb.Pin.OUT_PP)
upperArmStep = pyb.Pin('Y11', pyb.Pin.OUT_PP)
upperArmDirn = pyb.Pin('Y12', pyb.Pin.OUT_PP)

keybd = mpr121.MPR121(pyb.I2C(1, pyb.I2C.MASTER))
keybd.debounce(3,3)
for electr in range(4):
	keybd.threshold(electr, 50, 30)

pyb.LED(1).on()

lcd = pyb.LCD('X')
lcd.light(True)
pulseWidthUsecs = 1
betweenPulsesUsecs = 500

upperDegreesToMove = 15
# Degrees per step calculation
# Stepper motors move 1.8 degrees per full step
# In microstepping mode so 16 microsteps per step
# Motor shaft pulley has 20 teeth
# Upper arm pulley has 62 teeth
upperDegreesPerStep = (1.8/16)*(20/62)
upperStepsPerMove = upperDegreesToMove/upperDegreesPerStep
print("Upper steps per move", upperStepsPerMove)

lowerDegreesToMove = 90
# Degrees per step calculation
# Stepper motors move 1.8 degrees per full step
# In microstepping mode so 16 microsteps per step
# Motor shaft pulley has 20 teeth
# Upper arm pulley has 62 teeth
lowerDegreesPerStep = (1.8/16)*(20/62)
lowerStepsPerMove = lowerDegreesToMove/lowerDegreesPerStep
print("Lower steps per move", lowerStepsPerMove)

totalLower = 0
totalUpper = 0

while(True):
	lcd.fill(0)	
	lcd.text(str(keybd.elec_voltage(0)), 40,20,1)
	lcd.text(str(keybd.elec_voltage(1)), 0,20,1)
	lcd.text(str(keybd.elec_voltage(2)), 40,0,1)
	lcd.text(str(keybd.elec_voltage(3)), 0,0,1)
	lcd.show()
	if (keybd.elec_voltage(1) < 220):  # X Key
		lowerArmDirn.value(0)
		for i in range(lowerStepsPerMove):
			lowerArmStep.value(1)
			pyb.udelay(pulseWidthUsecs)
			lowerArmStep.value(0)
			pyb.udelay(betweenPulsesUsecs)
		totalLower += lowerStepsPerMove
	elif (keybd.elec_voltage(0) < 220):  # Y Key
		lowerArmDirn.value(1)
		for i in range(lowerStepsPerMove):
			lowerArmStep.value(1)
			pyb.udelay(pulseWidthUsecs)
			lowerArmStep.value(0)
			pyb.udelay(betweenPulsesUsecs)
		totalLower -= lowerStepsPerMove
	elif (keybd.elec_voltage(2) < 220):   # B Key
		upperArmDirn.value(0)
#		lowerArmDirn.value(1)
		for i in range(upperStepsPerMove):
			upperArmStep.value(1)
#			lowerArmStep.value(1)
			pyb.udelay(pulseWidthUsecs)
			upperArmStep.value(0)
#			lowerArmStep.value(0)
			pyb.udelay(betweenPulsesUsecs)
		totalUpper += upperStepsPerMove
	elif (keybd.elec_voltage(3) < 220):    # A Key
		upperArmDirn.value(1)
#		lowerArmDirn.value(0)
		for i in range(upperStepsPerMove):
			upperArmStep.value(1)
#			lowerArmStep.value(1)
			pyb.udelay(pulseWidthUsecs)
			upperArmStep.value(0)
#			lowerArmStep.value(0)
			pyb.udelay(betweenPulsesUsecs)
		totalUpper -= upperStepsPerMove
	print("TotalUpper " + str(totalUpper) + " TotalLower " + str(totalLower))
	pyb.delay(1000)

# for x in range(-80, 128):
#     lcd.fill(0)
#     lcd.text('Hello uPy!', x, 10, 1)
#     lcd.show()
#     pyb.delay(25)
    