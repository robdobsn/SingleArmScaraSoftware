import pyb
import mpr121

penMag = pyb.Pin('Y8', pyb.Pin.OUT_PP)

for i in range(2):
	penMag.value(1)
	pyb.delay(5000)
	penMag.value(0)
	pyb.delay(1000)

