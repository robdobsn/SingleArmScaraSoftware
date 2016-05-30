from pyb import UART

uart = UART(6, 115200)                         # init with given baudrate

cmdStr = ""
for i in range(1000):
	uart.write("hello ")
	pyb.delay(100)
	if uart.any() > 0:
		for i in range(uart.any()):
			ch = uart.readchar()
			if ch == 0x0d:
				print("Command is:", cmdStr)
				cmdStr = ""
				continue
			if ch == 0x0a:
				continue
			cmdStr += chr(ch)
			print (chr(ch))
