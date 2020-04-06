import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)   
GPIO.setup(4, GPIO.IN)


try: 
	while True:
		if(GPIO.input(4) == 0):
			print("Beam Broken")
			time.sleep(0.05)
		if(GPIO.input(4) == 1):
			print("Solid")
			time.sleep(0.05)
except KeyboardInterrupt:
	pass