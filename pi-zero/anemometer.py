import time 
import RPi.GPIO as GPIO

data = 0

def tick(clk, ts):
  time.sleep(0.00005 * ts)
  GPIO.output(clk, GPIO.HIGH)
  time.sleep(0.00005 * ts)
  GPIO.output(clk, GPIO.LOW)


csPIN = 18
clkPIN = 23
dinPIN = 24

GPIO.setmode(GPIO.BCM)
GPIO.setup(csPIN, GPIO.OUT, initial = GPIO.HIGH)
GPIO.setup(clkPIN, GPIO.OUT, initial = GPIO.LOW)

#security offset

ts = 1.05 

prev = data
GPIO.setup(dinPIN, GPIO.IN)
GPIO.output(csPIN, GPIO.LOW)

median = []
sum = 0

try:
  while True:
    GPIO.output(csPIN, GPIO.LOW)
    tick(clkPIN, ts)
    for i in range(8):
      tick(clkPIN, ts)
      data = (data << 1) | GPIO.input(dinPIN)
    GPIO.output(csPIN, GPIO.HIGH)
    print(data, "\t", '{:3f}'.format(data * (32.4 / 255)))
    if not prev == data:
        print(data,"\t", '{:3f}'.format(data * (32.4 / 255)))

    median.append(data * (32.4 / 255))
    prev = data
    data = 0
    if len(median) == 10:
    	median.sort()
    	print("median: ", median[2])
    	median = []

    time.sleep(1)
except KeyboardInterrupt:
  GPIO.cleanup()

