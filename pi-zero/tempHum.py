import board
import busio
import adafruit_si7021
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_si7021.SI7021(i2c)

while True:
	print("temperature: "'{:3f}'.format(sensor.temperature), "\t", "humidity: " '{:3f}'.format(sensor.relative_humidity))