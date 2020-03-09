import database
import time
import threading
import board
from busio import I2C
import adafruit_bme680
import adafruit_tsl2591
import paho.mqtt.client as mqtt

client = mqtt.Client("test")
client.connect("152.66.248.149", port=51883, keepalive=600)

i2c = I2C(board.SCL, board.SDA)
conn = database.create_db("test.sqlite")

def gas_warmup(bme680, warmup_time):
    start_time = time.time()
    curr_time = time.time()
    burn_in_time = warmup_time
    burn_in_data = []
    print('Collecting gas resistance burn-in data for 5 mins\n')
    while curr_time - start_time < burn_in_time:
        curr_time = time.time()
        gas = bme680.gas
        burn_in_data.append(gas)
        print('Gas: {0} Ohms'.format(gas))
        time.sleep(1)
    gas_baseline = sum(burn_in_data[-50:]) / 50.0
    hum_baseline = 40.0
    hum_weighting = 0.25
    print('Gas baseline: {0} Ohms, humidity baseline: {1:.2f} %RH\n'.format(
        gas_baseline,
        hum_baseline))
    return gas_baseline, hum_baseline, hum_weighting


def bme680():
    bme680 = adafruit_bme680.Adafruit_BME680_I2C(i2c, debug=False)
    bme680.sea_level_pressure = 101325 / 100
    gas_baseline, hum_baseline, hum_weighting = gas_warmup(bme680, 0)

    while True:
        temp = round(bme680.temperature, 2)
        database.create_record(conn, "Measurements", ("temp", temp))
        client.publish("greenhouse/temperature", temp)

        humidity = round(bme680.humidity, 2)
        database.create_record(conn, "Measurements", ("humidity", humidity))
        client.publish("greenhouse/humidity", humidity)

        pressure = round(bme680.pressure, 2)
        database.create_record(conn, "Measurements", ("pressure", pressure))
        client.publish("greenhouse/pressure", pressure)

        altitude = round(bme680.altitude, 2)
        database.create_record(conn, "Measurements", ("altitude", altitude))
        client.publish("greenhouse/altitude", altitude)

        gas = bme680.gas
        gas_offset = gas_baseline - gas
        hum = humidity
        hum_offset = hum - hum_baseline
        # Calculate hum_score as the distance from the hum_baseline.
        if hum_offset > 0:
            hum_score = (100 - hum_baseline - hum_offset)
            hum_score /= (100 - hum_baseline)
            hum_score *= (hum_weighting * 100)
        else:
            hum_score = (hum_baseline + hum_offset)
            hum_score /= hum_baseline
            hum_score *= (hum_weighting * 100)
        # Calculate gas_score as the distance from the gas_baseline.
        if gas_offset > 0:
            gas_score = (gas / gas_baseline)
            gas_score *= (100 - (hum_weighting * 100))
        else:
            gas_score = 100 - (hum_weighting * 100)
        # Calculate air_quality_score.
        air_quality_score = round(hum_score + gas_score, 2)

        database.create_record(conn, "Measurements", ("gas", air_quality_score))
        client.publish("greenhouse/gas", air_quality_score)

        print(temp, hum, pressure, altitude, air_quality_score)
        time.sleep(2)


def tsl():
    tsl = adafruit_tsl2591.TSL2591(i2c)
    while True:
        lux = round(tsl.lux, 2)
        database.create_record(conn, "Measurements", ("lux", lux))
        client.publish("greenhouse/lux", lux)
        print(lux)
        time.sleep(2)


def main():
    print("starting....")
    try:
        sensor1 = threading.Thread(target=bme680)
        sensor1.daemon = True
        sensor2 = threading.Thread(target=tsl)
        sensor2.daemon = True

        sensor1.start()
        sensor2.start()
        print("started")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("killing everything 💀")


if __name__ == "__main__":
    main()
