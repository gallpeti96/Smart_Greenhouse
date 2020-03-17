import database
import SIMCOM7000E
import time
import threading
import board
from busio import I2C
import adafruit_bme680
import adafruit_tsl2591
from adafruit_ina219 import ADCResolution, BusVoltageRange, INA219
import paho.mqtt.client as mqtt
import netifaces as ni
import RPi.GPIO as GPIO
from datetime import datetime
import json
import subprocess
import sys

client = mqtt.Client("test")
# if "ppp0" in ni.interfaces():
#     ip = ni.ifaddresses('ppp0')[ni.AF_INET][0]['addr']                              # nb-iot modemen át
#     client.connect("152.66.248.149", port=51883, keepalive=600, bind_address=ip)
#     client.loop_start()
#     print("connected using NB-IoT")
# else:
#     client.connect("152.66.248.149", port=51883, keepalive=600)                         # alapértelmezett gw
#     client.loop_start()
#     print("connected")

i2c = I2C(board.SCL, board.SDA)
conn = database.create_db("test.sqlite")

GPIO.setmode(GPIO.BCM)
relay_GPIO = 26
GPIO.setup(relay_GPIO, GPIO.OUT)


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
        # client.publish("greenhouse/temperature", temp)

        humidity = round(bme680.humidity, 2)
        database.create_record(conn, "Measurements", ("humidity", humidity))
        # client.publish("greenhouse/humidity", humidity)

        pressure = round(bme680.pressure, 2)
        database.create_record(conn, "Measurements", ("pressure", pressure))
        # client.publish("greenhouse/pressure", pressure)

        altitude = round(bme680.altitude, 2)
        database.create_record(conn, "Measurements", ("altitude", altitude))
        # client.publish("greenhouse/altitude", altitude)

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
        # client.publish("greenhouse/gas", air_quality_score)

        # print(temp, "°C, ", hum, "%, ", pressure, "hPa, ", altitude, "m, ", air_quality_score)
        time.sleep(2)


def tsl():
    tsl = adafruit_tsl2591.TSL2591(i2c)
    while True:
        lux = round(tsl.lux, 2)
        database.create_record(conn, "Measurements", ("lux", lux))
        # client.publish("greenhouse/lux", lux)
        # print(lux, " lux")
        time.sleep(2)


def power():
    ina219 = INA219(i2c)
    while True:
        power = round(ina219.power, 4)
        database.create_record(conn, "Measurements", ("power", power))
        # client.publish("greenhouse/power", power)
        # print(power, " Watt")
        time.sleep(2)


def GPS():
    modem = SIMCOM7000E.serial_connect("/dev/ttyUSB2")
    SIMCOM7000E.gnss_poweron(modem)
    gps_dict = SIMCOM7000E.get_gnss_data(modem)
    print("Waiting for position", end=" ")
    while gps_dict['Fix status'] == '0':
        print(".", end=" ")
        time.sleep(2)
        gps_dict = SIMCOM7000E.get_gnss_data(modem)
    print("Positioned!")
    while True:
        gps_dict = SIMCOM7000E.get_gnss_data(modem)
        lat, long = gps_dict["Latitude"], gps_dict["Longitude"]
        position = lat + ", " + long
        database.create_record(conn, "Measurements", ("gps", position))
        # client.publish("greenhouse/gps", position)
        # print(lat, "\t", long)
        time.sleep(2)


def get_action():
    client.subscribe("greenhouse/relay")
    print("Subscripted to topic")
    def on_message(client, userdata, message):
        print("message received ", str(message.payload.decode("utf-8")))
        print("message topic=", message.topic)
        action = str(message.payload.decode("utf-8"))
        topic = message.topic
        if topic == "greenhouse/relay":
            if action == "0":
                GPIO.output(relay_GPIO, GPIO.LOW)
            else:
                GPIO.output(relay_GPIO, GPIO.HIGH)

    client.on_message = on_message


def send_msg():
    temp_dict = {}
    humidity_dict = {}
    pressure_dict = {}
    altitude_dict = {}
    gas_dict = {}
    light_dict = {}
    pos_dict = {}
    power_dict = {}
    last_sent = database.select_last_entry(conn)
    if last_sent[0][0] is None:
        last_sent = "0"
    else:
        last_sent = str(last_sent[0][0])
    not_sent = database.get_not_sends(conn, last_sent)
    for i in range(0, len(not_sent)):
        if not_sent[i][2] == 'temp':
            temp_dict[not_sent[i][1]] = not_sent[i][3]
        elif not_sent[i][2] == 'humidity':
            humidity_dict[not_sent[i][1]] = not_sent[i][3]
        elif not_sent[i][2] == 'pressure':
            pressure_dict[not_sent[i][1]] = not_sent[i][3]
        elif not_sent[i][2] == 'altitude':
            altitude_dict[not_sent[i][1]] = not_sent[i][3]
        elif not_sent[i][2] == 'gas':
            gas_dict[not_sent[i][1]] = not_sent[i][3]
        elif not_sent[i][2] == 'lux':
            light_dict[not_sent[i][1]] = not_sent[i][3]
        if not_sent[i][2] == 'pos':
            pos_dict[not_sent[i][1]] = not_sent[i][3]
        if not_sent[i][2] == 'power':
            power_dict[not_sent[i][1]] = not_sent[i][3]

    client.publish("greenhouse/temp_json", json.dumps(temp_dict))
    client.publish("greenhouse/humidity_json", json.dumps(humidity_dict))
    client.publish("greenhouse/pressure_json", json.dumps(pressure_dict))
    client.publish("greenhouse/altitude_json", json.dumps(altitude_dict))
    client.publish("greenhouse/gas_json", json.dumps(gas_dict))
    client.publish("greenhouse/lux_json", json.dumps(light_dict))
    client.publish("greenhouse/pos_json", json.dumps(pos_dict))
    client.publish("greenhouse/power_json", json.dumps(power_dict))
    print("Sent:\t", len(temp_dict), "temp", len(humidity_dict), "humi", len(pressure_dict), "pressure", len(gas_dict), "gas", len(light_dict), "light", len(pos_dict), "pos", len(power_dict), "power")
    timestamp = datetime.now()
    for i in range(0, len(not_sent)):
        database.create_record(conn, "Communications", (not_sent[i][1], timestamp))


def connect_to_network_and_send(sleep_sec):
    while True:
        try:
            modem = SIMCOM7000E.serial_connect("/dev/ttyUSB3")
        except:
            print("exit and connect a modem first")
             # ide kene valami amivel az egesz programot kinyirja, nem csak ezt a threadet !!!
        back = SIMCOM7000E.set_CFUN(modem, '1')
        print(back)
        connected = 0
        while connected == 0:
            time.sleep(.1)
            back = SIMCOM7000E.get_conn_status(modem)
            print(back)
            if '1' in back:
                connected = 1
                print("connected to network")
        SIMCOM7000E.serial_disconnect(modem)
        time.sleep(1)

        bashCommand = "sudo pon"
        process = subprocess.Popen(bashCommand.split(), stderr=subprocess.PIPE, stdout=subprocess.PIPE)
        output, error = process.communicate()
        print(output)
        process1 = subprocess.Popen(['ip', 'a'], stderr=subprocess.PIPE, stdout=subprocess.PIPE)
        output1, error1 = process1.communicate()
        if b"Connection terminated." in output or b'unrecognized option' in output:
            print("ajaj")
            raise KeyboardInterrupt
        elif b'ppp0' not in output1:
            print("jajajj")
            raise KeyboardInterrupt
        else:
            print("connceted to nbiot")

        ip = ni.ifaddresses('ppp0')[ni.AF_INET][0]['addr']  # nb-iot modemen át
        client.connect("152.66.248.149", port=51883, keepalive=600, bind_address=ip)
        client.loop_start()
        get_action()
        print("connected using NB-IoT")
        send_msg()
        print("Waiting for action. . .")
        time.sleep(15)

        client.unsubscribe("greenhouse/relay")
        client.loop_stop()
        client.disconnect()
        bashCommand = "sudo poff"
        process = subprocess.Popen(bashCommand.split(), stderr=subprocess.PIPE, stdout=subprocess.PIPE)
        output, error = process.communicate()
        print("ppp killed")
        modem = SIMCOM7000E.serial_connect("/dev/ttyUSB3")
        back = SIMCOM7000E.set_CFUN(modem, '0')
        print(back)
        SIMCOM7000E.serial_disconnect(modem)
        print("Going to sleep for: ", sleep_sec, "seconds")
        time.sleep(sleep_sec)


def main():
    print("starting....")
    try:
        sensor1 = threading.Thread(target=bme680)
        sensor1.name = "bme680"
        sensor1.daemon = True
        sensor2 = threading.Thread(target=tsl)
        sensor2.name = 'tsl'
        sensor2.daemon = True
        sensor3 = threading.Thread(target=GPS)
        sensor3.name = 'GPS'
        sensor3.daemon = True
        sensor4 = threading.Thread(target=power)
        sensor4.name = 'power'
        sensor4.daemon = True

        rel = threading.Thread(target=get_action)
        rel.name = 'relay'
        rel.daemon = True

        connect_and_send = threading.Thread(target=connect_to_network_and_send, args=(30,))
        connect_and_send.name = 'Communication'
        connect_and_send.daemon = True

        sensor1.start()
        time.sleep(.1)
        sensor2.start()
        time.sleep(.1)
        # sensor3.start()
        # time.sleep(.1)
        sensor4.start()
        # rel.start()
        connect_and_send.start()
        print("started")
        threading.Lock()
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        client.disconnect()
        GPIO.cleanup()
        print("killing everything 💀")


if __name__ == "__main__":
    main()

# test
