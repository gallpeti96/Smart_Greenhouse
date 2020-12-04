import database
import SIMCOM7000E
import time
import threading
import sys
import board
from busio import I2C
import adafruit_bme680
import adafruit_tsl2591
from adafruit_ina219 import ADCResolution, BusVoltageRange, INA219
import adafruit_si7021

import paho.mqtt.client as mqtt
import netifaces as ni
import RPi.GPIO as GPIO
from datetime import datetime
import json
import subprocess
from termcolor import colored

client = mqtt.Client(client_id="RPi0_1")
# if "ppp0" in ni.interfaces():
#     ip = ni.ifaddresses('ppp0')[ni.AF_INET][0]['addr']                              # nb-iot modemen at
#     client.connect("152.66.248.149", port=51883, keepalive=600, bind_address=ip)
#     client.loop_start()
#     print("connected using NB-IoT")
# else:
#     client.connect("152.66.248.149", port=51883, keepalive=600)                         # default gw
#     client.loop_start()
#     print("connected")

i2c = I2C(board.SCL, board.SDA)
conn = database.create_db("test.sqlite")

try:
        modem = SIMCOM7000E.serial_connect("/dev/ttyUSB2")
except:
    print(colored("exit and connect a modem first", 'red'))
    sys.exit(0)

GPIO.setmode(GPIO.BCM)
pump_GPIO = 20
GPIO.setup(pump_GPIO, GPIO.OUT)
GPIO.output(pump_GPIO, GPIO.HIGH)

vent_GPIO = 16
GPIO.setup(vent_GPIO, GPIO.OUT)
GPIO.output(vent_GPIO, GPIO.HIGH)

Rled_GPIO = 0
GPIO.setup(Rled_GPIO, GPIO.OUT)
#GPIO.output(Rled_GPIO, GPIO.HIGH)

Gled_GPIO = 0
GPIO.setup(Gled_GPIO, GPIO.OUT)
#GPIO.output(Gled_GPIO, GPIO.HIGH)

Bled_GPIO = 0
GPIO.setup(Bled_GPIO, GPIO.OUT)
#GPIO.output(Bled_GPIO, GPIO.HIGH)

measurement_interval = 1
sectorId = 2
lock = threading.Lock()


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
        #print('Gas: {0} Ohms'.format(gas))
        time.sleep(1)
    gas_baseline = sum(burn_in_data[-50:]) / 50.0
    hum_baseline = 40.0
    hum_weighting = 0.25
    print('Gas baseline: {0} Ohms, humidity baseline: {1:.2f} %RH\n'.format(
        gas_baseline,
        hum_baseline))
    return gas_baseline, hum_baseline, hum_weighting


def bme680():
    print(colored("BME680 started. . .", 'green'))
    bme680 = adafruit_bme680.Adafruit_BME680_I2C(i2c, debug=False)
    bme680.sea_level_pressure = 101325 / 100
    gas_baseline, hum_baseline, hum_weighting = gas_warmup(bme680, 0)
    while True:
        temp = round(bme680.temperature, 2)
        humidity = round(bme680.humidity, 2)
        pressure = round(bme680.pressure, 2)
        altitude = round(bme680.altitude, 2)
        with lock:
            database.create_record(conn, "Measurements", ("temp", temp))
            # client.publish("greenhouse/temperature", temp)

            database.create_record(conn, "Measurements", ("humidity", humidity))
            # client.publish("greenhouse/humidity", humidity)

            database.create_record(conn, "Measurements", ("pressure", pressure))
            # client.publish("greenhouse/pressure", pressure)

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
        with lock:
            database.create_record(conn, "Measurements", ("gas", air_quality_score))
            # client.publish("greenhouse/gas", air_quality_score)

            # print(temp, "°C, ", hum, "%, ", pressure, "hPa, ", altitude, "m, ", air_quality_score)
        time.sleep(measurement_interval)


def si7021():
    print(colored("SI7021 started. . .", 'green'))
    si7021 = adafruit_si7021.SI7021(i2c)
    while True:
        temp = si7021.temperature
        humidity = si7021.relative_humidity
        with lock:
            database.create_record(conn, "Measurements", ("temp", temp))
            database.create_record(conn, "Measurements", ("humidity", humidity))
        time.sleep(measurement_interval)


def tsl():
    print(colored("TSL2591 started. . .", 'green'))
    tsl = adafruit_tsl2591.TSL2591(i2c)
    while True:
        lux = round(tsl.lux, 2)
        with lock:
            database.create_record(conn, "Measurements", ("lux", lux))
        # client.publish("greenhouse/lux", lux)
        # print(lux, " lux")
        time.sleep(measurement_interval)


def soil_moisture():
    print(colored("Soil moisture started. . .", 'green'))

    data = 0

    def tick(clk, ts):
        time.sleep(0.00005 * ts)
        GPIO.output(clk, GPIO.HIGH)
        time.sleep(0.00005 * ts)
        GPIO.output(clk, GPIO.LOW)

    csPIN = 18
    clkPIN = 24
    dinPIN = 23

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(csPIN, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(clkPIN, GPIO.OUT, initial=GPIO.LOW)
    # security offset
    ts = 1.05
    prev = data
    GPIO.setup(dinPIN, GPIO.IN)
    GPIO.output(csPIN, GPIO.LOW)
    sum = 0
    while True:
        GPIO.output(csPIN, GPIO.LOW)
        tick(clkPIN, ts)
        for i in range(8):
            tick(clkPIN, ts)
            data = (data << 1) | GPIO.input(dinPIN)
        GPIO.output(csPIN, GPIO.HIGH)
        moisturePercent = (255 - data) / 255 * 100
        #print(data, '\t', '{:3f}'.format(moisturePercent))
        #if not prev == data:
            #print(data, '\t', '{:3f}'.format(moisturePercent))
        prev = data
        with lock:
            database.create_record(conn, "Measurements", ("moisture", moisturePercent))
        data = 0
        time.sleep(measurement_interval)


def power():
    print(colored("INA219 started. . .", 'green'))
    ina219 = INA219(i2c, addr=0x41)
    powers = []
    while True:
        for i in range(0, 300):
            powers.append(ina219.power)
            time.sleep(.1)
        avg_power_consumption = sum(powers)/len(powers)
        with lock:
            database.create_record(conn, "Measurements", ("power", avg_power_consumption))
        powers = []
        # client.publish("greenhouse/power", avg_power_consumption)
        # print(avg_power_consumption, " Watt/hour")


def powerup_GPS():
    print(colored("GPS positioning started. . .", 'green'))
    SIMCOM7000E.gnss_poweron(modem)
    gps_dict = SIMCOM7000E.get_gnss_data(modem)
    print("Waiting for position", end=" ")
    while gps_dict['Fix status'] == '0':
        print(".", end=" ")
        time.sleep(2)
        gps_dict = SIMCOM7000E.get_gnss_data(modem)
    print("Positioned!")


def get_signal_strength():
    CSQ = SIMCOM7000E.get_CSQ(modem)
    signal_strength = int(CSQ[7:9])
    if signal_strength == 99:
        print(colored("Somethings wrong woth signal strength", "red"))
    elif signal_strength == 0:
        dBm = "-115 or less"
    elif signal_strength == 1:
        dBm = -111
    elif signal_strength == 31:
        dBm = "-52 or greater"
    else:
        dBm = -114+signal_strength*2

    return dBm


def get_action():
    client.subscribe("greenhouse/pump"+str(sectorId))
    print(colored("Subscripted to topic: " + "greenhouse/pump"+str(sectorId), 'green'))
    client.subscribe("greenhouse/vent"+str(sectorId))
    print(colored("Subscripted to topic: " + "greenhouse/vent"+str(sectorId), 'green'))
    client.subscribe("greenhouse/light1"+str(sectorId))
    print(colored("Subscripted to topic: " + "greenhouse/light1"+str(sectorId), 'green'))
    client.subscribe("greenhouse/light2"+str(sectorId))
    print(colored("Subscripted to topic: " + "greenhouse/light2"+str(sectorId), 'green'))


    def on_message(client, userdata, message):
        print("message received ", str(message.payload.decode("utf-8")))
        print("message topic=", message.topic)
        action = str(message.payload.decode("utf-8"))
        topic = message.topic
        if topic == "greenhouse/pump"+str(sectorId):
            if action == "0":
                GPIO.output(pump_GPIO, GPIO.HIGH)
            else:
                GPIO.output(pump_GPIO, GPIO.LOW)
        if topic == "greenhouse/vent"+str(sectorId):
            if action == "0":
                GPIO.output(vent_GPIO, GPIO.HIGH)
            else:
                GPIO.output(vent_GPIO, GPIO.LOW)
        if topic == "greenhouse/light1":
            print("light1")
        if topic == "greenhouse/light2":
            print("light2")
    client.on_message = on_message


def send_msg():
    temp_dict = {"sensorId": sectorId, "timestamp": [], "value": []}

    humidity_dict = {"sensorId": sectorId, "timestamp": [], "value": []}

    pressure_dict = {"sensorId": sectorId, "timestamp": [], "value": []}

    altitude_dict = {"sensorId": sectorId, "timestamp": [], "value": []}

    gas_dict = {"sensorId": sectorId, "timestamp": [], "value": []}

    light_dict = {"sensorId": sectorId, "timestamp": [], "value": []}

    moisture_dict = {"sensorId": sectorId, "timestamp": [], "value": []}

    power_dict = {"sensorId": sectorId, "timestamp": [], "value": []}

    last_sent = database.select_last_entry(conn)
    if last_sent[0][0] is None:
        last_sent = "0"
    else:
        last_sent = str(last_sent[0][0])
    not_sent = database.get_not_sends(conn, last_sent)
    for i in range(0, len(not_sent)):
        if not_sent[i][2] == 'temp':
            temp_dict['timestamp'].append(not_sent[i][1])
            temp_dict['value'].append(not_sent[i][3])
        elif not_sent[i][2] == 'humidity':
            humidity_dict['timestamp'].append(not_sent[i][1])
            humidity_dict['value'].append(not_sent[i][3])
        elif not_sent[i][2] == 'pressure':
            pressure_dict['timestamp'].append(not_sent[i][1])
            pressure_dict['value'].append(not_sent[i][3])
        elif not_sent[i][2] == 'altitude':
            altitude_dict['timestamp'].append(not_sent[i][1])
            altitude_dict['value'].append(not_sent[i][3])
        elif not_sent[i][2] == 'gas':
            gas_dict['timestamp'].append(not_sent[i][1])
            gas_dict['value'].append(not_sent[i][3])
        elif not_sent[i][2] == 'lux':
            light_dict['timestamp'].append(not_sent[i][1])
            light_dict['value'].append(not_sent[i][3])
        elif not_sent[i][2] == 'moisture':
            moisture_dict['timestamp'].append(not_sent[i][1])
            moisture_dict['value'].append(not_sent[i][3])
        if not_sent[i][2] == 'power':
            power_dict['timestamp'].append(not_sent[i][1])
            power_dict['value'].append(not_sent[i][3])

    dBm = get_signal_strength()
    client.publish("greenhouse/signal_strength"+str(sectorId), dBm)
    time.sleep(.2)

    gps_dict = SIMCOM7000E.get_gnss_data(modem)
    lat, long = gps_dict["Latitude"], gps_dict["Longitude"]
    if long and lat != 0:
        position = lat + ", " + long
        database.create_record(conn, "Measurements", ("gps", position))
        pos = {"name": "Sector" + str(sectorId), "lat": lat, "lon": long}
        client.publish("greenhouse/gps", json.dumps(pos))

    client.publish("test", "hahaha")
    print("hahaha to test")
    client.publish("greenhouse/temp_json"+str(sectorId), json.dumps(temp_dict))
    time.sleep(.2)
    client.publish("greenhouse/humidity_json"+str(sectorId), json.dumps(humidity_dict))
    time.sleep(.2)
    client.publish("greenhouse/pressure_json"+str(sectorId), json.dumps(pressure_dict))
    time.sleep(.2)
    client.publish("greenhouse/altitude_json"+str(sectorId), json.dumps(altitude_dict))
    time.sleep(.2)
    client.publish("greenhouse/gas_json"+str(sectorId), json.dumps(gas_dict))
    time.sleep(.2)
    client.publish("greenhouse/lux_json"+str(sectorId), json.dumps(light_dict))
    time.sleep(.2)
    client.publish("greenhouse/moisture_json"+str(sectorId), json.dumps(moisture_dict))
    time.sleep(.2)
    client.publish("greenhouse/power_json"+str(sectorId), json.dumps(power_dict))

    print("NB-IoT network signal strength: ", dBm, "dBm")
    print("Sent:\t", len(temp_dict['timestamp']), "temp", len(humidity_dict['timestamp']), "humi", len(pressure_dict['timestamp']), "pressure", len(gas_dict['timestamp']), "gas", len(light_dict['timestamp']), "light", len(moisture_dict['timestamp']), "moisture",  len(power_dict['timestamp']), "power")
    timestamp = time.time()
    start = time.time()
    with lock:
        database.create_record(conn, "Communications", (not_sent, timestamp))
    stop = time.time()
    print("logged in ", stop-start, "seconds")


def connect_to_network_and_send(sleep_sec):
    print(colored("Communication thread started. . .", 'green'))
    while True:
        # back = SIMCOM7000E.set_CFUN(modem, '0')
        back = SIMCOM7000E.set_CFUN(modem, '1')
        print(back)
        connected = 0
        try_number = 0
        while connected == 0:
            time.sleep(.1)
            back = SIMCOM7000E.get_conn_status(modem)
            # print(back)
            if '1' in str(back):
                connected = 1
                print("\rTried to connect to network:", try_number+1, "times")
                print("connected to network")
            else:
                try_number += 1
                print("\rTried to connect to network:", try_number, "times", end="")
            if try_number == 60:
                print(colored("cannot connect to the network, restarting communication thread", 'red'))
                back = SIMCOM7000E.set_CFUN(modem, '0')
                print(back)
                sys.exit(0)
        time.sleep(1)

        bashCommand = "sudo pon"
        process = subprocess.Popen(bashCommand.split(), stderr=subprocess.PIPE, stdout=subprocess.PIPE)
        output, error = process.communicate()
        print(output)
        time.sleep(1)
        process1 = subprocess.Popen(['ip', 'a'], stderr=subprocess.PIPE, stdout=subprocess.PIPE)
        output1, error1 = process1.communicate()
        if b"Connection terminated." in output or b'unrecognized option' in output:
            print("ajaj")
            process3 = subprocess.Popen(['sudo', 'poff'], stderr=subprocess.PIPE, stdout=subprocess.PIPE)
            output3, error3 = process1.communicate()
            print(output3, error3)
            sys.exit(0)
        elif b'ppp0' not in output1:
            print("jajajj")
            process3 = subprocess.Popen(['sudo', 'poff'], stderr=subprocess.PIPE, stdout=subprocess.PIPE)
            output3, error3 = process1.communicate()
            print(output3, error3)
            sys.exit(0)
        else:
            print(colored("Connected to NB-IoT network", 'green', attrs=['bold']))
        time.sleep(1)
        ip = ni.ifaddresses('ppp0')[ni.AF_INET][0]['addr']  # nb-iot modemen át
        print(ip)
        eredmeny = client.connect("gall-peter96.asuscomm.com", port=51883, bind_address=ip)
        print(eredmeny)
        client.loop_start()
        get_action()
        print("connected using NB-IoT")
        send_msg()
        print("Waiting for action. . .")
        time.sleep(30)

        client.unsubscribe("greenhouse/pump"+str(sectorId))
        client.loop_stop()
        client.disconnect()
        bashCommand = "sudo poff"
        process = subprocess.Popen(bashCommand.split(), stderr=subprocess.PIPE, stdout=subprocess.PIPE)
        output, error = process.communicate()
        time.sleep(2)
        print(colored("PPP stopped", 'yellow'))
        # back = SIMCOM7000E.set_CGATT(modem, '0')
        print(back)
        print(colored("Detached from network.", "yellow"))
        print(colored("Going to sleep for: " + str(sleep_sec) + "seconds", 'magenta'))
        time.sleep(sleep_sec)


def main():
    print(colored("STARTING....", 'green'))
    try:
        #sensor3 = threading.Thread(target=powerup_GPS())
        #sensor3.name = 'powerup_GPS'
        #sensor3.daemon = True

        #sensor1 = threading.Thread(target=bme680)
        #sensor1.name = "bme680"
        #sensor1.daemon = True
        sensor1 = threading.Thread(target=si7021)
        sensor1.name = "si7021"
        sensor1.daemon = True
        sensor2 = threading.Thread(target=tsl)
        sensor2.name = 'tsl'
        sensor2.daemon = True
        sensor5 = threading.Thread(target=soil_moisture)
        sensor5.name = 'moisture'
        sensor5.daemon = True
        sensor4 = threading.Thread(target=power)
        sensor4.name = 'power'
        sensor4.daemon = True

        rel = threading.Thread(target=get_action)
        rel.name = 'pump'
        rel.daemon = True

        connect_and_send = threading.Thread(target=connect_to_network_and_send, args=(30,))
        connect_and_send.name = 'Communication'
        connect_and_send.daemon = True

        #sensor3.start()
        time.sleep(.1)

        sensor1.start()
        time.sleep(.1)
        sensor2.start()
        time.sleep(.1)
        sensor4.start()
        sensor5.start()
        time.sleep(.1)
        # rel.start()
        connect_and_send.start()
        while True:
            time.sleep(5)

            if not sensor1.is_alive():
                sensor1 = None
                sensor1 = threading.Thread(target=si7021)
                sensor1.name = "si7021"
                sensor1.daemon = True
                sensor1.start()
            elif not sensor2.is_alive():
                sensor2 = None
                sensor2 = threading.Thread(target=tsl)
                sensor2.name = 'tsl'
                sensor2.daemon = True
                sensor2.start()
            elif not sensor4.is_alive():
                sensor4 = None
                sensor4 = threading.Thread(target=power)
                sensor4.name = 'power'
                sensor4.daemon = True
                sensor4.start()
            elif not sensor5.is_alive():
                sensor5 = None
                sensor5 = threading.Thread(target=soil_moisture)
                sensor5.name = 'moisture'
                sensor5.daemon = True
                sensor5.start()

            elif not connect_and_send.is_alive():
                print('restarting communication thread')
                connect_and_send = None
                time.sleep(5)
                connect_and_send = threading.Thread(target=connect_to_network_and_send, args=(30,))
                connect_and_send.name = 'Communication'
                connect_and_send.daemon = True
                connect_and_send.start()

    except KeyboardInterrupt:
        client.unsubscribe("greenhouse/pump")
        client.disconnect()

        bashCommand = "sudo poff"
        process = subprocess.Popen(bashCommand.split(), stderr=subprocess.PIPE, stdout=subprocess.PIPE)
        output, error = process.communicate()

        SIMCOM7000E.serial_disconnect(modem)
        GPIO.cleanup()
        print("killing everything💀")


if __name__ == "__main__":
    main()

# test
