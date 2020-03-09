from SIMCOM7000E import *
import serial

modem = serial_connect("/dev/ttyAMA0")

#gnss_poweron(modem)
#get_gnss_data(modem)
# gnss_poweroff(modem)

set_APN(modem, '"internet.iot.mt.co.hu"')
#set_APN_conf_mode(modem, 0)
#activate_network(modem)
#config_MQTT(modem, '"test.mosquitto.org"', '"gp"')
#get_MQTT_config(modem)
#time.sleep(1)
#connect_MQTT(modem)
#send_MQTT(modem, '"nbiothatgp"',"sziaadam")
serial_disconnect(modem)
