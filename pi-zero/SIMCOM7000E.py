import serial
import time
import sys


def serial_connect(port):
    print("Connecting to modem on port: ", port)
    ser = serial.Serial(port, 115200, xonxoff=True, rtscts=True, dsrdtr=True)
    ser.flushInput()
    time.sleep(1)
    print("Modem connected to send AT!")
    sys.stdout.flush()
    return ser


def serial_disconnect(ser):
    ser.close()
    print("Modem disconnected from AT port.")
    sys.stdout.flush()


def send_at(ser, command, back, timeout=2.0):
    rec_buff = ''
    ser.write((command + '\r').encode())
    # print(command, file=sys.stderr)
    time.sleep(timeout)
    if ser.inWaiting():
        time.sleep(0.01)
        rec_buff = ser.read(ser.inWaiting())
        rec_buff = rec_buff.decode().replace('\r', '')
    if rec_buff != '':
        #     if back not in rec_buff:
        #         print(command + ' ERROR', file=sys.stderr)
        #         print(command + ' back:\t' + rec_buff, file=sys.stderr)
        #     else:
        #         print(rec_buff, file=sys.stderr)
        return rec_buff
    else:
        #     print('ERROR', file=sys.stderr)
        return 0


def set_CFUN(ser, input):
    return send_at(ser, 'AT+CFUN='+input, 'OK', 3.0)


def get_conn_status(ser):
    return send_at(ser, 'AT+CGATT?', 'OK', 0.5)

def get_CSQ(ser):
    return send_at(ser, 'AT+CSQ', 'OK', 0.5)

def gnss_poweron(ser):
    send_at(ser, 'AT+CGNSPWR=1', 'OK')
    send_at(ser, 'AT+CGNSPORT=4', 'OK')

    print("GNSS power on")
    sys.stdout.flush()


def gnss_poweroff(ser):
    send_at(ser, 'AT+CGNSPWR=0', 'OK')
    print("GNSS power off")
    sys.stdout.flush()


def get_gnss_data(ser):
    answer = send_at(ser, 'AT+CGNSINF', '+CGNSINF: ')
    time.sleep(0.1)
    answer = answer[answer.find(" ") + 1:]
    answer = answer[:answer.find("\r")]

    splitted = [x.strip() for x in answer.split(',')]
    gps_dict = {"GNSS run status": splitted[0],
                "Fix status": splitted[1],
                "UTC date & Time": splitted[2],
                "Latitude": splitted[3],
                "Longitude": splitted[4],
                "MSL Altitude": splitted[5],
                "Speed Over Ground": splitted[6],
                "Course Over Ground": splitted[7],
                "Fix Mode": splitted[8],
                "Reserved1": splitted[9],
                "HDOP": splitted[10],
                "PDOP": splitted[11],
                "VDOP": splitted[12],
                "Reserved2": splitted[13],
                "GPS Satellites in View": splitted[14],
                "GNSS Satellites Used": splitted[15],
                "GLONASS Satellites in View": splitted[16],
                "Reserved3": splitted[17],
                "C/N0 max": splitted[18],
                "HPA": splitted[19],
                "VPA": splitted[20]
                }
    # print("{:<25} {:<50} ".format('Parameter', 'Value'))
    # for k, v in gps_dict.items():
    #    parameter = v
    #   print("{:<25} {:<50} ".format(k, parameter))
    sys.stdout.flush()
    return gps_dict


def set_APN(ser, APN):
    answer = send_at(ser, 'AT+CSTT?', "OK")
    if APN.replace('"', "") in answer:
        print("APN is already set up")
    else:
        send_at(ser, 'AT+CSTT=' + APN, "OK")
        print("APN set to:", APN)
    sys.stdout.flush()


def set_APN_conf_mode(ser, mode):
    send_at(ser, 'AT+CAPNMODE=' + str(mode), "OK")
    sys.stdout.flush()


def activate_network(ser):
    answer = send_at(ser, 'AT+CNACT?', "OK")
    if "1" in answer:
        print("Network is already activated")
    else:
        send_at(ser, 'AT+CNACT=1', "OK")
        time.sleep(1)
        print("Network activated")
    sys.stdout.flush()


def get_MQTT_config(ser):
    send_at(ser, 'AT+SMCONF?', "OK")


def config_MQTT(ser, broker, id):
    # send_at(ser, 'AT+SMCONF="CLIENTID",'+id, "OK")
    send_at(ser, 'AT+SMCONF="URL",' + broker, "OK")
    print("MQTT configured")
    sys.stdout.flush()


def connect_MQTT(ser):
    print("Connecting to MQTT broker . . . ", end=" ")
    answer = send_at(ser, "AT+SMCONN", "OK", timeout=10)
    # send_at(ser, "AT+SMSTATE?", "OK", timeout=5)
    while 'ERROR' in answer:
        send_at(ser, "AT+SMDISC", "OK", timeout=2)
        time.sleep(2)
        answer = send_at(ser, "AT+SMCONN", "OK", timeout=10)
    print("connected")


def send_MQTT(ser, topic, message):
    length = len(message) + 1
    send_at(ser, 'AT+SMPUB=' + topic + "," + str(length) + ",1,1", "OK")
    send_at(ser, message, "OK")
