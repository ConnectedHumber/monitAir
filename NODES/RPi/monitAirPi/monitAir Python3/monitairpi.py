#!/usr/bin/python3
'''
Full Payload Structure
    payload = {"timestamp":-1,
               "dev": CHAtP-dddddddd-x,
               "temp": ##.#,
               "humidity": ##,
               "pressure": ####,
               "PM10" : ###,
               "PM25" : ###,
               "iaq": ###,
               "dsTemp": ##.#,
               "long": ##.#####,
               "lat": ##.#####,
               "alt": ###
               }
'''

from time import sleep, time
from datetime  import datetime, timezone
import subprocess
import mqttclient  # local module, also need : sudo pip3 install paho-mqtt
import json
from statistics import median
import serial  # sudo pip3 install pyserial #GPIO/USB Serial read


######                                               ######
# ConnectedHumber : MonitAIR project authentication info. #
######                                               ######

MQTT_USERNAME  = "connectedhumber"
MQTT_PASSWORD  = "3fds8gssf6"
MQTT_CLIENT_ID = "" #CHAtp-dddddddd-x blank for autogenerate
MQTT_HOSTNAME = "mqtt.connectedhumber.org"
MQTT_PORT = 1883
MQTT_TOPIC = "airquality/data"

# Below enables sensors, set to "" to diasble or provide port/file name 
GNSS_PORT = ""  # "/dev/ttyACM0"
SDS_PORT = "/dev/serial0"
BME_BSEC = "./bsec_bme680"

W1THERM = True

DEVICE_TYPE = "S"  # "M" for mobile
LOCATION_NUMBER = "1" # increase if location is moved for the same device

if GNSS_PORT is not "":
    import pynmea2  # suso pip3 pynmea2     NMEA GNSS splitter

if SDS_PORT is not "":
    from sds011 import SDS011  # local module for SDS011/021 PM sensors

if W1THERM is True:
    from w1thermsensor import W1ThermSensor  #sudo pip3 install w1thermsensor     DS18B20 


payload = {}

#general
loop_delay = 0.05 #seconds
reading_delay = 60*6 # time between sending data
timestart = 0
waittime = 55  #initial estimate for time to perform all measurments

def get_serial():
    # Extract serial from cpuinfo file
    cpuserial = "N0N0N0N0"
    try:
        f = open('/proc/cpuinfo','r')
        for line in f:
            if line[0:6]=='Serial':
                cpuserial = line[18:26]
        f.close()
    except:
        cpuserial = "ERROR000"

    return cpuserial


def get_clientid():
    clientid = list("CHAtR-dddddddd-x")
    clientid[3] = DEVICE_TYPE
    clientid[6:14] = get_serial()
    clientid[15] = LOCATION_NUMBER

    return "".join(clientid)


def sds_start():
#SDS0x1
    print("about to create")
    # Now we have some details about it
    print("SDS011 sensor info:")
    print("Device ID: ", sds_sensor.device_id)
    print("Device firmware: ", sds_sensor.firmware)
    print("Current device cycle (0 is permanent on): ", sds_sensor.dutycycle)
    print(sds_sensor.workstate)
    print(sds_sensor.reportmode)
    sds_sensor.reset()
    sds_sensor.workstate = SDS011.WorkStates.Measuring

def bme680_go():
    for line in iter(proc.stdout.readline, ''):
        lineJSON = json.loads(line.decode("utf-8")) # process line-by-line
        lineDict = dict(lineJSON)

        listIAQ_Accuracy.append(int(lineDict['IAQ_Accuracy']))
        listPressure.append(float(lineDict['Pressure']))
        listTemperature.append(float(lineDict['Temperature']))
        listIAQ.append(float(lineDict['IAQ']))
        listHumidity.append(float(lineDict['Humidity']))

        #client.loop()
        if len(listIAQ_Accuracy) == 10:
            # generate the median for each value

            Pressure = median(listPressure)
            Temperature = median(listTemperature)
            IAQ = median(listIAQ)
            Humidity = median(listHumidity)

            # clear lists
            listIAQ_Accuracy.clear()
            listPressure.clear()
            listTemperature.clear()
            listIAQ.clear()
            listHumidity.clear()

            # Temperature Offset
            # Temperature = Temperature + 0
            return round(Temperature, 1), round(Humidity,1), int(Pressure), int(IAQ)

if GNSS_PORT is not "":
    def gnss_go():
        while True:
            strip = serialPort.readline()
            if strip.find(b'GGA') > 0:
                msg = pynmea2.parse(strip.decode('utf-8'))
                break
            sleep(0.01)
        return round(msg.longitude,5), round(msg.latitude,5), msg.altitude

def sds_go():

    for count in range(30):
        client.loop()
        try:
            PM_data = sds_sensor.get_values()
        except (IOError, OSError) as e:
            print("Error happened. {}".format(e.args[-1]))
            PM_data = None
        if PM_data is not None:
            list_PM25.append(float(PM_data[1]))
            list_PM10.append(float(PM_data[0]))
            sleep(0.01)
        else:
            print("None Error")
        
    # generate the median for each value, return -100 error on no data
    if len(list_PM25) == 0 or len(list_PM10) == 0:
        PM25 = -100
        PM10 = -100
    else:
        PM25 = int(median(list_PM25))
        PM10 = int(median(list_PM10))

    print("PM25 list ", list_PM25 , "= ",PM25)
    print("PM10 list ", list_PM10 , "= ",PM10)
    # clear lists
    list_PM25.clear()
    list_PM10.clear()
    # sds_sensor.workstate = SDS011.WorkStates.Sleeping
    return PM25, PM10


def timestamp_format():
    # 2018-08-25T14:23:45+00:00
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()

def on_message(message):
    print("on_message received: " + str(message))

def on_connect(message):
    print("on_connect received: " + str(message))


#-----Main
#Connect to MQTT

if MQTT_CLIENT_ID == "":
    payload["dev"] = get_clientid()
else:
    payload["dev"] = MQTT_CLIENT_ID


client = mqttclient.monitairMQTTClient()
client.on_message = on_message
client.on_connect = on_connect
client.begin(MQTT_USERNAME, MQTT_PASSWORD, MQTT_CLIENT_ID, MQTT_HOSTNAME, MQTT_PORT)

print("--- monitAIR Monitoring ---")

if BME_BSEC is not "":
    #BME680
    print("BME680 Humidity, Pressure, Air Temperature, IAQ Start")
    proc = subprocess.Popen([BME_BSEC], stdout=subprocess.PIPE)
    listIAQ_Accuracy = []
    listPressure = []
    listTemperature = []
    listIAQ = []
    listHumidity  = []

if W1THERM is True:
    #DS18B20
    print("DS18B20 Ground Temperature start")
    w1sensor = W1ThermSensor()

#GNSS
if GNSS_PORT is not "":
    print("GNSS start")
    serialPort = serial.Serial(GNSS_PORT, 9600, timeout=1.5)

if SDS_PORT is not "":
    print("SDS0x1 Start")
    sds_sensor = SDS011(SDS_PORT, timeout=2)
    print("port opened")
    sds_start()
    list_PM25 = []
    list_PM10 = []

try:
    print(payload)
    while True:
        client.loop()
        if (time() > timestart + reading_delay - waittime):
            timecheck = time()
            # print("timecheck = ", timecheck)

            if SDS_PORT is not "":
                #sds_sensor.workstate = SDS011.WorkStates.Measuring
                warmuptime = time()

            if BME_BSEC is not "":
                bme_data = bme680_go()
                payload["temp"] = bme_data[0]
                payload["humidity"] = bme_data[1]
                payload["pressure"] = bme_data[2]
                payload["iaq"] = bme_data[3]

            if GNSS_PORT is not "":
                gnss_data=gnss_go()
                payload["lon"] = gnss_data[0]
                payload["lat"] = gnss_data[1]
                payload["alt"] = gnss_data[2]

            if W1THERM is True:
                gtemp_data = round(w1sensor.get_temperature(),1)
                payload["dsTemp"]= gtemp_data

            if SDS_PORT is not "":
                while (time() < warmuptime + 29):
                    sleep(0.01)
                sds_data = sds_go()
                payload["PM25"] = int(sds_data[0])
                payload["PM10"] = int(sds_data[1])

            payload["timestamp"] = timestamp_format()

            waittime = time() - timecheck - 1
            # print("waittime = ", waittime)

            client.jsonWrite(MQTT_TOPIC, payload)
            # print("MQTT on it's way ::: ", payload)

            timestart = time()
        sleep(loop_delay)

except KeyboardInterrupt:
    sds_sensor.reset()
    sds_sensor = None
