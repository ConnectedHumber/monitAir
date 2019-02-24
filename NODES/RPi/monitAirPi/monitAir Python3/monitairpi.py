#!/usr/bin/python3

from time import sleep, gmtime, strftime, time
import subprocess
import mqttclient	#sudo pip3 install paho-mqtt
from sds011 import SDS011	#local module sds011.py
import json
from statistics import median
from w1thermsensor import W1ThermSensor		#sudo pip3 install w1thermsensor
import serial	#sudo pip3 install pyserial 
import pynmea2	#sudo pip3 install pynmea2




# ConnectedHumber : MonitAIR project authentication info.

MQTT_USERNAME  = "ch-username"
MQTT_CLIENT_ID = "name_pi01"
MQTT_HOSTNAME = "mqtt.connectedhumber.org"
MQTT_PORT = 1883
MQTT_TOPIC = "airquality/data"

GNSS_PORT = "/dev/ttyACM0"
SDS_PORT = "/dev/serial0"
BME_BSEC = "./bsec_bme680"

payload = {"timestamp":-1,
           "dev":MQTT_CLIENT_ID,
           "temp":-1,
           "humidity":-1,
           "pressure":-1,
           "PM10" : -1,
           "PM25" : -1,
           "iaq":-1,
           "groundTemp":-1,
           "lon":-1,
           "lat":-1,
           "alt":-1
           }

#general
timestamp = 0.0
loop_delay = 2 #seconds
reading_delay = 60*6 # time between sending data, every 6 minutes for monitAIR MQTT
timestart = 0 

def sds_start():
    # details about it
    print(sds_sensor.device_id)
    print(sds_sensor.firmware)
    print(sds_sensor.dutycycle)
    print(sds_sensor.workstate)
    print(sds_sensor.reportmode)
    # Set dutycyle to nocycle (permanent)
    sds_sensor.reset()
    sds_sensor.workstate = SDS011.WorkStates.Measuring
    #warm up delay removed as built in to main loop, add sleep(15) if using a fast loop

def bme680_go():
    for line in iter(proc.stdout.readline, ''):
        lineJSON = json.loads(line.decode("utf-8")) # process line-by-line
        lineDict = dict(lineJSON)

        listIAQ_Accuracy.append(int(lineDict['IAQ_Accuracy']))
        listPressure.append(float(lineDict['Pressure']))
        listGas.append(int(lineDict['Gas']))
        listTemperature.append(float(lineDict['Temperature']))
        listIAQ.append(float(lineDict['IAQ']))
        listHumidity.append(float(lineDict['Humidity']))
        listStatus.append(int(lineDict['Status']))

        if len(listIAQ_Accuracy) == 20:
            #generate the median for each value
            IAQ_Accuracy = median(listIAQ_Accuracy)
            Pressure = median(listPressure)
            Gas = median(listGas)
            Temperature = median(listTemperature)
            IAQ = median(listIAQ)
            Humidity = median(listHumidity)
            Status = median(listStatus)
            #clear lists
            listIAQ_Accuracy.clear()
            listPressure.clear()
            listGas.clear()
            listTemperature.clear()
            listIAQ.clear()
            listHumidity.clear()
            listStatus.clear()
            #Temperature Offset, quick and dirt method.  Best set in the BSEC for more accurate results.
            Temperature = Temperature + 0
            return round(Temperature, 1), round(Humidity,1), int(Pressure), int(IAQ)
        
def gnss_go():
    print("gnss is go")
    #serialPort = serial.Serial(GNSS_PORT, 9600, timeout=0.5)
    while True:
        strip = serialPort.readline()
        #print(strip)
        if strip.find(b'GGA') > 0:
            msg = pynmea2.parse(strip.decode('utf-8'))
            #print(msg.latitude,msg.longitude,msg.altitude)
            break
        sleep(0.01)
    return round(msg.longitude,5),round(msg.latitude,5),msg.altitude, msg.timestamp

def timestamp_format():
    return strftime("%a %b %d %Y %H:%M:%S GMT+0000",gmtime())

def on_message(message):
    print("on_message received: " + str(message))
    
def on_connect(message):
    print("on_connect received: " + str(message))    


#-----Main
#Connect to MQTT 
client = mqttclient.monitairMQTTClient()
client.on_message = on_message
client.on_connect = on_connect
client.begin(MQTT_USERNAME, MQTT_PASSWORD, MQTT_CLIENT_ID, MQTT_HOSTNAME, MQTT_PORT)

print("sleep 2 seconds for connect just incase..")
sleep(2)
print("slept")




print("--- monitAIR Monitoring ---")

#BME680
print("BME680 Humidity, Pressure, Air Temperature, IAQ Start")
proc = subprocess.Popen([BME_BSEC], stdout=subprocess.PIPE)
listIAQ_Accuracy = []
listPressure = []
listGas = []
listTemperature = []
listIAQ = []
listHumidity  = []
listStatus = []

#DS18B20
print("DS18B20 Ground Temperature start")
w1sensor = W1ThermSensor()

#GNSS
print("GNSS start")
serialPort = serial.Serial(GNSS_PORT, 9600, timeout=0.5)

#SDS0x1
print("SDS0x1 PM2.5 & PM10 Start")
sds_sensor = SDS011(SDS_PORT)
print("port opened")
sds_start()

try:

    print(payload)
    while True:
        client.loop()
        if (time() > timestart + reading_delay):
            bme_data = bme680_go()
            sds_data = sds_sensor.get_values()
            gnss_data = gnss_go()
            gtemp_data = round(w1sensor.get_temperature(),1)
            
            payload["temp"] = bme_data[0]
            payload["humidity"] = bme_data[1]
            payload["pressure"] = bme_data[2]
            payload["iaq"] = bme_data[3]
           
            payload["PM25"] = int(sds_data[1])
            payload["PM10"] = int(sds_data[0])
            payload["groundTemp"]= gtemp_data
            payload["lon"] = gnss_data[0]
            payload["lat"] = gnss_data[1]
            payload["alt"] = gnss_data[2]
            payload["timestamp"] = timestamp_format() #gnss_data[3] in future

            client.jsonWrite(MQTT_TOPIC, payload)
            timestart = time()
        sleep(loop_delay)

except KeyboardInterrupt:
    sds_sensor.workstate = SDS011.WorkStates.Sleeping
    pass
