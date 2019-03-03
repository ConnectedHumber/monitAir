/*
 * CH_PM_SENSOR_Testing
 * 
 * Author: Brian.N.Norman March 2019
 * 
 * The dust sensor uses serial comms and is switched on/off by the High Side Switch .
 * 
 * This sketch enables the high side switch at the outset then simply copies
 * any recieved data to the serial monitor as hex bytes.
 * 
 * Connections
 * 
 * Sensor                    WeMOS
 * 
 * J2 pin 3 RX <------------- TX on D6 (GPIO12)
 * J2 pin 4 TX -------------> RX on D7 (GPIO13)
 * 
 */

#include <SoftwareSerial.h>

#define SENSOR_ENABLE D5 

SoftwareSerial sensor(D6,D7); // may need reversing if PM Sensor wiring is different

void setup() {
    Serial.begin(9600);
    Serial.println(F("Serial comms test"));

    sensor.begin(9600); // majority of sensors use 9600

    // turn on the high side switch - could also place a link in JP2
    pinMode(SENSOR_ENABLE,OUTPUT);
    digitalWrite(SENSOR_ENABLE,HIGH);
    
    Serial.println();
}


void loop() { 
  // copy sensor output to the monitor
  if (sensor.available())
    {
      // this will not be human readable so display hex values
      // the start bytes of the sensor should be identifiable though
      Serial.print("0x");
      Serial.print(sensor.read(),HEX);
      Serial.print(" ");
    }
}
