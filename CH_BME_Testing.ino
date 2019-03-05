/*
 * CH_BME_Testing
 * 
 * Test the BME280 sensor using I2C
 * 
 * Taken from Adafruit example sketch.
 * 
 * 
 */

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>  // can use BME680library


#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C

uint8_t ADDR1=0x76; // some units default to 0x77
uint8_t ADDR2=0x77;

unsigned long delayTime;

void setup() {
    Serial.begin(9600);
    Serial.println(F("BME280 test"));

    bool status;
    
    // default settings
    status = bme.begin(ADDR1);  
    if (!status) {
        status=bme.begin(ADDR2);
        if (!status) {
          Serial.println("Could not find a BME280 sensor at addr 0x76 or 0x77, check wiring!");
          while (1) yield(); // yield is required to prevent WDT crash.
        }
    }
    
    Serial.println("-- BME Sensor Test --");
    delayTime = 1000;

    Serial.println();
}


void loop() { 
    printValues();
    delay(delayTime);
}


void printValues() {
    Serial.print("Temperature = ");
    Serial.print(bme.readTemperature());
    Serial.println(" *C");

    Serial.print("Pressure = ");

    Serial.print(bme.readPressure() / 100.0F);
    Serial.println(" hPa");

    Serial.print("Humidity = ");
    Serial.print(bme.readHumidity());
    Serial.println(" %");

    Serial.println();
}
