
//  Heltec esp32 code to send GPS & SDS021 code to TTN
// I've not written any of this code - just scammed it from others off the internet
// then stuck it together with glue, stickytape and bits of string

// adam - 12/08/18


//combination of OTAA code and GPS for Heltec ESP32 OLED LoRa module
// NCB Fri 12th Jan 2018
// Link with LMiC library available here: https://github.com/matthijskooijman/arduino-lmic/
// GPS get_coords function adapted from here: https://github.com/brady-aiello/Seeeduino_LoRaWAN_for_hybrid_gateways
/* Decode coordinate payload for TTN console
  function Bytes2Float32(bytes) {
    var sign = (bytes & 0x80000000) ? -1 : 1;
    var exponent = ((bytes >> 23) & 0xFF) - 127;
    var significand = (bytes & ~(-1 << 23));
    if (exponent == 128)
        return sign * ((significand) ? Number.NaN : Number.POSITIVE_INFINITY);
    if (exponent == -127) {
        if (significand == 0) return sign * 0.0;
        exponent = -126;
        significand /= (1 << 22);
    } else significand = (significand | (1 << 23)) / (1 << 23);
    return sign * significand * Math.pow(2, exponent);
  }
  function Decoder(bytes, port) {
  var lat = bytes[3] << 24 | bytes[2] << 16 | bytes[1] << 8 | bytes[0];
  var lon = bytes[7] << 24 | bytes[6] << 16 | bytes[5] << 8 | bytes[4];
  return{
    latitude:  Bytes2Float32(lat),
    longitude: Bytes2Float32(lon)
  };
  }
*/
#include <HardwareSerial.h>
#include <TinyGPS++.h>
//#include <lmic.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <U8x8lib.h>
//#include "SdsDustSensor.h"
//#include "processdata.h"

#define BUILTIN_LED 25
// The TinyGPS++ object
TinyGPSPlus gps;
typedef union {
  float f[4];               // Assigning fVal.f will also populate fVal.bytes;
  unsigned char bytes[16];   // Both fVal.f and fVal.bytes share the same 4 bytes of memory.
} floatArr2Val;
floatArr2Val latlong;
float latitude;
float longitude;
float h;
char s[16]; // used to sprintf for OLED display
#define GPS_RX 22
#define GPS_TX 23

#define RXD2 12
#define TXD2 13
//SdsDustSensor sds(Serial2);
HardwareSerial particleSensor(2);

float pm25; //2.5um particles detected in ug/m3
float pm10; //10um particles detected in ug/m3

unsigned int deviceID; //Two byte unique ID set by factor


HardwareSerial GPSSerial(1);

//HardwareSerial sds(0);

// the OLED used

U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock=*/ 15, /* data=*/ 4, /* reset=*/ 16);
// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3, 0x70.

//device id
static const u1_t PROGMEM APPEUI[8] = {  }; //DevID here
void os_getArtEui (u1_t* buf) {
  memcpy_P(buf, APPEUI, 8);
}

//device id
// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8] = {  }; //DeviceID here
void os_getDevEui (u1_t* buf) {
  memcpy_P(buf, DEVEUI, 8);
}
// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// 

//appID
static const u1_t PROGMEM APPKEY[16] = {  }; //AppID here
void os_getDevKey (u1_t* buf) {
  memcpy_P(buf, APPKEY, 16);
}
static osjob_t sendjob;
// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 30;
// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 14,
  .dio = {26, 33, 32},
};
void get_coords () {
  while (GPSSerial.available())
    gps.encode(GPSSerial.read());
  latitude  = gps.location.lat();
  longitude = gps.location.lng();
  // Only update if location is valid and has changed
  if ((latitude && longitude) && latitude != latlong.f[0]
      && longitude != latlong.f[1]) {
    latlong.f[0] = latitude;
    latlong.f[1] = longitude;
    latlong.f[2]=  pm25;
    latlong.f[3] = pm10;
    for (int i = 0; i < 16; i++)
      Serial.print(latlong.bytes[i], HEX);
    Serial.println();
  }
  u8x8.setCursor(0, 2);
  u8x8.print("Lat: ");
  u8x8.setCursor(5, 2);
  sprintf(s, "%f", latitude);
  u8x8.print(s);
  u8x8.setCursor(0, 3);
  u8x8.print("Lng: ");
  u8x8.setCursor(5, 3);
  sprintf(s, "%f", longitude);
  u8x8.print(s);
}
void onEvent (ev_t ev) {
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      u8x8.drawString(0, 7, "EV_SCAN_TIMEOUT");
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      u8x8.drawString(0, 7, "EV_BEACON_FOUND");
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      u8x8.drawString(0, 7, "EV_BEACON_MISSED");
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      u8x8.drawString(0, 7, "EV_BEACON_TRACKED");
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      u8x8.drawString(0, 7, "EV_JOINING   ");
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      u8x8.drawString(0, 7, "EV_JOINED    ");
      // Disable link check validation (automatically enabled
      // during join, but not supported by TTN at this time).
      LMIC_setLinkCheckMode(0);
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      u8x8.drawString(0, 7, "EV_RFUI");
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      u8x8.drawString(0, 7, "EV_JOIN_FAILED");
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      u8x8.drawString(0, 7, "EV_REJOIN_FAILED");
      //break;
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      u8x8.drawString(0, 7, "EV_TXCOMPLETE");
      digitalWrite(BUILTIN_LED, LOW);
      if (LMIC.txrxFlags & TXRX_ACK) {
        Serial.println(F("Received ack"));
        u8x8.drawString(0, 7, "Received ACK");
      }
      if (LMIC.dataLen) {
        Serial.println(F("Received "));
        u8x8.drawString(0, 6, "RX ");
        Serial.println(LMIC.dataLen);
        u8x8.setCursor(4, 6);
        u8x8.printf("%i bytes", LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
        u8x8.setCursor(0, 7);
        u8x8.printf("RSSI %d SNR %.1d", LMIC.rssi, LMIC.snr);
      }
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      u8x8.drawString(0, 7, "EV_LOST_TSYNC");
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      u8x8.drawString(0, 7, "EV_RESET");
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      u8x8.drawString(0, 7, "EV_RXCOMPLETE");
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      u8x8.drawString(0, 7, "EV_LINK_DEAD");
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      u8x8.drawString(0, 7, "EV_LINK_ALIVE");
      break;
    default:
      Serial.println(F("Unknown event"));
      u8x8.setCursor(0, 7);
      u8x8.printf("UNKNOWN EVENT %d", ev);
      break;
  }
}
void do_send(osjob_t* j) {
  // Check if there is not a current TX/RX job running




if (dataAvailable())
  {
    Serial.print("Particle Matter [2.5]:");
    Serial.print(pm25, 1);
    Serial.print("ug/m3 [10]:");
    Serial.print(pm10, 1);
    Serial.print("ug/m3");
    Serial.print(" DeviceID:0x");
    Serial.print(deviceID, HEX);
    Serial.println();

   
    
  }
  else
  {
    Serial.println("Timeout or CRC error");
    Serial.println("Double check blue wire goes to pin 10");
  }
  
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
    u8x8.drawString(0, 7, "OP_TXRXPEND, not sent");
  } else {
    // Prepare upstream data transmission at the next possible time.
    get_coords();

    //LMIC_setTxData2(1, (uint8_t*) coords, sizeof(coords), 0);
    LMIC_setTxData2(1, latlong.bytes, 16, 0);
    Serial.println(F("Packet queued"));
    u8x8.drawString(0, 7, "PACKET QUEUED");
    digitalWrite(BUILTIN_LED, HIGH);
  }
  // Next TX is scheduled after TX_COMPLETE event.
}//-------------------



//Scans for incoming packet
//Times out after 1500 miliseconds
boolean dataAvailable(void)
{
  //Spin until we hear meassage header byte
  long startTime = millis();

  while (1)
  {
    while (!particleSensor.available())
    {
      delay(1);
      if (millis() - startTime > 1500) return (false); //Timeout error
    }

    if (particleSensor.read() == 0xAA) break; //We have the message header
  }

  //Read the next 9 bytes
  byte sensorValue[10];
  for (byte spot = 1 ; spot < 10 ; spot++)
  {
    startTime = millis();
    while (!particleSensor.available())
    {
      delay(1);
      if (millis() - startTime > 1500) return (false); //Timeout error
    }

    sensorValue[spot] = particleSensor.read();
  }

  //Check CRC
  byte crc = 0;
  for (byte x = 2 ; x < 8 ; x++) //DATA1+DATA2+...+DATA6
    crc += sensorValue[x];
  if (crc != sensorValue[8])
    return (false); //CRC error

  if (sensorValue[1] == 0xC0) //This is just a normal reading
  {
    //Update the global variables
    pm25 = ((float)sensorValue[3] * 256 + sensorValue[2]) / 10;
    pm10 = ((float)sensorValue[5] * 256 + sensorValue[4]) / 10;

    deviceID = sensorValue[6] * 256 + sensorValue[7];
  }
  else if (sensorValue[1] == 0xC5) //Response to command
  {
    Serial.println();
    Serial.println("Response to command found");

    if (sensorValue[2] == 7) //Firmware response
    {
      Serial.print("Firmware version Y/M/D: ");
      Serial.print(sensorValue[3]);
      Serial.print("/");
      Serial.print(sensorValue[4]);
      Serial.print("/");
      Serial.print(sensorValue[5]);
      Serial.println();
    }
    else if (sensorValue[2] == 6) //Query/Set work and sleep modes
    {
      if (sensorValue[3] == 1) //Response to set mode
      {
        Serial.print("Sensor is going to ");
        if (sensorValue[4] == 0) Serial.println("sleep");
        else if (sensorValue[4] == 1) Serial.println("work");
      }
    }

    Serial.println();
  }

  Serial.print("Raw data:");
  for (int x = 1 ; x < 10 ; x++)
  {
    Serial.print(" ");
    Serial.print(x);
    Serial.print(":0x");
    Serial.print(sensorValue[x], HEX);
  }
  Serial.println();

  return (true); //We've got a good reading!
}//----------------

//Print the firmware version
void getFirmwareVersion(void)
{
  sendCommand(7, 0, 0); //Command number is 7, no databytes
}

//Tell the module to go to sleep
void goToSleep(void)
{
  sendCommand(6, 1, 0); //Command number is 6, set mode = 1, sleep = 0
}

//Tell module to start working!
void wakeUp(void)
{
  sendCommand(6, 1, 1); //Command number is 6, set mode = 1, work = 1
}



//Send a command packet to the module
//Requires the command number and two setting bytes
//Calculates CRC and attaches all header/ender bytes
//Assumes you are only talking to one sensor
void sendCommand(byte commandNumber, byte dataByte2, byte dataByte3)
{
  byte packet[19]; //It's 19 bytes big
  packet[0] = 0xAA; //Message header
  packet[1] = 0xB4; //Packet type = Command
  packet[2] = commandNumber; //Type of command we want to do
  packet[3] = dataByte2; //These are specific to each command
  packet[4] = dataByte3;

  for (byte x = 5; x < 15 ; x++)
    packet[x] = 0; //Reserved bytes

  packet[15] = 0xFF; //Talk to whatever sensor we are connected to. No specific device ID.
  packet[16] = 0xFF; //Talk to whatever sensor we are connected to. No specific device ID.

  //packet[15] = 0xA4; //Talk to specific sensor
  //packet[16] = 0xE6; //Talk to specific sensor

  //Caculate CRC
  byte crc = 0;
  for (byte x = 2 ; x < 17 ; x++)
    crc += packet[x];

  packet[17] = crc;
  packet[18] = 0xAB; //Tail

  //Display the contents of the command packet for debugging
  /*Serial.print("Command packet:");
    for(int x = 0 ; x < 19 ; x++)
    {
    Serial.print(" ");
    Serial.print(x);
    Serial.print(":0x");
    Serial.print(packet[x], HEX);
    }
    Serial.println();*/

  //The sensor seems to fail to respond to the first 2 or 3 times we send a command
  //Hardware serial doesn't have this issue but software serial does.
  //Sending 10 throw away characters at it gets the units talking correctly
  for (byte x = 0 ; x < 10 ; x++)
    particleSensor.write('!'); //Just get the software serial working

  //Send command packet
  for (byte x = 0 ; x < 19 ; x++)
    particleSensor.write(packet[x]);

  //Now look for response
  dataAvailable();
}

void setup() {
  u8x8.begin();
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.drawString(0, 0, "TTN test 1");
  Serial.begin(115200);
  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  GPSSerial.setTimeout(2);
  particleSensor.begin(9600, SERIAL_8N1,16,17); //initilise the SDS021 on pins 15/16 - move these to variables silly boy!
  Serial.println(F("Starting setup"));
 


 // Pm25 = 0;
 // Pm10 = 0;
  SPI.begin(5, 19, 27);
  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  // Start job (sending automatically starts OTAA too)
  do_send(&sendjob);
  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, LOW);
}
void loop() {
  os_runloop_once();
}