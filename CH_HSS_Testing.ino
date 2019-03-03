/*
  CH_HSS_Test.ino

 Author: Brian.N.Norman March 2019
 
 Test to confirm the ConnectedHumber HighSide switch is operational using a variation of Blink.

 Jumper J2 MUST be removed (normal state)

 Connect an LED and suitable resistor in series between the sensor 5V and GND.

 The Sensor switched 5v can be accessed using the lefthand pin of jumper JP2 or from J2

 J2 pin 1 ... 5v
 J2 pin 2 ... GND
  

*/

#define SENSOR_ENABLE D5  // GPIO14/SCK


// the setup function runs once when you press reset or power the board
void setup() {

  pinMode(SENSOR_ENABLE,OUTPUT);
  digitalWrite(SENSOR_ENABLE,LOW); // 5V switch
  
  Serial.begin(9600);
  while (!Serial) ;
  
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN,LOW);
  delay(5000);

  Serial.println("Setup done");
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(SENSOR_ENABLE,HIGH);
  delay(500);                 
  digitalWrite(SENSOR_ENABLE,LOW);
  delay(500);      
}
