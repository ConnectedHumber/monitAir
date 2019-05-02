/*
 * DEEP_SLEEP.ino
 * 
 * Checks the functioning of deep sleep
 * 
 * Some Cheap WeMOS D1 Mini clones fail this test
 * 
 * 
 * a link between 3V3 and A0 will stop the WeMOS going to sleep
 * a link between D5 and RST will enable the WeMOS to wakeup from sleep
 * 
 * 
*/

const int sleepSeconds=10;

void setup() {
  // put your setup code here, to run once:
  pinMode(D0, WAKEUP_PULLUP);
  pinMode(D5,OUTPUT);

  digitalWrite(D5,HIGH);

  Serial.begin(74880);
  Serial.println("\nWake up - to stop me going into deepsleep link 3V3->A0 and reset.");

  // max range is 1024 so anything less than
  // half shows the link exists
  
  if (analogRead(A0)<512)
  {
  Serial.println("To enable wakeup link D0->RST");

  delay(sleepSeconds*1000);
  
  // convert to microseconds
  Serial.print("Going to sleep for ");
  Serial.print(sleepSeconds);
  Serial.println("s");
  
  ESP.deepSleep(sleepSeconds * 1000000);
  }
  else{
    Serial.println("Not sleeping. Remove the D0->RST link to reprogram.");
  }
}

void loop() {
  // put your main code here, to run repeatedly:
}
