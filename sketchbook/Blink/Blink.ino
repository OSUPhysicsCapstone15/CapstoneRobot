/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.
 
  This example code is in the public domain.
 */
 
// Pin 13 has an LED connected on most Arduino boards.
// give it a name:
int led = 13;
long startTime;

// the setup routine runs once when you press reset:
void setup() {                
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);     
  startTime = millis();
  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop() {
  long currentTime = millis();
  long deltaTime = currentTime - startTime;
  //Serial.println("**********************");
  //Serial.println(startTime);
  //Serial.println(currentTime);
  //Serial.println(deltaTime);
  if(deltaTime > 1000 && deltaTime < 2000) {
    Serial.print("Set");
    digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  } else if(deltaTime > 2000) {
    Serial.println("Reset");
    digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW 
    startTime = currentTime;
  }// wait for a second
}
