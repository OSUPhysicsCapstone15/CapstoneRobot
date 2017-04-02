/*
  Analog input, analog output, serial output

 Reads an analog input pin, maps the result to a range from 0 to 255
 and uses the result to set the pulsewidth modulation (PWM) of an output pin.
 Also prints the results to the serial monitor.

 The circuit:
 * potentiometer connected to analog pin 0.
   Center pin of the potentiometer goes to the analog pin.
   side pins of the potentiometer go to +5V and ground
 * LED connected from digital pin 9 to ground

 created 29 Dec. 2008
 modified 9 Apr 2012
 by Tom Igoe

 This example code is in the public domain.

 */

// These constants won't change.  They're used to give names
// to the pins used:
const int GyroReadingPin = A2;  // Analog input pin that the potentiometer is attached to
const int FSR_Pin = A5;         // Analog input pin the reads the voltage across the FSR (Force Sensative Resistor)
const int LED_PIN = 8; // Analog output pin that the LED is attached to

// FUNCTION: Turnning(), Returns True or false based on readings from Gyro
// This function is run first in the main to determine if gyro is turning
  // Included Variables
  const int HighTurnMargin = 100; // Huge values are likely anomolies, ingore these
  const int  LowTurnMargin = 1;   // No need to worry about verry small fluctuations
  int sensorValue = 0;            // value read from the Gyro
  int lastValue = 0;              // Used to check the difference between readings

bool Turnning ( int Val, int lastVal ){
  // if the new val is whith the margin, we are turning
  int epsilon = abs(Val - lastVal); // define epsilon as the error
  if (epsilon > LowTurnMargin & epsilon < HighTurnMargin){
    return true;
  }
  else {return false;}
}

//FUNCTION: Pickup_Sucess(), Returns True or Flase based on readings from FSR
  // Included Variables
  int FSR_Voltage = 0;  // Value read from FSR analog pin
  const int Max_Voltage = 0; // If the maximum voltage is exeeded, we assume that the sample is on the resistor and therefore in the bin

bool Pickup_Sucess( int Val ){
  if (Val > Max_Voltage ){ return true;}
  else {return false;}
}

  



void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
  pinMode(LED_PIN, OUTPUT);
}

void loop() {

  /*#############################################################
  // Gyro Section: This section deals with external motion sensors
  //#############################################################
  // read the analog in value:
  sensorValue = analogRead(GyroReadingPin);
  
  // Check if turning
  if (Turnning(sensorValue, lastValue)){ // Turnning Returns TRUE
     Serial.print("Turn UP!! \n ");
     digitalWrite( LED_PIN, HIGH);
  } 
  else { // Trunning returns FALSE
     Serial.print("Turn down!! \n ");
     digitalWrite( LED_PIN, LOW);
  }
  //print the results to the serial monitor:
  Serial.print("sensor = ");
  Serial.print(sensorValue);
  Serial.print("\t");
  //Serial.println(outputValue);
 
  // wait 2 milliseconds before the next loop
  // for the analog-to-digital converter to settle
  // after the last reading:
  lastValue = sensorValue;
  delay(2);
  // $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
  // End Gyro Section
  */

  // ############################################################
  // FSR Section: This section reads the voltage across the FSR to confirm Retrval
  //#############################################################
  FSR_Voltage = analogRead(FSR_Pin);
  Serial.print("FSR Voltage = ");
  Serial.print(FSR_Voltage);
  Serial.print(" \n");
    

  
}
