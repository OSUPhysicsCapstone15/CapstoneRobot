/*
  Example Bluetooth Serial Passthrough Sketch
 by: Jim Lindblom
 SparkFun Electronics
 date: February 26, 2013
 license: Public domain

 This example sketch converts an RN-42 bluetooth module to
 communicate at 9600 bps (from 115200), and passes any serial
 data between Serial Monitor and bluetooth module.
 */
#include <SoftwareSerial.h> 
#include <Encoder.h>
#include "DualVNH5019MotorShield.h"


int bluetoothTx = 2;  // TX-O pin of bluetooth mate, Arduino D2
int bluetoothRx = 3;  // RX-I pin of bluetooth mate, Arduino D3

DualVNH5019MotorShield md1;

SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);

void setup()
{

  Serial.begin(9600);  // Begin the serial monitor at 9600bps

  bluetooth.begin(115200);  // The Bluetooth Mate defaults to 115200bps
//  bluetooth.print("$");  // Print three times individually
//  bluetooth.print("$");
//  bluetooth.print("$");  // Enter command mode
//  delay(100);  // Short delay, wait for the Mate to send back CMD
//  bluetooth.println("U,9600,N");  // Temporarily Change the baudrate to 9600, no parity
//  // 115200 can be too fast at times for NewSoftSerial to relay the data reliably
//  bluetooth.begin(9600);  // Start bluetooth serial at 9600
}

void loop()
{
  md1.setM1Speed(150);
  md1.setM2Speed(150);
  delay(1000);
  md1.setM1Speed(0);
  md1.setM2Speed(0);
//  if(bluetooth.available())  // If the bluetooth sent any characters
//  { 
//    // Send any characters the bluetooth prints to the serial monitor
//    int command=(int)bluetooth.read();
//    if(command==179){ // If "g" is read
//      Serial.println("Motors on.");
//      md1.setM1Speed(150);
//      md1.setM2Speed(150);
//    }
//    else if (command==187) { // If "s" is read
//      Serial.println("Motors off.");
//      md1.setM1Speed(0);
//      md1.setM2Speed(0);
//    }
//    Serial.print(command);
//  }
//  if(Serial.available())  // If stuff was typed in the serial monitor
//  {
//    // Send any characters the Serial monitor prints to the bluetooth
//    bluetooth.print((char)Serial.read());
//  }
//  // and loop forever and ever!
}

