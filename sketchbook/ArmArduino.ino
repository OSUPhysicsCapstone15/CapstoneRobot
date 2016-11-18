
#include <Servo.h>  // Loads 'Servo' Library
Servo ClawServo;    // Define ClawServo variable of class 'Servo'

int LED_PIN = 12;
int state; 

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(9600);

  ClawServo.attach(10); // Attack Claw to Pin 9
}

// the loop function runs over and over again forever
void loop() {

  // This loop Checks from Grab Message
  if (Serial.available() > 0)
  {
    if (Serial.peek() == 'g')
    {
      Serial.read();
      state = Serial.parseInt();
      digitalWrite(LED_PIN, state);
    }
    while (Serial.available() > 0)
    {
      Serial.read(); 
    }
  }
  
  // Arm Control
  if (state == 1)
  {
    // #### Execute this Block when grab commnad recieved ###

    // Step 0: Make sure Bin Actuator is Moved to The left

    // Step 1: Open Claw
    ClawServo.write (90);

    // Step 2: Extend Arm Actuator

    // Step 3: Close Claw, grab sample

    // Step 4: Retract Arm

    // Step 5: Move Bin Actuator Under Arm

    // Step 6: Arm Drop Sample Into Bin

    // Step 7: Confirm Retreval

    // Steo 8: Return All systems to original Positions

    // Step 9 Return Finished Mesage
    state = 0; 
    digitalWrite(LED_PIN, state);
  }
}
