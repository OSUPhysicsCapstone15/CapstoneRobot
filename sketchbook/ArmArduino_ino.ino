#include<Servo.h>
//Standard PWM DC control
const int E1 = 5;     //M1 Speed Control
const int E2 = 6;     //M2 Speed Control
const int M1 = 4;    //M1 Direction Control
const int M2 = 7;    //M1 Direction Control

// TIME CONSTANTS
// Moving at max speed, arm and bin actuators will take some time to get into place
// Tweak these constants to change how long actuators will move
const int ARM_TIME = 2000;
const int BIN_TIME = 2000; 

// Servo Declarations
Servo ArmAct; // Arm actuator 
Servo BinAct; // Bin Actuator
Servo Claw;   // Claw Servo

// CLAW Constants
const int CLAW_OPEN = 90;
const int CLAW_CLOSE = 0;

void stop(void)                    //Stop
{
  digitalWrite(E1,LOW);   
  digitalWrite(E2,LOW);      
}   
void advanceARM(char a,char b)          //Move forward
{
  analogWrite (E1,a);      //PWM Speed Control
  digitalWrite(M1,HIGH);    
  analogWrite (E2,b);    
  digitalWrite(M2,HIGH);
}  
void back_offARM (char a,char b)          //Move backward
{
  analogWrite (E1,a);
  digitalWrite(M1,LOW);   
  analogWrite (E2,b);    
  digitalWrite(M2,LOW);
}

void GRAB_DAT()
{
 // #### Execute this Block when grab commnad recieved ###

    // Step 0: Make sure Bin Actuator is Moved to The left

    // Step 1: Open Claw
    Claw.write (CLAW_OPEN);
    // Step 2: Extend Arm Actuator
    advanceARM (255,255);
    delay(ARM_TIME);
    stop();
    // Step 3: Close Claw, grab sample
    Claw.write(CLAW_CLOSE);
    // Step 4: Retract Arm
    back_offARM (255,255); 
    delay(ARM_TIME); // arm now back in place 
    stop();
    // Step 5: Move Bin Actuator Under Arm
    
    // Step 6: Arm Drop Sample Into Bin
    Claw.write (CLAW_OPEN);
    delay(1000);
    // Step 7: Confirm Retreval
    
    delay(1000);
    // Steo 8: Return All systems to original Positions

    // Step 9 Return Finished Mesage
}

void setup(void) 
{ 
  Serial.begin(9600);
  pinMode(6,OUTPUT);
  
  ArmAct.attach(6);
  
  int i;
  for(i=4;i<=7;i++)
    pinMode(i, OUTPUT);  
  Serial.begin(19200);      //Set Baud Rate
  Serial.println("Run keyboard control");
} 
void loop(void) 
{
  /*if(Serial.available()){
    char val = Serial.read();
    if(val != -1)
    {
      switch(val)
      {
      case 'w'://Move Forward
        advance (255,255);   //move forward in max speed
        Serial.println("Onward!");
        break;
      case 's'://Move Backward
        back_off (255,255);   //move back in max speed
        Serial.println("GET BACK!");
        break;

      case 'z':
        Serial.println("Hello");
        break;
      case 'x':
        stop();
        break;
      }
    }
    else stop();  
  }
  */
}  
