#include<Servo.h>
//Standard PWM DC control
int E1 = 5;     //M1 Speed Control
int E2 = 6;     //Bin Speed Control
int M1 = 4;    //M1 Direction Control
int M2 = 7;    //M1 Direction Control

// Pin Declarations
const int BIN_PIN = 9;
const int ARM_PIN = 8;
const int CAMERA_PIN = 12;
const int CLAW_PIN = 3;

// TIME CONSTANTS
// These are the times (ms) required for actuators to move into position
const int SECOND= 1000;
const double ARM_DOWN = 32*SECOND;
const double ARM_SAMPLE = ARM_DOWN/1.75;
const double ARM_UP = ARM_DOWN - ARM_SAMPLE;
const double BIN_TIME = 15.5*SECOND;

// OTHER CONSTSTANTS 
const int CLAW_ANGLE = 33; // Degree claw makes when fully open
const int CLAW_PINCH = 45; // Close to a higher angle so that the servo pinches down
// CAMERA#1 VARIALES
#define FULL_TURN 1300 //360 degrees
#define TURN_TIME 132 //30 degrees
#define STOP_POINT 92 //No movement
#define CLOCKWISE 0 // Full speed clockwise
#define C_CLOCKWISE 180 // Full speed counter-clockwise

// SERVO DECLARATIONS
Servo ArmAct; // Arm actuator 
Servo BinAct; // Bin Actuator
Servo Claw;   // Claw Servo
Servo Camera; // Camera Servo


// FUNCTIONS: ########################################################################################################
// ###################################################################################################################
void stop(void)                    //Stop
{
  digitalWrite(E1,LOW);   
  digitalWrite(E2,LOW);      
  }   

void arm_advance(char a)          //Move Arm forward
{
  analogWrite (E1,a);      //PWM Speed Control
  digitalWrite(M1,LOW);    
  }  

void arm_back (char a)          //Move backward
{
  analogWrite (E1,a);
  digitalWrite(M1,HIGH);   
  }
  
void Claw_Open(Servo Claw, int a)          // Open CLaw
{
 for (int pos = 0; pos <= a; pos += 1) { // goes from 0 degrees to 'a' degrees
    // in steps of 1 degree
    Claw.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
    }
  }

void Claw_Close( Servo Claw, int a )  // CLose Claw
{
  for (int pos = a; pos >= 0; pos -= 1) { // goes from 'a' degrees to 0 degrees
    Claw.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
    }
  }  

void bin_left(char a)             //Turn Left
{    
  analogWrite (E2,a);    
  digitalWrite(M2,HIGH);
  }

void bin_right(char a)             //Turn Left
{    
  analogWrite (E2,a);    
  digitalWrite(M2,LOW);
  }

void open_claw(int a, Servo b)             //Turn Right
{
  b.write(a);
  }

// SETUP: ######################################################################################################
// #############################################################################################################
void setup(void) 
{ 
  Serial.begin(9600);
  //pinMode(E2,OUTPUT);
  //pinMode(E1,OUTPUT);
 
  int i;
  for(i=3;i<=9;i++)
    pinMode(i, OUTPUT);  
  Serial.begin(19200);      //Set Baud Rate
  Serial.println("Run keyboard control");

  // Define Servo Names and Attach
  BinAct.attach(8);
  ArmAct.attach(9);
  Claw.attach(CLAW_PIN);
  Camera.attach(CAMERA_PIN);
  
  // Set Camera Position
  Camera.write(STOP_POINT);
} 


// MAIN: #######################################################################################################
// #############################################################################################################

void loop(void) 
{
  if(Serial.available()){
    char val = Serial.read();
    if(val != -1)
    {
      switch(val)
      {
      case 'w'://Move Forward
        arm_advance (255);   //move forward in max speed
        Serial.println("Onward!");
        break;
      
      case 's'://Move Backward
        arm_back(255);   //move back in max speed
        Serial.println("GET BACK!");
        break;
      
      case 'a'://Turn Left
        bin_left(255);
        break;  
      
      case 'b'://Turn Right
        bin_right(255);
        break;       
        
       case 'c' : // Open/Close Claw
       Claw_Open(  Claw,CLAW_ANGLE );
       break;
       
       case 'd' :
       Claw_Close( Claw,CLAW_ANGLE);
       break;
         
        
      case 'l'://Turn Right
        open_claw(50, Claw);
        Serial.println("Opening Claw");
        break;
      case 'z':
        Serial.println("Hello");
        break;
      
      /* Camera Rotation
      case 'c':
        for (int j = 1; j < 13; j++){
          Serial.println("Rotating camera");
          Camera.write(CLOCKWISE);
          delay(TURN_TIME);
          Camera.write(STOP_POINT);
          Serial.println("Looking for object");
          delay(2000);
          feedback = j+j*TURN_TIME;
          Serial.print("Rotated ");
          Serial.print(feedback);
          Serial.print("degrees");
          Serial.println();
        }
        Camera.write(C_CLOCKWISE);
        delay(FULL_TURN);
        Camera.write(STOP_POINT);
        break;
      */
      
      
      // $$$$$$$$$ GRAB COMMAND $$$$$$$$$$$$$$$$$$$$$
      case 'g': 
        Serial.println("Initiate Retrevial!");
        // Open Claw
        open_claw(50, Claw);
        // Begin Arm Descent
        arm_advance(255);
        delay(ARM_DOWN);
        stop();
        // Arm a lowest position, close claw
        open_claw(0, Claw);
        // Retract Arm so that it is above bin
        arm_back(255);
        delay(ARM_SAMPLE);
        stop();
        // Move Bin into place
        bin_right(255);
        delay(BIN_TIME);
        // Drop sample
        open_claw(60, Claw);
        // Confirm Retreval
        // FIXME - For now just delay 5s
        delay(5*SECOND);
        // Move bin back to left
        bin_left(255);
        delay(BIN_TIME);
        // Return Arm to upright position
        arm_back(255);
        delay(ARM_UP);     
        stop();
        // Return 
        break;
        // $$$$$$$$$$$$$$$$ END GRAB COMMAND $$$$$$$$$$$$$$$$
 
      // E-Stop       
      case 'x':
        stop();
        Serial.println("HOLD UP!");

        break;
      }
    }
    else stop();  
  }
}  
