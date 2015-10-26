/* Keep track of gyro angle over time
 * Connect Gyro to Analog Pin 0
 *
 * Sketch by eric barch / ericbarch.com
 * v. 0.1 - simple serial output
 *
 */

int gyroPin = A0;               //Gyro is connected to analog pin 0
float gyroVoltage = 3.3;         //Gyro is running at 3.3V
float gyroZeroVoltage = 1.23;   //Gyro is zeroed at 1.23V
float gyroSensitivity = .0005;  //Typical sensitivity for our LPY550AL gyro is 0.5 mV/deg/sec(not amplified)
float rotationThreshold  = 1;   //Minimum deg/sec to keep track of - helps with gyro drifting

float currentAngle = 0;          //Keep track of our current angle

void setup() {
  Serial.begin (9600);
}

void loop() {
//  Serial.println(analogRead(gyroPin));
  //This line converts the 0-1023 signal to 0-5V
  float gyroRate = (analogRead(gyroPin) * gyroVoltage) / 1023;
  //This line finds the voltage offset from sitting still
  gyroRate -= gyroZeroVoltage;
  //This line divides the voltage we found by the gyro's sensitivity
  gyroRate /= gyroSensitivity;

//  Serial.println(gyroRate);
  //Ignore the gyro if our angular velocity does not meet our threshold
  if (gyroRate >= rotationThreshold || gyroRate <= -rotationThreshold) {
    //This line divides the value by 100 since we are running in a 10ms loop (1000ms/10ms)
    gyroRate /= 100;
    currentAngle += gyroRate;
  }

  //Keep our angle between 0-359 degrees
  if (currentAngle < 0){
    currentAngle += 360;}
  else if (currentAngle > 359){
    currentAngle -= 360;}

  //DEBUG
//  Serial.println(currentAngle);

  delay(10);
}
