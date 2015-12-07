#include <Wire.h>
#include <L3G.h>
L3G gyro;

void dc_offset() {
  Serial.begin(9600);
Wire.begin();

while (!gyro.init());
gyro.enableDefault();

int sampleNum=500;
int dc_offset=0;

//Calculate initial DC offset and noise level of gyro
for(int n=0;n<sampleNum;n++){
gyro.read();
dc_offset+=(int)gyro.g.z;
}
dc_offset=dc_offset/sampleNum;
//print dc offset and noise level
Serial.println();
Serial.print("DC Offset: ");
Serial.print(dc_offset);
Serial.println();

}
