#include <Adafruit_HMC5883_U.h>
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
 sensors_event_t event;
 float mx,my,mz;
 float min_mx,max_mx,min_my,max_my,min_mz,max_mz;
 float x_offset,y_offset,z_offset;
 int i;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000);
   Serial.println("---------------------------");
  Serial.println("Magnetometer Calibration Starting");
  Serial.println("data points to be collected: 2000");
  Serial.println("Rotate magnetometer in all directions");
  delay(100);
  for(i=0;i<=2000;i++){
    mag.getEvent(&event);
    mx = (event.magnetic.x);
    my = (event.magnetic.y);
    mz = (event.magnetic.z);
    min_mx = mx<min_mx?mx:min_mx;
    max_mx = mx>max_mx?mx:max_mx;
    min_my = mx<min_my?mx:min_my;
    max_my = mx>max_my?mx:max_my;
    min_mz = mx<min_mz?mx:min_mz;
    max_mz = mx>max_mz?mx:max_mz;
    String s = String(mx)+","+String(my)+","+String(mz);
    Serial.println(s);
    delay(50);
  }
  Serial.println("Calibration Completed");
  x_offset = (min_mx+max_mx)/2;
  y_offset = (min_my+max_my)/2;
  z_offset = (min_mz+max_mz)/2;
  Serial.println("x offset:"+(String)x_offset);
  Serial.println("y offset:"+(String)y_offset);
  Serial.println("z offset:"+(String)z_offset);
  Serial.println("---------------------------");

void loop() {
  // put your main code here, to run repeatedly:


}
