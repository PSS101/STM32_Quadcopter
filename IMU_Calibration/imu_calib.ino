
#include <Adafruit_MPU6050.h>
Adafruit_MPU6050 mpu;
float gx_e,gy_e,gz_e,ax_e,ay_e;
int i;
void setup() {
  // put your setup code here, to run once:
  Serial.println("---------------------------");
  Serial.println("IMU Calibration");
  Serial.println("Place IMU on a flat surface");
  delay(100);
  Serial.println("Starting calibration");
  sensors_event_t a, g, temp;
  for(int i=0;i<10000;i++){
    mpu.getEvent(&a, &g, &temp);
    gx_e += g.gyro.x;
    gy_e += g.gyro.y;
    gz_e += g.gyro.z;
    ax_e +=a.acceleration.x;
    ay_e += a.acceleration.y;
    
  }
  gx_e/= 10000;
  gy_e/= 10000;
  gz_e/= 10000;
  ax_e/= 10000;
  ay_e/= 10000;
}
  Serial.println("---------------------------");
  Serial.println("Calibration Completed");
  Serial.println("---------------------------");
  Serial.println("gx offset:"+(String)gx_e);
  Serial.println("gy offset:"+(String)gx_e);
  Serial.println("gx offset:"+(String)gy_e);
  Serial.println("gz offset:"+(String)gz_e);
  Serial.println("ax offset:"+(String)ax_e);
  Serial.println("ay offset:"+(String)ay_e);
  Serial.println("---------------------------");

void loop() {
  // put your main code here, to run repeatedly:

}
