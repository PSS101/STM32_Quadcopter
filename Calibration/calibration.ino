#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_HMC5883_U.h>
Adafruit_MPU6050 mpu;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
Servo m1;
Servo m2;
Servo m3;
Servo m4;
sensors_event_t event;
int x = 1000,i,mode,inp=0;
float gx_e,gy_e,gz_e,ax_e,ay_e;
float mx,my,mz;
float min_mx,max_mx,min_my,max_my,min_mz,max_mz;
float x_offset,y_offset,z_offset;
String input="";
void stop(){
  m1.writeMicroseconds(1000);
  m2.writeMicroseconds(1000);
  m3.writeMicroseconds(1000);
  m4.writeMicroseconds(1000);    
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000);
  Serial.println("---------------------------");
  Serial.println("Enter 1 to Calibrate ESC'S");
  Serial.println("Enter 2 to Calibrate IMU");
  Serial.println("Enter 3 to Calibrate Magnetometer");
  Serial.println("Enter 4 to Exit");
  Serial.println("---------------------------");
  delay(1000);
  while(input==""){
    if(Serial.available()){
      input = Serial.readString();
      input.trim();
      mode = input.toInt();
    }
  }
  
  switch(mode){
    case 1:
      Serial.println("--------------------");
      Serial.println("ESC Calibration");
      m1.attach(PB_1,2000);
      m2.attach(PB_0,2000);
      m3.attach(PA_2,2000);
      m4.attach(PA_1,2000);
      
      delay(5000); 
      Serial.println("Starting calibration");
      delay(100);
      
      Serial.println("Writing max throttle");
      m1.writeMicroseconds(2000);
      m2.writeMicroseconds(2000);
      m3.writeMicroseconds(2000);
      m4.writeMicroseconds(2000);
      delay(5000);
      
      Serial.println("Writing min throttle");
      m1.writeMicroseconds(1000);
      m2.writeMicroseconds(1000);
      m3.writeMicroseconds(1000);
      m4.writeMicroseconds(1000);
      
      delay(3000);
      
      Serial.println("Calibration completed");
      Serial.println("--------------------");
      while(x>=1000){
        if(Serial.available()){
           m1.writeMicroseconds(x);
           m2.writeMicroseconds(x);
           m3.writeMicroseconds(x);
           m4.writeMicroseconds(x);
           Serial.print("Throttle set to: ");
           Serial.println(x);
          input = Serial.readString();
          input.trim();
          x = input.toInt();       
        }
      }
      break;

      case 2:
          Serial.println("---------------------------");
          Serial.println("IMU Calibration");
          Serial.println("Place IMU on a flat surface");
          Serial.println("Starting calibration");
          if (!mpu.begin(0x68)) {
            Serial.println("Failed to find MPU6050 chip");
          }
          mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
          mpu.setGyroRange(MPU6050_RANGE_250_DEG);
          mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);
          sensors_event_t a, g, temp;
          delay(1000);
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
          break;
      
      case 3:
        delay(1000);
        Serial.println("---------------------------");
        mag.begin();
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
          delay(10);
        }
        Serial.println("Calibration Completed");
        x_offset = (min_mx+max_mx)/2;
        y_offset = (min_my+max_my)/2;
        z_offset = (min_mz+max_mz)/2;
        Serial.println("x offset:"+(String)x_offset);
        Serial.println("y offset:"+(String)y_offset);
        Serial.println("z offset:"+(String)z_offset);
        Serial.println("---------------------------");
        break;

      default:
        Serial.println("Input must be between 1-4");

      
}
}

void loop() {
  // put your main code here, to run repeatedly:

}
