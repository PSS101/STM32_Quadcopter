#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
float pitch_kp=1,pitch_ki=0.05,pitch_kd=0.6,set =0.0;
float roll_kp=1,roll_ki=0.05,roll_kd=0.6;
float pitch_angle2=0.0;
float roll_angle2 =0.0;
double pitch_p,pitch_i,pitch_d;
double roll_p,roll_i,roll_d;
unsigned long time_pres,time_prev;
Adafruit_MPU6050 mpu;
RF24 radio(PB_7, PB_8); // CE, CSN
Servo m1;
Servo m2;
Servo m3;
Servo m4;


const byte address[6] = "00001";

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  m1.attach(PA_1);
  m2.attach(PA_2);
  m3.attach(PB_0);
  m4.attach(PB_1);
  pinMode(PB_2,OUTPUT);
   if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
   }
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("");
  delay(100);
  
}

void loop() {
  if (radio.available()) {
    digitalWrite(PB_2,HIGH);
    char text[32] = "";
    radio.read(&text, sizeof(text));
    
    int x = atoi(text);
    Serial.print(x);
    Serial.print(",");
 
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

  
    float ax =a.acceleration.x/8192 ;
    float ay = a.acceleration.y/8192 ;
    float az =a.acceleration.z/8192 ;
    time_pres = millis();
    float p_angle = atan((-1*ax)/(sqrt(pow(ay,2)+pow(az,2))));
    float r_angle = atan((ay)/(sqrt(pow(ax,2)+pow(az,2))));
    float roll_angle =  (r_angle*(180/PI));
    float pitch_angle = (p_angle*(180/PI));

   


    //pitch pid
    pitch_p = (set-pitch_angle);
    pitch_i += ((set-pitch_angle)*(time_pres-time_prev))/1000;
    pitch_d = (((set-pitch_angle2)-(set-pitch_angle))/(time_pres-time_prev))*1000;
    double pitch_pid = pitch_p*pitch_kp+pitch_ki*pitch_i+pitch_d*pitch_kd;
    //roll pid
    roll_p = (set-roll_angle);
    roll_i += ((set-roll_angle)*(time_pres-time_prev))/1000;
    roll_d = (((set-roll_angle2)-(set-roll_angle))/(time_pres-time_prev))*1000;
    double roll_pid = roll_p*roll_kp+roll_ki*roll_i+roll_d*roll_kd;

    int th1 = x-pitch_pid+roll_pid;
    int th2 = x-pitch_pid-roll_pid;
    int th3 = x+pitch_pid-roll_pid;
    int th4 = x+pitch_pid+roll_pid;
    //int th1 = x+roll_pid;
    //int th2 = x-roll_pid;
    //int th3 = x-roll_pid;
    //int th4 = x+roll_pid;

    if(th1<1000){th1=1000;}else if(th1>2000){th1=2000;} 
    if(th2<1000){th2=1000;}else if(th2>2000){th2=2000;}
    if(th3<1000){th3=1000;}else if(th3>2000){th3=2000;}
    if(th4<1000){th4=1000;}else if(th4>2000){th4=2000;}
    m1.writeMicroseconds(th1);
    m2.writeMicroseconds(th2);
    m3.writeMicroseconds(th3);
    m4.writeMicroseconds(th4);
    Serial.print(set);
    Serial.print(",");
    Serial.print(roll_angle);
    Serial.print(",");
    Serial.print(pitch_angle);
    Serial.println("");

    delay(100);
    pitch_angle2 = pitch_angle;
    roll_angle2 = roll_angle;
    time_prev = time_pres;
    


}



  }

 
