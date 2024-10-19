#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include<math.h> 

float pitch_kp=0.5,pitch_ki=0.0405,pitch_kd=0.1,set =0.0;
float roll_kp=0.5,roll_ki=0.0405,roll_kd=0.1;
float pitch_angle2=0.0;
float roll_angle2 =0.0;
int c1,c2,c3;
float C1,C2,C3;
double pitch_p,pitch_i,pitch_d;
double roll_p,roll_i,roll_d;
unsigned long time_pres,time_prev;
String vbat;
Adafruit_MPU6050 mpu;
RF24 radio(PB_7, PB_8); // CE, CSN
Servo m1;
Servo m2;
Servo m3;
Servo m4;

int t=0;
const uint64_t address = 0x5DA871E78AC9AB;

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
  pinMode(PB_6,OUTPUT);
  pinMode(PB_5,OUTPUT);
  pinMode(PB_2,OUTPUT);
  pinMode(PA_3,INPUT);
  pinMode(PA_4,INPUT);
  pinMode(PA_0,INPUT);
   if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
   }
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("");
  m1.writeMicroseconds(1000);
  m2.writeMicroseconds(1000);
  m3.writeMicroseconds(1000);
  m4.writeMicroseconds(1000);
  
  delay(100);

  
}

void loop() {
   analogReadResolution(12);
   for(int i=0;i<10;i++){
   c1 += analogRead(PA_0);
   c2 += analogRead(PA_3);
   c3 += analogRead(PA_4);
   }
   c1=c1/10;c2=c2/10;c3=c3/10;
   C1 = (c1*4.767/4096)-0.05;
   C3 = (c3*13.322/4096)-0.15;
   C2 = (c2*9.24/4096)-0.1;
   String kk = String()+","+String(c2)+","+String(c1);
   vbat = String(C3)+","+String((C3-C2))+","+String((C2-C1))+","+String(C1);
   Serial.println(vbat);
   Serial.println(kk);
   if(C1<= 3.8 || (C2-C1)<=3.8 || (C3-C2)<=3.8){
    digitalWrite(PB_2,HIGH);
    digitalWrite(PB_6,LOW);
    digitalWrite(PB_5,LOW);
    delay(100);
    digitalWrite(PB_2,LOW);
    digitalWrite(PB_6,LOW);
    digitalWrite(PB_5,LOW);

   }
   else{
   if (radio.available()) {
    
    char text[32] = "";
    radio.read(&text, sizeof(text));
    int x = atoi(text);
    if(x<800){
   
   
    digitalWrite(PB_2,LOW);
    digitalWrite(PB_6,HIGH);
    digitalWrite(PB_5,LOW);
    
    m1.writeMicroseconds(1000);
    m2.writeMicroseconds(1000);
    m3.writeMicroseconds(1000);
    m4.writeMicroseconds(1000); 
    t=0;
    }
    else{
    
     digitalWrite(PB_2,LOW);
    digitalWrite(PB_6,LOW);
    digitalWrite(PB_5,HIGH);
    
    
    
 
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float gx = g.gyro.x/131;
    float gy = g.gyro.y/131;
    float gz = g.gyro.z/131;
    float ax =a.acceleration.x/8192 ;
    float ay = a.acceleration.y/8192 ;
    float az =a.acceleration.z/8192 ;
    time_pres = millis();
    float p_angle = atan((-1*ax)/(sqrt(pow(ay,2)+pow(az,2))));
    float r_angle = atan((ay)/(sqrt(pow(ax,2)+pow(az,2))));
    float roll_angle =  (r_angle*(180/PI));
    float pitch_angle = (p_angle*(180/PI));
    if(!(isnan(roll_angle) || isnan(pitch_angle))){
    //complementary filter
    roll_angle = 0.98*(roll_angle+gx*(time_pres-time_prev))+0.02*(roll_angle);
    pitch_angle = 0.98*(pitch_angle+gy*(time_pres-time_prev))+0.02*(pitch_angle);
   
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
    
    if(th1<1000){th1=1000;}else if(th1>2000){th1=2000;} 
    if(th2<1000){th2=1000;}else if(th2>2000){th2=2000;}
    if(th3<1000){th3=1000;}else if(th3>2000){th3=2000;}
    if(th4<1000){th4=1000;}else if(th4>2000){th4=2000;}
 
    m1.writeMicroseconds(th1);
    m2.writeMicroseconds(th2);
    m3.writeMicroseconds(th3);
    m4.writeMicroseconds(th4);
    pitch_angle2 = pitch_angle;
    roll_angle2 = roll_angle;
    time_prev = time_pres;
    String o = String(x)+","+String(set)+","+String(roll_angle)+","+String(pitch_angle);
    Serial.println(o);
    t=0;
    
    }
    
    else{
    m1.writeMicroseconds(1000);
    m2.writeMicroseconds(1000);
    m3.writeMicroseconds(1000);
    m4.writeMicroseconds(1000);     
    }
        delay(100);
    
    


  }
  }
  else if(!radio.available()){
      t++;
    }
  if(t>=50000){
    digitalWrite(PB_2,HIGH);
    digitalWrite(PB_6,LOW);
    digitalWrite(PB_5,LOW);
    
    m1.writeMicroseconds(1000);
    m2.writeMicroseconds(1000);
    m3.writeMicroseconds(1000);
    m4.writeMicroseconds(1000);

  }
  
  }
  c1=0;c2=0;c3=0;
  
}

 
