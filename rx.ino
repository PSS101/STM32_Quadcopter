#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include<math.h> 
unsigned long time_pres,time_prev;
double pitch_p,pitch_i,pitch_d;
double yaw_p,yaw_i,yaw_d;
double roll_p,roll_i,roll_d;
double yaw_pid,roll_pid,pitch_pid;
float gx,gy,gz,ax,ay,az,mx,my;
float gx_e,gy_e,gz_e,ax_e,ay_e,az_e;
float p_angle=0.0,r_angle=0.0,y_angle=0.0,pitch_angle=0.0,roll_angle=0.0,yaw_angle=0.0;

float pitch_kp=0.5,pitch_ki=0.04,pitch_kd=0.0,set =0.0,yaw_set=0;
float roll_kp=0.5,roll_ki=0.04,roll_kd=0.0;
float yaw_kp=0.0,yaw_ki=0.0,yaw_kd=0.0;

float pitch_angle2=0.0;
float roll_angle2 =0.0;
float yaw_angle2 =0.0;
float C1,C2,C3;
int c1,c2,c3;
int low_bat;
int th1,th2,th3,th4;

String vbat;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
float dec = -0.58333;
Adafruit_MPU6050 mpu;
RF24 radio(PB_7, PB_8); // CE, CSN
Servo m1;
Servo m2;
Servo m3;
Servo m4;

int t=0;
const uint64_t address = 0x5DA871E78AC9AB;
//const byte address[6] = "00001";
void stop(){
  m1.writeMicroseconds(1000);
  m2.writeMicroseconds(1000);
  m3.writeMicroseconds(1000);
  m4.writeMicroseconds(1000);    
}

void check(){
  delay(100);
  m1.writeMicroseconds(1500);
  delay(200);
  m1.writeMicroseconds(1000);
  delay(1000);
  m2.writeMicroseconds(1500);
  delay(200);
  m2.writeMicroseconds(1000);
  delay(1000);
  m3.writeMicroseconds(1500);
  delay(200);
  m3.writeMicroseconds(1000);
  delay(1000);
  m4.writeMicroseconds(1500);
  delay(200);
  m4.writeMicroseconds(1000);
}

void calibrate(){
  sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    gx_e = g.gyro.x/131;
    gy_e = g.gyro.y/131;
    gz_e = g.gyro.z/131;
    ax_e =a.acceleration.x/8192;
    ay_e = a.acceleration.y/8192;
    az_e =a.acceleration.z/8192;
}

void setup() {
  delay(100);
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
  pinMode(PB_9,OUTPUT);
  pinMode(PA_3,INPUT);
  pinMode(PA_4,INPUT);
  pinMode(PA_0,INPUT);
   if (!mpu.begin(0x68)) {
    Serial.println("Failed to find MPU6050 chip");
   }
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  mag.begin();
  //set yaw
  sensors_event_t event; 
  mag.getEvent(&event);
  float s_mx = (event.magnetic.x);
  float s_my = (event.magnetic.y);
  float s_angle = atan2(s_my,s_mx);
  s_angle = s_angle * (180 / PI);
  s_angle-=90;
  if(s_angle<0){s_angle+=360;}
  yaw_set = ((int)(s_angle/90))*90;

  stop();
  Serial.begin(9600);
  //check motors
  //check();
  delay(100);
  calibrate();
}

void loop() {
   //check for low battery voltage
   analogReadResolution(12);
   for(int i=0;i<10;i++){
   c1 += analogRead(PA_0);
   c2 += analogRead(PA_3);
   c3 += analogRead(PA_4);
   }
   c1=c1/10;c2=c2/10;c3=c3/10;
   C1 = (c1*4.767/4096)-0.05;
   C3 = (c3*13.322/4096)+0.82;
   C2 = (c2*9.24/4096)-0.1;
   //vbat = String(C3)+","+String((C3-C2))+","+String((C2-C1))+","+String(C1);
   //Serial.println(vbat);
   
   if(C1<= 3.8 || (C2-C1)<=3.8 || (C3-C2)<=3.8){
    if(C1< 3.7 || (C2-C1)<3.7 || (C3-C2)<3.7){
      low_bat = 2;
    }
    else{
      low_bat = 1;
    }
   }
   
   //check if rf module is connected
   if (radio.available()) {
    
    char text[32] = "";
    radio.read(&text, sizeof(text));
    int x = atoi(text);
    //Serial.println(x);
    if(x<800){
    if(low_bat==0){
    digitalWrite(PB_9,LOW);
    digitalWrite(PB_6,HIGH);
    digitalWrite(PB_5,LOW);
    }
    else{
    digitalWrite(PB_9,HIGH);
    digitalWrite(PB_6,HIGH);
    digitalWrite(PB_5,LOW);     
    }
    stop();
    t=0;
    }
    else{
    if(low_bat==0){
     digitalWrite(PB_9,LOW);
    digitalWrite(PB_6,LOW);
    digitalWrite(PB_5,HIGH);
    }
      else{
    digitalWrite(PB_9,HIGH);
    digitalWrite(PB_6,LOW);
    digitalWrite(PB_5,HIGH);     
    } 
    
    
    //sensor reading
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    gx = (g.gyro.x/131)-gx_e;
    gy = (g.gyro.y/131)-gy_e;
    gz = (g.gyro.z/131)-gz_e;
    ax = (a.acceleration.x/8192)-ax_e;
    ay = (a.acceleration.y/8192)-ay_e;
    az = (a.acceleration.z/8192)-az_e;
    sensors_event_t event; 
    mag.getEvent(&event);
    mx = (event.magnetic.x);
    my = (event.magnetic.y);
   
    
    
    //angle calculation
    p_angle = atan((-1*ax)/(sqrt(pow(ay,2)+pow(az,2))));
    r_angle = atan((ay)/(sqrt(pow(ax,2)+pow(az,2))));
    roll_angle =  (r_angle*(180/PI));
    pitch_angle = (p_angle*(180/PI));
    y_angle = atan2(my,mx);
    yaw_angle = y_angle * (180 / PI);
    yaw_angle = yaw_angle-90;
    if(yaw_angle<0){
      yaw_angle+=360;
    }
    yaw_angle+= dec;
    if(!(isnan(roll_angle) || isnan(pitch_angle) || isnan(yaw_angle))){\

    //complementary filter
    time_pres = millis();
    roll_angle = 0.98*(roll_angle+gx*(time_pres-time_prev))+0.02*(roll_angle);
    pitch_angle = 0.98*(pitch_angle+gy*(time_pres-time_prev))+0.02*(pitch_angle);
    
    //pitch pid
    time_pres = millis();
    pitch_p = (set-pitch_angle);
    pitch_i += ((set-pitch_angle)*(time_pres-time_prev))/1000;
    pitch_d = (((set-pitch_angle2)-(set-pitch_angle))/(time_pres-time_prev))*1000;
    pitch_pid = (pitch_p*pitch_kp)+(pitch_i*pitch_ki)+(pitch_d*pitch_kd);

    //roll pid
    time_pres = millis();
    roll_p = (set-roll_angle);
    roll_i += ((set-roll_angle)*(time_pres-time_prev))/1000;
    roll_d = (((set-roll_angle2)-(set-roll_angle))/(time_pres-time_prev))*1000;
    roll_pid = (roll_p*roll_kp)+(roll_i*roll_ki)+(roll_d*roll_kd);
    
    //yaw pid
    time_pres = millis();
    yaw_p = (yaw_set-yaw_angle);
    yaw_i += ((yaw_set-yaw_angle)*(time_pres-time_prev))/1000;
    yaw_d = (((yaw_set-yaw_angle2)-(yaw_set-yaw_angle))/(time_pres-time_prev))*1000;
    yaw_pid = (yaw_p*yaw_kp)+(yaw_i*yaw_ki)+(yaw_d*yaw_kd);
    
    //speed mapping to motors
    th1 = x-pitch_pid+roll_pid+yaw_pid;
    th2 = x-pitch_pid-roll_pid-yaw_pid;
    th3 = x+pitch_pid-roll_pid+yaw_pid;
    th4 = x+pitch_pid+roll_pid-yaw_pid;
    
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
    yaw_angle2 = yaw_angle;
    time_prev = time_pres;
    //String o = String(x)+","+String(set)+","+String(roll_angle)+","+String(pitch_angle)+","+String(yaw_angle);
    //String o2 = String(yaw_set)+','+String(roll_pid)+","+String(pitch_pid)+","+String(yaw_pid);
    //Serial.println(o);
    //Serial.println(o2);
    t=0;    
    }
    
    else{
    stop();    
    }
        //delay(100);

  }
  }
   //check if rf module is not connected
  else if(!radio.available()){
      t++;
    }
  if(t>=1500){
    if(low_bat==0){
    digitalWrite(PB_9,HIGH);
    digitalWrite(PB_6,LOW);
    digitalWrite(PB_5,LOW);
    }
    else{
    digitalWrite(PB_9,HIGH);
    digitalWrite(PB_6,LOW);
    digitalWrite(PB_5,HIGH);     
    }
    stop();

  }
  c1=0;c2=0;c3=0;
  
}
  


 

 
