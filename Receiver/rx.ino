#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include<math.h> 
#define min_throttle 1000
#define max_throttle 2000
#define dec  -0.516667

unsigned long time_pres,time_prev;
double pitch_p,pitch_i,pitch_d;
double yaw_p,yaw_i,yaw_d;
double roll_p,roll_i,roll_d;
double yaw_pid,roll_pid,pitch_pid;
float gx,gy,gz,ax,ay,az,mx,my;
float gx_e=0.0,gy_e=0.0,gz_e=0.0,ax_e=0.0,ay_e=0.0,az_e=0.0;
float p_angle=0.0,r_angle=0.0,y_angle=0.0,roll_a,pitch_a,pitch_angle=0.0,roll_angle=0.0,yaw_angle=0.0;
float pitch_angle_prev=0.0,roll_angle_prev =0.0,yaw_angle_prev =0.0;
float pitch_kp=0.0,pitch_ki=0.0,pitch_kd=0.0,set =0.0,yaw_set=0;
float roll_kp=3.5,roll_ki=0.0,roll_kd=0.5;
float yaw_kp=0.0,yaw_ki=0.0,yaw_kd=0.0;

float roll_kalman,roll_kalman_prev,roll_uncertainity,roll_uncertainity_prev,roll_kalman_gain;
float pitch_kalman,pitch_kalman_prev,pitch_uncertainity,pitch_uncertainity_prev,pitch_kalman_gain;

float dt,C1,C2,C3;
int c1,c2,c3;
int low_bat=0;
int th1,th2,th3,th4,th;
bool valid_angle,check_angle,is_timed_out;
String vbat;

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
Adafruit_MPU6050 mpu;
RF24 radio(PB_7, PB_8); // CE, CSN
Servo m1;
Servo m2;
Servo m3;
Servo m4;
sensors_event_t a, g, temp;
 sensors_event_t event;
char input[5] = "";
int time_out=0;
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
  delay(2000);
  m2.writeMicroseconds(1500);
  delay(200);
  m2.writeMicroseconds(1000);
  delay(2000);
  m3.writeMicroseconds(1500);
  delay(200);
  m3.writeMicroseconds(1000);
  delay(2000);
  m4.writeMicroseconds(1500);
  delay(200);
  m4.writeMicroseconds(1000);
}

void calibrate(){
  sensors_event_t a, g, temp;
  for(int i=0;i<2000;i++){
    mpu.getEvent(&a, &g, &temp);
    gx_e += g.gyro.x;
    gy_e += g.gyro.y;
    gz_e += g.gyro.z;
    ax_e +=a.acceleration.x;
    ay_e += a.acceleration.y;
    digitalWrite(PB_9,HIGH);
    digitalWrite(PB_6,HIGH);
    digitalWrite(PB_5,LOW);
    
  }
  gx_e/= 2000;
  gy_e/= 2000;
  gz_e/= 2000;
  ax_e/= 2000;
  ay_e/= 2000;
}

void setup() {
  stop();
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MAX);
  radio.startListening();
  m1.attach(PB_1,1000,2000,1000);
  m2.attach(PB_0,1000,2000,1000);
  m3.attach(PA_2,1000,2000,1000);
  m4.attach(PA_1,1000,2000,1000);
  stop();
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
  mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);
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
  delay(1000);
  //Serial.begin(115200);
  //check motors
  calibrate();
  delay(100);
  //check();
  delay(100);
}

void loop() {
   //check for low battery voltage
   /*
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
   }*/
   
   //check if rf module is connected
   if (radio.available()) {
    
    //
    
    radio.read(&input, sizeof(input));
    th = atoi(input);
    //Serial.println(x);
    time_out=0;
    if(th<800){
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
    
        //delay(100);

  }
  }
   //check if rf module is not connected
  else if(!radio.available()){
      time_out++;
    }

      if(th>1000 && !is_timed_out){ 
    //sensor reading
    
    mpu.getEvent(&a, &g, &temp);
    gx = (g.gyro.x)-gx_e;
    gy = (g.gyro.y)-gy_e;
    gz = (g.gyro.z)-gz_e;
    ax = (a.acceleration.x)-ax_e;
    ay = (a.acceleration.y)-ay_e;
    az = (a.acceleration.z);
    mag.getEvent(&event);
    mx = (event.magnetic.x);
    my = (event.magnetic.y);
   
    
    
    //angle calculation
    p_angle = atan((-1*ax)/(sqrt(pow(ay,2)+pow(az,2))));
    r_angle = atan((ay)/(sqrt(pow(ax,2)+pow(az,2))));
    roll_a =  (r_angle*(180/PI));
    pitch_a= (p_angle*(180/PI));
    y_angle = atan2(my,mx);
    yaw_angle = y_angle * (180 / PI);
    yaw_angle = yaw_angle-90;
    if(yaw_angle<0){
      yaw_angle+=360;
    }
    yaw_angle+= dec;
    check_angle = (roll_angle<=180 && roll_angle>=-180) && (pitch_angle<=90 && pitch_angle>=-90);
    valid_angle = !(isnan(roll_angle) || isnan(pitch_angle) || isnan(yaw_angle));
    
    if(valid_angle && !is_timed_out && check_angle){

     
   
    time_pres = micros();
    dt = ((float)(time_pres-time_prev))/1000000.0;
    if(dt<0.000001){dt=0.000001;}

    //roll 1D kalman filter
    roll_kalman = roll_kalman_prev+dt*gx;
    roll_uncertainity = roll_uncertainity_prev+(dt*dt)*(4*4);
    roll_kalman_gain = roll_uncertainity/(roll_uncertainity+(1.5*1.5));
    roll_kalman = roll_kalman+roll_kalman_gain*(roll_a-roll_kalman);
    roll_uncertainity = (1-roll_kalman_gain)*roll_uncertainity;
    roll_angle = roll_kalman;
    roll_kalman_prev = roll_kalman;
    roll_uncertainity_prev = roll_uncertainity;

    //pitch 1D kalman filter
    pitch_kalman = pitch_kalman_prev+dt*gx;
    pitch_uncertainity = pitch_uncertainity_prev+(dt*dt)*(4*4);
    pitch_kalman_gain = pitch_uncertainity/(pitch_uncertainity+(1.5*1.5));
    pitch_kalman = pitch_kalman+pitch_kalman_gain*(pitch_a-pitch_kalman);
    pitch_uncertainity = (1-pitch_kalman_gain)*pitch_uncertainity;
    pitch_angle = pitch_kalman;
    pitch_kalman_prev = pitch_kalman;
    pitch_uncertainity_prev = pitch_uncertainity;

    //pitch pid
    pitch_p = (set-pitch_angle);
    pitch_i += (set-pitch_angle)*dt;
    pitch_d = ((set-pitch_angle_prev)-(set-pitch_angle))/dt;
    pitch_pid = (pitch_p*pitch_kp)+(pitch_i*pitch_ki)+(pitch_d*pitch_kd);
    pitch_pid = constrain(pitch_pid,-100,100);

    //roll pid
    roll_p = (set-roll_angle);
    roll_i += (set-roll_angle)*dt;
    roll_d = ((set-roll_angle_prev)-(set-roll_angle))/dt;
    roll_pid = (roll_p*roll_kp)+(roll_i*roll_ki)+(roll_d*roll_kd);
    roll_pid = constrain(roll_pid,-200,200);

    //yaw pid
    yaw_p = (yaw_set-yaw_angle);
    yaw_i += (yaw_set-yaw_angle)*dt;
    yaw_d = ((yaw_set-yaw_angle_prev)-(yaw_set-yaw_angle))/dt;
    yaw_pid = (yaw_p*yaw_kp)+(yaw_i*yaw_ki)+(yaw_d*yaw_kd);
    yaw_pid = constrain(yaw_pid,-100,100);
    //speed mapping to motors
    th1 = th-pitch_pid+roll_pid+yaw_pid;
    th2 = th-pitch_pid-roll_pid-yaw_pid;
    th3 = th+pitch_pid-roll_pid+yaw_pid;
    th4 = th+pitch_pid+roll_pid-yaw_pid;
    
    th1 = constrain(th1,min_throttle,max_throttle)+0.5;
    th2 = constrain(th2,min_throttle,max_throttle)+0.5;
    th3 = constrain(th3,min_throttle,max_throttle)+0.5;
    th4 = constrain(th4,min_throttle,max_throttle)+0.5;

    m1.writeMicroseconds(th1);
    m2.writeMicroseconds(th2);
    m3.writeMicroseconds(th3);
    m4.writeMicroseconds(th4);

    pitch_angle_prev = pitch_angle;
    roll_angle_prev = roll_angle;
    yaw_angle_prev = yaw_angle;
    time_prev = time_pres;

     }
     
       
    //String o = String(x)+", "+String(roll_angle)+", "+String(th1)+", "+String(th2)+", "+String(th3)+", "+String(th4);
    //String o2 = String(roll_angle)+","+String(set)+","+String(roll_pid)+","+String(th);
    //Serial.println(o2);
  
    }
    
    else{
    stop();    
    
    }


  if(is_timed_out){
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
  is_timed_out = time_out>=400?1:0;
  //c1=0;c2=0;c3=0;
  //Serial.println(millis());
}
  

 
 