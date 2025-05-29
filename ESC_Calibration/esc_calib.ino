#include <Servo.h>

Servo m1;
Servo m2;
Servo m3;
Servo m4;
int x = 1000;
String input;
void setup() {
  Serial.begin(9600);
  delay(2000);
   Serial.println("--------------------");
  Serial.println("ESC Calibration");
  m1.attach(PB_1);
  m2.attach(PB_0);
  m3.attach(PA_2);
  m4.attach(PA_1);
  delay(5000); 
  Serial.println("starting calib");
  delay(100);
  
  Serial.println("max throttle");
  m1.writeMicroseconds(2000);
  m2.writeMicroseconds(2000);
  m3.writeMicroseconds(2000);
  m4.writeMicroseconds(2000);
  delay(5000);
  
  Serial.println("min throttle");
  m1.writeMicroseconds(1000);
  m2.writeMicroseconds(1000);
  m3.writeMicroseconds(1000);
  m4.writeMicroseconds(1000);
  
  delay(3000);
  
  Serial.println("Calibration completed");
  Serial.println("--------------------");
}

void loop() {
  if(Serial.available()){
      input = Serial.readString();
      input.trim();
      x = input.toInt();
      m1.writeMicroseconds(x);
      m2.writeMicroseconds(x);
      m3.writeMicroseconds(x);
      m4.writeMicroseconds(x);
      Serial.print("Throttle set to: ");
      Serial.println(x);
  }
    
}

