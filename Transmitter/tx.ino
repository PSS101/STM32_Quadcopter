#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include<String.h>
#include <Arduino.h>
#include<Wire.h>
RF24 radio(PB_7, PB_8); // CE, CSN



const uint64_t address = 0x5DA871E78AC9AB;
int x,x_prev,throttle,arm,arm_prev,refresh=0;
char buf[5]="";

void setup() {
  

  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MAX);
  radio.setRetries(1, 1); 
  radio.stopListening();
  pinMode(PA_0,INPUT);
  pinMode(PB_0,INPUT);
  pinMode(PB_1,OUTPUT);
  pinMode(PB_2,OUTPUT);
  pinMode(PB_10,OUTPUT);
  Serial.begin(9600);
analogReadResolution(12);
x = analogRead(PA_0);
arm = digitalRead(PB_0);
while(x>=10|| arm==1){
  digitalWrite(PB_10,HIGH);
  digitalWrite(PB_2,LOW);
  digitalWrite(PB_1,LOW);
  x = analogRead(PA_0);
arm = digitalRead(PB_0);
}

}

void loop() {
  
  x = analogRead(PA_0);
 
  arm = digitalRead(PB_0); 
  if(arm==1){
  digitalWrite(PB_10,LOW);
  digitalWrite(PB_1,HIGH);
  digitalWrite(PB_2,LOW);
  x = int(x/200);
  x=x>8?8:x;
  throttle = map(x,0,20,1000,2000);
  itoa(throttle,buf,10);
  }
  else{
  digitalWrite(PB_10,LOW);
  digitalWrite(PB_1,LOW);
  digitalWrite(PB_2,HIGH);
    throttle=800;
    itoa(-1,buf,10);

  }
  radio.write(&buf, sizeof(buf));
 Serial.println(throttle);

  

  
}