#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include<String.h>
#include <Arduino.h>
#include <Arduino.h>
#include <U8g2lib.h>
U8G2_SSD1306_128X64_NONAME_2_SW_I2C u8g2 (U8G2_R0, A5, A4);
RF24 radio(7, 8); // CE, CSN

const uint64_t address = 0x5DA871E78AC9AB;

void setup() {
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
  pinMode(A2,INPUT);
  pinMode(A5,INPUT);
  pinMode(4,OUTPUT);




}

void loop() {
  int x = analogRead(A2);
  int sb = digitalRead(5);
  
char buf[32];
  if(sb==1){
  digitalWrite(4,HIGH);
  int throttle = map(x,0,1023,1000,2000);

  String t = String(int((throttle-1000)/100))+"%";
  itoa(throttle,buf,10);
  }
  else{
    digitalWrite(4,LOW);
    itoa("-1",buf,10);
  }
  radio.write(&buf, sizeof(buf));
  
 

  

  
}
