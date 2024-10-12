#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include<String.h>
RF24 radio(7, 8); // CE, CSN

const byte address[6] = "00001";

void setup() {
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
  pinMode(A2,INPUT);
 

}

void loop() {
  int x = analogRead(A2);
  
char buf[32];
  int throttle = map(x,0,1023,1000,2000);

  itoa(throttle,buf,10);

  radio.write(&buf, sizeof(buf));
  
}
