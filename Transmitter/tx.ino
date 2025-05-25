#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include<String.h>
#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
RF24 radio(PB_7, PB_8); // CE, CSN


LiquidCrystal_I2C lcd(0x27,20,4);

const uint64_t address = 0x5DA871E78AC9AB;
int x,x_prev,throttle,arm,arm_prev,refresh=0;
char buf[32]="";
void setup() {
  lcd.init();
   lcd.backlight();
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
  pinMode(PA_0,INPUT);
  pinMode(PB_0,INPUT);
  //pinMode(4,OUTPUT);

   Serial.begin(9600);
   /*
     x = analogRead(A2);
  while(x>=50){
    
    lcd.setCursor(0,0);
    lcd.print("Set Throttle to");
    lcd.setCursor(0,1);
    lcd.print("zero");
      x = analogRead(A2);
  }
*/
}

void loop() {
  x = analogRead(PA_0);
  arm = digitalRead(PB_0); 
  lcd.setCursor(0,0);
  if(arm==1){
  //digitalWrite(4,HIGH);

  x = int(x/50);
  throttle = map(x,0,20,1000,2000);
   lcd.print("Armed");
  String t = "Throttle: "+String(int((throttle-1000)/10))+"%";
  itoa(throttle,buf,10);
  lcd.setCursor(0,1);
  lcd.print(t);
  refresh = x!=x_prev?1:0;
  x_prev=x;
  }
  else{
    //digitalWrite(4,LOW);
    throttle=800;
    itoa("-1",buf,10);
    lcd.print("Disrmed");
  }
  radio.write(&buf, sizeof(buf));
  refresh = (arm!=arm_prev || refresh);
  if(refresh){
    lcd.clear();
  }
  arm_prev=arm;


  

  
}
