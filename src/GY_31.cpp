#include "GY_31.h"

GY_31::GY_31(int S2pin, int S3pin, int outpin, int ledpin, unsigned long timeout) {
  S2_Pin  = S2pin;
  S3_Pin  = S3pin;
  OUT_Pin = outpin; 
  LED_Pin = ledpin;
  time_out = timeout;

  pinMode(LED_Pin,OUTPUT);
  pinMode(S2_Pin,OUTPUT);
  pinMode(S3_Pin,OUTPUT);
  pinMode(OUT_Pin,INPUT);

  //digitalWrite(EN_Pin, LOW);
};

int GY_31::getRED() {
   digitalWrite(S2_Pin,LOW);        //S2/S3 levels define which set of photodiodes we are using LOW/LOW is for RED LOW/HIGH is for Blue and HIGH/HIGH is for green
   digitalWrite(S3_Pin,LOW);
   dat=pulseIn(OUT_Pin,LOW,time_out);  //here we wait until "out" go LOW, we start measuring the duration      and stops when "out" is HIGH again
   return dat;
};

int GY_31::getBLUE() {
   digitalWrite(S2_Pin,LOW);
   digitalWrite(S3_Pin,HIGH);
   dat=pulseIn(OUT_Pin,LOW,time_out);  //here we wait until "out" go LOW, we start measuring the duration and stops when "out" is HIGH again
   return dat;
};

void GY_31::disableLEDs(){
    digitalWrite(LED_Pin, LOW);
};
void GY_31::enableLEDs(){
    digitalWrite(LED_Pin, HIGH);
};
void GY_31::enableLEDs(boolean enable){
  if(enable){
    digitalWrite(LED_Pin, HIGH);
  }else{
    digitalWrite(LED_Pin, LOW);
  }
};



