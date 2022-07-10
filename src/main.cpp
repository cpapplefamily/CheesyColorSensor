#include <Arduino.h>

    /*
  GY-31-TCS3200-Color-Recognition-Sensor-Module
  Modified on 29 Dec 2020
  by Amir Mohammad Shojaee @ Electropeak
  Home

  based on create.arduino.cc Example
*/

//#define s0 8       
//#define s1 9
#define s2 10
#define s3 9
#define out 11
#define led_EN 8

int data=0;        

void setup() 
{
   //pinMode(s0,OUTPUT);   
   //pinMode(s1,OUTPUT);
   pinMode(s2,OUTPUT);
   pinMode(s3,OUTPUT);
   pinMode(out,INPUT);
   pinMode(led_EN, OUTPUT);

   Serial.begin(9600);   
   
   //digitalWrite(s0,HIGH); //Putting S0/S1 on HIGH/HIGH levels means the output frequency scalling is at 100% (recommended)
   //digitalWrite(s1,HIGH); 
   digitalWrite(led_EN,HIGH); 

   
}

void loop()                  //Every 0.2s we select a photodiodes set and read its data
{

   digitalWrite(s2,LOW);        //S2/S3 levels define which set of photodiodes we are using LOW/LOW is for RED LOW/HIGH is for Blue and HIGH/HIGH is for green
   digitalWrite(s3,LOW);
   Serial.print("Red value= "); 
   data=pulseIn(out,LOW);  //here we wait until "out" go LOW, we start measuring the duration      and stops when "out" is HIGH again
   Serial.print(map(data,60,15,0,100));        
   Serial.print("\t");          
   delay(20);
                      
   digitalWrite(s2,LOW);
   digitalWrite(s3,HIGH);
   Serial.print("Blue value= ");
   data=pulseIn(out,LOW);  //here we wait until "out" go LOW, we start measuring the duration and stops when "out" is HIGH again
   Serial.print(map(data,80,11,0,100));          
   Serial.print("\t");          
   delay(20);

   digitalWrite(s2,HIGH);
   digitalWrite(s3,HIGH);
   Serial.print("Green value= ");
   data=pulseIn(out,LOW);  //here we wait until "out" go LOW, we start measuring the duration and stops when "out" is HIGH again
   Serial.print(map(data,80,20,0,100));          
   Serial.print("\t");          
   delay(20);

   Serial.println();

   delay(200);
}
