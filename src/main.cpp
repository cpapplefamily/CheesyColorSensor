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
//Sensor 0
#define led_EN_0 8
#define s2_0 10
#define s3_0 9
#define out_0 11

//Sensor 2
#define led_EN_1 4
#define s2_1 6
#define s3_1 5
#define out_1 7

int pdata=0;        

#include "GY_31.h"

GY_31 sensor1(s2_0,s3_0,out_0,led_EN_0);
GY_31 sensor2(s2_1, s3_1, out_1, led_EN_1);

void setup() 
{

   Serial.begin(9600);   
   sensor1.enableLEDs();  
   sensor2.enableLEDs();  
}


void loop()                  //Every 0.2s we select a photodiodes set and read its data
{
   long int t1 = millis();
   
   
   Serial.print("Red 1 value= "); 
   pdata=sensor1.getRED();// pulseIn(out_0,LOW);  //here we wait until "out" go LOW, we start measuring the duration      and stops when "out" is HIGH again
   Serial.print(map(pdata,60,15,0,100));        
   Serial.print("\t");          
   delay(20);

   Serial.print("Blue 1 value= ");
   pdata=sensor1.getBLUE();// pulseIn(out_0,LOW);  //here we wait until "out" go LOW, we start measuring the duration and stops when "out" is HIGH again
   Serial.print(map(pdata,80,11,0,100));          
   Serial.print("\t");          
   delay(20);
   
   Serial.print("Red 2 value= "); 
   pdata=sensor2.getRED();// pulseIn(out_0,LOW);  //here we wait until "out" go LOW, we start measuring the duration      and stops when "out" is HIGH again
   Serial.print(map(pdata,60,15,0,100));        
   Serial.print("\t");          
   delay(20);

   Serial.print("Blue 2 value= ");
   pdata=sensor2.getBLUE();// pulseIn(out_0,LOW);  //here we wait until "out" go LOW, we start measuring the duration and stops when "out" is HIGH again
   Serial.print(map(pdata,80,11,0,100));          
   Serial.print("\t");          
   delay(20);
       


   long int t2 = millis();
   Serial.println("Time taken by the task: "); 
   Serial.print(t2-t1); Serial.println(" milliseconds");
   Serial.println();

   delay(200);
}
