#include <Arduino.h>
#include "FastLED.h"


#define NUM_LEDS      8
#define LED_TYPE   WS2812B
#define COLOR_ORDER   GRB
#define DATA_PIN        3
//#define CLK_PIN       4
#define VOLTS          5
#define MAX_MA       1000

    /*
  GY-31-TCS3200-Color-Recognition-Sensor-Module
  Modified on 29 Dec 2020
  by Amir Mohammad Shojaee @ Electropeak
  Home

  based on create.arduino.cc Example
*/

//Sensor 1
#define led_EN_1 8
#define s2_1 10
#define s3_1 9
#define out_1 11

//Sensor 2
#define led_EN_2 4
#define s2_2 6
#define s3_2 5
#define out_2 7

//Sensor 3
#define led_EN_3 17
#define s2_3 15
#define s3_3 16
#define out_3 14

//Sensor 4
#define led_EN_4 21
#define s2_4 19
#define s3_4 20
#define out_4 18

//Sensor 5
#define led_EN_5 29
#define s2_5 27
#define s3_5 28
#define out_5 26

//Sensor 6
#define led_EN_6 33
#define s2_6 31
#define s3_6 32
#define out_6 30

//Sensor 7
#define led_EN_7 37
#define s2_7 35
#define s3_7 36
#define out_7 34

//Sensor 8
#define led_EN_8 41
#define s2_8 39
#define s3_8 40
#define out_8 38



int reddata=0;        
int bluedata=0;   
bool enplot = true;     

#include "GY_31.h"

GY_31 sensor1(s2_1, s3_1, out_1, led_EN_1);
GY_31 sensor2(s2_2, s3_2, out_2, led_EN_2);
GY_31 sensor3(s2_3, s3_3, out_3, led_EN_3);
GY_31 sensor4(s2_4, s3_4, out_4, led_EN_4);
GY_31 sensor5(s2_5, s3_5, out_5, led_EN_5);
GY_31 sensor6(s2_6, s3_6, out_6, led_EN_6);
GY_31 sensor7(s2_7, s3_7, out_7, led_EN_7);
GY_31 sensor8(s2_8, s3_8, out_8, led_EN_8);

CRGBArray<NUM_LEDS> leds;


void TestSensor1(){
   long int t1 = millis();
   
   reddata=sensor1.getRED();
   bluedata=sensor1.getBLUE();
   if(!enplot){
      Serial.print("Red 1 value= "); 
      Serial.print(map(reddata,700,75,0,100));        
   }else{
      //Serial.print("Red: "); 
      Serial.print(map(bluedata,700,75,0,100)); 
      Serial.print(","); 

      Serial.println(map(reddata,700,75,0,100));        
   };
   if(!enplot){
      Serial.print("\t"); 
      long int t2 = millis();
      Serial.println("Time taken by the task: "); 
      Serial.print(t2-t1); Serial.println(" milliseconds");
      Serial.println();   
   }
   delay(200);
}

void marqueLED(){
   unsigned long stepdelay = 1000;

   sensor1.enableLEDs();
   delay(stepdelay);
   sensor1.disableLEDs();
   sensor2.enableLEDs();
   delay(stepdelay);
   sensor2.disableLEDs();
   sensor3.enableLEDs();
   delay(stepdelay);
   sensor3.disableLEDs();
   sensor4.enableLEDs();
   delay(stepdelay);
   sensor4.disableLEDs();
   sensor5.enableLEDs();
   delay(stepdelay);
   sensor5.disableLEDs();
   sensor6.enableLEDs();
   delay(stepdelay);
   sensor6.disableLEDs();
   sensor7.enableLEDs();
   delay(stepdelay);
   sensor7.disableLEDs();
   sensor8.enableLEDs();
   delay(stepdelay);
   sensor8.disableLEDs();
}

void ScanValues(){
   
   long int t1 = millis();
   
   
   Serial.print("Red 1 value= "); 
   reddata=sensor1.getRED();
   Serial.print(map(reddata,60,15,0,100));        
   Serial.print("\t");          
   delay(20);

   Serial.print("Blue 1 value= ");
   reddata=sensor1.getBLUE();
   Serial.print(map(reddata,80,11,0,100));          
   Serial.print("\t");          
   delay(20);
   
   Serial.print("Red 2 value= "); 
   reddata=sensor2.getRED();
   Serial.print(map(reddata,60,15,0,100));        
   Serial.print("\t");          
   delay(20);

   Serial.print("Blue 2 value= ");
   reddata=sensor2.getBLUE();
   Serial.print(map(reddata,80,11,0,100));          
   Serial.print("\t");          
   delay(20);
       


   long int t2 = millis();
   Serial.println("Time taken by the task: "); 
   Serial.print(t2-t1); Serial.println(" milliseconds");
   Serial.println();

   delay(200);
}

void testSensor(GY_31 sensor0){
   long int t1 = millis();
   
   reddata=sensor0.getRED();
   bluedata=sensor0.getBLUE();
   if(!enplot){
      Serial.print("Red 1 value= "); 
      Serial.print(map(reddata,700,75,0,100));        
   }else{
      //Serial.print("Red: "); 
      Serial.print(map(bluedata,700,75,0,100)); 
      Serial.print(","); 

      Serial.println(map(reddata,700,75,0,100));        
   };
   if(!enplot){
      Serial.print("\t"); 
      long int t2 = millis();
      Serial.println("Time taken by the task: "); 
      Serial.print(t2-t1); Serial.println(" milliseconds");
      Serial.println();   
   }
   
}

void setup() {

   Serial.begin(9600);   
   sensor1.enableLEDs(false);  
   sensor2.enableLEDs(false);  
   sensor3.enableLEDs(false);  
   sensor4.enableLEDs(false);  
   sensor5.enableLEDs(false);  
   sensor6.enableLEDs(false);  
   sensor7.enableLEDs(false);  
   sensor8.enableLEDs(false);  

   FastLED.setMaxPowerInVoltsAndMilliamps( VOLTS, MAX_MA);
   FastLED.addLeds<LED_TYPE,DATA_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
   FastLED.setBrightness(10);
}
String sensor;
void loop(){
   leds[0] = CRGB::Blue;
   leds[1] = CRGB::Red;
   leds[2] = CRGB::Blue;
   leds[3] = CRGB::Red;
   leds[4] = CRGB::Blue;
   leds[5] = CRGB::Red;
   leds[6] = CRGB::Blue;
   leds[7] = CRGB::Red;
   FastLED.show();
   sensor = "sensor" + 1;
   testSensor(sensor1);
   testSensor(sensor2);
   testSensor(sensor3);
   testSensor(sensor4);
   testSensor(sensor5);
   testSensor(sensor6);
   delay(200);
}
