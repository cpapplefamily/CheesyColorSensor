#include <Arduino.h>
#include "FastLED.h"


#define NUM_LEDS      9
#define LED_TYPE   WS2812B
#define COLOR_ORDER   GRB
#define DATA_PIN        3
//#define CLK_PIN       4
#define VOLTS          5
#define MAX_MA       1000

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

#define NUM_SENSORS 8

enum SensorState {NONE, RED, BLUE, GREEN};
//Eight storage location for the sensor States
SensorState sensorState[NUM_SENSORS];
SensorState _sensorState[NUM_SENSORS];

int reddata=0;        
int bluedata=0;   
bool enplot = true;     

#include "GY_31.h"

GY_31 sensors[] = {  {s2_1, s3_1, out_1, led_EN_1}, 
                     {s2_2, s3_2, out_2, led_EN_2}, 
                     {s2_3, s3_3, out_3, led_EN_3}, 
                     {s2_4, s3_4, out_4, led_EN_4}, 
                     {s2_5, s3_5, out_5, led_EN_5}, 
                     {s2_6, s3_6, out_6, led_EN_6}, 
                     {s2_7, s3_7, out_7, led_EN_7}, 
                     {s2_8, s3_8, out_8, led_EN_8}};


/* GY_31 sensor1(s2_1, s3_1, out_1, led_EN_1);
GY_31 sensors[1](s2_2, s3_2, out_2, led_EN_2);
GY_31 sensors[2](s2_3, s3_3, out_3, led_EN_3);
GY_31 sensors[3](s2_4, s3_4, out_4, led_EN_4);
GY_31 sensors[4](s2_5, s3_5, out_5, led_EN_5);
GY_31 sensors[5](s2_6, s3_6, out_6, led_EN_6);
GY_31 sensors[6](s2_7, s3_7, out_7, led_EN_7);
GY_31 sensors[7](s2_8, s3_8, out_8, led_EN_8); */

CRGBArray<NUM_LEDS> leds;


void TestSensor1(){
   long int t1 = millis();
   
   reddata=sensors[0].getRED();
   bluedata=sensors[0].getBLUE();
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

   sensors[0].enableLEDs();
   delay(stepdelay);
   sensors[0].disableLEDs();
   sensors[1].enableLEDs();
   delay(stepdelay);
   sensors[1].disableLEDs();
   sensors[2].enableLEDs();
   delay(stepdelay);
   sensors[2].disableLEDs();
   sensors[3].enableLEDs();
   delay(stepdelay);
   sensors[3].disableLEDs();
   sensors[4].enableLEDs();
   delay(stepdelay);
   sensors[4].disableLEDs();
   sensors[5].enableLEDs();
   delay(stepdelay);
   sensors[5].disableLEDs();
   sensors[6].enableLEDs();
   delay(stepdelay);
   sensors[6].disableLEDs();
   sensors[7].enableLEDs();
   delay(stepdelay);
   sensors[7].disableLEDs();
}

void ScanValues(){
   
   long int t1 = millis();
   
   Serial.print("Red 1 value= "); 
   reddata=sensors[0].getRED();
   Serial.print(map(reddata,60,15,0,100));        
   Serial.print("\t");          
   delay(20);

   Serial.print("Blue 1 value= ");
   reddata=sensors[0].getBLUE();
   Serial.print(map(reddata,80,11,0,100));          
   Serial.print("\t");          
   delay(20);
   
   Serial.print("Red 2 value= "); 
   reddata=sensors[1].getRED();
   Serial.print(map(reddata,60,15,0,100));        
   Serial.print("\t");          
   delay(20);

   Serial.print("Blue 2 value= ");
   reddata=sensors[1].getBLUE();
   Serial.print(map(reddata,80,11,0,100));          
   Serial.print("\t");          
   delay(20);
       


   long int t2 = millis();
   Serial.println("Time taken by the task: "); 
   Serial.print(t2-t1); Serial.println(" milliseconds");
   Serial.println();

   delay(200);
}

SensorState testSensor(GY_31 sensor, CRGB led){
   SensorState state;
   reddata=map(sensor.getRED(),700,75,0,100);
   bluedata=map(sensor.getBLUE(),700,75,0,100);
   int redThresh = 70;
   int blueThresh = 70;
   if(reddata>redThresh){
      led = CRGB::Red;
      state = SensorState::RED;
   }else if(bluedata>blueThresh){
      led = CRGB::Blue;
      state = SensorState::BLUE;
   }else{
      led = CRGB::Black;
      state = SensorState::NONE;
   }
   if(!enplot){
      Serial.print("Red value= "); 
      Serial.print(reddata);   
      Serial.print("\t");   
      Serial.print("Blue value= "); 
      Serial.println(bluedata);      
   }else{
      Serial.print(bluedata); 
      Serial.print(","); 
      Serial.println(reddata);        
   };

   return state;
}


void setup() {
   Serial.begin(9600); 

   for(int i=0; i<NUM_SENSORS ; i++){
      sensorState[i] = SensorState::NONE; 
      _sensorState[i] = SensorState::NONE; 
      sensors[i].enableLEDs(false);
   }   
   
   FastLED.setMaxPowerInVoltsAndMilliamps( VOLTS, MAX_MA);
   FastLED.addLeds<LED_TYPE,DATA_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
   FastLED.setBrightness(10);
}

long int hartBeatTck = 0;
long int currentTime = 0;
boolean hartBeat = false;

void loop(){
   long int currentTime = millis();

/*    sensorState[0] = testSensor(sensors[0], leds[1]);
   sensorState[1] = testSensor(sensors[1], leds[2]);
   sensorState[2] = testSensor(sensors[2], leds[3]);
   sensorState[3] = testSensor(sensors[3], leds[4]);
   sensorState[4] = testSensor(sensors[4], leds[5]);
   sensorState[5] = testSensor(sensors[5], leds[6]);
   sensorState[6] = testSensor(sensors[6], leds[7]);
   sensorState[7] = testSensor(sensors[7], leds[8]); */

   for(int i=0; i<NUM_SENSORS ; i++){
      sensorState[i] = testSensor(sensors[i], leds[i+1]);
      if(_sensorState[i]!=sensorState[i]){
         _sensorState[i]=sensorState[i];
      } 
   }

   //Flip Hartbeat LED
   if(currentTime > hartBeatTck){
      hartBeatTck = currentTime + 500;
      hartBeat = !hartBeat;
   }
   if(hartBeat){
      leds[0] = CRGB::White;
   }else{
      leds[0] = CRGB::Black;
   }

   FastLED.show();
   delay(200);
}
