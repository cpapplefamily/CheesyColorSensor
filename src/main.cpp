#include <Arduino.h>
#include "FastLED.h"
//#include <Servo.h>
#include <Adafruit_TiCoServo.h>
#include <ArduinoJson.h>

#define Serial1_FMS_Amp Serial1
#define Serial_Debug Serial

//#define ALLIANCE CRGB::Red
#define ALLIANCE CRGB::Blue

#define test_Note_ID_Pin 5
//Servo myservo;  // create servo object to control a servo
Adafruit_TiCoServo myservo;  // create servo object to control a servo

#define PWM_PIN        2
int potpin = A0;  // analog pin used to connect the potentiometer
int val;

//LEDs start at count 0
#define NUM_LEDS      200
#define NUM_AMP1_LEDS_START 1
#define NUM_AMP1_LEDS_LEN   46
#define NUM_AMP2_LEDS_START 47
#define NUM_AMP2_LEDS_LEN   46 //last LED 92
#define NUM_COOP_LEDS_START 93
#define NUM_COOP_LEDS_LEN   48 //Last LED 139
#define NUM_SPEAKER_LEDS_START 140
#define NUM_SPEAKER_LEDS_LEN   48 //Last LED 139

#define LED_TYPE   WS2812B
#define COLOR_ORDER   GRB
#define DATA_PIN        3
//#define CLK_PIN       4
#define VOLTS           5
#define MAX_MA       1000

//Sensor Freq Scaling
#define S0pin  31
#define S1pin  30
/* S0 S1 
      L  L  Power Down  0
      L  H  2%          1
      H  L  20%         2
      H  H  100%        3*/
int sensorPowerScale = 1;
int redThresh = 95;
int blueThresh = 90;

//Sensor 1
#define led_EN_1  8
#define s2_1      10
#define s3_1      9
#define out_1     11

//Sensor 2
#define led_EN_2  4
#define s2_2      6
#define s3_2      5
#define out_2     7

//Cooperation Button
#define coopBTN_led 26
#define coopBTN_input 27

//Cooperation Button
#define amplifyBTN_led 28
#define amplifyBTN_input 29

#define NUM_AMP_SENSORS 1

#define LOWER_SCALE_LIM 0
#define UPPER_SCALE_LIM 100

#define SIGN_OF_LIFE_AR 140
#define SIGN_OF_LIFE_PI SIGN_OF_LIFE_AR + 1


boolean EN_CALIBRATE_PLOT = false;


enum SensorState {
                  NONE,
                  RED,
                  BLUE,
                  GREEN
                  };

enum MatchState {
               PreMatch,         // 20
               StartMatch,       // 21
               WarmupPeriod,     // 22
               AutoPeriod,       // 23
               PausePeriod,      // 24
               TeleopPeriod,     // 25
               PostMatch,        // 26
               TimeoutActive,    // 27
               PostTimeout,      // 28
               None              // 29
               };

/*
matchState_char_msg_map = {
    "0": '20',
    "1": '21',
    "2": '22',
    "3": '23',
    "4": '24',
    "5": '25',
    "6": '26',
    "7": '27',
    "8": '28',
    "9": '29'
}

bankedAmpNotes_char_msg_map = {
    "0": '30',
    "1": '31',
    "2": '32'
}

speakerState_char_msg_map = {
    "0": '40',
    "1": '41',
    "2": '42'
}

coopState_char_msg_map = {
    "0": '50',
    "1": '51'
}
*/

CRGB MatchState_LEDs = CRGB::Black;
int matchState_int; //Not set
int bankedAmpNotes_int;
int coopState_int;
int speakerState_int;

//Eight storage location for the sensor States
SensorState ampSensorState[NUM_AMP_SENSORS];
SensorState ampSensorScored_ONS[NUM_AMP_SENSORS];


#include "ColorTrigger.h"

ColorTrigger ampTrigger[NUM_AMP_SENSORS];


#include "Debouncer.h"
double debounceTime_RED = 60;  //Sensor must be on or off
double debounceTime_BLUE = 60;  //Sensor must be on or off
Debouncer::DebounceType debounceType = Debouncer::DebounceType::kBoth;

Debouncer debouncAMP[NUM_AMP_SENSORS] = {
                                             {debounceTime_RED, debounceType}
                                             };



//**********************//
// Set Up Color sensors //
//**********************//
int reddata=0;        
int bluedata=0;   

#include "GY_31.h"
unsigned long timeout_micros = 5000;// 2000 = 2.00ms//unsigned long = 1000000UL

GY_31 ampSensors[] = {  
                     {s2_1, s3_1, out_1, led_EN_1, timeout_micros},
                     {s2_2, s3_2, out_2, led_EN_2, timeout_micros}
                     
                     };


//**********************//
// Set Up led strip     //
//**********************//
CRGBArray<NUM_LEDS> leds;


//REturn the Sensor State and set the LED color CGRB byReference
//SensorState getSensorState(GY_31 sensor, CRGB& led){
 
bool plot_raw_data = false;
//REturn the Sensor State
SensorState getSensorState(GY_31 sensor){
   SensorState state;
   
   //Get Sensor Color Reading
   if(plot_raw_data){
      reddata=sensor.getRED();
      bluedata=sensor.getBLUE();
   }else{
      reddata=map(sensor.getRED(),700,75,LOWER_SCALE_LIM,UPPER_SCALE_LIM);
      bluedata=map(sensor.getBLUE(),700,75,LOWER_SCALE_LIM,UPPER_SCALE_LIM);
      // the lower value for easier plotting. Value idiles @ -300
      // valus above Upper scale limit indicates no sensor
      if(reddata<-10 | reddata > UPPER_SCALE_LIM){
         //reddata = -10;
      }
      if(bluedata<-10 | bluedata > UPPER_SCALE_LIM){
       //bluedata = -10;
      }
   }
   
   //Set some Trigger Threasholds
   if((reddata > redThresh) & (reddata > bluedata)){
      state = SensorState::RED;
   }else if((bluedata > blueThresh) & (bluedata > reddata)){
      state = SensorState::BLUE;
   }else{
      state = SensorState::NONE;
   }
   if(EN_CALIBRATE_PLOT){
      Serial_Debug.print(bluedata); 
      Serial_Debug.print(","); 
      Serial_Debug.println(reddata);        
   }; 

   return state;
}

void configerSensorLED(boolean Enable_All){
   //Set sensor state and turn on all LED's
   for(int i=0; i<NUM_AMP_SENSORS ; i++){
      ampSensorState[i] = SensorState::NONE; 
      ampSensorScored_ONS[i] = SensorState::NONE; 
      ampTrigger[i].reset();
      ampSensors[i].enableLEDs(Enable_All);
   }   
      
}
void setup() {
   // Initialize "debug" serial port
  // The data rate must be much higher than the "link" serial port
   Serial_Debug.begin(115200);

   // Initialize the "link" serial port
   // Use a low data rate to reduce the error ratio
   Serial1_FMS_Amp.begin(9600);

   myservo.attach(PWM_PIN);  // attaches the servo on PWM_PIN to the servo object

   boolean Enable_All;
   if(EN_CALIBRATE_PLOT){
      Enable_All = false;
   }else{
      Enable_All = true;
   }
   //Set sensor state and turn on all LED's
   configerSensorLED(Enable_All);
   pinMode(S0pin,OUTPUT);
   pinMode(S1pin,OUTPUT);
   /* S0 S1 
      L  L  Power Down  0
      L  H  2%          1
      H  L  20%         2
      H  H  100%        3*/
   
   digitalWrite(S0pin, (sensorPowerScale >> 0) & 1);
   digitalWrite(S1pin, (sensorPowerScale >> 1) & 1);

   matchState_int = 99;
   bankedAmpNotes_int = 99;
   coopState_int = 99;
   speakerState_int = 99;
    
   //Set up RGB LED strip lights
   FastLED.setMaxPowerInVoltsAndMilliamps( VOLTS, MAX_MA);
   FastLED.addLeds<LED_TYPE,DATA_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
   FastLED.setBrightness(10);
 
   //Set up coop buttons
   pinMode(coopBTN_led,OUTPUT);
   pinMode(coopBTN_input,INPUT_PULLUP);
   
   //Set up amplify buttons
   pinMode(amplifyBTN_led,OUTPUT);
   pinMode(amplifyBTN_input,INPUT_PULLUP);

}

void Setup_CALIBRATE_PLOT(int incomingByte){
   for(int i=0; i<NUM_LEDS; i++){
      leds[i] = CRGB::Black;
   }

   configerSensorLED(false); 

   if((incomingByte>=0) & (incomingByte <= (NUM_AMP_SENSORS - 1))){
      ampSensors[incomingByte].enableLEDs(true);
      Serial_Debug.println("Upper");
   }

}
/**
 * @param fill //Start of fill
 * @param block //Length of block
*/
void fill_Block(int fill, int block, CRGB color){
   for(int l=fill; l < (fill+block); l++){
      leds[l] = color;
    }
}

CRGB setMatchStateLED(int matchState){
   switch (matchState){
      case 20: //PreMatch
         return CRGB::Green;
         break;
      case 21 ... 25: //StartMatch
         return CRGB::Black;
         break;
      case 26: // PostMatch
         return CRGB::Violet;
         break;
      case 27 ... 28: // TimeoutActive
         return CRGB::Black;
         break;
      default:
         return CRGB::Black;
         break;
   }
}

void run_Agitator(boolean run){
   if(run){
      val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023)
      val = map(val, 0, 1023, 45, 135);     // scale it for use with the servo (value between 0 and 180)
   }else{
      val = 90;
   }
   myservo.write(val);   
}


const byte numChars = 32;
char receivedChars[numChars];   // an array to store the received data

boolean newData = false;

void recvWithEndMarker() {
    static byte ndx = 0;
    char endMarker = '\n';
    char rc;
    
    while (Serial1_FMS_Amp.available() > 0 && newData == false) {
        rc = Serial1_FMS_Amp.read();
        
        if (rc != endMarker) {
            receivedChars[ndx] = rc;
            ndx++;
            if (ndx >= numChars) {
                ndx = numChars - 1;
            }
        }
        else {
            receivedChars[ndx] = '\0'; // terminate the string
            ndx = 0;
            newData = true;
        }
    }
}

/**
 * Main Loop
 * 
 * 
 */

long int hartBeatTck = 0;
long int currentTime = 0;
boolean hartBeat = false;

/* goal_char_msg_map = {
    "W": '{ "type": "W" }',
    "R": '{ "type": "R" }',
    "P": '{ "type": "P" }',
    "O": '{ "type": "O" }
} */
int incoming_Debug_Byte = 0; // for incoming Debug serial data
int incoming_FMS_Byte = 0; // for incoming FMS serial data
int int_Calibrate = 0; // for incoming serial data
bool signOfLifePi = false;
bool signOfLifePi_State = false;

void loop(){
   long int currentTime = millis();

   // Check the Debug serial port
   if (Serial_Debug.available() > 0) {
      // read the incoming byte:
      // send a 9 to the console to start Calibration Mode
      // Send a 0-numver of sensors to sellect the desired sensor
      // Send a 8 to end calibration
      incoming_Debug_Byte = Serial_Debug.read();
        
      if(((incoming_Debug_Byte) == '\r' || (incoming_Debug_Byte) == '\n')){
         //Do Nothing
      }else{
         //Check if incomming byto is a calibration command
         //Convert to int
         int_Calibrate = incoming_Debug_Byte - '0';
         if(int_Calibrate == 9){
            EN_CALIBRATE_PLOT = true;           
         }else if(int_Calibrate == 8){
            EN_CALIBRATE_PLOT = false; 
            configerSensorLED(true);           
         }
      }
   }

   //Check the FMS Serial Port
   recvWithEndMarker();
   if (newData == true) {
       //Serial.print("This just in ... ");
       //Serial.println(receivedChars);

      // read the incoming byte:
      //incomingByte = Serial1.parseInt();
      incoming_FMS_Byte = 0;
      incoming_FMS_Byte = atoi(receivedChars);

      Serial_Debug.print("incoming Number: ");
      Serial_Debug.println(incoming_FMS_Byte);

      newData = false;
      
      signOfLifePi = true;
   
      if(((incoming_FMS_Byte) == '\r' || (incoming_FMS_Byte) == '\n')){
         //Do Nothing
      }else{
         //Check if incoming Byte is Match State
         if((incoming_FMS_Byte) >= 20 & (incoming_FMS_Byte <= 29)){
            matchState_int = incoming_FMS_Byte;
            Serial_Debug.print("Incoming  matchState_int: ");
            Serial_Debug.println(matchState_int);
         }

         //Check if incoming Byte is BankedAmpNotes
         if((incoming_FMS_Byte >= 30) & (incoming_FMS_Byte <= 39)){
            bankedAmpNotes_int = incoming_FMS_Byte;
         }
         /* if((incoming_FMS_Byte >= 40) & (incoming_FMS_Byte <= 49)){
            speakerState_int = incoming_FMS_Byte;
         }
         if((incoming_FMS_Byte >= 50) & (incoming_FMS_Byte <= 59)){
            coopState_int = incoming_FMS_Byte;
         }
          if((incoming_FMS_Byte >= 100) & (incoming_FMS_Byte <= 200)){
            Serial_Debug.println("*****************");
            float num = (incoming_FMS_Byte-100);
            Serial_Debug.print("num: ");
            Serial_Debug.println(num);
            if(speakerState_int != 40 ){
               amp_percent = float( (incoming_FMS_Byte-100)/100.0);
               Serial_Debug.print("amp_percent: ");
               Serial_Debug.println(amp_percent);
            }
            
         } */
      }
   }
   //set the led backgound color
   MatchState_LEDs = setMatchStateLED(matchState_int);

   // Run Agitator if match running
   if(matchState_int>20 & matchState_int<26){
         run_Agitator(true);
   }else{
      run_Agitator(false);
   }

   if(EN_CALIBRATE_PLOT){
      if((int_Calibrate>=0) & (int_Calibrate <= (NUM_AMP_SENSORS - 1))){
         ampSensorState[int_Calibrate] = getSensorState(ampSensors[int_Calibrate]);
      }   
   }else{
      //Loop through upper sensors
      for(int i=0; i < NUM_AMP_SENSORS ; i++){
      
         long int startsence = micros();  //For Time diagnostics

         ampSensorState[i] = getSensorState(ampSensors[i]);
      
         boolean printSensor_time = false;
         if((i == 0) & (printSensor_time)){ 
            Serial_Debug.print("Sensor 0 ");
            Serial_Debug.print(micros() - startsence);
            Serial_Debug.println(" us");
         }
      
         if(debouncAMP[i].calculate((ampSensorState[i] == SensorState::RED))){
            //Serial_Debug.println("Is red");
            if((ampSensorScored_ONS[i]!= SensorState::RED)){
               //Serial_Debug.println("**********RED**************");
               Serial1_FMS_Amp.print("A");
               ampSensorScored_ONS[i]= SensorState::RED;
               //delay(3000);
               //fill_Block(NUM_AMP1_LEDS_START + (i * NUM_AMP1_LEDS_LEN), NUM_AMP1_LEDS_LEN, CRGB::Red); 
               leds[0] = ALLIANCE;
            }
            
         }else{
            ampSensorScored_ONS[i]= SensorState::NONE;
            //fill_Block(NUM_AMP1_LEDS_START + (i * NUM_AMP1_LEDS_LEN), NUM_AMP1_LEDS_LEN, MatchState_LEDs);  
            leds[0] = CRGB::Black;   
         };   
      }
   
      switch (bankedAmpNotes_int){
      case 31: //
         fill_Block(NUM_AMP1_LEDS_START , NUM_AMP1_LEDS_LEN, ALLIANCE);
         break;
      case 32: //
         fill_Block(NUM_AMP1_LEDS_START , NUM_AMP1_LEDS_LEN, ALLIANCE);
         fill_Block(NUM_AMP2_LEDS_START , NUM_AMP2_LEDS_LEN, ALLIANCE);
         break;
      default:
         fill_Block(NUM_AMP1_LEDS_START , NUM_AMP1_LEDS_LEN, MatchState_LEDs);
         fill_Block(NUM_AMP2_LEDS_START , NUM_AMP2_LEDS_LEN, MatchState_LEDs);
         break;
      }

      
      switch (coopState_int){
      case 41: //
         fill_Block(NUM_COOP_LEDS_START , NUM_COOP_LEDS_LEN, CRGB::Yellow);
         break;
      default:
         fill_Block(NUM_COOP_LEDS_START , NUM_COOP_LEDS_LEN, MatchState_LEDs);
         break;
      }

      switch (speakerState_int){
      case 51:
         fill_Block(NUM_SPEAKER_LEDS_START , NUM_SPEAKER_LEDS_LEN, ALLIANCE);
         break;
      default:
         fill_Block(NUM_SPEAKER_LEDS_START , NUM_SPEAKER_LEDS_LEN, MatchState_LEDs);
         break;
      }
      

   }

   //****************************************************TEMP
   if (!digitalRead(coopBTN_input)){
      digitalWrite(coopBTN_led, HIGH);
      leds[test_Note_ID_Pin] = CRGB::Red;
      Serial1_FMS_Amp.print("C");
   }else{
      leds[test_Note_ID_Pin] = CRGB::Black;
      digitalWrite(coopBTN_led, LOW);
   }                
   if (!digitalRead(amplifyBTN_input)){
      digitalWrite(amplifyBTN_led, HIGH);
      leds[test_Note_ID_Pin] = CRGB::Red;
      Serial1_FMS_Amp.print("K");
   }else{
      leds[test_Note_ID_Pin] = CRGB::Black;
      digitalWrite(amplifyBTN_led, LOW);
   }
   //***********************************************************
  
   //Sing Of Life Arduino
   CRGB SOL = CRGB::White;
   long int now = millis();
   //Hartbeat LED Color
   if((now - currentTime)>=50){
      SOL = CRGB::Red;
   }else if ((now - currentTime)>=25){
      SOL = CRGB::Yellow;
   }else{
      SOL = CRGB::White;
   }

   if(currentTime > hartBeatTck){
      hartBeatTck = currentTime + 500;
      hartBeat = !hartBeat;
   }

   if(hartBeat){
      leds[SIGN_OF_LIFE_AR] = SOL;
   }else{
      leds[SIGN_OF_LIFE_AR] = CRGB::Black;
   }

   //Sing Of Life Raspberry Pi
   if(signOfLifePi){
      signOfLifePi_State = !signOfLifePi_State;
      signOfLifePi = false;
   }

   if(signOfLifePi_State){
      leds[SIGN_OF_LIFE_PI] = CRGB::White;
   }else{
      leds[SIGN_OF_LIFE_PI] = CRGB::Black;
   }

   // Show Match state on LEDs 3-10
   //leds[matchState_int-18] = CRGB::White;

   FastLED.show();
   
   //As of 8/4/2022 this program ran at 20ms This assures this is the min
   while((millis() - currentTime)<20){}
   
   //ToDo add warning if over ?ms
}

