

/************************************
 * Ver 1.5 for Arduino UNO and ESP32
 * 
 * 
 */

#define OLED_Display
#define ESP32 // vs. Arduino Uno

#ifdef ESP32
  #include <Preferences.h>
  #include <WiFi.h>
  #include <WiFiUdp.h>
  #include "AsyncUDP.h"
  #include <ArduinoOTA.h>

  const char * ssid = "********";
  const char * password = "********";
  
  AsyncUDP udps;
  AsyncUDP udpc;


  Preferences preferences;

  unsigned char* udpdata; // declare a pointer to a char array
  unsigned int udplength;


  unsigned char led1on[]  = {0x2f, 0x31, 0x2f, 0x6c, 0x65, 0x64, 0x31, 0x2F, 0x00, 0x00, 0x00, 0x00, 0x2c, 0x66, 0x00, 0x00, 0x3f, 0x80, 0x00, 0x00 };
  unsigned char led1off[] = {0x2f, 0x31, 0x2f, 0x6c, 0x65, 0x64, 0x31, 0x2F, 0x00, 0x00, 0x00, 0x00, 0x2c, 0x66, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  unsigned char* pled1; 
  
  unsigned char led2on[]  = {0x2f, 0x31, 0x2f, 0x6c, 0x65, 0x64, 0x32, 0x2F, 0x00, 0x00, 0x00, 0x00, 0x2c, 0x66, 0x00, 0x00, 0x3f, 0x80, 0x00, 0x00 };
  unsigned char led2off[] = {0x2f, 0x31, 0x2f, 0x6c, 0x65, 0x64, 0x32, 0x2F, 0x00, 0x00, 0x00, 0x00, 0x2c, 0x66, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  unsigned char* pled2; 
  
  unsigned char led3on[]  = {0x2f, 0x31, 0x2f, 0x6c, 0x65, 0x64, 0x33, 0x2F, 0x00, 0x00, 0x00, 0x00, 0x2c, 0x66, 0x00, 0x00, 0x3f, 0x80, 0x00, 0x00 };
  unsigned char led3off[] = {0x2f, 0x31, 0x2f, 0x6c, 0x65, 0x64, 0x33, 0x2F, 0x00, 0x00, 0x00, 0x00, 0x2c, 0x66, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  unsigned char* pled3; 
  
  unsigned char led4on[]  = {0x2f, 0x31, 0x2f, 0x6c, 0x65, 0x64, 0x34, 0x2F, 0x00, 0x00, 0x00, 0x00, 0x2c, 0x66, 0x00, 0x00, 0x3f, 0x80, 0x00, 0x00 };
  unsigned char led4off[] = {0x2f, 0x31, 0x2f, 0x6c, 0x65, 0x64, 0x34, 0x2F, 0x00, 0x00, 0x00, 0x00, 0x2c, 0x66, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  unsigned char* pled4; 

  enum {
  leftFADER, rightFADER
};

 //fader1
  unsigned char fader1[] = {0x2F, 0x31, 0x2F, 0x66, 0x61, 0x64, 0x65, 0x72, 0x31, 0x00, 0x00, 0x00, 0x2C, 0x66, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // last four bytes are the float value of the fader position
  unsigned char* pfader1; 
  float fader1Pos;
  float oldfader1Pos;

  //fader2
  unsigned char  fader2[] = {0x2F, 0x31, 0x2F, 0x66, 0x61, 0x64, 0x65, 0x72, 0x32, 0x00, 0x00, 0x00, 0x2C, 0x66, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // last four bytes are the float value of the fader position
  unsigned char* pfader2;
  float fader2Pos;
  float oldfader2Pos;

  byte * tmp;

  byte result = 0;
  
  
  String txt= "";
  
#else
  #include <EEPROM.h>
#endif


#include <Bounce2.h>
 

#ifdef OLED_Display
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>
  
  // Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
  // The pins for I2C are defined by the Wire-library.
  // On an arduino UNO:       A4(SDA), A5(SCL)
  // On an arduino MEGA 2560: 20(SDA), 21(SCL)
  // On an arduino LEONARDO:   2(SDA),  3(SCL), ...
  
  #define SCREEN_WIDTH 128 // OLED display width, in pixels
  #define SCREEN_HEIGHT 32 // OLED display height, in pixels
  #define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
  #define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

  Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#endif

// Arduino Pin-mapping

//IBT_2 BTS7960 based motor control module
// VCC connects to 5V of the arduino PCB

#ifdef ESP32 // pins

  //const byte R_IS = 1; 
  //const byte L_IS = 4;
  //const byte R_EN = 7;
  //const byte L_EN = 5;
  const byte R_PWM = 26;
  const byte L_PWM = 25;

// Hall Sensor module KY-003 (A3144 chip) w/ digital output
// Note: Hall sensor module KY-035 (AH49E chip) was not used 

const byte interruptPin = 34;

const byte LED1 = 17;
const byte LED2 = 16;

const byte emulatedPushButton1 = 32;
const byte emulatedPushButton2 = 33;

const byte butPin[] = {             // Pin-Nummern der angeschlossenen Tasten
  27, 14
};

const int ADC_pin = 36;


#else

  const byte R_IS = 1; 
  const byte L_IS = 4;
  const byte R_EN = 7;
  const byte L_EN = 5;
  const byte R_PWM = 3;
  const byte L_PWM = 6;

// Hall Sensor module KY-003 (A3144 chip) w/ digital output
// Note: Hall sensor module KY-035 (AH49E chip) was not used 

const byte interruptPin = 2;

const byte LED1 = 10;
const byte LED2 = 11;

const byte butPin[] = {             // Pin-Nummern der angeschlossenen Tasten
  12, 13
};

const int ADC_pin = A0;

#endif


#ifdef ESP32

// setting PWM properties
const int freq = 1000;
const int R_PWMChannel = 0;
const int L_PWMChannel = 1;
const int resolution = 8; //bit

//uint8_t* OSC_Rx;
int  OSC_len;
String OSCcmd = "";

#endif



const byte PositionAllUp = 0;  // all up
unsigned int  PositionAllDown  = 0;                // this detined the allDown position. It is set during calibration
float currentPosition = 0;              // this will be tracked during up/down movement. The range is min. PositionAllUp and max. PossitionAllDown.
float oldPosition = 0;
unsigned int writes = 0; // byte writes = 0;
int POSdir = +1;
bool engineRunning = false;
String TMP;




// There are only two push-buttons to control and calibrate the system
// 
// normal mode:
//    "move blinds up": single push of up-button
//    "move blinds down: single push of down-button
//    "stop at current possition if moving": single push of up-or donw-button
// 
// calibration mode: 
//    "enter calibration mode": push both buttons simultaniously until sound is audible ( 4 low-tone beeps followed by 5 high-tone beeps)
//    "calibrate upper blind possition": 
//    "calibrate lower blind possition": 

unsigned long now;                  // Aktueller Zeitpunkt

// _Entprellung

#define INPUTMODE INPUT             // INPUT oder INPUT_PULLUP


#define NUM_BUTTONS sizeof(butPin)  // Die Anzahl der Tasten durch die Anzahl der Bytes des Arrays butPin ermitteln (wird automatisch definiert)
Bounce debouncer[NUM_BUTTONS];      // Mehrere Bounce-Instanzen erstellen
bool bState[NUM_BUTTONS];           // Speichert den Zustand des Pins
enum {
  UNHOLD, HOLD
};
byte bCmd[NUM_BUTTONS];             // Speichert Pinbefehle, welche dem Sketch übergeben werden
enum {
  WAIT, PUSH, RELEASE, S_HOLD, M_HOLD
};
byte bCount;                        // Anzahl der gleichzeitig gedrückt gehaltenen Tasten

byte bMenue;
byte bMenueOld;
enum {
  MOVE, CAL
};

byte bCommand;
byte previous_bCommand;
enum {
  UP, DOWN, STOP
};

byte calState;
enum {
  upper, lower
};




bool requestSTOP = false;


String ver = __FILE__;
String ver2, ver3;

float Sample;
float Voltage;
long lowVoltage= 0;

long cal_wait_time =0;
long emulate_Hall_time =0;
bool EmulatedHallTrigger = false;


unsigned char fader1step= 0;
unsigned char fader2step= 0;
float fadersteps[]={.2, .5, .8};


long oldCrone1time = 0;
long oldCrone2time = 0;
bool ISR_hallFlag = false;

bool ISR_GPIO= true; 

String helper;


// various pre-sets
  
int cal_wait_duration = 3000; // ms  the is the wait time during cal(-ibration mode. x ms after the last key push to switch from uper limit cal to lowerlimit cal abd then x ms to finish cal.


/*
 * *********************************************************
 * *********************************************************
 * *********************************************************
 */

String check = "";



/*
 * *********************************************************
 * *************** SETUP ***********************************
 * *********************************************************
 */


void setup() { //§1
  Serial.begin(115200);

#ifdef ESP32

  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);

  pinMode(emulatedPushButton1, OUTPUT);
  pinMode(emulatedPushButton2, OUTPUT);
  digitalWrite(emulatedPushButton1, LOW);
  digitalWrite(emulatedPushButton2, LOW);
  

#else
  // Motorsteuerung
  pinMode(R_IS, OUTPUT);
  pinMode(R_EN, OUTPUT);
  pinMode(L_IS, OUTPUT);
  pinMode(L_EN, OUTPUT);

  digitalWrite(R_IS, LOW);
  digitalWrite(L_IS, LOW);
  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);

  pinMode(R_PWM, OUTPUT);
  pinMode(L_PWM, OUTPUT);


  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);

#endif

  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
      Serial.println("WiFi Failed");
      while(1) {
          delay(1000);
      }
  }


  ArduinoOTA.setHostname("shutter_master");
  ArduinoOTA.setPassword("1234");
  ArduinoOTA.begin();


    if(udps.listen(1235)) {
       // Serial.print("UDP Listening on IP: ");
       // Serial.println(WiFi.localIP());
        udps.onPacket([](AsyncUDPPacket packets) {
       //     Serial.print("UDP Packet Type: ");
       //     Serial.print(packets.isBroadcast()?"Broadcast":packets.isMulticast()?"Multicast":"Unicast");
       //     Serial.print(", From: ");
       //     Serial.print(packets.remoteIP());
       //     Serial.print(":");
       //     Serial.print(packets.remotePort());
       //     Serial.print(", To: ");
       //     Serial.print(packets.localIP());
       //     Serial.print(":");
       //     Serial.print(packets.localPort());
       //     Serial.print(", Length: ");
       //     Serial.print(packets.length());
            //Serial.print(", Datax: ");

            udpdata = packets.data();
            udplength = packets.length();            
     
    
              
             // UP1
            if (packets.data()[7] == 49){ //push1

              if(packets.data()[16] == 63){ // pushed
             //   Serial.println("UP1 - pushed");
                digitalWrite(emulatedPushButton1, HIGH);
              }

              if(packets.data()[16] == 0){ // released
              //  Serial.println("UP1 - released");
                digitalWrite(emulatedPushButton1, LOW);
              }
            }
           // DOWN1
           if (packets.data()[7] == 50){ //push2

            if(packets.data()[16] == 63){ // pushed
              //Serial.println("DOWN1 - pushed");
              digitalWrite(emulatedPushButton2, HIGH);
            }

            if(packets.data()[16] == 0){ // released
             // Serial.println("DOWN1 - released");
              digitalWrite(emulatedPushButton2, LOW);
            }
          }          
     

       
            udpc.write(udpdata, udplength); // also send OSC data to #2
            
        });

        
    }

    if(udpc.connect(IPAddress(192,168, 178, 53), 1236)) {
       // Serial.println("UDP connected");
        udpc.onPacket([](AsyncUDPPacket packet) {
        /*    Serial.print("UDP Packet Type: ");
            Serial.print(packet.isBroadcast()?"Broadcast":packet.isMulticast()?"Multicast":"Unicast");
            Serial.print(", From: ");
            Serial.print(packet.remoteIP());
            Serial.print(":");
            Serial.print(packet.remotePort());
            Serial.print(", To: ");
            Serial.print(packet.localIP());
            Serial.print(":");
            Serial.print(packet.localPort());
            Serial.print(", Length: ");
            Serial.print(packet.length());
            Serial.print(", Datay: ");
            Serial.write(packet.data(), packet.length());
            Serial.println();
            */
            //reply to the client
            packet.printf("Got %u bytes of data", packet.length());
        });
        //Send unicast
        //udpc.print("Hello Server!");
        
    }
    



  
  bMenue = MOVE;
  bMenueOld = -1;
  bCommand = STOP;


#ifdef ESP32
   // initialize the Flash storage (ESP32)
  
   preferences.begin("APPstatus", false);// initialize Flash name space 

     // configure LED PWM functionalitites
  ledcSetup(R_PWMChannel, freq, resolution);
  ledcSetup(L_PWMChannel, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(R_PWM, R_PWMChannel);
  ledcAttachPin(L_PWM, L_PWMChannel);
#endif


 //int counter = preferences.getInt("counter", 0);


 
#ifdef ESP32
  writes = preferences.getUInt("FlashWrites", 0);
#else
  writes = EEPROM.read(28); // write counter
#endif

  
  ver2 =  (ver.substring((ver.lastIndexOf(".")), (ver.lastIndexOf("\\")) + 1));
  ver3 = (ver2.substring( (ver2.lastIndexOf("_")+1), (ver2.length()) ));

#ifdef OLED_Display  // ######################
  //OLED start
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) { //§1/1
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }//§1/1
  display.clearDisplay();
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0, 0);            // Start at top-left corner
  display.println(ver2);
  display.println();
  display.print("[");
  display.print(ver3);
  display.print("] EE#");
  display.print(writes);
 
 
  display.display();

  delay(5000);
 #endif             // ######################
 



   if((preferences.getUInt("HWflag1", 0) != 13) || (preferences.getUInt("HWflag2", 0) != 211)) {  //  §1/2 HWflag1 (16) and HWflag2 (17)

    preferences.putUInt("FlashWrites", 1); // initialize write counter

  }
  
  unsigned int fail;
  delay(10000);
  fail = preferences.getUInt("fail",0);
  Serial.print("fail* : ");
  Serial.println(fail);
 
 
  if (fail != 222) {
      Serial.println("WTF");
      delay(10000);
      preferences.putUInt("PositionAllDown", 0); // PositionAllDown init 
  float pointer = 0;    
      preferences.putFloat("currentPosition", pointer); //currentPosition init 

      cal_wait_duration = -1;
      bMenue = CAL;
               
      bCommand = STOP;
      calState = upper;
      
      toneLowToHigh();
      cal_wait_time = millis();

  }


  //preferences.putUInt("fail", 0); // Reset the fail flag of 222
  

  PositionAllDown = preferences.getUInt("PositionAllDown", 0);
  currentPosition = preferences.getFloat("currentPosition", 0);



  for (int i = 0; i < NUM_BUTTONS; i++)
  {
    pinMode(butPin[i], INPUTMODE);
    debouncer[i].attach(butPin[i]);
    debouncer[i].interval(10);
  }




  // Hall sensor config

  // Lege den Interruptpin als Inputpin mit Pullupwiderstand fest
  pinMode(interruptPin, INPUT_PULLUP);
  // Lege die ISR 'blink' auf den Interruptpin mit Modus 'CHANGE':
  // "Bei wechselnder Flanke auf dem Interruptpin" --> "Führe die ISR aus"

#ifdef ESP32

  //attachInterrupt(interruptPin, ISR_hallTMP, FALLING);

#else
  attachInterrupt(digitalPinToInterrupt(interruptPin), ISR_hall, FALLING);

#endif

  
} //§1

/*
 * *********************************************************
 * *********************************************************
 * *********************************************************
 */

void loop()
{


  ArduinoOTA.handle();

  Serial.println(preferences.getUInt("fail", 0));

 
  if (!ISR_GPIO & digitalRead(interruptPin) == 0) {

    ISR_hallTMP();
    ISR_GPIO= true;
  }
  else if (digitalRead(interruptPin) == 1) {
    ISR_GPIO = false; 
  };

 // emulate_Hall_Sensor();
  
  
#ifdef ESP32
  writes = preferences.getUInt("FlashWrites", 0);
#else  
  writes = EEPROM.read(28);
#endif
  
  now = millis(); // Aktueller Zeitpunkt
  debounce();
  
  delay(10); // this 10ms delay was requied is no OLED Display (also a delay) is used. It looked like polling the ADC too oftern in the loop caused wrong voltage readings which in turn shut the system down.
  Sample = analogRead(ADC_pin);
  
  Voltage = map(Sample, 1150, 3850, 50, 150);
  Voltage = (Voltage/10)+.5;
  Voltage = 3;
  //Serial.println(Sample);
 // Serial.println(Voltage);

  if (Voltage < 11.50) { //11.00V
  
   lowVoltage = lowVoltage + 1;
  }
  else {
    lowVoltage = 0;
  }

  if (lowVoltage > 3) { //11


#ifdef ESP32    
    Serial.println("*");
    helper = "passed";
    preferences.putFloat("currentPosition", currentPosition);
   // writes = preferences.getUInt("FlashWrites", 0);
   // writes++;
   // preferences.putUInt("FlashWrites", writes);
    preferences.putUInt("fail", 222);
    Serial.println("-");
#else 
    EEPROM.put(18,currentPosition);
    writes = EEPROM.read(28);
    writes = writes + 1;
    EEPROM.write(28,writes);
    EEPROM.write(38,222); // flag for power failure
#endif    

    #ifdef OLED_Display  // ###################### 
    MoveModeOLEDwrite("going2DIE", lowVoltage, 1);
    #endif                // ######################

        
    motor(STOP); // Try to cut power to the motor to win few ms time for the EEPROM write to finish.  
    
    //for (;;); // Don't proceed, loop forever
        
  }

 
  checkCalRequested();  // 

  
  // State machine with two main modes (MOVE and CAL) and each three sub-modes (UP, DOWN, STOP) 
  
  switch (bMenue) { // MOVE or CAL mode

    case MOVE:
       
      checkUpStartOrStopRequested();    // if Up-Button was pressed, check if motor is on. If yes, stop it and enter MOVE-STOP mode. If no, start motor and enter MOVE-UP mode

      checkDownStartOrStopRequested();  // if Down-Button was pressed, check if motor is on. If yes, stop it and enter MOVE-STOP mode. If no, start motor to move down.

  
      switch (bCommand) { // Within MOVE mode: UP, DOWN or STOP mode

        case UP:
          if (currentPosition > PositionAllUp) { // if not all up

            //Serial.println(currentPosition);

            if (!engineRunning) motor(UP);  // start motor to move up if its not already running                            
            requestSTOP = false;            // don't stop it during next ISR call
            POSdir = -1;                    // as this is up movement, need to decrement the Hall sensor impulse counts in the ISR 
            
            #ifdef OLED_Display // ######################
            MoveModeOLEDwrite("MOVE - up", currentPosition, PositionAllDown);      
            #endif              // ######################



         
           }
           else {                    // shutter is all up, stop motor
            
            bCommand = STOP;         // State machine now: MOVE-STOP mode
            requestSTOP = true;      //stop the motor the next time the ISR triggers. Stopping the motor only in the ISR ensures it always stops at a magnet, and not in between.        

            #ifdef OLED_Display  // ######################
            MoveModeOLEDwrite("MOVE -stop", currentPosition, PositionAllDown);            
            #endif               // ######################



            
          }
          
          break;

        case DOWN:

          if (currentPosition < PositionAllDown) { // if not all down

            if (!engineRunning) motor(DOWN);  // start motor to move down if its not already running              
            requestSTOP = false;
            POSdir = 1;

            #ifdef OLED_Display  // ######################
            MoveModeOLEDwrite("MOVE -down", currentPosition, PositionAllDown);  
            #endif               // ######################
          
 
            
          }
          else {
            bCommand = STOP;
            requestSTOP = true;  //stop the motor the next time the ISR triggers. Stopping the motor only in the ISR ensures it always stops at a magnet, and not in between.  
            
           #ifdef OLED_Display    // ###################### 
           MoveModeOLEDwrite("MOVE -stop", currentPosition, PositionAllDown);
           #endif                 // ######################    

    

          }

          break;

        case STOP:
          requestSTOP = true; //stop the motor the next time the ISR triggers. Stopping the motor only in the ISR ensures it always stops at a magnet, and not in between.

          #ifdef OLED_Display    // ######################    
          MoveModeOLEDwrite("MOVE -stop", currentPosition, PositionAllDown);
          #endif                 // ######################


#ifdef ESP32            
            //UDP_Pos2Fader(leftFADER);
#endif           
          
          break;
      }
      break;




    case CAL:
   
      checkUpStartRequested();     // check if Up button is held down, if yes enter CAL-UP mode. If not, check if CAL-DOWN mode is running, other wise stop the motor
      
      checkDownStartRequested();   // check if Down button is held down, if yes enter CAL-DOWN mode. If not, check if CAL-UP mode is running, other wise stop the motor
 
      switch (bCommand) { // Within CAL mode: UP, DOWN or STOP mode

        case UP:
          cal_wait_duration = 3000;

          
          requestSTOP = false;
          POSdir = -1;
          if (!engineRunning) motor(UP); 
                
      
          
          #ifdef OLED_Display    // ######################
          if (calState == upper) {
            CalModeOLEDwrite("CAL -upper", "use up/dwn");
          }
          else {
            CalModeOLEDwrite("CAL -lower", "use up/dwn");          
          }
          #endif                 // ######################
          

          cal_wait_time = millis();
          break;

        case DOWN:
          cal_wait_duration = 3000;
          requestSTOP = false;
          POSdir = 1;

          if (!engineRunning) {
             motor(DOWN);
          }         
          
      
        

          #ifdef OLED_Display   // ######################
          if (calState == upper) {
            CalModeOLEDwrite("CAL -upper", "use up/dwn");
          }
          else {
            CalModeOLEDwrite("CAL -lower", "use up/dwn");          
          }
          #endif                 // ######################
          
          
          cal_wait_time = millis();  
          break;

        case STOP:
          requestSTOP = true;

          
          if (calState == upper) {

            #ifdef OLED_Display    // ######################
            CalModeOLEDwrite("CAL -upper", "use up/dwn");
            #endif                 // ######################
             
            currentPosition = 0;
          
  
            if (cal_wait_duration != -1){ // if arrive from HE reset or normal power up, force to CAL mode and wait for action
              
            
                if (millis()- cal_wait_time >= cal_wait_duration) {
                  calState = lower;
                  toneHighToLow();
               
                  cal_wait_time = millis(); 
    
                #ifdef OLED_Display    // ######################
                CalModeOLEDwrite("CAL -lower", "use up/dwn");
                #endif                 // ######################
    
                }  
              }
          }

          if (calState == lower) {
            PositionAllDown = currentPosition;

          
            if (millis()- cal_wait_time >= cal_wait_duration) {
              
#ifdef ESP32
              preferences.putUInt("PositionAllDown", PositionAllDown);
              preferences.putUInt("HWflag1", 13);
              preferences.putUInt("HWflag2", 211);
              preferences.putFloat("currentPosition", currentPosition);
  
              writes = preferences.getUInt("FlashWrites", 0);
              writes++;
              preferences.putUInt("FlashWrites", writes);



#else   
              EEPROM.update(15,PositionAllDown);
              EEPROM.update(16, 13);
              EEPROM.update(17,211);
              EEPROM.put(18,currentPosition);
              
              writes = EEPROM.read(28);
              writes = writes + 1;
              EEPROM.write(28,writes);
    
#endif
           
              bMenue = MOVE;
              toneLowToLow();
              UDP_Pos2Fader(leftFADER);
              
            }
          }
  
        break;

      }

      break;
  }

  
  crone1(5000, 1000);

} // loop




void checkCalRequested()
{
    if (bCmd[0] == M_HOLD && bCmd[1] == M_HOLD && bMenue == MOVE)  {
   
      bMenue = CAL;
      
      bCommand = STOP;
      requestSTOP = true;
      calState = upper;
      
     
      bState[0] = UNHOLD;
      bState[1] = UNHOLD;
      toneLowToHigh();
      cal_wait_time = millis();
    
    }
}


void checkUpStartOrStopRequested()
{
    if (bCmd[0] == S_HOLD) {  // was up key pressed?

      if (bCommand == UP) {
        bCommand = STOP;
      }
      else if (bCommand == DOWN) {
        bCommand = STOP;
      }
      else {
        bCommand = UP;
      }
   } 
}


void checkDownStartOrStopRequested()
{
    if (bCmd[1] == S_HOLD) { // was down key pressed?
      if (bCommand == DOWN) {
        bCommand = STOP;         
      }
      else if (bCommand == UP) {
        bCommand = STOP;
      }
      else {
        bCommand = DOWN;
      }
   }
}


void checkUpStartRequested()
{
    if (bState[0] == HOLD) bCommand = UP;
     
    else {
      if (bCommand != DOWN) bCommand = STOP;
    }
}

void checkDownStartRequested()
{
    if (bState[1] == HOLD) bCommand = DOWN;
    
    else {
      if (bCommand != UP) bCommand = STOP;
    }

}

#ifdef OLED_Display    // ######################
void MoveModeOLEDwrite(String text, float val1, int val2)
{
      display.clearDisplay();
      display.setTextSize(2);             
      display.setTextColor(SSD1306_WHITE);       
      display.setCursor(0, 0);            
      display.println(text);
      display.print("[");
      display.print(int((val1 / val2) * 100));
      display.print("%]");
      display.display();  
}

void CalModeOLEDwrite(String txt1, String txt2)
{
      display.clearDisplay();
      display.setTextSize(2);             // Normal 1:1 pixel scale
      display.setTextColor(SSD1306_WHITE);        // Draw white text
      display.setCursor(0, 0);            // Start at top-left corner
      display.println(txt1);
      display.print(txt2);
      display.display();
}
#endif                 // ######################


void motor(byte state)
{
    //Serial.println(state);
    switch(state)
    {

#ifdef ESP32    
        case STOP:
          engineRunning = false;
          ledcWrite(L_PWMChannel, 0);
          ledcWrite(R_PWMChannel, 0);
          EmulatedHallTrigger = false;
          break;
      
        case UP: // UP
          ledcWrite(R_PWMChannel, 0);
          ledcWrite(L_PWMChannel, 255);
          engineRunning = true;
          EmulatedHallTrigger = true;
          break;
         
       case DOWN:
          ledcWrite(L_PWMChannel, 0);
          ledcWrite(R_PWMChannel, 255);
          engineRunning = true;
          EmulatedHallTrigger = true;
          break;


#else
        case STOP:
          engineRunning = false;
          analogWrite(L_PWM, 0);
          analogWrite(R_PWM, 0);
          break;
      
        case UP: // UP
          analogWrite(R_PWM, 0);
          analogWrite(L_PWM, 255);
          engineRunning = true;
          break;
         
       case DOWN:
          analogWrite(L_PWM, 0);
          analogWrite(R_PWM, 255);
          engineRunning = true;
          break;
#endif          
    }

}


void toneLowToHigh()
{
  
     digitalWrite(LED1, HIGH);

#ifdef ESP32



     for (int i = 0; i <= 3; i++) {    
          ledcWrite(L_PWMChannel, 0);
          ledcWrite(R_PWMChannel, 10);

          delay(200);

          ledcWrite(L_PWMChannel, 0);
          ledcWrite(R_PWMChannel, 0);

          delay(200);
     }

     for (int i = 0; i <= 4; i++) {    
          ledcWrite(L_PWMChannel, 10);
          ledcWrite(R_PWMChannel, 0);

          delay(100);

          ledcWrite(L_PWMChannel, 0);
          ledcWrite(R_PWMChannel, 0);

          delay(100);
          
     }  
     pled1 = led1on;
     udpc.write(pled1, 20); 

#else
     for (int i = 0; i <= 3; i++) {    
          analogWrite(L_PWM, 0);
          analogWrite(R_PWM, 10);

          delay(200);

          analogWrite(L_PWM, 0);
          analogWrite(R_PWM, 0);

          delay(200);
     }

     for (int i = 0; i <= 4; i++) {    
          analogWrite(L_PWM, 10);
          analogWrite(R_PWM, 0);

          delay(100);

          analogWrite(L_PWM, 0);
          analogWrite(R_PWM, 0);

          delay(100);
          
     }     
#endif
     
     digitalWrite(LED1, LOW);   
}


void toneHighToLow()
{
     digitalWrite(LED2, HIGH);

#ifdef ESP32

 
     for (int i = 0; i <= 3; i++) {    
          ledcWrite(L_PWMChannel, 10);
          ledcWrite(R_PWMChannel, 0);

          delay(200);

          ledcWrite(L_PWMChannel, 0);
          ledcWrite(R_PWMChannel, 0);

          delay(200);
     }

     for (int i = 0; i <= 4; i++) {    
          ledcWrite(L_PWMChannel, 0);
          ledcWrite(R_PWMChannel, 10);

          delay(100);

          ledcWrite(L_PWMChannel, 0);
          ledcWrite(R_PWMChannel, 0);

          delay(100);
     }   
     pled1 = led1off;
     udpc.write(pled1, 20);

     pled2 = led2on;
     udpc.write(pled2, 20);

#else
     for (int i = 0; i <= 3; i++) {    
          analogWrite(L_PWM, 10);
          analogWrite(R_PWM, 0);

          delay(200);

          analogWrite(L_PWM, 0);
          analogWrite(R_PWM, 0);

          delay(200);
     }

     for (int i = 0; i <= 4; i++) {    
          analogWrite(L_PWM, 0);
          analogWrite(R_PWM, 10);

          delay(100);

          analogWrite(L_PWM, 0);
          analogWrite(R_PWM, 0);

          delay(100);
     }   
#endif
               
     digitalWrite(LED2, LOW);
}

void toneLowToLow()
{
     digitalWrite(LED1, HIGH);
     digitalWrite(LED2, HIGH);

#ifdef ESP32


     pled1 = led1on;
     udpc.write(pled1, 20);

     pled2 = led2on;
     udpc.write(pled2, 20);

     for (int i = 0; i <= 6; i++) {    
          ledcWrite(L_PWMChannel, 0);
          ledcWrite(R_PWMChannel, 10);

          delay(50);

          ledcWrite(L_PWMChannel, 0);
          ledcWrite(R_PWMChannel, 0);

          delay(50);
     }
     pled1 = led1off;
     udpc.write(pled1, 20);

     pled2 = led2off;
     udpc.write(pled2, 20); 
#else
     for (int i = 0; i <= 6; i++) {    
          analogWrite(L_PWM, 0);
          analogWrite(R_PWM, 10);

          delay(50);

          analogWrite(L_PWM, 0);
          analogWrite(R_PWM, 0);

          delay(50);
     }
#endif

     digitalWrite(LED1, LOW);
     digitalWrite(LED2, LOW);         
}


// debounce2 lib is used but still needed some code to check if multiple keys are pressed at the same time. 
// the below code (debounce()) from the arduino forum did the job
//https://forum.arduino.cc/t/tastenabfrage-mehrerer-tasten-inkl-entprellung-und-sonderfunktionalitaten/378613


/*

   Tastenauswertungsalgorithmus

   Tastenzustände können folgendermaßen abgefragt werden:

   Beispiele:

   if (bState[7] == HOLD)                                         // Wird die Taste 7 momentan gedrückt gehalten?
   if (bCmd[3] == PUSH)                                           // Wurde die Taste 3 gedrückt?
   if (bCmd[1] == S_HOLD)                                         // Wurde die Taste 1 pushDur ms lang gedrückt gehalten?
   if (bCmd[2] == M_HOLD && bCmd[4] == M_HOLD)                    // Wurden die Tasten 2 und 4 pushDur ms lang gedrückt gehalten, unabhängig davon, ob weiteren Tasten gedrückt gehalten werden?
   if (bCmd[5] == M_HOLD && bCmd[6] == M_HOLD && bCount == 2)     // Wurden die Tasten 5 und 6 pushDur ms lang gedrückt gehalten, während keine weiteren Tasten gedrückt gehalten werden?
                                                                  // bCount sollte der Summe der abzufragenden Tasten entsprechen.

   Werden mehr als 255 Tasten benötigt, muss der Wertebereich von butPin und bCount jeweils in Integer geändert werden.

*/

void debounce()
{
  static unsigned long pushTime;    // Definiert, wann zuletzt eine Taste gedrückt gehalten wurde
  const  int pushDur = 50;         // Definiert die Haltedauer, im Anschluss derer eine Aktion ausgelöst werden soll
  static bool action = false;       // Definiert, ob eine Aktion bereits ausgeführt wurde
  now = millis();
  for (int i = 0; i < NUM_BUTTONS; i++)
  {
    switch (bCmd[i])
    {
      case WAIT:
        if (now - pushTime >= pushDur && action == false &&  bState[i] == HOLD)
        {
          if (bCount == 1) bCmd[i] = S_HOLD;   // Aktion fürs gedrückt halten einer Taste
          if (bCount >= 2) bCmd[i] = M_HOLD;   // Aktion fürs gedrückt halten mehrerer Tasten
        }
        break;
      case PUSH:
        if (!bCount)       // Wurde bereits noch keine Tasten weitere Taste gedrückt?
        {
          pushTime = now;  // Druck der ersten Taste
          action = false;  // Actionstatus zurücksetzen
        }
        bCount++;          // Anzahl der gleichzeitig gedrückt gehaltenen Tasten inkrementieren
        bCmd[i] = WAIT;
        break;
      case RELEASE:
        if (bCount) bCount--;         // Anzahl der gleichzeitig gedrückt gehaltenen Tasten inkrementieren, falls noch welche gedrückt gehalten werden
        if (!bCount) action = false;  // Gibt das Auslösen von Aktionen wieder frei, sobald keine Tasten mehr gedrückt gehalten werden
        bCmd[i] = M_HOLD; // An dieser Stelle wurde bewusst auf break; verzichtet
      case S_HOLD:
        bCmd[i] = M_HOLD; // An dieser Stelle wurde bewusst auf break; verzichtet
      case M_HOLD:
        action = true;  // Verhindert das mehrmalige Auslösen bereits ausgelöster Aktionen
        bCmd[i] = WAIT;
        break;
    }
    debouncer[i].update();   // Status der Tasten prüfen

    if (debouncer[i].rose())
    {
      
      bCmd[i] = PUSH;
      bState[i] = HOLD;
    }
    else if (debouncer[i].fell())
    {
      
      bCmd[i] = RELEASE;
      bState[i] = UNHOLD;
    }
  }
}



#ifdef ESP32
void UDP_Pos2Fader(byte bfader) {

   switch(bfader){

      case leftFADER:
      
            fader1Pos = currentPosition/PositionAllDown;
            tmp = (byte*)&fader1Pos;
            fader1[16] = *(tmp+3);
            fader1[17] = *(tmp+2);
            fader1[18] = *(tmp+1);
            fader1[19] = *tmp;

            pfader1 = fader1;

            udpc.write(pfader1, 20);  

            break;

      case rightFADER:

            fader2Pos = currentPosition/PositionAllDown;
            tmp = (byte*)&fader2Pos;
            fader2[16] = *(tmp+3);
            fader2[17] = *(tmp+2);
            fader2[18] = *(tmp+1);
            fader2[19] = *tmp;

            pfader2 = fader2;

            udpc.write(pfader2, 20); 

            break;            
   }
}



void UDP_Fader_stepdown(byte bfader) {

   switch(bfader){

      case leftFADER:

            if(fader1step < 2){
              fader1step += 1;
            } else {
              fader1step = 0;
            }

            fader1Pos = fadersteps[fader1step];
            if (fader1Pos != oldfader1Pos) { 
              tmp = (byte*)&fader1Pos;
              fader1[16] = *(tmp+3);
              fader1[17] = *(tmp+2);
              fader1[18] = *(tmp+1);
              fader1[19] = *tmp;
  
              pfader1 = fader1;
  
              udpc.write(pfader1, 20);
              oldfader1Pos = fader1Pos;  
            }
            break;

      case rightFADER:

            if(fader2step < 2){
              fader2step += 1;
            } else {
              fader2step = 0;
            }

            fader2Pos = fadersteps[fader2step];
   
            //if (fader2Pos != oldfader2Pos) { 
              tmp = (byte*)&fader2Pos;
              fader2[16] = *(tmp+3);
              fader2[17] = *(tmp+2);
              fader2[18] = *(tmp+1);
              fader2[19] = *tmp;
  
              pfader2 = fader2;
  
              udpc.write(pfader2, 20); 
           //  oldfader2Pos = fader2Pos;
           // }
            break;            
   }
   
}

void UDP_Fader_stepup(byte bfader) {

   switch(bfader){

      case leftFADER:
            
            if(fader1step > 0){
              fader1step -= 1;
            } else {
              fader1step = 2;
            }

            fader1Pos = fadersteps[fader1step];
            if (fader1Pos != oldfader1Pos) { 
              tmp = (byte*)&fader1Pos;
              fader1[16] = *(tmp+3);
              fader1[17] = *(tmp+2);
              fader1[18] = *(tmp+1);
              fader1[19] = *tmp;
  
              pfader1 = fader1;
  
              udpc.write(pfader1, 20);
              oldfader1Pos = fader1Pos;  
            }
            break;

      case rightFADER:

            if(fader2step > 0){
              fader2step -= 1;
            } else {
              fader2step = 2;
            }

            fader2Pos = fadersteps[fader2step];
            if (fader2Pos != oldfader2Pos) { 
              tmp = (byte*)&fader2Pos;
              fader2[16] = *(tmp+3);
              fader2[17] = *(tmp+2);
              fader2[18] = *(tmp+1);
              fader2[19] = *tmp;
  
              pfader2 = fader2;
  
              udpc.write(pfader2, 20); 
             oldfader2Pos = fader2Pos;
            }
            break;            
   }
   
}

#endif


void emulate_Hall_Sensor() {

   if (millis()- emulate_Hall_time >= 333 & EmulatedHallTrigger) {

     emulate_Hall_time = millis();
     ISR_hallTMP();
   }
  
}

void ISR_hallTMP() {

 
 
  ISR_hallFlag = true;
  
  currentPosition = currentPosition + POSdir;

 

 
  if (bMenue == MOVE) {
    UDP_Pos2Fader(leftFADER);
    if (currentPosition <= PositionAllUp || currentPosition >= PositionAllDown || requestSTOP) {
        
          bCommand = STOP;       
        motor(STOP);
  

      }
  }
  else if (bMenue == CAL) {
  if (POSdir >0 ){

     UDP_Fader_stepdown(leftFADER);
  }else{

     UDP_Fader_stepup(leftFADER);
  }
 

    if (requestSTOP) {
        
        bCommand = STOP; 
        motor(STOP);
 
      }
    
  }

}



#ifdef ESP32

void ISR_hall() {
  
  currentPosition = currentPosition + POSdir;
 
  if (bMenue == MOVE) {
    if (currentPosition <= PositionAllUp || currentPosition >= PositionAllDown || requestSTOP) {
        
          bCommand = STOP;       
        motor(STOP);
  

      }
  }
  else if (bMenue == CAL) {

    if (requestSTOP) {
        
        bCommand = STOP; 
        motor(STOP);
 
      }
    
  }

}

#else

void ISR_hall() {
  
  currentPosition = currentPosition + POSdir;
 
  if (bMenue == MOVE) {
    if (currentPosition <= PositionAllUp || currentPosition >= PositionAllDown || requestSTOP) {
        
          bCommand = STOP;       
        motor(STOP);
  

      }
  }
  else if (bMenue == CAL) {

    if (requestSTOP) {
        
        bCommand = STOP; 
        motor(STOP);
 
      }
    
  }

}
#endif

void crone1(long wait, int repeat){  // set just started OSC app with Led and Fader status

  
  now= millis();
  if (now > wait){
    if (now - oldCrone1time > repeat){

      udpc.write(pled1, 20);
      if (!engineRunning){

         UDP_Pos2Fader(leftFADER);
      }
     

      oldCrone1time = now;
    }
  }

}


//Emergency STOP 
// case one: loss of sensor ... if motor is on but no Hall sensor inputs for 1 second, stop motor and force calibration
// case two: networkloss during calibration run. If engine on but no Sensor inputs for 1 second, assume motor is stuck. 
void crone2(long wait, int repeat){  // set just started OSC app with Led and Fader status

  now= millis();
  if (now > wait){
    if (now - oldCrone2time > repeat){

      if (engineRunning && ISR_hallFlag==false){

        motor(STOP);       
#ifdef OLED_Display  
        MoveModeOLEDwrite("STOP", 1, 1);
        delay(5000);
       

#endif  
      }
      oldCrone2time = now;
      ISR_hallFlag = false;
    }
  }

}


 
