/*  PROJECT SHUTTER CONTROLLER (Contoller for two electric motors to move two independent shutter up/down)
 *  By: nav2go (Peter Voss)
 *  FIRMWARE STATUS: Verified Stable Build Version
 *  
 *  -----------------------------------------------------------------------------------------------------------
 *  DATE CREATED:  July 2022 
 *  DATE MODIFIED: 29.10.2022
 *  -----------------------------------------------------------------------------------------------------------

//================================  Schematics & Components ========================================//
// Below is a simple schematics and wiring information for the project.                             //
//==================================================================================================//

/*

   Required Components:
   - ESP32 Dev Module (U1)
  
              |----- antenna -----|
         3.3V o                   o GND
              |                   |
              o                   o
              |                   |
       ADC_in o (36)              o
              |                   |
              o                   o
              |                   |
       IRQ_in o (34)              o
              |                   |
              o                   o
              |                   |
              o (32)              o
              |                   |
              o (33)              o
              |                   |
      DWN_PWM o (25)              o
              |                   |
        UP_PWM o (26)              o
              |                   |
              o (27)              o
              |                   |
              o (14)              o
              |                   |
              o                   o
              |                   |
          GND o                   o
              |                   |
              o                   o
              |                   |
              o                   o
              |                   |
              o                   o
              |                   |
              o                   o
              |                   |
           5V o                   o
              |_ _ _ _ USB _ _ _ _|


  - IBT-2 BTS7960B PWM H-Bridge motor driver (U2)

              |-------------------|
              |                   |
              |                   |
              |                   |
              |                   |
              |                   |
              |                   () B-
          GND o--o VCC              |
              |  |                () B+
         L_IS o  o R_IS           | 
              |  |                () M+
         L_EN o  o R_EN           |
              |  |                () M-
         LPWM o__o RPWM           |
              |                   |
              |                   |
              |_ _ _ _ _ _ _ _ _ _| 

  - 2 lines OLED Display 0.91" I2C (optional) (U3)    

              |----------------------------|
          SDA o                            |
          SCK o                            |
          VCC o                            |
          GND o                            |
              |_ _ _ _ _ _ _ _ _ _ _ _ _ _ |     

  - digital Hall Sensor KY-003 (S1)

              |----------------|
          GND o                |
        +3.3V o                | 
            S o                |
              |_ _ _ _ _ _ _ _ |     

  - 230V AC/ 12V DC 5A switching Power Supply (P1)

              |-------|
           AC o \     o +12V
         230V |  \    | 
           AC o   \   o GND
              |_ _ \_ |   

  - 12V DC/ 5V DC switching Power Supply (P2)

              |-------|
         +12V o \     o +5V 
              |  \    | 
          GND o   \   o GND
              |_ _ \_ |   

  - Voltage Divider (S2) 

              |----------------|
            - o                () Uin
          n/c o                | 
            S o                () GND
              |_ _ _ _ _ _ _ _ |          


 - DC gear motor (M1) 

              |----------------|
          M+ ()                |
              |                | 
          M- ()                |
              |_ _ _ _ _ _ _ _ |          


  - Diode 1N4004 (D1)

          in o-------|>|-------o out

  - Capacitor 4700uF (C1)

           + o-------||--------o -  


Wiring 

 - ESP32 (U1) with Hall Sensor (S1)

 (U1) 5V ----------- (S1) +
 (U1) 34 ----------- (S1) S
 (U1) GND ---------- (S1) GND

 - ESB32 (U1) Gpio bridging

 (U1) 32 ----------- (U1) 27
 (U1) 33 ----------- (U1) 14

 - ESP32 (U1) with Voltage Devider (S2) and 12V Power Supply (P1)

 (U1) 36 ----------- (S2) S
 (U1) GND ---------- (S2) -
                     (S2) Uin -------- (P1) 12V
                     (S2) GND -------- (P1) GND

 - ESP32 (U1) and OLED display (U3)

 (U1) 21 ----------- (U3) SDA
 (U1) 22 ----------- (U3) SCK
 (U1) 3.3V --------- (U3) VCC
 (U1) GND ---------- (U3) GND

 - ESP32 (U1) and motor driver (U2)

 (U1) 25 ----------- (U2) LPWM
 (U1) 26 ----------- (U2) RPWM
 (U1) 3.3V --------- (U2) R_EN & L_EN
 (U1) GND  --------- (U2) R_IS & L_IS & GND
                     (U2) VCC not connected

 - 12V Power supply (P1) and Voltage devider (S2)

 (P1) +12V --------- (S2) Uin
 (P1) GND ---------- (S2) GND

  - 12V Power supply (P1) and motor driver (U2) 

  (P1) +12V -------- (U2) B+
  (P1) GND --------- (U2) B-

  - 12V Power Supply, Diode (D1), capacitor (C1), 5V Pwr supply

  (P1) +12V -------- (D1) in 
                     (D1) out ------------- (P2) 12V
                     (D1) out ------------- (C1) +
  (P1) GND -------------------------------- (P2) GND & (C1) - 

  - 5V Power supply (P2) and ESP32 (U1)

  (P2) USB out ----------- (U1) USB in

  - motor driver (U2) and motor (M1) 

  (U2) M+ ---------------- (M1) M+
  (U2) M- ---------------- (M1) M-

*/
       
           


#define MASTER // use: MASTER or SLAVE

// define use:
// HALL = normal operation, real HALL sensor connected to pin GPIO 34
// HW_EMU = debugg operation without Hall sensor. GIPO 17 emulates Sensor signal. Needs tbe wire jumpered to GPIO 34 (IRQ input)
// SW_EMU = debugg operation without Hall sensor. ISR is called periodically by SW. 

#define HALL // or HW_EMU or SW_EMU or HALL

#define OLED_Display


           
//====================== ARDUINO LIBRARIES (ESP32 Compatible Libraries) ============================//
// You will have to download and install the following libraries below.                             //
//==================================================================================================//

#include "arduino_secrets.h"
#include <Preferences.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "AsyncUDP.h"
#include <ArduinoOTA.h>
#include <Bounce2.h>

#ifdef OLED_Display

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#endif


//=================================== WLAN credentials =============================================//
// Enter WLAN credentials here.                                                                     //
//==================================================================================================//

  const char * ssid                   = SECRET_SSID;
  const char * password               = SECRET_PASS;

//=================================== IP Addresses and Ports =======================================//
// IP's and Ports required to  interact with the touchOSC app GUI                                   //
//==================================================================================================//


  //   |---> OSCapp ------ [touch events] ------> MASTER ------ [touch events, possition & status] ------> SLAVE ------ [Possition & status ] ----->|
  //   |                                                                                                                                            |
  //   |<-------------------------------------------------------------------------------------------------------------------------------------------|

 
 
#ifdef MASTER

const int ListenPort                  = 1235;    // Master 
const int SendToIPAddr[]              = {192, 168, 2, 21}; // Slave IP 
const int SendToPort                  = 1234;   // Slave Port 

#endif

#ifdef SLAVE

const int ListenPort                  = 1236;    // Slave
const int SendToIPAddr[]              = {192, 168, 2, 21}; // OSCapp IP
const int SendToPort                  = 1234;    // OSCApp Port

#endif 
 
 
//=================================== SYSTEM PIN DEFINITIONS =======================================//
// defining pins used on the ESP32   (note: pin number is GPIO number)                              //
//==================================================================================================//

const byte DWN_PWM                    = 26;      // SYSTEM PIN DEF: PWM output for Down movement
const byte UP_PWM                     = 25;      // SYSTEM PIN DEF: PWM output for UP movement
const byte interruptPin               = 34;      // SYSTEM PIN DEF: Input pin for Hall Sensor module KY-003 (A3144 chip)
const byte emulatedPushButton1        = 32;      // SYSTEM PIN DEF: GUI output pin for Up button 
const byte emulatedPushButton2        = 33;      // SYSTEM PIN DEF: GUI output pin for Down button 
const byte butPin[]                   = {27,14}; // SYSTEM PIN DEF: input pins for Up and down buttons 
const byte ADC_pin                    = 36;      // SYSTEM PIN DEF: input pin for voltage devider
const byte emulatedHallSignal         = 17;      // SYSTEM PIN DEF: this pin gives emulated a Hall sensor output (HW_EMU mode)


const int freq                        = 1000;    // SYSTEM PIN DEF: PWM frequency
const int DWN_PWMChannel              = 0;       // SYSTEM PIN DEF: PWM channel for Down movement
const int UP_PWMChannel               = 1;       // SYSTEM PIN DEF: PWM channel for up moevement
const int resolution                  = 8;       // SYSTEM PIN DEF: PWM bit resolution 

//=================================== SYSTEM PARAMETERS ============================================//
//                                                                                                  //
//==================================================================================================//
const byte PositionAllUp              = 0;       // SYSTEM PARAMETER: Counter value for sgutter all up position
int  PositionAllDown                  = 0;       // this detined the allDown position. It is set during calibration
float currentPosition                 = 0;       // this will be tracked during up/down movement. The range is min. PositionAllUp and max. PossitionAllDown.
float oldPosition                     = 0;
int POSdir                            = +1;
bool engineRunning                    = false;
unsigned int writes                   = 0; 


//=================================== INSTANCE DEFINITION ==========================================//
// Defining instances                                                                               //
//==================================================================================================//
  
  AsyncUDP udps;                                  // INSTANCE DEFINITION: UDP Server instance
  AsyncUDP udpc;                                  // INSTANCE DEFINITION: UDP Client instance
  Preferences preferences;                        // INSTANCE DEFINITION: Flash read/write instance
  

//=================================== UDP Rx DATA DEFINITION =======================================//
// Defining Pointer to the Rx buffer and int variable holding the length of the Buffer              //                                                                      
//==================================================================================================//

  unsigned char* udpdata; // declare a pointer to a char array
  unsigned int udplength;

//=============================== UDP Tx MESSAGES FOR THE OSC GUI ==================================//
//                                                                                                  //                                                                      
//==================================================================================================//


#ifdef MASTER
  unsigned char led1on[]  = {0x2f, 0x31, 0x2f, 0x6c, 0x65, 0x64, 0x31, 0x2F, 0x00, 0x00, 0x00, 0x00, 0x2c, 0x66, 0x00, 0x00, 0x3f, 0x80, 0x00, 0x00 };
  unsigned char led1off[] = {0x2f, 0x31, 0x2f, 0x6c, 0x65, 0x64, 0x31, 0x2F, 0x00, 0x00, 0x00, 0x00, 0x2c, 0x66, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  unsigned char* pled1 = led1off; 
  
  unsigned char led2on[]  = {0x2f, 0x31, 0x2f, 0x6c, 0x65, 0x64, 0x32, 0x2F, 0x00, 0x00, 0x00, 0x00, 0x2c, 0x66, 0x00, 0x00, 0x3f, 0x80, 0x00, 0x00 };
  unsigned char led2off[] = {0x2f, 0x31, 0x2f, 0x6c, 0x65, 0x64, 0x32, 0x2F, 0x00, 0x00, 0x00, 0x00, 0x2c, 0x66, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  unsigned char* pled2  = led2off;
#endif

#ifdef SLAVE
  unsigned char led1on[]  = {0x2f, 0x31, 0x2f, 0x6c, 0x65, 0x64, 0x33, 0x2F, 0x00, 0x00, 0x00, 0x00, 0x2c, 0x66, 0x00, 0x00, 0x3f, 0x80, 0x00, 0x00 };
  unsigned char led1off[] = {0x2f, 0x31, 0x2f, 0x6c, 0x65, 0x64, 0x33, 0x2F, 0x00, 0x00, 0x00, 0x00, 0x2c, 0x66, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  unsigned char* pled1 = led1off;
  
  unsigned char led2on[]  = {0x2f, 0x31, 0x2f, 0x6c, 0x65, 0x64, 0x34, 0x2F, 0x00, 0x00, 0x00, 0x00, 0x2c, 0x66, 0x00, 0x00, 0x3f, 0x80, 0x00, 0x00 };
  unsigned char led2off[] = {0x2f, 0x31, 0x2f, 0x6c, 0x65, 0x64, 0x34, 0x2F, 0x00, 0x00, 0x00, 0x00, 0x2c, 0x66, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  unsigned char* pled2  = led2off;
#endif



#ifdef MASTER
 //fader1
  unsigned char fader1[] = {0x2F, 0x31, 0x2F, 0x66, 0x61, 0x64, 0x65, 0x72, 0x31, 0x00, 0x00, 0x00, 0x2C, 0x66, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // last four bytes are the float value of the fader position
  unsigned char* pfader1; 
  float fader1Pos;
  float oldfader1Pos;
#endif

#ifdef SLAVE
  //fader2
  unsigned char  fader1[] = {0x2F, 0x31, 0x2F, 0x66, 0x61, 0x64, 0x65, 0x72, 0x32, 0x00, 0x00, 0x00, 0x2C, 0x66, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // last four bytes are the float value of the fader position
  unsigned char* pfader1;
  float fader1Pos;
  float oldfader1Pos;
#endif


//=================================== DISPLAY DEFINITIONS ==========================================//
//                                                                                                  //
//==================================================================================================//
#ifdef OLED_Display
  
  // Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
  // The pins for I2C are defined by the Wire-library.
  // On an arduino UNO:       A4(SDA), A5(SCL)
  // On an ESP32:             21(SDA), 22(SCK)
  
  
  #define SCREEN_WIDTH 128 // OLED display width, in pixels
  #define SCREEN_HEIGHT 32 // OLED display height, in pixels
  #define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
  #define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

  Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#endif



 




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


// various pre-sets
  
int cal_wait_duration = 3000; // ms  the is the wait time during cal(-ibration mode. x ms after the last key push to switch from uper limit cal to lowerlimit cal abd then x ms to finish cal.

bool triggeredISR= false;

String sIP;
float avgVoltage = 0;
float avgSample = 0;
int Uavg = 0;
float supplyVoltage = 0;

/*
 * *********************************************************
 * *********************************************************
 * *********************************************************
 */

void stateMachine();
void supplyVoltageMonitor();
void checkCalRequested();
void checkUpStartOrStopRequested();
void debounce();
void motor(byte state);
void toneLowToHigh();
void toneHighToLow();
void toneLowToLow();

void UDP_Pos2Fader();
void UDP_Fader_stepdown();
void UDP_Fader_stepup();

void emulate_Hall_Sensor();

void MoveModeOLEDwrite(String text, float val1, int val2);
void CalModeOLEDwrite(String txt1, String txt2);


void crone1(long wait, int repeat);
void UDPhandling();

void IRAM_ATTR ISR_hallTMP();
void ISR_action();


//=========================================== SETUP ================================================//
//                                                                                                  //
//==================================================================================================//


void setup() { 

  Serial.begin(115200);

  pinMode(emulatedHallSignal, OUTPUT);
  digitalWrite(emulatedHallSignal, HIGH);


  pinMode(emulatedPushButton1, INPUT);
  pinMode(emulatedPushButton2, INPUT);
  //digitalWrite(emulatedPushButton1, LOW);
  //digitalWrite(emulatedPushButton2, LOW);
  


  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
      Serial.println("WiFi Failed");
      while(1) {
          delay(1000);
      }
  
 
  }

  IPAddress myIP;
  myIP= WiFi.localIP();
  sIP = String(myIP[0])+ "." + String(myIP[1])+ "." + String(myIP[2])+ "." + String(myIP[3]);

#ifdef MASTER
  ArduinoOTA.setHostname("shutter_master");
#endif 

#ifdef SLAVE
  ArduinoOTA.setHostname("shutter_slave");
#endif 

  ArduinoOTA.setPassword("1234");
  ArduinoOTA.begin();

  UDPhandling();

 
  
  bMenue = MOVE;
  bMenueOld = -1;
  bCommand = STOP;



   // initialize the Flash storage (ESP32)
  
  preferences.begin("APPstatus", false);// initialize Flash name space 

     // configure LED PWM functionalitites
  ledcSetup(DWN_PWMChannel, freq, resolution);
  ledcSetup(UP_PWMChannel, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(DWN_PWM, DWN_PWMChannel);
  ledcAttachPin(UP_PWM, UP_PWMChannel);



 //int counter = preferences.getInt("counter", 0);


 

  writes = preferences.getUInt("FlashWrites", 0);


  
  ver2 =  (ver.substring((ver.lastIndexOf(".")), (ver.lastIndexOf("\\")) + 1));
  ver3 = (ver2.substring( (ver2.lastIndexOf("_")+1), (ver2.length()) ));

  #ifdef OLED_Display  // ######################
  //OLED start
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) { 
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }
  display.clearDisplay();
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0, 0);            // Start at top-left corner
  display.println(ver2); 
  display.println();
  //display.print("[");
  display.print(ver3); 
  //display.print("] EE#");
  //display.print(writes);
 
 
  display.display();

  delay(5000);
 #endif             // ######################
 

  IPAddrOLEDwrite(sIP, "***");
  delay(5000);

  if((preferences.getUInt("HWflag1", 0) != 13) || (preferences.getUInt("HWflag2", 0) != 211)) {  //  §1/2 HWflag1 (16) and HWflag2 (17)

    preferences.putUInt("FlashWrites", 1); // initialize write counter

  }
  
  unsigned int fail;
  fail = preferences.getUInt("fail",0);
  if (fail != 222) {
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






  preferences.putUInt("fail", 0); // Reset the fail flag of 222
  

  PositionAllDown = preferences.getUInt("PositionAllDown", 0);
  currentPosition = preferences.getFloat("currentPosition", 0);


  for (int i = 0; i < NUM_BUTTONS; i++)
  {
    pinMode(butPin[i], INPUTMODE);
    debouncer[i].attach(butPin[i]);
    debouncer[i].interval(10);
  }



#ifdef HALL // real Hall sensor coneected to pin GPIO 34
  // Hall sensor config

  // Lege den Interruptpin als Inputpin mit Pullupwiderstand fest
  pinMode(interruptPin, INPUT);
  // Lege die ISR 'blink' auf den Interruptpin mit Modus 'CHANGE':
  // "Bei wechselnder Flanke auf dem Interruptpin" --> "Führe die ISR aus"


  attachInterrupt(interruptPin, ISR_hallTMP, FALLING);
#endif

#ifdef HW_EMU //emulation with GPIO 17 jumpered to IRQ input (GPIO 34)
  // Hall sensor config

  // Lege den Interruptpin als Inputpin mit Pullupwiderstand fest
  pinMode(interruptPin, INPUT);
  // Lege die ISR 'blink' auf den Interruptpin mit Modus 'CHANGE':
  // "Bei wechselnder Flanke auf dem Interruptpin" --> "Führe die ISR aus"


  attachInterrupt(interruptPin, ISR_hallTMP, FALLING);
#endif


  
} 


//=========================================== LOOP =================================================//
//                                                                                                  //
//==================================================================================================//

void loop()
{


  ArduinoOTA.handle();                                  //start OTA capability

#ifdef SW_EMU 
  emulate_Hall_Sensor();
#endif

#ifdef HW_EMU
  emulate_Hall_Sensor();
#endif
  
  if (triggeredISR){

    ISR_action();

  }

  writes = preferences.getUInt("FlashWrites", 0);

  
  now = millis(); // Aktueller Zeitpunkt

  debounce();
  
  supplyVoltageMonitor();

  stateMachine();
  
  crone1(5000, 1000);

} // loop











 
