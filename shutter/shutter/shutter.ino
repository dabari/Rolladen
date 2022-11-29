

/************************************
 * Shutter Control Master 
 * 
 * 
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
        R_PWM o (25)              o
              |                   |
        L_PWM o (26)              o
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
          +5V o                | (check if really 5V or 3.3V)
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


***************************************************************************************
*/

#define MASTER // use: MASTER or SLAVE

// define use:
// HALL = normal operation, real HALL sensor connected to pin GPIO 34
// HW_EMU = debugg operation without Hall sensor. GIPO 17 emulates Sensor signal. Needs tbe wire jumpered to GPIO 34 (IRQ input)
// SW_EMU = debugg operation without Hall sensor. ISR is called periodically by SW. 

#define HALL // or HW_EMU or SW_EMU or HALL

#define OLED_Display

// includes
  #include "arduino_secrets.h"
  #include <Preferences.h>
  #include <WiFi.h>
  #include <WiFiUdp.h>
  #include "AsyncUDP.h"
  #include <ArduinoOTA.h>
  #include "esp_attr.h" // TODO: check if still required
  #include <Bounce2.h>
#ifdef OLED_Display
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>
#endif


// constant definition

// WLAN credential

  const char * ssid     = SECRET_SSID;
  const char * password = SECRET_PASS;

 // IP addresses and ports

  //   |---> OSCapp ------ [touch events] ------> MASTER ------ [touch events, possition & status] ------> SLAVE ------ [Possition & status ] ----->|
  //   |                                                                                                                                            |
  //   |<-------------------------------------------------------------------------------------------------------------------------------------------|

 
 
#ifdef MASTER
  const int ListenPort = 1235; // Master 
  const int SendToIPAddr[] = {192, 168, 2, 77}; // Slave IP 
  const int SendToPort = 1236; // Slave Port 
#endif

#ifdef SLAVE
  const int ListenPort = 1236; // Slave
  const int SendToIPAddr[] = {192, 168, 2, 21}; // OSCapp IP
  const int SendToPort = 1234; // OSCApp Port
#endif 
 
 
  // - motor control pins


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


// setting PWM properties
  const int freq = 1000;
  const int R_PWMChannel = 0;
  const int L_PWMChannel = 1;
  const int resolution = 8; //bit

  const byte PositionAllUp = 0;  // all up

 
  
  AsyncUDP udps;
  AsyncUDP udpc;


  Preferences preferences;

  unsigned char* udpdata; // declare a pointer to a char array
  unsigned int udplength;

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


  enum {
  leftFADER, rightFADER
};

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


  byte * tmp;

  byte result = 0;
  
  
  String txt= "";
  

  
#ifdef OLED_Display
  
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

 









//uint8_t* OSC_Rx;
int  OSC_len;
String OSCcmd = "";






int  PositionAllDown  = 0;                // this detined the allDown position. It is set during calibration
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


// various pre-sets
  
int cal_wait_duration = 3000; // ms  the is the wait time during cal(-ibration mode. x ms after the last key push to switch from uper limit cal to lowerlimit cal abd then x ms to finish cal.

bool triggeredISR= false;

/*
 * *********************************************************
 * *********************************************************
 * *********************************************************
 */

String check = "";


void ISR_action() {

 triggeredISR = false;
 currentPosition = currentPosition + POSdir;
  
 
 
  if (bMenue == MOVE) { // MOVE
     UDP_Pos2Fader();
    if (currentPosition <= PositionAllUp || currentPosition >= PositionAllDown || requestSTOP) { //requestSTOP is set e.g. in CAL STOP, MOVE STOP or MOVE is upper or lower limit is reached
        
      bCommand = STOP;       
      EmulatedHallTrigger= false;
      motor(STOP);
  

      }
  }
  else if (bMenue == CAL) {
    if (POSdir >0 ){

      UDP_Fader_stepdown(); 
    }else{

      UDP_Fader_stepup();  
    }
 

    if (requestSTOP) { //requestSTOP is set e.g. in CAL STOP, MOVE STOP or MOVE is upper or lower limit is reached
              
       bCommand = STOP; 
       EmulatedHallTrigger= false;
       motor(STOP);
 
      }
    
  }

}

void IRAM_ATTR ISR_hallTMP() { 
  
    
    triggeredISR = true;

} 

/*
 * *********************************************************
 * *************** SETUP ***********************************
 * *********************************************************


 */


void setup() { //§1
  Serial.begin(115200);

  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, LOW);

  pinMode(emulatedPushButton1, OUTPUT);
  pinMode(emulatedPushButton2, OUTPUT);
  digitalWrite(emulatedPushButton1, LOW);
  digitalWrite(emulatedPushButton2, LOW);
  


  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
      Serial.println("WiFi Failed");
      while(1) {
          delay(1000);
      }
  }


#ifdef MASTER
  ArduinoOTA.setHostname("shutter_master");
#endif 

#ifdef SLAVE
  ArduinoOTA.setHostname("shutter_slave");
#endif 

  ArduinoOTA.setPassword("1234");
  ArduinoOTA.begin();


    if(udps.listen(ListenPort)) {
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

#ifdef MASTER     
            int pushup = 49;
            int pushDown = 50;
#endif

#ifdef SLAVE
            int pushup = 51;
            int pushDown = 52;
#endif
             // UP1
            if (packets.data()[7] == pushup){ //push1 || push3

              if(packets.data()[16] == 63){ // pushed
                //Serial.println("UP1 - pushed");
                digitalWrite(emulatedPushButton1, HIGH);
              }

              if(packets.data()[16] == 0){ // released
                //Serial.println("UP1 - released");
                digitalWrite(emulatedPushButton1, LOW);
              }
            }
           // DOWN1
           if (packets.data()[7] == pushDown ){ //push2 || push4

            if(packets.data()[16] == 63){ // pushed
              //Serial.println("DOWN1 - pushed");
              digitalWrite(emulatedPushButton2, HIGH);
            }

            if(packets.data()[16] == 0){ // released
              //Serial.println("DOWN1 - released");
              digitalWrite(emulatedPushButton2, LOW);
            }
          }          
     

       
            udpc.write(udpdata, udplength); // also send OSC data to #2
            
        });

        
    }

    if(udpc.connect(IPAddress(SendToIPAddr[0],SendToIPAddr[1],SendToIPAddr[2], SendToIPAddr[3]), SendToPort)) {
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



   // initialize the Flash storage (ESP32)
  
  preferences.begin("APPstatus", false);// initialize Flash name space 

     // configure LED PWM functionalitites
  ledcSetup(R_PWMChannel, freq, resolution);
  ledcSetup(L_PWMChannel, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(R_PWM, R_PWMChannel);
  ledcAttachPin(L_PWM, L_PWMChannel);



 //int counter = preferences.getInt("counter", 0);


 

  writes = preferences.getUInt("FlashWrites", 0);


  
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





//for (;;);
  
} //§1

/*
 * *********************************************************
 * ***************** LOOP **********************************
 * *********************************************************
 */

void loop()
{


  ArduinoOTA.handle();

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
  
  delay(10); // this 10ms delay was requied is no OLED Display (also a delay) is used. It looked like polling the ADC too oftern in the loop caused wrong voltage readings which in turn shut the system down.
  Sample = analogRead(ADC_pin);
  
  Voltage = map(Sample, 50, 3000, 1, 124);
  Voltage = Voltage/10;
  //Serial.print(Sample);
  //Serial.println(Voltage);

  if (Voltage < 11.00) { //11.00V
  
   lowVoltage = lowVoltage + 1;
  }
  else {
    lowVoltage = 0;
  }

  if (lowVoltage > 11) { //11

 
    preferences.putFloat("currentPosition", currentPosition);
    writes = preferences.getUInt("FlashWrites", 0);
    writes++;
    preferences.putUInt("FlashWrites", writes);
    preferences.putUInt("fail", 222);

    #ifdef OLED_Display  // ###################### 
    MoveModeOLEDwrite("going2DIE", lowVoltage, 1);
    #endif                // ######################

        
    motor(STOP); // Try to cut power to the motor to win few ms time for the EEPROM write to finish.  
    
    for (;;); // Don't proceed, loop forever
        
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
              
              preferences.putUInt("PositionAllDown", PositionAllDown);
              preferences.putUInt("HWflag1", 13);
              preferences.putUInt("HWflag2", 211);
              preferences.putFloat("currentPosition", currentPosition);
  
              writes = preferences.getUInt("FlashWrites", 0);
              writes++;
              preferences.putUInt("FlashWrites", writes);

           
              bMenue = MOVE;
              toneLowToLow();
              UDP_Pos2Fader();
              
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
    if ( bCmd[0] == S_HOLD) {  // was up key pressed?

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

    }

}


void toneLowToHigh()
{
  
     
     for (int i = 0; i <= 3; i++) {    
          ledcWrite(L_PWMChannel, 0);
          ledcWrite(R_PWMChannel, 10);

          delay(200);

          ledcWrite(L_PWMChannel, 0);
          ledcWrite(R_PWMChannel, 0);

          delay(200);
     }

     for (int i = 0; i <= 4; i++) {    
          ledcWrite(L_PWMChannel, 20);
          ledcWrite(R_PWMChannel, 0);

          delay(100);

          ledcWrite(L_PWMChannel, 0);
          ledcWrite(R_PWMChannel, 0);

          delay(100);
          
     }  
     pled1 = led1on;
     udpc.write(pled1, 20); 

     
      
}


void toneHighToLow()
{
     digitalWrite(LED2, HIGH);


 
     for (int i = 0; i <= 3; i++) {    
          ledcWrite(L_PWMChannel, 20);
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

               
     digitalWrite(LED2, LOW);
}

void toneLowToLow()
{



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


void UDP_Pos2Fader() {

 
    fader1Pos = currentPosition/PositionAllDown;
    tmp = (byte*)&fader1Pos;
    fader1[16] = *(tmp+3);
    fader1[17] = *(tmp+2);
    fader1[18] = *(tmp+1);
    fader1[19] = *tmp;

    pfader1 = fader1;

    udpc.write(pfader1, 20);  

     
}



void UDP_Fader_stepdown() {

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
      
}

void UDP_Fader_stepup() {
         
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
}




void emulate_Hall_Sensor() {

   if (millis()- emulate_Hall_time >= 333 & EmulatedHallTrigger) {

    emulate_Hall_time = millis();
#ifdef SW_EMU
    ISR_hallTMP();  
#endif

#ifdef HW_EMU
    digitalWrite(LED1, LOW);
    delay(50);
    digitalWrite(LED1, HIGH);
#endif 
   }
    
  
}



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


void crone1(long wait, int repeat){  // set just started OSC app with Led and Fader status

  
  now= millis();
  if (now > wait){
    if (now - oldCrone1time > repeat){

      udpc.write(pled1, 20);
      udpc.write(pled2, 20);
      if (!engineRunning){

         UDP_Pos2Fader();
      }
     

      oldCrone1time = now;
    }
  }

}




 
