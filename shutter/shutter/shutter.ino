

/************************************
 * Ver 1.4 with initial EEProm storage
 * -POSmin
 * -POSmax
 * -POSactual
 * 
 */



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

// OLED display Lib

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library.
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define NUMFLAKES     10 // Number of snowflakes in the animation example

#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16
static const unsigned char PROGMEM logo_bmp[] =
{ 0b00000000, 0b11000000,
  0b00000001, 0b11000000,
  0b00000001, 0b11000000,
  0b00000011, 0b11100000,
  0b11110011, 0b11100000,
  0b11111110, 0b11111000,
  0b01111110, 0b11111111,
  0b00110011, 0b10011111,
  0b00011111, 0b11111100,
  0b00001101, 0b01110000,
  0b00011011, 0b10100000,
  0b00111111, 0b11100000,
  0b00111111, 0b11110000,
  0b01111100, 0b11110000,
  0b01110000, 0b01110000,
  0b00000000, 0b00110000
};

//Bounce Lib

#include <Bounce2.h>



// Motorsteuerung

int R_IS = 1;
int R_EN = 7;
int R_PWM = 3;
int L_IS = 4;
int L_EN = 5;
int L_PWM = 6;

// Motorsteuerung finish


//Hall Sensor
const byte interruptPin = 2;
volatile int counter = 0;

int POSmin = 0;  // all up
int POSmax = 0; // all down
float POSactual = 0;

int POSdir = +1;
bool engineRunning = false;
String TMP;


// end


unsigned long now;                  // Aktueller Zeitpunkt

// _Entprellung

#define INPUTMODE INPUT             // INPUT oder INPUT_PULLUP
const byte butPin[] = {             // Pin-Nummern der angeschlossenen Tasten
  12, 13
};
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
  MAIN, CAL
};

byte bCommand;
byte previous_bCommand;
enum {
  UP, DOWN, STOPPED
};

byte calState;
enum {
  upper, lower
};



long hangoutCNTR = 0;
bool prepareSTOP = false;


String ver = __FILE__;
String ver2, ver3;

float Sample;
float Voltage;
long lowVoltage= 0;

void setup()
{
  Serial.begin(9600);

  if((EEPROM.read(6) != 12) || (EEPROM.read(7) != 210)) {
    EEPROM.update(5, 0); // POSmax init
    EEPROM.put(8,0); //POSactual init
  }

  POSmax = EEPROM.read(5);
  EEPROM.get(8, POSactual);

  ver2 =  (ver.substring((ver.lastIndexOf(".")), (ver.lastIndexOf("\\")) + 1));
  ver3 = (ver2.substring( (ver2.lastIndexOf("_")+1), (ver2.length()) ));

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
            display.print("[");
            display.print(ver3);
            display.print("]");
           
           
            display.display();
            delay(5000);

       

 


  for (int i = 0; i < NUM_BUTTONS; i++)
  {
    pinMode(butPin[i], INPUTMODE);
    debouncer[i].attach(butPin[i]);
    debouncer[i].interval(10);
  }
  bMenue = MAIN;
  bMenueOld = -1;
  bCommand = STOPPED;

  // Hall sensor config

  // Lege den Interruptpin als Inputpin mit Pullupwiderstand fest
  pinMode(interruptPin, INPUT_PULLUP);
  // Lege die ISR 'blink' auf den Interruptpin mit Modus 'CHANGE':
  // "Bei wechselnder Flanke auf dem Interruptpin" --> "Führe die ISR aus"
  attachInterrupt(digitalPinToInterrupt(interruptPin), ISR_hall, FALLING);



  // Motorsteuerung
  pinMode(R_IS, OUTPUT);
  pinMode(R_EN, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  pinMode(L_IS, OUTPUT);
  pinMode(L_EN, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  digitalWrite(R_IS, LOW);
  digitalWrite(L_IS, LOW);
  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);
}



void loop()
{
  now = millis(); // Aktueller Zeitpunkt
  debounce();

  Sample = analogRead(A0);
  Voltage = map(Sample, 0, 1023, 0, 250);
  Voltage = Voltage/10;
  //Serial.println(Voltage);

  if (Voltage < 11.00) {
  
   lowVoltage = lowVoltage + 1;
  }
  else {
    lowVoltage = 0;
  }
  
  if (lowVoltage > 5) {
    MainModeOLEDwrite("going2DIE", lowVoltage, 1);
    if (bCommand == DOWN && POSactual > 0) {
      EEPROM.put(8,POSactual+1);
    }
    else if (bCommand == UP && POSactual < POSmax){
      EEPROM.put(8,POSactual-1);
    }
    else {
      EEPROM.put(8,POSactual);
    }
    
    for (;;); // Don't proceed, loop forever
        
  }

 
  selectMainOrCalMode();  // to enter CAL mode, push both buttons (up and down) at the same time.The return to Main is automatically after a delay counter.


  switch (bMenue) { // MAIN or CAL mode


    

    case MAIN:
       
      UpTriggerButtonToggle();    // if Up-Button was pressed, check if motor is on. If yes, stop it and enter MAIN-STOPPED mode. If no, start motor and enter MAIN-UP mode

      DownTriggerButtonToggle();  // if Down-Button was pressed, check if motor is on. If yes, stop it and enter MAIN-STOPPED mode. If no, start motor to move down.

  
      switch (bCommand) { // Within MAIN mode: UP, DOWN or STOPPED mode

        case UP:
          if (POSactual > POSmin) { // if not all up

            if (!engineRunning) motor(UP);  // start motor to move up if its not already running                            
            prepareSTOP = false;            // don't stop it during next ISR call
            POSdir = -1;                    // as this is up movement, need to decrement the Hall sensor impulse counts in the ISR 
            MainModeOLEDwrite("MAIN - up", POSactual, POSmax);      
          }
          else {                    // shutter is all up, stop motor
            bCommand = STOPPED;
            prepareSTOP = true;     //stop the motor the next time the ISR triggers. Stopping the motor only in the ISR ensures it always stops at a magnet, and not in between.                               
            MainModeOLEDwrite("MAIN -stop", POSactual, POSmax);            
          }
          
          break;

        case DOWN:

          if (POSactual < POSmax) { // if not all down

            if (!engineRunning) motor(DOWN);  // start motor to move down if its not already running              
            prepareSTOP = false;
            POSdir = 1;
            MainModeOLEDwrite("MAIN -down", POSactual, POSmax);  
            //
          }
          else {
            bCommand = STOPPED;
            prepareSTOP = true;  //stop the motor the next time the ISR triggers. Stopping the motor only in the ISR ensures it always stops at a magnet, and not in between.         
           MainModeOLEDwrite("MAIN -stop", POSactual, POSmax);
           //MainModeOLEDwrite("MAIN -stop", Voltage, 1);
          }

          break;

        case STOPPED:
          prepareSTOP = true; //stop the motor the next time the ISR triggers. Stopping the motor only in the ISR ensures it always stops at a magnet, and not in between.
          MainModeOLEDwrite("MAIN -stop", POSactual, POSmax);
          //MainModeOLEDwrite("MAIN -stop", Voltage, 1);
          break;
      }
      break;




    case CAL:
      
      UpButtonHold();     // check if Up button is held down, if yes enter CAL-UP mode. If not, check if CAL-DOWN mode is running, other wise stop the motor
      
      DownButtonHold();   // check if Down button is held down, if yes enter CAL-DOWN mode. If not, check if CAL-UP mode is running, other wise stop the motor
 
      switch (bCommand) { // Within CAL mode: UP, DOWN or STOPPED mode

        case UP:

          prepareSTOP = false;
          POSdir = -1;
          if (!engineRunning) motor(UP); 
                
          hangoutCNTR = 0;

          if (calState == upper) {
            CalModeOLEDwrite("CAL -upper", "use up/dwn");
          }
          else {
            CalModeOLEDwrite("CAL -lower", "use up/dwn");
          
          }

    
          break;

        case DOWN:

          prepareSTOP = false;
          POSdir = 1;

          if (!engineRunning) {
             motor(DOWN);
          }         
          
          //Serial.println("CAL - going down");
          hangoutCNTR = 0;

 
          if (calState == upper) {
            CalModeOLEDwrite("CAL -upper", "use up/dwn");
          }
          else {
            CalModeOLEDwrite("CAL -lower", "use up/dwn");
          
          }
          
            
          break;

        case STOPPED:
          prepareSTOP = true;

          
          if (calState == upper) {

            CalModeOLEDwrite("CAL -upper", "use up/dwn");
             
            POSactual = 0;
            hangoutCNTR = hangoutCNTR + 1;
 
            if (hangoutCNTR == 100 ) {

              calState = lower;
              toneHighToLow();
              hangoutCNTR = 0;


            CalModeOLEDwrite("CAL -lower", "use up/dwn");

            }  
          }

          if (calState == lower) {
            POSmax = POSactual;
     
            
            hangoutCNTR = hangoutCNTR + 1;
  
            if (hangoutCNTR == 50000) {
              
              EEPROM.update(5,POSmax);
              EEPROM.update(6, 12);
              EEPROM.update(7,210);
              EEPROM.put(8,POSactual);
              bMenue = MAIN;
              toneLowToLow();
              hangoutCNTR = 0;
            }
          }
  
        break;

      }

      break;
  }

}




void selectMainOrCalMode()
{
    if (bCmd[0] == M_HOLD && bCmd[1] == M_HOLD && bMenue == MAIN)  {
   
      bMenue = CAL;
      bCommand = STOPPED;
      prepareSTOP = true;
      calState = upper;
      
     
      bState[0] = UNHOLD;
      bState[1] = UNHOLD;
      toneLowToHigh();
    
    }
}


void UpTriggerButtonToggle()
{
    if (bCmd[0] == S_HOLD) {  // was up key pressed?

      if (bCommand == UP) {
        bCommand = STOPPED;
      }
      else if (bCommand == DOWN) {
        bCommand = STOPPED;
      }
      else {
        bCommand = UP;
      }
   } 
}


void DownTriggerButtonToggle()
{
    if (bCmd[1] == S_HOLD) { // was down key pressed?
      if (bCommand == DOWN) {
        bCommand = STOPPED;         
      }
      else if (bCommand == UP) {
        bCommand = STOPPED;
      }
      else {
        bCommand = DOWN;
      }
   }
}


void UpButtonHold()
{
    if (bState[0] == HOLD) bCommand = UP;
     
    else {
      if (bCommand != DOWN) bCommand = STOPPED;
    }
}

void DownButtonHold()
{
    if (bState[1] == HOLD) bCommand = DOWN;
    
    else {
      if (bCommand != UP) bCommand = STOPPED;
    }

}


void MainModeOLEDwrite(String text, float val1, int val2)
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

void motor(byte state)
{
    switch(state)
    {
        case STOPPED:
          engineRunning = false;
          analogWrite(L_PWM, 0);
          analogWrite(R_PWM, 0);
          break;
          
        case UP:
          analogWrite(R_PWM, 0);
          analogWrite(L_PWM, 255);
          engineRunning = true;
          break;
          
       case DOWN:
          analogWrite(L_PWM, 0);
          analogWrite(R_PWM, 255);
          engineRunning = true;
          break;
        
    }

}


void toneLowToHigh()
{
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
         
}


void toneHighToLow()
{
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
         
}


void toneLowToLow()
{
     for (int i = 0; i <= 6; i++) {    
          analogWrite(L_PWM, 0);
          analogWrite(R_PWM, 10);

          delay(50);

          analogWrite(L_PWM, 0);
          analogWrite(R_PWM, 0);

          delay(50);
     }

   
         
}

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
      // Serial.println("rose");
      bCmd[i] = PUSH;
      bState[i] = HOLD;
    }
    else if (debouncer[i].fell())
    {
      // Serial.println("fell");
      bCmd[i] = RELEASE;
      bState[i] = UNHOLD;
    }
  }
}




void ISR_hall() {
  
  POSactual = POSactual + POSdir;

 
  if (bMenue == MAIN) {
    if (POSactual <= POSmin || POSactual >= POSmax || prepareSTOP) {
        
          bCommand = STOPPED; 
      
        motor(STOPPED);
  

      }
  }
  else if (bMenue == CAL) {

    if (prepareSTOP) {
        
        bCommand = STOPPED; 
        motor(STOPPED);
 
      }
    
  }

 
}

//OLED stuff

void testdrawline() {
  int16_t i;

  display.clearDisplay(); // Clear display buffer

  for (i = 0; i < display.width(); i += 4) {
    display.drawLine(0, 0, i, display.height() - 1, SSD1306_WHITE);
    display.display(); // Update screen with each newly-drawn line
    delay(1);
  }
  for (i = 0; i < display.height(); i += 4) {
    display.drawLine(0, 0, display.width() - 1, i, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  delay(250);

  display.clearDisplay();

  for (i = 0; i < display.width(); i += 4) {
    display.drawLine(0, display.height() - 1, i, 0, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  for (i = display.height() - 1; i >= 0; i -= 4) {
    display.drawLine(0, display.height() - 1, display.width() - 1, i, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  delay(250);

  display.clearDisplay();

  for (i = display.width() - 1; i >= 0; i -= 4) {
    display.drawLine(display.width() - 1, display.height() - 1, i, 0, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  for (i = display.height() - 1; i >= 0; i -= 4) {
    display.drawLine(display.width() - 1, display.height() - 1, 0, i, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  delay(250);

  display.clearDisplay();

  for (i = 0; i < display.height(); i += 4) {
    display.drawLine(display.width() - 1, 0, 0, i, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  for (i = 0; i < display.width(); i += 4) {
    display.drawLine(display.width() - 1, 0, i, display.height() - 1, SSD1306_WHITE);
    display.display();
    delay(1);
  }

  delay(2000); // Pause for 2 seconds
}

void testdrawrect(void) {
  display.clearDisplay();

  for (int16_t i = 0; i < display.height() / 2; i += 2) {
    display.drawRect(i, i, display.width() - 2 * i, display.height() - 2 * i, SSD1306_WHITE);
    display.display(); // Update screen with each newly-drawn rectangle
    delay(1);
  }

  delay(2000);
}

void testfillrect(void) {
  display.clearDisplay();

  for (int16_t i = 0; i < display.height() / 2; i += 3) {
    // The INVERSE color is used so rectangles alternate white/black
    display.fillRect(i, i, display.width() - i * 2, display.height() - i * 2, SSD1306_INVERSE);
    display.display(); // Update screen with each newly-drawn rectangle
    delay(1);
  }

  delay(2000);
}

void testdrawcircle(void) {
  display.clearDisplay();

  for (int16_t i = 0; i < max(display.width(), display.height()) / 2; i += 2) {
    display.drawCircle(display.width() / 2, display.height() / 2, i, SSD1306_WHITE);
    display.display();
    delay(1);
  }

  delay(2000);
}

void testfillcircle(void) {
  display.clearDisplay();

  for (int16_t i = max(display.width(), display.height()) / 2; i > 0; i -= 3) {
    // The INVERSE color is used so circles alternate white/black
    display.fillCircle(display.width() / 2, display.height() / 2, i, SSD1306_INVERSE);
    display.display(); // Update screen with each newly-drawn circle
    delay(1);
  }

  delay(2000);
}

void testdrawroundrect(void) {
  display.clearDisplay();

  for (int16_t i = 0; i < display.height() / 2 - 2; i += 2) {
    display.drawRoundRect(i, i, display.width() - 2 * i, display.height() - 2 * i,
                          display.height() / 4, SSD1306_WHITE);
    display.display();
    delay(1);
  }

  delay(2000);
}

void testfillroundrect(void) {
  display.clearDisplay();

  for (int16_t i = 0; i < display.height() / 2 - 2; i += 2) {
    // The INVERSE color is used so round-rects alternate white/black
    display.fillRoundRect(i, i, display.width() - 2 * i, display.height() - 2 * i,
                          display.height() / 4, SSD1306_INVERSE);
    display.display();
    delay(1);
  }

  delay(2000);
}

void testdrawtriangle(void) {
  display.clearDisplay();

  for (int16_t i = 0; i < max(display.width(), display.height()) / 2; i += 5) {
    display.drawTriangle(
      display.width() / 2  , display.height() / 2 - i,
      display.width() / 2 - i, display.height() / 2 + i,
      display.width() / 2 + i, display.height() / 2 + i, SSD1306_WHITE);
    display.display();
    delay(1);
  }

  delay(2000);
}

void testfilltriangle(void) {
  display.clearDisplay();

  for (int16_t i = max(display.width(), display.height()) / 2; i > 0; i -= 5) {
    // The INVERSE color is used so triangles alternate white/black
    display.fillTriangle(
      display.width() / 2  , display.height() / 2 - i,
      display.width() / 2 - i, display.height() / 2 + i,
      display.width() / 2 + i, display.height() / 2 + i, SSD1306_INVERSE);
    display.display();
    delay(1);
  }

  delay(2000);
}

void testdrawchar(void) {
  display.clearDisplay();

  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  display.cp437(true);         // Use full 256 char 'Code Page 437' font

  // Not all the characters will fit on the display. This is normal.
  // Library will draw what it can and the rest will be clipped.
  for (int16_t i = 0; i < 256; i++) {
    if (i == '\n') display.write(' ');
    else          display.write(i);
  }

  display.display();
  delay(2000);
}

void testdrawstyles(void) {
  display.clearDisplay();

  display.setTextSize(2);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0, 0);            // Start at top-left corner
  display.println(F("Cal up"));
  //display.setCursor(0,5);             // Start at top-left corner
  display.println(F("Cal up2"));
  //display.startscrollleft(0x00, 0x0F);
  //display.println(F("Line2"));
  //display.println(F("Line3"));
  //  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // Draw 'inverse' text
  //  display.println(3.141592);

  //  display.setTextSize(2);             // Draw 2X-scale text
  //  display.setTextColor(SSD1306_WHITE);
  //  display.print(F("0x")); display.println(0xDEADBEEF, HEX);

  display.display();
  delay(2000);
}

void testscrolltext(void) {
  display.clearDisplay();

  display.setTextSize(2); // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 0);
  display.println(F("scroll"));
  display.display();      // Show initial text
  delay(100);

  // Scroll in various directions, pausing in-between:
  display.startscrollright(0x00, 0x0F);
  delay(2000);
  display.stopscroll();
  delay(1000);
  display.startscrollleft(0x00, 0x0F);
  delay(2000);
  display.stopscroll();
  delay(1000);
  display.startscrolldiagright(0x00, 0x07);
  delay(2000);
  display.startscrolldiagleft(0x00, 0x07);
  delay(2000);
  display.stopscroll();
  delay(1000);
}

void testdrawbitmap(void) {
  display.clearDisplay();

  display.drawBitmap(
    (display.width()  - LOGO_WIDTH ) / 2,
    (display.height() - LOGO_HEIGHT) / 2,
    logo_bmp, LOGO_WIDTH, LOGO_HEIGHT, 1);
  display.display();
  delay(1000);
}

#define XPOS   0 // Indexes into the 'icons' array in function below
#define YPOS   1
#define DELTAY 2

void testanimate(const uint8_t *bitmap, uint8_t w, uint8_t h) {
  int8_t f, icons[NUMFLAKES][3];

  // Initialize 'snowflake' positions
  for (f = 0; f < NUMFLAKES; f++) {
    icons[f][XPOS]   = random(1 - LOGO_WIDTH, display.width());
    icons[f][YPOS]   = -LOGO_HEIGHT;
    icons[f][DELTAY] = random(1, 6);
    Serial.print(F("x: "));
    Serial.print(icons[f][XPOS], DEC);
    Serial.print(F(" y: "));
    Serial.print(icons[f][YPOS], DEC);
    Serial.print(F(" dy: "));
    Serial.println(icons[f][DELTAY], DEC);
  }

  for (;;) { // Loop forever...
    display.clearDisplay(); // Clear the display buffer

    // Draw each snowflake:
    for (f = 0; f < NUMFLAKES; f++) {
      display.drawBitmap(icons[f][XPOS], icons[f][YPOS], bitmap, w, h, SSD1306_WHITE);
    }

    display.display(); // Show the display buffer on the screen
    delay(200);        // Pause for 1/10 second

    // Then update coordinates of each flake...
    for (f = 0; f < NUMFLAKES; f++) {
      icons[f][YPOS] += icons[f][DELTAY];
      // If snowflake is off the bottom of the screen...
      if (icons[f][YPOS] >= display.height()) {
        // Reinitialize to a random position, just off the top
        icons[f][XPOS]   = random(1 - LOGO_WIDTH, display.width());
        icons[f][YPOS]   = -LOGO_HEIGHT;
        icons[f][DELTAY] = random(1, 6);
      }
    }
  }
}
