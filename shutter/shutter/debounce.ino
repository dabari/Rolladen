

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