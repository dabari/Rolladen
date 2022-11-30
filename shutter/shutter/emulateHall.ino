

void emulate_Hall_Sensor() {

   if (millis()- emulate_Hall_time >= 333 & EmulatedHallTrigger) {

    emulate_Hall_time = millis();
#ifdef SW_EMU
    ISR_hallTMP();  
#endif

#ifdef HW_EMU
    digitalWrite(emulatedHallSignal, LOW);
    delay(50);
    digitalWrite(emulatedHallSignal, HIGH);
#endif 
   }
    
  
}
