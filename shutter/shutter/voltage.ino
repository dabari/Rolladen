void supplyVoltageMonitor(){ 


  delay(10); // this 10ms delay was requied is no OLED Display (also a delay) is used. It looked like polling the ADC too oftern in the loop caused wrong voltage readings which in turn shut the system down.
  Sample = analogRead(ADC_pin);
  supplyVoltage = linADC(Sample);
  
 // Voltage = map(Sample, 50, 3000, 1, 124);
 // Voltage = Voltage/10;
/*
  if (Uavg < 1){

    //avgVoltage = avgVoltage + Voltage;
    avgSample = avgSample + Sample;
   
    Uavg += 1;

  } else {
     //avgVoltage = avgVoltage / 100;
     avgSample = avgSample / 4;
     supplyVoltage = linADC(avgSample);
      
     //MoveModeOLEDwrite((String(supplyVoltage)+ "V"), 1, 1);
     Uavg = 0;
    //avgVoltage = 0;
    avgSample = 0;
  }

 */ 
  if (supplyVoltage < 11.30) { //11.00V
  
   lowVoltage = lowVoltage + 1;
  }
  else {
    lowVoltage = 0;
  }

  if (lowVoltage > 5) { //11

 
    preferences.putFloat("currentPosition", currentPosition);
    writes = preferences.getUInt("FlashWrites", 0);
    writes++;
    preferences.putUInt("FlashWrites", writes);
    preferences.putUInt("fail", 222);

    #ifdef OLED_Display  // ###################### 

    LowVoltageOLEDwrite("going2DIE", String(supplyVoltage));
     
   
    #endif                // ######################

        
    motor(STOP); // Try to cut power to the motor to win few ms time for the EEPROM write to finish.  
    
   for (;;); // Don't proceed, loop forever
        
  }
}