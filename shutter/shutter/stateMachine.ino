void stateMachine(){ 

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

}