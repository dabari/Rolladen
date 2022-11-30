

void IRAM_ATTR ISR_hallTMP() { 
  
    
    triggeredISR = true;

} 


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