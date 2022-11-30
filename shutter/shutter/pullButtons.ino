
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
