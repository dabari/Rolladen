void motor(byte state)
{
    //Serial.println(state);
    switch(state)
    {

  
        case STOP:
          engineRunning = false;
          ledcWrite(UP_PWMChannel, 0);
          ledcWrite(DWN_PWMChannel, 0);
          EmulatedHallTrigger = false;
          break;
      
        case UP: // UP
          ledcWrite(DWN_PWMChannel, 0);
          ledcWrite(UP_PWMChannel, 255);
          engineRunning = true;
          EmulatedHallTrigger = true;
          break;
         
       case DOWN:
          ledcWrite(UP_PWMChannel, 0);
          ledcWrite(DWN_PWMChannel, 255);
          engineRunning = true;
          EmulatedHallTrigger = true;
          break;

    }

}


void toneLowToHigh()
{
  
     
     for (int i = 0; i <= 3; i++) {    
          ledcWrite(UP_PWMChannel, 0);
          ledcWrite(DWN_PWMChannel, 10);

          delay(200);

          ledcWrite(UP_PWMChannel, 0);
          ledcWrite(DWN_PWMChannel, 0);

          delay(200);
     }

     for (int i = 0; i <= 4; i++) {    
          ledcWrite(UP_PWMChannel, 20);
          ledcWrite(DWN_PWMChannel, 0);

          delay(100);

          ledcWrite(UP_PWMChannel, 0);
          ledcWrite(DWN_PWMChannel, 0);

          delay(100);
          
     }  
     pled1 = led1on;
     udpc.write(pled1, 20); 

     
      
}


void toneHighToLow()
{
  
     for (int i = 0; i <= 3; i++) {    
          ledcWrite(UP_PWMChannel, 20);
          ledcWrite(DWN_PWMChannel, 0);

          delay(200);

          ledcWrite(UP_PWMChannel, 0);
          ledcWrite(DWN_PWMChannel, 0);

          delay(200);
     }

     for (int i = 0; i <= 4; i++) {    
          ledcWrite(UP_PWMChannel, 0);
          ledcWrite(DWN_PWMChannel, 10);

          delay(100);

          ledcWrite(UP_PWMChannel, 0);
          ledcWrite(DWN_PWMChannel, 0);

          delay(100);
     }   
     pled1 = led1off;
     udpc.write(pled1, 20);

     pled2 = led2on;
     udpc.write(pled2, 20);

               
}

void toneLowToLow()
{

     pled1 = led1on;
     udpc.write(pled1, 20);

     pled2 = led2on;
     udpc.write(pled2, 20);

     for (int i = 0; i <= 6; i++) {    
          ledcWrite(UP_PWMChannel, 0);
          ledcWrite(DWN_PWMChannel, 10);

          delay(50);

          ledcWrite(UP_PWMChannel, 0);
          ledcWrite(DWN_PWMChannel, 0);

          delay(50);
     }
     pled1 = led1off;
     udpc.write(pled1, 20);

     pled2 = led2off;
     udpc.write(pled2, 20); 

        
}
