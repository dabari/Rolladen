
byte * tmp;

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
