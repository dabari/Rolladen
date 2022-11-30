
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