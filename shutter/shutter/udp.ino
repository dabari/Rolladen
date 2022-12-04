void UDPhandling(){  

  if(udps.listen(ListenPort)) {
    
    udps.onPacket([](AsyncUDPPacket packets) {

    #ifdef MASTER     
      const byte pushup = 49;
      const byte pushDown = 50;
    #endif

    #ifdef SLAVE
      const byte pushup = 51;
      const byte pushDown = 52;
    #endif    

      udpdata = packets.data();
      udplength = packets.length();            


      // UP1
      if (packets.data()[7] == pushup){ //push1 || push3

        if(packets.data()[16] == 63){ // pushed
          pinMode(emulatedPushButton1, OUTPUT);
          digitalWrite(emulatedPushButton1, HIGH);
        }

        if(packets.data()[16] == 0){ // released
          pinMode(emulatedPushButton1, INPUT);
          //digitalWrite(emulatedPushButton1, LOW);
        }
      }

      // DOWN1
      if (packets.data()[7] == pushDown ){ //push2 || push4

        if(packets.data()[16] == 63){ // pushed
          pinMode(emulatedPushButton2, OUTPUT);
          digitalWrite(emulatedPushButton2, HIGH);
        }

        if(packets.data()[16] == 0){ // released
          pinMode(emulatedPushButton2, INPUT);
          //digitalWrite(emulatedPushButton2, LOW);
        }
      }          
       
      udpc.write(udpdata, udplength); // also send OSC data to #2
            
    });

        
  }

  if(udpc.connect(IPAddress(SendToIPAddr[0],SendToIPAddr[1],SendToIPAddr[2], SendToIPAddr[3]), SendToPort)) {
      
    udpc.onPacket([](AsyncUDPPacket packet) {
      //reply to the client
      packet.printf("Got %u bytes of data", packet.length());
    });
  }
}    

