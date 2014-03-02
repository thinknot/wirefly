void clockSync( uint16_t c = COLOR_JELLY, byte opts = 0){
  //unsigned long time_ON = 750;
  //I'm not sure how long it takes to check for a packet
  // so x should be number of checks roughly equivaent to 750ms
  //unsigned long time_OFF = 750;
  unsigned long time_half_cycle = 750; //milliseconds
  unsigned long time_start = 0;
  unsigned long time_end = 0;

  unsigned long sum_ON = 0;
  unsigned long sum_OFF = 0;
  int ON_longer = 0;
  int OFF_longer = 0;
  int ON_count = 0;
  int OFF_count = 0;

  off(opts);

  debounceInputs();
  delay(random(0,2001)); //make sure everyone starts at a somewhat different time
  debounceInputs();

  int startIndex = (IGNORE_FRONT(opts)?PIXEL_FRONT_LAST+1:0);
  int endIndex = (IGNORE_BACK(opts)?PIXEL_FRONT_LAST+1:strip.numPixels());

  while(true){

    ON_count = 0;
    sum_ON = 0;
    //digitalWrite(A3, HIGH); //turn LED on
    for(uint8_t p=startIndex; p<endIndex; p++) {
      debounceInputs();
      strip.setPixelColor(p, c);
    }
    strip.show();
    //transmit a packet
    if( rf12_canSend() ) {
      uint8_t send_data = PATTERN_CLOCKSYNC;
      rf12_sendStart(0, &send_data, 1);
    }
    time_start = millis();
    time_end = time_start + time_half_cycle + ON_longer;

    while(millis()<time_end){ //how far are we from the start, replace with a millis() or micros()
      debounceInputs();
      if( rf12_recvDone() && !rf12_crc ) { //did we get a good packet?
        sum_ON += millis()-time_start;
        ON_count++;
      }
    }

    //digitalWrite(A3, LOW); //turn LED off
    for(uint8_t p=startIndex; p<endIndex; p++) {
      debounceInputs();
      strip.setPixelColor(p, 0);
    }
    strip.show();

    OFF_count = 0;
    time_start = millis();
    time_end = time_start + time_half_cycle + OFF_longer;

    while(millis()<time_end){
      debounceInputs();
      if (rf12_recvDone() && rf12_crc == 0){
        sum_OFF += time_end - millis(); //how far are we from the start of the next cycle
        OFF_count++;
      }
    }

    // Jeremy, ESPLAIN PLEAZ ...
    if (ON_count > OFF_count){
      ON_longer = 0;
      OFF_longer = (sum_ON/ON_count) >> 1;
    } 
    else if(ON_count < OFF_count){
      OFF_longer = 0;
      ON_longer = (sum_ON/ON_count) >> 1;
    } 
    else if (ON_count == 0 && OFF_count == 0){
      ON_longer = 0;
      OFF_longer = 0;
    } 
    else if(ON_count == OFF_count){
      if (sum_ON < sum_OFF){
        ON_longer = 0;
        OFF_longer = (sum_ON/ON_count) >> 1;
      } 
      else {
        OFF_longer = 0;
        ON_longer = (sum_OFF/OFF_count) >> 1;
      }
    }

    if( handleInputs() ) {
      return;
    }

  }
}

void loop() {
    if (Serial.available())
        handleInput(Serial.read());

    if (rf12_recvDone()) {
        byte n = rf12_len;
        if (rf12_crc == 0) {
            Serial.print("OK");
        } else {
            if (quiet)
                return;
            Serial.print(" ?");
            if (n > 20) // print at most 20 bytes if crc is wrong
                n = 20;
        }
        if (config.group == 0) {
            Serial.print("G ");
            Serial.print((int) rf12_grp);
        }
        Serial.print(' ');
        Serial.print((int) rf12_hdr);
        for (byte i = 0; i < n; ++i) {
            Serial.print(' ');
            Serial.print((int) rf12_data[i]);
        }
        Serial.println();
        
        if (rf12_crc == 0) {
            activityLed(1);

            if (df_present())
                df_append((const char*) rf12_data - 2, rf12_len + 2);

            if (RF12_WANTS_ACK && (config.nodeId & COLLECT) == 0) {
                Serial.println(" -> ack");
                rf12_sendStart(RF12_ACK_REPLY, 0, 0);
            }
            
            activityLed(0);
        }
    }

    if (cmd && rf12_canSend()) {
        activityLed(1);

        Serial.print(" -> ");
        Serial.print((int) sendLen);
        Serial.println(" b");
        byte header = cmd == 'a' ? RF12_HDR_ACK : 0;
        if (dest)
            header |= RF12_HDR_DST | dest;
        Serial.print("Header: "); Serial.print((int)header);
        Serial.print(" Data: ");
        for( int d=0; d<sendLen; d++)
          Serial.print((int) databuffer[d]);
        Serial.print(" Length: "); Serial.print((int) sendLen);
        Serial.println();

        rf12_sendStart(header, databuffer, sendLen);
        cmd = 0;

        activityLed(0);
    }
}
