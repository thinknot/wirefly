#include "RF12.h"
#include "firefly.h"

// Based on http://jeelabs.net/projects/jeelib/wiki/RF12demo

static boolean wirefly_needToSend;
static MilliTimer wirefly_sendTimer;
static MilliTimer wirefly_immuneTimer;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Main loop functions, top-level / timing functions 

// = = = = = = = = = = = = = = = = = = = = = = = = = = =
void loop() {
	//PHASE1: input
	wirefly_interrupt(); // note that patterns can call pattern_interrupt(), on their own time

	//PHASE2: communicate
	wirefly_send(); //send a pending message, if (needToSend != 0)

	//PHASE3: display
	pattern_run();  // switches control to the active pattern
}

// = = = = = = = = = = = = = = = = = = = = = = = = = = =
// my_interrupt()
// Call this function whenever reasonably possible to keep network comm and serial IO going. 
// returns the output of handleInputs()
int wirefly_interrupt() {
	boolean trigger = false;
#ifdef DEBUG
	//check for serial input via handleSerialInput()
	if (Serial.available()) {
                char input = Serial.read();
		handleInput(input);
		trigger = true;
	}
#endif
	//  debounceInputs();

	// check for network input via my_recvDone()
	// note that wirefly_recvDone() may alter wirefly_pattern, if the crc is good
	trigger |= wirefly_recvDone();
	return trigger;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Control functions for patterns

// = = = = = = = = = = = = = = = = = = = = = = = = = = =
// pattern_interrupt()
// A pattern should call this function, or some equivalent of the my_interrupt()
boolean pattern_interrupt(int current_pattern) {
	//PHASE1: check for a change in pattern from network / serial
	boolean inputReceived = wirefly_interrupt();
	boolean patternChanged = (wirefly_pattern != current_pattern);

	//PHASE2:
	if (wirefly_sendTimer.poll(2096))
		wirefly_needToSend = 1;
	wirefly_send();

	if (patternChanged) {
#ifdef DEBUG
		// log if new pattern detected
		Serial.print("pattern_interrupt() old: ");
		Serial.print(current_pattern);
		Serial.print(" new: ");
		Serial.println(wirefly_pattern);
#endif
		wirefly_immuneTimer.set(16384);
	}
	//report status
	return (patternChanged);
}

// = = = = = = = = = = = = = = = = = = = = = = = = = = =
// runPattern()
// Pattern control switch!
void pattern_run() {
	// Update this switch if adding a pattern! (primitive callback)
	switch (wirefly_pattern) {
	default:
	case PATTERN_OFF:
		pattern_off();  // all lights off, including/especially the lantern
		break;
        case PATTERN_RGBTEST:
                pattern_testLED();
                break;
	case PATTERN_TWINKLE:
		pattern_randomTwinkle(); //blinks randomly at full intensity
		break;
	case PATTERN_FIREFLY:
		pattern_teamFirefly(); // slowly beginning to blink together, timed tx/rx
		break;
	case PATTERN_CLOCKSYNC:
		pattern_clockSync();  // synchronizing firefly lanterns
		break;
	case PATTERN_FADER:
		pattern_rgbFader(); // rgb fading in 3-space. matrix math is fun
		break;
	case PATTERN_PULSER:
		pattern_rgbpulse(); // more primitive rgb fader
	}
}

// = = = = = = = = = = = = = = = = = = = = = = = = = = =
// delay functions - // use these, don't waste cycles with delay()

//poll for new inputs and break if an interrupt is detected
int pattern_delay(unsigned long wait_time, uint8_t current_pattern) {
	unsigned long t0 = millis();
	while ((millis() - t0) < wait_time)
		if (pattern_interrupt(current_pattern))
			return 1;
	return 0;
}
// poll for new inputs, detect interrupts but do not break the delay
void wirefly_delay(unsigned long wait_time) {
	unsigned long t0 = millis();
	while ((millis() - t0) < wait_time)
		wirefly_interrupt();
}


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// = = = = = = = = = = = = = = = = = = = = = = = = = = =
// Network communication

// = = = = = = = = = = = = = = = = = = = = = = = = = = =
// my_recvDone
// returns 1 if a message was received successfully
int wirefly_recvDone() {
	int msgReceived = 0;
	// (receive a network message if one exists; keep the RF12 library happy)
	// rf12_recvDone() needs to be constantly called in order to recieve new transmissions.
	// It checks to see if a packet has been received, returns true if it has
	if (rf12_recvDone()) {
#ifdef DEBUG
		debugRecv(); //print the message if in debug mode
#endif
		// if we got a bad crc, then no message was received.
		msgReceived = !rf12_crc;
		if (rf12_crc == 0) {
			// If a new transmission comes in and CRC is ok, don't poll recv state again -
			// otherwise rf12_crc, rf12_len, and rf12_data will be reset.
			activityLed(1);
			//ack if requested
			if (RF12_WANTS_ACK && (config.collect_mode) == 0) {
				showString(PSTR("Send -> ack\n"));
				rf12_sendStart(RF12_ACK_REPLY, 0, 0);
				rf12_sendWait(1); // don't power down too soon
			}
			// grab the pattern that we recieve, unless no data...
			if (rf12_len == 1) {
				int new_pattern = rf12_data[0];
				// ...if we get a crazy clocksync ping msg, then just ignore it
				if (msgReceived = (new_pattern != PATTERN_CLOCKSYNC_PING)) {
					//otherwise, msgReceived is true and set the new pattern
					if (!wirefly_immuneTimer.poll()) {
						wirefly_pattern = new_pattern;
						wirefly_sendTimer.set(0);
#ifdef DEBUG
						Serial.print("my_recvDone() "); Serial.println(wirefly_pattern);
#endif
					}
				}
			}
			activityLed(0);
		}
	}
	return msgReceived;
}

// = = = = = = = = = = = = = = = = = = = = = = = = = = =
// my_send
int wirefly_send() {
	//if rf12_canSend returns 1, then you must subsequently call rf12_sendStart.
	if (wirefly_needToSend && rf12_canSend()) {
		activityLed(1);
		//do yo thang:
		wirefly_msg_stack[0] = wirefly_pattern;
		wirefly_msg_sendLen = 1;
		wirefly_msg_dest = 0; //broadcast message
#ifdef DEBUG
				showString(PSTR("Send -> "));
				Serial.println((byte) wirefly_msg_stack[0]);
#endif
		//make this an ack if requested
		byte header = wirefly_msg_cmd == 'a' ? RF12_HDR_ACK : 0;
		// if not broadcast, set the destination node
		if (wirefly_msg_dest)
			header |= RF12_HDR_DST | wirefly_msg_dest;
		//actually send the message
		wirefly_needToSend = 0;
		rf12_sendStart(header, wirefly_msg_stack, wirefly_msg_sendLen);
		wirefly_msg_cmd = 0;
		activityLed(0);
	}
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Setup
// = = = = = = = = = = = = = = = = = = = = = = = = = = =
void setup() {
	if (rf12_config()) {
		config.nodeId = eeprom_read_byte(RF12_EEPROM_ADDR);
		config.group = eeprom_read_byte(RF12_EEPROM_ADDR + 1);
	}
#ifdef DEBUG
	else {
		config.nodeId = 0x41; // node A1 @ 433 MHz
		config.group = 0xD4;// group 212 (valid values: 0-212)
		saveConfig();
	}

	pinMode(A1, OUTPUT);
	Serial.begin(SERIAL_BAUD);
	Serial.println();
#endif

	wirefly_pattern = PATTERN_OFF;

	if (rf12_configSilent()) {
		loadConfig();
	} else {
		memset(&config, 0, sizeof config);
		config.nodeId = 0x81;       // 868 MHz, node 1
		config.group = 0xD4;        // default group 212
		config.frequency_offset = 1600;
		config.quiet_mode = true;   // Default flags, quiet on
		saveConfig();
		rf12_configSilent();
	}

	rf12_configDump();
	df_initialize();

	randomSeed(analogRead(0));
	wirefly_sendTimer.set(0);
}
