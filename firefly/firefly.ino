/// Configure some values in EEPROM for easy config of the RF12 later on.
// 2009-05-06 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

// Based on http://jeelabs.net/projects/jeelib/wiki/RF12demo
// this version adds flash memory support, 2009-11-19
// Adding frequency features, author JohnO, 2013-09-05
// Major EEPROM format change, refactoring, and cleanup for v12, 2014-02-13

#include <JeeLib.h>
#include <util/crc16.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <util/parity.h>

#include "Arduino.h"
#include "firefly.h"

static unsigned long now () {
	// FIXME 49-day overflow
	return millis() / 1000;
}

void activityLed (byte on) {
#ifdef LED_PIN
	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, !on);
#endif
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Main loop functions, top-level / timing functions 

// = = = = = = = = = = = = = = = = = = = = = = = = = = =
void loop() {
	//PHASE1: input
	my_interrupt(); // note that patterns should also check for interrupts, on their own time
	//PHASE2: communicate
	my_send(); //send a pending message, if (msg_cmd != 0)
	//PHASE3: display
	runPattern();  // switches control to the active pattern
}

// = = = = = = = = = = = = = = = = = = = = = = = = = = =
// my_interrupt()
// Call this function whenever reasonably possible to keep network comm and serial IO going. 
// returns the output of handleInputs()
int my_interrupt()
{
	int trigger = 0;
#ifdef DEBUG
	//check for serial input via handleSerialInput()
	if (Serial.available()) {
		handleSerialInput(Serial.read());
		trigger = 1;
	}
#endif
	//  debounceInputs();

	// check for network input via my_recvDone()
	// note that my_recvDone() may alter g_pattern, if the crc is good
	trigger |= my_recvDone();

	return trigger;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Control functions for patterns

// = = = = = = = = = = = = = = = = = = = = = = = = = = =
// pattern_interrupt()
// A pattern should call this function, or some equivalent of the my_interrupt()
boolean pattern_interrupt(int current_pattern)
{
	//check for a change in pattern from network / serial
	my_interrupt();
	boolean patternChanged = (g_pattern != current_pattern);

	if (patternChanged)
		// respond to a new pattern command
	{
#ifdef DEBUG
		Serial.print("pattern_interrupt() old: ");
		Serial.print(current_pattern);
		Serial.print("pattern_interrupt() new: ");
		Serial.println(g_pattern);
#endif
		// request that new pattern be broadcasted to the network
		msg_cmd = 'p';
		msg_stack[0] = msg_value;
		msg_sendLen = 1;
		msg_dest = 0; //broadcast message
	}

	//report status
	return (patternChanged);
}

// = = = = = = = = = = = = = = = = = = = = = = = = = = =
// runPattern()
// Pattern control switch!
void runPattern() {
	activityLed(1);
#ifdef DEBUG
	Serial.print("run_pattern() "); Serial.println(g_pattern);
#endif

	// Update this switch if adding a pattern! (primitive callback)
	switch( g_pattern ) {
	default:
	case PATTERN_OFF:
		pattern_off();  // all lights off, including/especially the lantern
		break;
	case PATTERN_TWINKLE:
		pattern_randomTwinkle();
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

	activityLed(0);
}

// = = = = = = = = = = = = = = = = = = = = = = = = = = =
// delay functions - // use these, don't waste cycles with delay()

//poll for new inputs and break if an interrupt is detected
void my_delay_with_break(unsigned long wait_time) {
	unsigned long t0 = millis();
	while( (millis() - t0) < wait_time )
		if ( my_interrupt() ) return;
}
// poll for new inputs, detect interrupts but do not break the delay
void my_delay(unsigned long wait_time) {
	unsigned long t0 = millis();
	while( millis() - t0 < wait_time )
		my_interrupt();
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// = = = = = = = = = = = = = = = = = = = = = = = = = = =
// Helper functions 

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

word crc16Calc (const void* ptr, byte len) {
	word crc = ~0;
	for (byte i = 0; i < len; ++i)
		crc = _crc16_update(crc, ((const byte*) ptr)[i]);
	return crc;
}

static void loadConfig () {
	// eeprom_read_block(&config, RF12_EEPROM_ADDR, sizeof config);
	// this uses 166 bytes less flash than eeprom_read_block(), no idea why
	for (byte i = 0; i < sizeof config; ++ i)
		((byte*) &config)[i] = eeprom_read_byte(RF12_EEPROM_ADDR + i);
}

void saveConfig () {
	config.format = MAJOR_VERSION;
	config.crc = crc16Calc(&config, sizeof config - 2);
	// eeprom_write_block(&config, RF12_EEPROM_ADDR, sizeof config);
	// this uses 170 bytes less flash than eeprom_write_block(), no idea why
	eeprom_write_byte(RF12_EEPROM_ADDR, ((byte*) &config)[0]);
	for (byte i = 0; i < sizeof config; ++ i)
		eeprom_write_byte(RF12_EEPROM_ADDR + i, ((byte*) &config)[i]);

	if (rf12_configSilent())
		rf12_configDump();
	else
		showString(INITFAIL);
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// = = = = = = = = = = = = = = = = = = = = = = = = = = =
// Network communication

// = = = = = = = = = = = = = = = = = = = = = = = = = = =
// my_recvDone
// returns 1 if a message was received successfully
int my_recvDone() {
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
				showString(PSTR(" -> ack\n"));
				rf12_sendStart(RF12_ACK_REPLY, 0, 0);
			}
#ifdef DEBUG
			Serial.print("rf12_len "); Serial.println(rf12_len);
			Serial.print("rf12_data[0] "); Serial.println(rf12_data[0]);
#endif
			// grab the pattern that we recieve, unless no data...
			int new_pattern = ((rf12_len > 0) ? rf12_data[0] : g_pattern);

			// ...if we get a crazy clocksync ping msg, then just ignore it
			if (msgReceived = (new_pattern != PATTERN_CLOCKSYNC_PING)) {
				//otherwise, msgReceived is true and set the new pattern
				g_pattern = new_pattern;
			}
#ifdef DEBUG
			Serial.print("pattern "); Serial.println(g_pattern);
#endif
			activityLed(0);
		}
#ifdef DEBUG
		Serial.print("msgReceived: "); Serial.println(msgReceived);
#endif
	}
	return msgReceived;
}

// = = = = = = = = = = = = = = = = = = = = = = = = = = =
// my_send
int my_send() {
	if (msg_cmd) {
		//if rf12_canSend returns 1, then you must subsequently call rf12_sendStart.
		if (rf12_canSend()) {
			//do yo thang:
			activityLed(1);
#ifdef DEBUG
			showString(PSTR("Send -> "));
			Serial.print((char) msg_cmd); Serial.print(" ");
			Serial.print((word) msg_sendLen);
			showString(PSTR(" b\n"));
#endif
			//make this an ack if requested
			byte header = msg_cmd == 'a' ? RF12_HDR_ACK : 0;
			// if not broadcast, set the destination node
			if (msg_dest)
				header |= RF12_HDR_DST | msg_dest;
			//actually send the message
			rf12_sendStart(header, msg_stack, msg_sendLen);
			msg_cmd = 0;

			activityLed(0);
		}
	}
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Setup
// = = = = = = = = = = = = = = = = = = = = = = = = = = =
void setup() {
#ifdef DEBUG
	pinMode(A1, OUTPUT);
#endif

	if (rf12_config()) {
		config.nodeId = eeprom_read_byte(RF12_EEPROM_ADDR);
		config.group = eeprom_read_byte(RF12_EEPROM_ADDR + 1);
	}
#ifdef DEBUG
	else {
		config.nodeId = 0x41; // node A1 @ 433 MHz
		config.group = 0xD4;  // group 212 (valid values: 0-212)
		saveConfig();
	}

	Serial.begin(SERIAL_BAUD);
	Serial.println();
	displayVersion();
#endif

	g_pattern = PATTERN_OFF;

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

#ifndef DEBUG
	// turn the radio off in the most power-efficient manner
	Sleepy::loseSomeTime(32);
	rf12_sleep(RF12_SLEEP);
	// wait another 2s for the power supply to settle
	Sleepy::loseSomeTime(2000);
#endif

	randomSeed(analogRead(0));
}
