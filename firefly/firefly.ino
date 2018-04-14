#include <Ports.h>
#include <JeeLib.h>
#include "RF12.h"
#include "firefly.h"

/*                 JeeNode / JeeNode USB / JeeSMD
 -------|-----------------------|----|-----------------------|----
|       |D3  A1 [Port2]  D5     |    |D3  A0 [port1]  D4     |    |
|-------|IRQ AIO +3V GND DIO PWR|    |IRQ AIO +3V GND DIO PWR|    |
| D1|TXD|                                           ---- ----     |
| A5|SCL|                                       D12|MISO|+3v |    |
| A4|SDA|   Atmel Atmega 328                    D13|SCK |MOSI|D11 |
|   |PWR|   JeeNode / JeeNode USB / JeeSMD         |RST |GND |    |
|   |GND|                                       D8 |BO  |B1  |D9  |
| D0|RXD|                                           ---- ----     |
|-------|PWR DIO GND +3V AIO IRQ|    |PWR DIO GND +3V AIO IRQ|    |
|       |    D6 [Port3]  A2  D3 |    |    D7 [Port4]  A3  D3 |    |
 -------|-----------------------|----|-----------------------|----
*/

static boolean wirefly_needToSend;
static MilliTimer wirefly_sendTimer; //broadcast the pattern on this interval
static int WIREFLY_TIMER_BROADCAST = 4096;
static MilliTimer pattern_immuneTimer; //stop listening after pattern change
static int WIREFLY_TIMER_IMMUNE = 16384;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Main loop functions, top-level / timing functions 

// = = = = = = = = = = = = = = = = = = = = = = = = = = =
void loop() {
	//PHASE1: input, listen
	wirefly_interrupt(); // note that patterns can call pattern_interrupt(), on their own time

	//PHASE2: communicate
	wirefly_send(); //send a pending message, if (needToSend != 0)

	//PHASE3: display
	pattern_run();  // switches control to the active pattern
}

// = = = = = = = = = = = = = = = = = = = = = = = = = = =
// wirefly_interrupt()
// Call this function whenever reasonably possible to keep network comm and serial I/O going.
// returns true if input of any kind was received
int wirefly_interrupt() {
	int current_pattern = pattern_get();
  //first check for serial input / commands
#if TINY
    if (_receive_buffer_index)
        handleInput(inChar());
#else
    if (Serial.available())
        handleInput(Serial.read());
#endif

	// check for network input via rf12_recvDone()
	if (wirefly_recvDone())
	{
		// check for the pattern data and grab it
		if (rf12_data[0] == WIREFLY_SEND_PATTERN && !pattern_immuneTimer.poll())
		{
			int new_pattern = rf12_data[1];
			//set the new pattern
			pattern_set(new_pattern);
		}
	}
	//check to see if either serial or network changed the pattern
	boolean patternChanged = (pattern_get() != current_pattern);
	if (patternChanged) {
#ifdef DEBUG
		// log if new pattern detected
		Serial.print("wirefly_interrupt() old pattern: ");
		Serial.print(current_pattern);
		Serial.print("  new: ");
		Serial.println(pattern_get());
#endif
		pattern_immuneTimer.set(WIREFLY_TIMER_IMMUNE); //don't listen for a little while
		wirefly_sendTimer.set(0); //we want to send a message quickly
	}

	return patternChanged;
}

// = = = = = = = = = = = = = = = = = = = = = = = = = = =
// delay function - // use these, don't waste cycles with delay()
// poll for new inputs and break if an interrupt is detected
boolean wirefly_delay(unsigned long wait_time) {
	unsigned long t0 = millis();
	while ((millis() - t0) < wait_time)
		if (wirefly_interrupt())
			return false;
	return true;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// = = = = = = = = = = = = = = = = = = = = = = = = = = =
// RF12 Network communication

// = = = = = = = = = = = = = = = = = = = = = = = = = = =
// wirefly_recvDone
// returns 1 if a message was received successfully
int wirefly_recvDone() {
  int msgReceived = 0;
  // (receive a network message if one exists; keep the RF12 library happy)
  // rf12_recvDone() needs to be constantly called in order to recieve new transmissions.
  // It checks to see if a packet has been received, returns true if it has
  if (rf12_recvDone()) {
    byte n = rf12_len;
    // if we got a bad crc, then no message was received.
    msgReceived = !rf12_crc;
    if (rf12_crc == 0) {
      showString(PSTR("OK"));
      // If a new transmission comes in and CRC is ok, don't poll recv state again -
      // otherwise rf12_crc, rf12_len, and rf12_data will be reset.
      activityLed(1);
      //ack if requested
      if (RF12_WANTS_ACK && (config.collect_mode) == 0) {
        showString(PSTR("Send -> ack\n"));
        rf12_sendStart(RF12_ACK_REPLY, 0, 0);
        rf12_sendWait(1); // don't power down too soon
      }
      activityLed(0);
    }
    else {
        if (config.quiet_mode)
            return msgReceived;
        showString(PSTR(" ?"));
        if (n > 20) // print at most 20 bytes if crc is wrong
            n = 20;
    }
  }
  return msgReceived;
}

// = = = = = = = = = = = = = = = = = = = = = = = = = = =
// wirefly_send
// Call this a lot, it will decide whether to send the broadcast or not.
int wirefly_send() {
	// every four seconds
	if (wirefly_sendTimer.poll(WIREFLY_TIMER_BROADCAST))
		wirefly_needToSend = 1;
  //if rf12_canSend returns 1, then you must subsequently call rf12_sendStart.
  if (wirefly_needToSend && rf12_canSend()) {
    activityLed(1);
    //do yo thang:
    wirefly_msg_stack[0] = WIREFLY_SEND_PATTERN;
    wirefly_msg_stack[1] = pattern_get(); //send the pattern as integer
    wirefly_msg_sendLen = 2;
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
    //actually send the message:
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
  rf12_setup();

	pattern_set(PATTERN_OFF);

	randomSeed(analogRead(0));
	wirefly_sendTimer.set(0); //we want to send a message quickly

  //set the AIO pin on the jeeNode to be an output pin
  pinMode(A1, OUTPUT);
}

