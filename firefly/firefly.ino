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

static void activityLed (byte on) {
#ifdef LED_PIN
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, !on);
#endif
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// = = = = = = = = = = = = = = = = = = = = = = = = = = =
// RF12 configuration setup code

/// @details
/// For the EEPROM layout, see http://jeelabs.net/projects/jeelib/wiki/RF12demo
/// Useful url: http://blog.strobotics.com.au/2009/07/27/rfm12-tutorial-part-3a/

// RF12 configuration area
typedef struct {
    byte nodeId;            // used by rf12_config, offset 0
    byte group;             // used by rf12_config, offset 1
    byte format;            // used by rf12_config, offset 2
    byte hex_output   :2;   // 0 = dec, 1 = hex, 2 = hex+ascii
    byte collect_mode :1;   // 0 = ack, 1 = don't send acks
    byte quiet_mode   :1;   // 0 = show all, 1 = show only valid packets
    byte spare_flags  :4;
    word frequency_offset;  // used by rf12_config, offset 4
    byte pad[RF12_EEPROM_SIZE-8];
    word crc;
} RF12Config;

static RF12Config config;

// cmd may be set to: [0, 'a', 'c']
// 0   no command
// 'a' send request ack
// 'c' send
static char msg_cmd;
static word msg_value;
static byte msg_stack[RF12_MAXDATA+4], msg_top, msg_sendLen, msg_dest;
static byte msg_testCounter; //number of test packets sent

static void addCh (char* msg, char c) {
  byte n = strlen(msg);
  msg[n] = c;
}

static void addInt (char* msg, word v) {
  if (v >= 10)
    addInt(msg, v / 10);
  addCh(msg, '0' + v % 10);
}

static void showNibble (byte nibble) {
    char c = '0' + (nibble & 0x0F);
    if (c > '9')
        c += 7;
    Serial.print(c);
}

static void showByte (byte value) {
    if (config.hex_output) {
        showNibble(value >> 4);
        showNibble(value);
    } else
        Serial.print((word) value);
}

static word crc16Calc (const void* ptr, byte len) {
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

static void saveConfig () {
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

static byte bandToFreq (byte band) {
     return band == 4 ? RF12_433MHZ : band == 8 ? RF12_868MHZ : band == 9 ? RF12_915MHZ : 0;
}

static void printOneChar (char c) {
    Serial.print(c);
}

static void displayASCII (const byte* data, byte count) {
    for (byte i = 0; i < count; ++i) {
        printOneChar(' ');
        char c = (char) data[i];
        printOneChar(c < ' ' || c > '~' ? '.' : c);
    }
    Serial.println();
}

static void displayVersion () {
    showString(PSTR(VERSION));
    showString(PSTR("\nBlue pin: "));
    Serial.println(BLUEPIN);
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// DataFlash code

#if DATAFLASH
#include "dataflash.h"
#else // DATAFLASH

#define df_present() 0
#define df_initialize()
#define df_dump()
#define df_replay(x,y)
#define df_erase(x)
#define df_wipe()
#define df_append(x,y)

#endif

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// FIXME update this help menu based on handleSerialInput()
const char helpText1[] PROGMEM =
    "\n"
    "Available commands:\n"
    "  <nn> i     - set node ID (standard node ids are 1..30)\n"
    "  <n> b      - set MHz band (4 = 433, 8 = 868, 9 = 915)\n"
    "  <nnnn> o   - change frequency offset within the band (default 1600)\n"
    "               96..3903 is the range supported by the RFM12B\n"
    "  <nnn> g    - set network group (RFM12 only allows 212, 0 = any)\n"
    "  <n> c      - set collect mode (advanced, normally 0)\n"
    "  t          - broadcast max-size test packet, request ack\n"
    "  ...,<nn> a - send data packet to node <nn>, request ack\n"
    "  ...,<nn> s - send data packet to node <nn>, no ack\n"
    "  <n> q      - set quiet mode (1 = don't report bad packets)\n"
    "  <n> x      - set reporting format (0: decimal, 1: hex, 2: hex+ascii)\n"
    "  123 z      - total power down, needs a reset to start up again\n"
    "Remote control commands:\n"
    "  <hchi>,<hclo>,<addr>,<cmd> f     - FS20 command (868 MHz)\n"
    "  <addr>,<dev>,<on> k              - KAKU command (433 MHz)\n"
;

const char helpText2[] PROGMEM =
    "Flash storage (JeeLink only):\n"
    "    d                                  - dump all log markers\n"
    "    <sh>,<sl>,<t3>,<t2>,<t1>,<t0> r    - replay from specified marker\n"
    "    123,<bhi>,<blo> e                  - erase 4K block\n"
    "    12,34 w                            - wipe entire flash memory\n"
;

static void showString (PGM_P s) {
    for (;;) {
        char c = pgm_read_byte(s++);
        if (c == 0)
            break;
        if (c == '\n')
            printOneChar('\r');
        printOneChar(c);
    }
}

static void showHelp () {
    showString(helpText1);
    if (df_present())
        showString(helpText2);
    showString(PSTR("Current configuration:\n"));
    rf12_configDump();
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// = = = = = = = = = = = = = = = = = = = = = = = = = = =
// serial I/O

//this function is intended only to run in debug mode!!
static void handleSerialInput (char c) {
    if ('0' <= c && c <= '9') {
        msg_value = 10 * msg_value + c - '0';
        return;
    }

    if (c == ',') {
        if (msg_top < sizeof msg_stack)
            msg_stack[msg_top++] = msg_value; // truncated to 8 bits
        msg_value = 0;
        return;
    }

    if ('a' <= c && c <= 'z') {
        showString(PSTR("> "));
        for (byte i = 0; i < msg_top; ++i) {
            Serial.print((word) msg_stack[i]);
            printOneChar(',');
        }
        Serial.print(msg_value);
        Serial.println(c);
    }
    
    // keeping this out of the switch reduces code size (smaller branch table)
    if (c == '>') {
      // special case, send to specific band and group, and don't echo cmd
      // input: band,group,node,header,data...
      msg_stack[msg_top++] = msg_value;
      // TODO: frequency offset is taken from global config, is that ok?
      rf12_initialize(msg_stack[2], bandToFreq(msg_stack[0]), msg_stack[1],
                          config.frequency_offset);
      rf12_sendNow(msg_stack[3], msg_stack + 4, msg_top - 4);
      rf12_sendWait(2);
      rf12_configSilent();
    } 
    else if (c > ' ') {
      // base case, handle alpha commands
      // (only set up any messages; use my_interrupt() to send)
      switch (c) {

        case 'i': // set node id
            config.nodeId = (config.nodeId & 0xE0) + (msg_value & 0x1F);
            saveConfig();
            break;

        case 'b': // set frequency band in MHz: 4 = 433, 8 = 868, 9 = 915
            msg_value = bandToFreq(msg_value);
            if (msg_value) {
                config.nodeId = (msg_value << 6) + (config.nodeId & 0x3F);
                config.frequency_offset = 1600;
                saveConfig();
            }
            break;
        case 'o': { // Increment frequency within band
// Stay within your country's ISM spectrum management guidelines, i.e.
// allowable frequencies and their use when selecting operating frequencies.
            if ((msg_value > 95) && (msg_value < 3904)) { // supported by RFM12B
                config.frequency_offset = msg_value;
                saveConfig();
            }
            break;
        }
            
        case 'g': // set network group
            config.group = msg_value;
            saveConfig();
            break;

        case 'c': // set collect mode (off = 0, on = 1)
            config.collect_mode = msg_value;
            saveConfig();
            break;

        case 't': // broadcast a maximum size test packet, request an ack
            msg_cmd = 'a';
            msg_sendLen = RF12_MAXDATA;
            msg_dest = 0;
            for (byte i = 0; i < RF12_MAXDATA; ++i)
                msg_stack[i] = i + msg_testCounter;
            showString(PSTR("test "));
            showByte(msg_testCounter); // first byte in test buffer
            ++msg_testCounter;
            break;

        case 'a': // send packet to node ID N, request an ack
        case 's': // send packet to node ID N, no ack
            msg_cmd = c;
            msg_sendLen = msg_top;
            msg_dest = msg_value;
            break;
/*
        case 'f': // send FS20 command: <hchi>,<hclo>,<addr>,<cmd>f
            rf12_initialize(0, RF12_868MHZ, 0);
            activityLed(1);
            fs20cmd(256 * msg_stack[0] + msg_stack[1], msg_stack[2], msg_value);
            activityLed(0);
            rf12_configSilent();
            break;

        case 'k': // send KAKU command: <addr>,<dev>,<on>k
            rf12_initialize(0, RF12_433MHZ, 0);
            activityLed(1);
            kakuSend(stack[0], msg_stack[1], msg_value);
            activityLed(0);
            rf12_configSilent();
            break;
*/
        case 'q': // turn quiet mode on or off (don't report bad packets)
            config.quiet_mode = msg_value;
            saveConfig();
            break;

        case 'x': // set reporting mode to decimal (0), hex (1), hex+ascii (2)
            config.hex_output = msg_value;
            saveConfig();
            break;

        case 'v': //display the interpreter version and configuration
            displayVersion();
            rf12_configDump();

        case 'l': // turn activity LED on or off
            activityLed(msg_value);
            break;
/*
        case 'd': // dump all log markers
            if (df_present())
                df_dump();
            break;

        case 'r': // replay from specified seqnum/time marker
            if (df_present()) {
                word seqnum = (msg_stack[0] << 8) | msg_stack[1];
                long asof = (msg_stack[2] << 8) | msg_stack[3];
                asof = (asof << 16) | ((msg_stack[4] << 8) | msg_value);
                df_replay(seqnum, asof);
            }
            break;

        case 'e': // erase specified 4Kb block
            if (df_present() && msg_stack[0] == 123) {
                word block = (msg_stack[1] << 8) | msg_value;
                df_erase(block);
            }
            break;

        case 'w': // wipe entire flash memory
            if (df_present() && stack[0] == 12 && value == 34) {
                df_wipe();
                showString(PSTR("erased\n"));
            }
            break;
*/
        case 'p': // select a new pattern (side effect: broadcast new pattern to the network)
            //immediately change pattern, rather than wait on a network msg:
            g_pattern = msg_value;
            //broadcast the new pattern:
            msg_cmd = c;
            msg_stack[0] = msg_value;
            msg_sendLen = 1;
            msg_dest = 0; //broadcast message
#ifdef DEBUG
            Serial.println("New pattern " + g_pattern);
#endif
            break;
            
        default:
            showHelp();
        }
    }

    msg_value = msg_top = 0;
  }
//    memset(stack, 0, sizeof stack);
/*
  else if ('A' <= c && c <= 'Z') {
    config.nodeId = (config.nodeId & 0xE0) + (c & 0x1F);
    saveConfig();
  } 
  else if (c > ' ')
    showHelp();
    */


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// = = = = = = = = = = = = = = = = = = = = = = = = = = =
// Helper functions 

// Create a 15 bit color value from R,G,B
unsigned int Color(uint8_t r, uint8_t g, uint8_t b)
{
  //Take the lowest 5 bits of each value and append them end to end
#ifdef UPSIDE_DOWN_LEDS
  //swap green and blue bits
  return( ((unsigned int)b & 0x1F )<<10 | ((unsigned int)g & 0x1F)<<5 | (unsigned int)r & 0x1F);
#else
  return( ((unsigned int)g & 0x1F )<<10 | ((unsigned int)b & 0x1F)<<5 | (unsigned int)r & 0x1F);
#endif
}

//Input a value 0 to 127 to get a color value.
//The colours are a transition r - g -b - back to r
unsigned int Wheel(byte WheelPos)
{
  byte r,g,b;
  switch(WheelPos >> 5)
  {
  case 0:
    r=31- WheelPos % 32;   //Red down
    g=WheelPos % 32;      // Green up
    b=0;                  //blue off
    break; 
  case 1:
    g=31- WheelPos % 32;  //green down
    b=WheelPos % 32;      //blue up
    r=0;                  //red off
    break; 
  case 2:
    b=31- WheelPos % 32;  //blue down 
    r=WheelPos % 32;      //red up
    g=0;                  //green off
    break; 
  }
  return(Color(r,g,b));
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// = = = = = = = = = = = = = = = = = = = = = = = = = = =
// Network communication

// this just outputs received msgs to serial. Only gets called ifdef DEBUG
void debugRecv() {
    byte n = rf12_len;
    if (rf12_crc == 0)
        showString(PSTR("OK"));
    else {
        if (config.quiet_mode)
            return;
        showString(PSTR(" ?"));
        if (n > 20) // print at most 20 bytes if crc is wrong
            n = 20;
    }
    if (config.hex_output)
        printOneChar('X');
    if (config.group == 0) {
        showString(PSTR(" G"));
        showByte(rf12_grp);
    }
    printOneChar(' ');
    showByte(rf12_hdr);
    for (byte i = 0; i < n; ++i) {
        if (!config.hex_output)
            printOneChar(' ');
        showByte(rf12_data[i]);
    }
    Serial.println();

    if (config.hex_output > 1) { // also print a line as ascii
        showString(PSTR("ASC "));
        if (config.group == 0) {
            showString(PSTR(" II "));
        }
        printOneChar(rf12_hdr & RF12_HDR_DST ? '>' : '<');
        printOneChar('@' + (rf12_hdr & RF12_HDR_MASK));
        displayASCII((const byte*) rf12_data, n);
    }
}

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
        showString(PSTR(" -> "));
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
// top-level / timing functions 

// = = = = = = = = = = = = = = = = = = = = = = = = = = =
// handleInputs()
// returns 1 if either network or serial input was received
int handleInputs() {
    int trigger = 0;
    
    if (Serial.available()) {
        handleSerialInput(Serial.read());
        trigger = 1;
    }

//  debounceInputs();

    // note that my_recvDone() may alter g_pattern, if the crc is good
    trigger |= my_recvDone();

#ifdef DEBUG
    if (trigger) Serial.println(".");
#endif

    return trigger;
}

// = = = = = = = = = = = = = = = = = = = = = = = = = = =
// my_interrupt()
// A pattern should call this function (or some equivalent of sub-functions)
// whenever reasonably possible to keep network comm and serial IO going. 
// returns the output of handleInputs()
void my_interrupt()
{
    handleInputs();  //check for serial input, call my_recvDone()
    my_send(); //try to send, if (cmd != 0)

    return;
}

// = = = = = = = = = = = = = = = = = = = = = = = = = = =
// delay functions - // don't waste cycles with delay(); 

//poll for new inputs and break if an interrupt is detected
void my_delay_with_break(unsigned long wait_time) {
  unsigned long t0 = millis();
  while( (millis() - t0) < wait_time )
    if ( handleInputs() ) return;
}
// poll for new inputs, detect interrupts but do not break the delay
void my_delay(unsigned long wait_time) {
  unsigned long t0 = millis();
  while( millis() - t0 < wait_time )
    handleInputs();
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
  else {
    config.nodeId = 0x41; // node A1 @ 433 MHz
    config.group = 0xD4;  // group 212 (valid values: 0-212)
    saveConfig();
  }

  g_pattern = PATTERN_OFF;

  Serial.begin(SERIAL_BAUD);
  Serial.println();
  displayVersion();

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

// = = = = = = = = = = = = = = = = = = = = = = = = = = =
// Pattern control switch!
void runPattern() {
  activityLed(1);
#ifdef DEBUG
  Serial.print("Run pattern "); Serial.println(g_pattern);
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

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Main loop
// = = = = = = = = = = = = = = = = = = = = = = = = = = =
void loop() {
    my_interrupt(); // note that patterns should also check for interrupts, on their own time
    runPattern();  // switches control to the active pattern
}


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// = = = = = = = = = = = = = = = = = = = = = = = = = = =
// PATTERN functions begin here

// = = = = = = = = = = = = = = = = = = = = = = = = = = =
// Utility functions for patterns
boolean pattern_interrupt(int current_pattern)
{
  my_interrupt();
#ifdef DEBUG
  if (g_pattern != current_pattern)
  {
      Serial.print("old pattern: ");
      Serial.print(current_pattern);
      Serial.print("   new pattern: ");
      Serial.println(g_pattern);
  }
#endif
  //we respond to a new pattern command
  return (current_pattern != g_pattern);
}
