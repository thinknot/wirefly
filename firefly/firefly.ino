/// Configure some values in EEPROM for easy config of the RF12 later on.
// 2009-05-06 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

// this version adds flash memory support, 2009-11-19
// Adding frequency features, author JohnO, 2013-09-05
// Major EEPROM format change, refactoring, and cleanup for v12, 2014-02-13

#include <JeeLib.h>
#include <util/crc16.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <util/parity.h>

#define MAJOR_VERSION RF12_EEPROM_VERSION // bump when EEPROM layout changes
#define MINOR_VERSION 2                   // bump on other non-trivial changes
#define VERSION "[Wirefly 10-2014]"
#define TINY        0
#define SERIAL_BAUD 57600   // adjust as needed
#define DATAFLASH   0       // set to 0 for non-JeeLinks, else 4/8/16 (Mbit)
//#define LED_PIN     9       // activity LED, comment out to disable
/// Save a few bytes of flash by declaring const if used more than once.
const char INVALID1[] PROGMEM = "\rInvalid\n";
const char INITFAIL[] PROGMEM = "config save failed\n";

// comment out below before compiling production codez!
//#define DEBUG 1

#define RF12_BUFFER_SIZE	66
static uint8_t my_data[RF12_BUFFER_SIZE];

#define COLLECT 0x20 // collect mode, i.e. pass incoming without sending acks

// Select platform features:
//#define LUXMETER
//#define TIMER
//#define LED_STRIP
//#define LED_NEOPIX
#define LED_SUPERFLUX

#define REDPIN 5        // DIO2
#define GREENPIN 6      // DIO3
#define BLUEPIN 3   // IRQ2
//#define BLUEPIN 9   // SEL1 LedNode
    
PortI2C myBus (3);
LuxPlug sensor (myBus, 0x39);
byte highGain;

// For PATTERN_FADER and rgbSet(): 
// Used to adjust the limits for the LED, especially if it has a lower ON threshold
#define  MIN_RGB_VALUE  10   // no smaller than 10. TODO add gamma correction 
#define  MAX_RGB_VALUE  255  // no bigger than 255. (dark)

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// = = = = = = = = = = = = = = = = = = = = = = = = = = =
// Pattern control, pattern variables
#define PATTERN_OFF		0
#define PATTERN_TWINKLE		1
#define PATTERN_FIREFLY         2
#define PATTERN_FADER           3
#define PATTERN_PULSER          4
#define PATTERN_CLOCKSYNC	10
#define PATTERN_CLOCKSYNC_PING	11
#define PATTERN_LUXMETER        90

ISR(WDT_vect) { Sleepy::watchdogEvent(); }

#define PULSE_COLORSPEED 5     // For PATTERN_PULSE, make this higher to slow down

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

static void rgbSet(byte r, byte g, byte b)
{
	analogWrite(REDPIN, r);
	analogWrite(GREENPIN, g);
	analogWrite(BLUEPIN, b);
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

static uint8_t g_pattern = 0;

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
        case 'p': // select a new pattern 
                  // (side effect: broadcast a pattern command to the network)
            //immediately change pattern, rather than wait on a network msg
            g_pattern = msg_value; 
            //broadcast the new pattern
            msg_cmd = c;
            msg_stack[0] = msg_value;
            msg_sendLen = 1;
            msg_dest = 0;
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

// = = = = = = = = = = = = = = = = = = = = = = = = = = =
// PATTERN_OFF: No color value, clear LEDs
void pattern_off(byte opts = 0) {
#ifdef DEBUG
    Serial.println("Pattern Off");
#endif                
/*
  int startIndex = (IGNORE_FRONT(opts)?PIXEL_FRONT_LAST+1:0);
  int endIndex = (IGNORE_BACK(opts)?PIXEL_FRONT_LAST+1:strip.numPixels());
  for(uint8_t p=startIndex; p<endIndex; p++) {
    debounceInputs();
    strip.setPixelColor(p,0);
  }
  strip.show();
*/
  rgbSet(MAX_RGB_VALUE, MAX_RGB_VALUE, MAX_RGB_VALUE);
  
  my_delay(50);  //TODO: need to go into low power mode here instead of delay
//  debounceInputs();
}

// = = = = = = = = = = = = = = = = = = = = = = = = = = =
// PATTERN_LUXMETER: low-power periodic monitoring
void pattern_luxMeter() {
#ifdef DEBUG
    Serial.println("Begin pattern: luxMeter. waiting for sunset");
#endif
  while (true) {
    sensor.begin();
    sensor.setGain(highGain);
    delay(1000); // Wait for proper powerup.
    
    const word* photoDiodes = sensor.getData();
    Serial.print("LUX ");
    Serial.print(photoDiodes[0]);
    Serial.print(' ');
    Serial.print(photoDiodes[1]);
    Serial.print(' ');
    Serial.print(sensor.calcLux());
    Serial.print(' ');
    Serial.println(highGain);
    sensor.poweroff(); // Power off when we've got the data.
    
    highGain = ! highGain;
    delay(6000);
  }
}

// = = = = = = = = = = = = = = = = = = = = = = = = = = =
// PATTERN_TWINKLE:
void pattern_randomTwinkle() {
#ifdef DEBUG
    Serial.println("Begin pattern: randomTwinkle");
#endif                
    int lantern_on = 0; // LED 1 = enabled, 0 = disabled 
    int wait_millis = 0; // time to stay either on or off
    unsigned long clock_elapsed = 0; // counting how long it's been on or off

    unsigned long clock_current = 0; //this variable will be updated each iteration of loop
    unsigned long clock_recent = 0; //no time has passed yet
    
    while (true) {        
        clock_recent = clock_current; //save clock_current from the previous loop() to clock_recent
        clock_current = millis(); //update to "now"
        
        //this is roll-over proof, if clock_current is small, and clock_recent large, 
        //the result rolls over to a small positive value:
        clock_elapsed = clock_elapsed + (clock_current - clock_recent);    

//        debounceInputs();
        
        if (clock_elapsed >= wait_millis) { //time to switch
            if (lantern_on == 1) { // previously on
                wait_millis = random(1500, 6999); // time to stay off
                rgbSet(MAX_RGB_VALUE, MAX_RGB_VALUE, MAX_RGB_VALUE);
                lantern_on = 0;
            }
            else { //the light was previously turned off
                wait_millis = random(500, 900); // time to stay on
                rgbSet(23, 23, 23);
                lantern_on = 1;
            }
            clock_elapsed = 0; //reset
        }
        //check with the outside world:
        if (pattern_interrupt(PATTERN_TWINKLE)) return;
    }
}

// = = = = = = = = = = = = = = = = = = = = = = = = = = =
// PATTERN_FIREFLY:
void pattern_teamFirefly() {
#ifdef DEBUG
    Serial.println("Begin pattern: teamFirefly");
#endif                

/*    
MilliTimer g_sendTimer;
byte g_needToSend;
    // check the timer for sending a network message
    if (g_sendTimer.poll(3000)) // 3 seconds
        g_needToSend = 1;
*/        

        //check with the outside world:
        if (pattern_interrupt(PATTERN_FIREFLY)) return;
}

// = = = = = = = = = = = = = = = = = = = = = = = = = = =
// PATTERN_FADER:
/*
 RGB LED - Automatic Smooth Color Cycling - 
 Marco Colli
 April 2012 see http://forum.arduino.cc/index.php?topic=102040.0
 
 Uses the properties of the RGB Colour Cube
 The RGB colour space can be viewed as a cube of colour. If we assume a cube of dimension 1, then the 
 coordinates of the vertices for the cubve will range from (0,0,0) to (1,1,1) (all black to all white).
 The transitions between each vertex will be a smooth colour flow and we can exploit this by using the 
 path coordinates as the LED transition effect. 
*/

// Slowing things down we need ...
#define  FADE_TRANSITION_DELAY  70   // in milliseconds, between individual light changes
#define  FADE_WAIT_DELAY        500  // in milliseconds, at the end of each traverse
//
// Total traversal time is ((MAX_RGB_VALUE - MIN_RGB_VALUE) * TRANSITION_DELAY) + WAIT_DELAY
// eg, ((255-0)*70)+500 = 18350ms = 18.35s

// Structure to contain a 3D coordinate
typedef struct
{
  byte  x, y, z;
} coord;

static coord  v; // the current rgb coordinates (colour) being displayed

//The RGB colour space can be visualised as a cube whose (x, y, z) coordinates range from (0, 0, 0) 
//  or white, to (255, 255, 255) or black. 
//More generally the cube is defined in the 3D space (0,0,0) to (1,1,1), scaled by 255. 
//The vertices of this cube define the boundaries of the colour space, and moving along the 3D 
//  coordinates from one point to the next will naturally provide smooth colour transitions.
/*
 Vertices of a cube
      
    C+----------+G
    /|        / |
  B+---------+F |
   | |       |  |    y   
   |D+-------|--+H   ^  7 z
   |/        | /     | /
  A+---------+E      +--->x

*/
const coord vertex[] = 
{
 //x  y  z      name
  {0, 0, 0}, // A or 0
  {0, 1, 0}, // B or 1
  {0, 1, 1}, // C or 2
  {0, 0, 1}, // D or 3
  {1, 0, 0}, // E or 4
  {1, 1, 0}, // F or 5
  {1, 1, 1}, // G or 6
  {1, 0, 1}  // H or 7
};

/*
 A list of vertex numbers encoded 2 per byte.
 Hex digits are used as vertices 0-7 fit nicely (3 bits 000-111) and have the same visual
 representation as decimal, so bytes 0x12, 0x34 ... should be interpreted as vertex 1 to 
 v2 to v3 to v4 (ie, one continuous path B to C to D to E).
*/
const byte path[] =
{
  0x01, 0x23, 0x76, 0x54, 0x03, 0x21, 0x56, 0x74,  // trace the edges
  0x13, 0x64, 0x16, 0x02, 0x75, 0x24, 0x35, 0x17, 0x25, 0x70,  // do the diagonals
};

#define  MAX_PATH_SIZE  (sizeof(path)/sizeof(path[0]))  // size of the array

void traverse(int dx, int dy, int dz)
// Move along the colour line from where we are to the next vertex of the cube.
// The transition is achieved by applying the 'delta' value to the coordinate.
// By definition all the coordinates will complete the transition at the same 
// time as we only have one loop index.
{
  if ((dx == 0) && (dy == 0) && (dz == 0))   // no point looping if we are staying in the same spot!
    return;
    
  for (int i = 0; i < MAX_RGB_VALUE-MIN_RGB_VALUE; i++, v.x += dx, v.y += dy, v.z += dz)
  {
    // set the colour in the LED
    analogWrite(REDPIN, v.x);
    analogWrite(GREENPIN, v.y);
    analogWrite(BLUEPIN, v.z);
    
    delay(FADE_TRANSITION_DELAY);
    // wait for the transition delay
  }

  delay(FADE_WAIT_DELAY);
  // give it an extra rest at the end of the traverse
}

void pattern_rgbFader()
{
#ifdef DEBUG
    Serial.println("Begin pattern: rgbPulse");
#endif                
	while (true) 
	{
		int v1, v2 = 0;    // the new vertex and the previous one

		// initialise the place we start from as the first vertex in the array
		v.x = (vertex[v2].x ? MAX_RGB_VALUE : MIN_RGB_VALUE);
		v.y = (vertex[v2].y ? MAX_RGB_VALUE : MIN_RGB_VALUE);
		v.z = (vertex[v2].z ? MAX_RGB_VALUE : MIN_RGB_VALUE);

		// Now just loop through the path, traversing from one point to the next
		for (int i = 0; i < 2 * MAX_PATH_SIZE; i++)
		{
			// !! loop index is double what the path index is as it is a nybble index !!
			v1 = v2;
			if (i & 1)  // odd number is the second element and ...
				v2 = path[i >> 1] & 0xf;  // ... the bottom nybble (index /2) or ...
			else      // ... even number is the first element and ...
				v2 = path[i >> 1] >> 4;  // ... the top nybble

			traverse(vertex[v2].x - vertex[v1].x,
				vertex[v2].y - vertex[v1].y,
				vertex[v2].z - vertex[v1].z);

			if (pattern_interrupt(PATTERN_FADER))
			  return;
		}
	}
}

// = = = = = = = = = = = = = = = = = = = = = = = = = = =
// PATTERN_PULSER:
void pattern_rgbpulse() {
  int r, g, b;
#ifdef DEBUG
  Serial.println("Begin pattern: rgbPulse");
#endif                
  while (true) 
  {    
    // fade from blue to violet
    for (r = 0; r < 256; r++) { 
      analogWrite(REDPIN, r);
      my_delay_with_break(PULSE_COLORSPEED);
      if (pattern_interrupt(PATTERN_PULSER)) return;
    } 
    // fade from violet to red
    for (b = 255; b > 0; b--) { 
      analogWrite(BLUEPIN, b);
      my_delay_with_break(PULSE_COLORSPEED);
      if (pattern_interrupt(PATTERN_PULSER)) return;
    }
    // fade from red to yellow
    for (g = 0; g < 256; g++) { 
      analogWrite(GREENPIN, g);
      my_delay_with_break(PULSE_COLORSPEED);
      if (pattern_interrupt(PATTERN_PULSER)) return;
    }
    // fade from yellow to green
    for (r = 255; r > 0; r--) { 
      analogWrite(REDPIN, r);
      my_delay_with_break(PULSE_COLORSPEED);
      if (pattern_interrupt(PATTERN_PULSER)) return;
    }
    // fade from green to teal
    for (b = 0; b < 256; b++) { 
      analogWrite(BLUEPIN, b);
      my_delay_with_break(PULSE_COLORSPEED);
      if (pattern_interrupt(PATTERN_PULSER)) return;
    }
    // fade from teal to blue
    for (g = 255; g > 0; g--) { 
      analogWrite(GREENPIN, g);
      my_delay_with_break(PULSE_COLORSPEED);
      if (pattern_interrupt(PATTERN_PULSER)) return;
    }
  }
}

// = = = = = = = = = = = = = = = = = = = = = = = = = = =
// PATTERN_CLOCKSYNC:
void pattern_clockSync( byte opts = 0){
#ifdef DEBUG
    Serial.println("Begin pattern: clockSync");
#endif                    
  // rf12b-calibrated, about the time it takes to send a packet in milliseconds
  unsigned long time_cycle = 750;
//  unsigned long time_on = 750; //experimental

  // loop variables use to keep track of how long to listen  
  unsigned long time_start = 0;
  unsigned long time_end = 0;
  
  // track total time on and off, millis
  unsigned long sum_ON = 0;
  unsigned long sum_OFF = 0;
  
  // used to make adjustments to the time_cycle
  int ON_longer = 0;
  int OFF_longer = 0;
  
  // track pings received in either on/off state
  int ON_count = 0;
  int OFF_count = 0;

  //turn all lights off
  pattern_off(opts);

//  debounceInputs();
  my_delay(random(0, 2001)); //make sure everyone starts at a somewhat different time
//  debounceInputs();

  while(true)
  {
    //reset counters for this loop
    ON_count = 0;
    sum_ON = 0;
    OFF_count = 0;
    
    //Phase1
    rgbSet(123,123,123); //turn LED on
    
    //transmit a packet, while the LED is on
    if (rf12_canSend()) {
      uint8_t send_data = PATTERN_CLOCKSYNC_PING;
      rf12_sendStart(0, &send_data, 1);
    }
    //welcome back from radio land
    
    time_start = millis(); //start time is now
    time_end = time_start + time_cycle + ON_longer; // decide how long to listen w/ LED on

    while (millis() < time_end) { //listen for pings until the time is up
//      debounceInputs();
      if (rf12_recvDone() && !rf12_crc) { //if we get a good packet
      //this means somebody else is on at the same time as me, keep track
        sum_ON += millis() - time_start;
        ON_count++;
      }
    }

    //Phase2
    rgbSet(MAX_RGB_VALUE, MAX_RGB_VALUE, MAX_RGB_VALUE); //turn LED off
    
    time_start = millis(); //reset start time to now
    time_end = time_start + time_cycle + OFF_longer; //how long to listen w/ LED off

    while (millis() < time_end) { //listen for pings until the time is up
//      debounceInputs();
      if (rf12_recvDone() && rf12_crc == 0){ //if we get a good packet
      //this means somebody else is on when I am off, keep track
        sum_OFF += time_end - millis(); 
        OFF_count++;
      }
    }
  
    if (ON_count > OFF_count) // I am more in-sync than out-of-sync
    {
      ON_longer = 0;
      OFF_longer = (sum_ON/ON_count) >> 1;
    } 
    else if (ON_count < OFF_count) //I am more out-of-sync than in-sync
    {
      OFF_longer = 0;
      ON_longer = (sum_ON/ON_count) >> 1;
    } 
    else if (ON_count == 0 && OFF_count == 0) //initial state...
    {
      ON_longer = 0;
      OFF_longer = 0;
    } 
    else if (ON_count == OFF_count) //I am just plain out of phase
    {
      if (sum_ON < sum_OFF) // use a more precise method
      {
        ON_longer = 0;
        OFF_longer = (sum_ON/ON_count) >> 1;
      } 
      else 
      {
        OFF_longer = 0;
        ON_longer = (sum_OFF/OFF_count) >> 1;
      }
    }

    if( handleInputs() && (g_pattern <= PATTERN_CLOCKSYNC)) {
		return;
    }

  }
}

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
        Serial.print("msgRx "); Serial.println(msgReceived);
#endif
    }
    return msgReceived;
}

// = = = = = = = = = = = = = = = = = = = = = = = = = = =
// my_send
int my_send() {
    if (msg_cmd) {
      if (rf12_canSend()) {
        //if rf12_canSend returns 1, then you must subsequently call rf12_sendStart.
        //do yo thang:
        activityLed(1);
#ifdef DEBUG
        showString(PSTR(" -> "));
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

    // my_recvDone may alter g_pattern if the crc is good
    trigger |= my_recvDone();

#ifdef DEBUG
    if (trigger) Serial.println("Input!");
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
        pattern_off(1);  // all lights off, including/especially the lantern
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
