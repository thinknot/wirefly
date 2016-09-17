#ifndef __FIREFLY_H
#define __FIREFLY_H

/// Configure some values in EEPROM for easy config of the RF12 later on.
// 2009-05-06 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#define MAJOR_VERSION RF12_EEPROM_VERSION // bump when EEPROM layout changes
#define MINOR_VERSION 2                   // bump on other non-trivial changes
#define VERSION "[Wirefly 05-2015]"
#define TINY        0
#define SERIAL_BAUD 57600   // adjust as needed
#define DATAFLASH   0       // set to 0 for non-JeeLinks, else 4/8/16 (Mbit)
//#define LED_PIN     9       // activity LED, comment out to disable
/// Save a few bytes of flash by declaring const if used more than once.
const char INVALID1[] PROGMEM = "\rInvalid\n";
const char INITFAIL[] PROGMEM = "config save failed\n";

// comment out below before compiling production codez!
#define DEBUG 1

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// = = = = = = = = = = = = = = = = = = = = = = = = = = =
// RF12 configuration setup code
#define RF12_BUFFER_SIZE	66
static uint8_t my_data[RF12_BUFFER_SIZE];
static byte msg_stack[RF12_MAXDATA+4], msg_top, msg_sendLen, msg_dest;
// cmd may be set to: [0, 'a', 'c']
// 0   no command
// 'a' send request ack
// 'c' send
static char msg_cmd;

#define COLLECT 0x20 // collect mode, i.e. pass incoming without sending acks

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

// http://jeelabs.net/pub/docs/jeelib/classSleepy.html
// WDT inturrupt handler, required to use Sleepy::loseSomeTime()
//ISR(WDT_vect) { Sleepy::watchdogEvent(); }

// Select at features:
//#define LUXMETER 10
//#define RTCTIMER 11
#define LED_MONO 100
//#define LED_RGB 101
//#define LED_ADDR 102
//#define RGB_STRIP 200
//#define RGB_NEOPIX 201
#define LED_SUPERFLUX 202

// For PATTERN_FADER and rgbSet(): 
// Used to adjust the limits for the LED, especially if it has a lower ON threshold
#define  MIN_RGB_VALUE  10   // no smaller than 10. TODO add gamma correction 
#define  MAX_RGB_VALUE  255  // no bigger than 255. (dark)

#ifdef LED_MONO
  #define LEDPIN 5
#endif
#ifdef LED_RGB
  #define REDPIN 5        // DIO2
  #define GREENPIN 6      // DIO3
  #define BLUEPIN 3   // IRQ2
  //#define BLUEPIN 9   // SEL1 LedNode
#endif
  static void rgbSet(byte r, byte g, byte b)
  {
#ifdef LED_RGB
	analogWrite(REDPIN, r);
	analogWrite(GREENPIN, g);
	analogWrite(BLUEPIN, b);
#endif
#ifdef LED_MONO
        byte x = 0.299*r + 0.587*g + 0.114*b;
        analogWrite(LEDPIN, x);
#endif
  }

#ifdef LUXMETER
PortI2C myBus (3);
LuxPlug sensor (myBus, 0x39);
#endif

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// = = = = = = = = = = = = = = = = = = = = = = = = = = =
// Pattern control, pattern variables
extern uint8_t g_pattern;

#define PATTERN_OFF             0
#define PATTERN_TWINKLE         1
#define PATTERN_FIREFLY         2
#define PATTERN_FADER           3
#define PATTERN_PULSER          4
#define PATTERN_CLOCKSYNC      10
#define PATTERN_CLOCKSYNC_PING 11
#define PATTERN_LUXMETER       90

#define PULSE_COLORSPEED 5     // For PATTERN_PULSE, make this higher to slow down

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// = = = = = = = = = = = = = = = = = = = = = = = = = = =
// Function prototypes
void saveConfig();
void showString (PGM_P s);
void handleSerialInput (char c);
void activityLed (byte on);
word crc16Calc (const void* ptr, byte len);
int my_send();
int my_interrupt();
int pattern_delay(unsigned long wait_time, uint8_t current_pattern);
void my_delay(unsigned long wait_time);
boolean pattern_interrupt(int current_pattern);
void runPattern();
void pattern_off();
void pattern_randomTwinkle();
void pattern_teamFirefly();
void pattern_clockSync();
void pattern_rgbFader();
void pattern_rgbpulse();
void handleSerialInput (char c);

#ifdef DEBUG
void debugRecv();
void displayVersion ();
#endif

#endif

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
