#ifndef __FIREFLY_H
#define __FIREFLY_H

#define MAJOR_VERSION RF12_EEPROM_VERSION // bump when EEPROM layout changes
#define MINOR_VERSION 2                   // bump on other non-trivial changes
#define VERSION "[Wirefly 12-2014]"
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

// http://jeelabs.net/pub/docs/jeelib/classSleepy.html
// WDT inturrupt handler, required to use Sleepy::loseSomeTime()
//ISR(WDT_vect) { Sleepy::watchdogEvent(); }

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
    
//PortI2C myBus (3);
//LuxPlug sensor (myBus, 0x39);
byte highGain;

// For PATTERN_FADER and rgbSet(): 
// Used to adjust the limits for the LED, especially if it has a lower ON threshold
#define  MIN_RGB_VALUE  10   // no smaller than 10. TODO add gamma correction 
#define  MAX_RGB_VALUE  255  // no bigger than 255. (dark)

static void rgbSet(byte r, byte g, byte b)
{
	analogWrite(REDPIN, r);
	analogWrite(GREENPIN, g);
	analogWrite(BLUEPIN, b);
}

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
int my_send();
void my_interrupt();
void my_delay_with_break(unsigned long wait_time);
void my_delay(unsigned long wait_time);
boolean pattern_interrupt(int current_pattern);
void runPattern();
void pattern_off();
void pattern_randomTwinkle();
void pattern_teamFirefly();
void pattern_clockSync();
void pattern_rgbFader();
void pattern_rgbpulse();

#endif
