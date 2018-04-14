#ifndef __FIREFLY_H
#define __FIREFLY_H

#define WIREFLY_VERSION "[Wirefly 08-2017]"
// comment out below before compiling production codez!
#define SERIAL_DEBUG 1

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// = = = = = = = = = = = = = = = = = = = = = = = = = = =
// RF12 configuration setup code
#define RF12_BUFFER_SIZE	66
static uint8_t wirefly_msg_data[RF12_BUFFER_SIZE];
static byte wirefly_msg_stack[RF12_MAXDATA+4], wirefly_msg_top, wirefly_msg_sendLen, wirefly_msg_dest;
// cmd may be set to: [0, 'a', 'c']
// 0   no command
// 'a' send request ack
// 'c' send
static char wirefly_msg_cmd;

#define COLLECT 0x20 // collect mode, i.e. pass incoming without sending acks

// http://jeelabs.net/pub/docs/jeelib/classSleepy.html
// WDT inturrupt handler, required to use Sleepy::loseSomeTime()
//ISR(WDT_vect) { Sleepy::watchdogEvent(); }

// Select at features:
//#define LUXMETER 10
//#define RTCTIMER 11
//#define LED_MONO 100
#define LED_RGB 101
//#define LED_ADDR 102
//#define RGB_STRIP 200
//#define RGB_NEOPIX 201
#define LED_SUPERFLUX 202

// For PATTERN_FADER and rgbSet(): 
// Used to adjust the limits for the LED, especially if it has a lower ON threshold
#define  MIN_RGB_VALUE  10   // no brighter than 10. TODO add gamma correction 
#define  MAX_RGB_VALUE  255  // no darker than 255. (darkest)

#ifdef LED_MONO
  #define LEDPIN 5
#endif
#ifdef LED_RGB
  #define REDPIN 5        // DIO2
  #define GREENPIN 6      // DIO3
  #define BLUEPIN 3   // IRQ2
  //#define BLUEPIN 9   // SEL1 LedNode
#endif

#ifdef LUXMETER
PortI2C myBus (3);
LuxPlug sensor (myBus, 0x39);
#endif

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// = = = = = = = = = = = = = = = = = = = = = = = = = = =
// Network messages
#define WIREFLY_SEND_PATTERN    2
#define WIREFLY_SEND_CLOCKSYNC  10

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// = = = = = = = = = = = = = = = = = = = = = = = = = = =
// Pattern control, pattern variables

#define PATTERN_OFF             0
#define PATTERN_TWINKLE         1
#define PATTERN_FIREFLY         2
#define PATTERN_FADER           3
#define PATTERN_PULSER          4
#define PATTERN_RGBTEST         5
#define PATTERN_CLOCKSYNC      10
#define PATTERN_LUXMETER       90

#define PULSE_COLORSPEED 5     // For PATTERN_PULSE, make this higher to slow down

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// = = = = = = = = = = = = = = = = = = = = = = = = = = =
int wirefly_send();
int wirefly_interrupt();
boolean wirefly_delay(unsigned long wait_time);
void pattern_run();
void pattern_off();
void pattern_set(int value);
int pattern_get();

#endif

