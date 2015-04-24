#include <TimerOne.h>
#include "LPD6803.h"

#include <Ports.h>
#include <RF12.h>
#include <util/crc16.h>
#include <util/parity.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>

// comment out below before compiling production codez!
#define DEBUG 1

#define UPSIDE_DOWN_LEDS 1

// Configure some values in EEPROM for easy config of the RF12 later on.
// 2009-05-06 <jcw@equi4.com> http://opensource.org/licenses/mit-license.php
// $Id: RF12demo.pde 7754 2011-08-22 11:38:59Z jcw $

// this version adds flash memory support, 2009-11-19

#define DATAFLASH   1   // check for presence of DataFlash memory on JeeLink
#define FLASH_MBIT  16  // support for various dataflash sizes: 4/8/16 Mbit

//#define LED_PIN     9   // activity LED, comment out to disable

#define COLLECT 0x20 // collect mode, i.e. pass incoming without sending acks

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// RF12 configuration setup code

typedef struct {
    byte nodeId;
    byte group;
    char msg[RF12_EEPROM_SIZE-4];
    word crc;
} RF12Config;

static RF12Config config;

static char cmd;
static byte value, stack[RF12_MAXDATA], top, sendLen, dest, quiet;
static byte databuffer[RF12_MAXDATA], testCounter;

static void addCh (char* msg, char c) {
    byte n = strlen(msg);
    msg[n] = c;
}

static void addInt (char* msg, word v) {
    if (v >= 10)
        addInt(msg, v / 10);
    addCh(msg, '0' + v % 10);
}

static void saveConfig () {
    // set up a nice config string to be shown on startup
    memset(config.msg, 0, sizeof config.msg);
    strcpy(config.msg, " ");
    
    byte id = config.nodeId & 0x1F;
    addCh(config.msg, '@' + id);
    strcat(config.msg, " i");
    addInt(config.msg, id);
    if (config.nodeId & COLLECT)
        addCh(config.msg, '*');
    
    strcat(config.msg, " g");
    addInt(config.msg, config.group);
    
    strcat(config.msg, " @ ");
    static word bands[4] = { 315, 433, 868, 915 };
    word band = config.nodeId >> 6;
    addInt(config.msg, bands[band]);
    strcat(config.msg, " MHz ");
    
    config.crc = ~0;
    for (byte i = 0; i < sizeof config - 2; ++i)
        config.crc = _crc16_update(config.crc, ((byte*) &config)[i]);

    // save to EEPROM
    for (byte i = 0; i < sizeof config; ++i) {
        byte b = ((byte*) &config)[i];
        eeprom_write_byte(RF12_EEPROM_ADDR + i, b);
    }
    
    if (!rf12_config())
        Serial.println("config save failed");
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// DataFlash code

#if DATAFLASH

#define DF_ENABLE_PIN   8           // PB0

#if FLASH_MBIT == 4
// settings for 0.5 Mbyte flash in JLv2
#define DF_BLOCK_SIZE   16          // number of pages erased at same time
#define DF_LOG_BEGIN    32          // first 2 blocks reserved for future use
#define DF_LOG_LIMIT    0x0700      // last 64k is not used for logging
#define DF_MEM_TOTAL    0x0800      // 2048 pages, i.e. 0.5 Mbyte
#define DF_DEVICE_ID    0x1F44      // see AT25DF041A datasheet
#define DF_PAGE_ERASE   0x20        // erase one block of flash memory
#endif

#if FLASH_MBIT == 8
// settings for 1 Mbyte flash in JLv2
#define DF_BLOCK_SIZE   16          // number of pages erased at same time
#define DF_LOG_BEGIN    32          // first 2 blocks reserved for future use
#define DF_LOG_LIMIT    0x0F00      // last 64k is not used for logging
#define DF_MEM_TOTAL    0x1000      // 4096 pages, i.e. 1 Mbyte
#define DF_DEVICE_ID    0x1F45      // see AT26DF081A datasheet
#define DF_PAGE_ERASE   0x20        // erase one block of flash memory
#endif

#if FLASH_MBIT == 16
// settings for 2 Mbyte flash in JLv3
#define DF_BLOCK_SIZE   256         // number of pages erased at same time
#define DF_LOG_BEGIN    512         // first 2 blocks reserved for future use
#define DF_LOG_LIMIT    0x1F00      // last 64k is not used for logging
#define DF_MEM_TOTAL    0x2000      // 8192 pages, i.e. 2 Mbyte
#define DF_DEVICE_ID    0x2020      // see M25P16 datasheet
#define DF_PAGE_ERASE   0xD8        // erase one block of flash memory
#endif

// structure of each page in the log buffer, size must be exactly 256 bytes
typedef struct {
    byte data [248];
    word seqnum;
    long timestamp;
    word crc;
} FlashPage;

// structure of consecutive entries in the data area of each FlashPage
typedef struct {
    byte length;
    byte offset;
    byte header;
    byte data[RF12_MAXDATA];
} FlashEntry;

static FlashPage dfBuf;     // for data not yet written to flash
static word dfLastPage;     // page number last written
static byte dfFill;         // next byte available in buffer to store entries

static byte df_present () {
    return dfLastPage != 0;
}

static void df_enable () {
    // digitalWrite(ENABLE_PIN, 0);
    bitClear(PORTB, 0);
}

static void df_disable () {
    // digitalWrite(ENABLE_PIN, 1);
    bitSet(PORTB, 0);
}

static byte df_xfer (byte cmd) {
    SPDR = cmd;
    while (!bitRead(SPSR, SPIF))
        ;
    return SPDR;
}

void df_command (byte cmd) {
    for (;;) {
        cli();
        df_enable();
        df_xfer(0x05); // Read Status Register
        byte status = df_xfer(0);
        df_disable();
        sei();
        // don't wait for ready bit if there is clearly no dataflash connected
        if (status == 0xFF || (status & 1) == 0)
            break;
    }    

    cli();
    df_enable();
    df_xfer(cmd);
}

static void df_deselect () {
    df_disable();
    sei();
}

static void df_writeCmd (byte cmd) {
    df_command(0x06); // Write Enable
    df_deselect();
    df_command(cmd);
}

void df_read (word block, word off, void* buf, word len) {
    df_command(0x03); // Read Array (Low Frequency)
    df_xfer(block >> 8);
    df_xfer(block);
    df_xfer(off);
    for (word i = 0; i < len; ++i)
        ((byte*) buf)[(byte) i] = df_xfer(0);
    df_deselect();
}

void df_write (word block, const void* buf) {
    df_writeCmd(0x02); // Byte/Page Program
    df_xfer(block >> 8);
    df_xfer(block);
    df_xfer(0);
    for (word i = 0; i < 256; ++i)
        df_xfer(((const byte*) buf)[(byte) i]);
    df_deselect();
}

// wait for current command to complete
void df_flush () {
    df_read(0, 0, 0, 0);
}

static void df_wipe () {
    Serial.println("DF W");
    
    df_writeCmd(0xC7); // Chip Erase
    df_deselect();
    df_flush();
}

static void df_erase (word block) {
    Serial.print("DF E ");
    Serial.println(block);
    
    df_writeCmd(DF_PAGE_ERASE); // Block Erase
    df_xfer(block >> 8);
    df_xfer(block);
    df_xfer(0);
    df_deselect();
    df_flush();
}

static word df_wrap (word page) {
    return page < DF_LOG_LIMIT ? page : DF_LOG_BEGIN;
}

static void df_saveBuf () {
    if (dfFill == 0)
        return;

    dfLastPage = df_wrap(dfLastPage + 1);
    if (dfLastPage == DF_LOG_BEGIN)
        ++dfBuf.seqnum; // bump to next seqnum when wrapping
    
    // set remainder of buffer data to 0xFF and calculate crc over entire buffer
    dfBuf.crc = ~0;
    for (byte i = 0; i < sizeof dfBuf - 2; ++i) {
        if (dfFill <= i && i < sizeof dfBuf.data)
            dfBuf.data[i] = 0xFF;
        dfBuf.crc = _crc16_update(dfBuf.crc, dfBuf.data[i]);
    }
    
    df_write(dfLastPage, &dfBuf);
    dfFill = 0;
    
    // wait for write to finish before reporting page, seqnum, and time stamp
    df_flush();
    Serial.print("DF S ");
    Serial.print(dfLastPage);
    Serial.print(' ');
    Serial.print(dfBuf.seqnum);
    Serial.print(' ');
    Serial.println(dfBuf.timestamp);
    
    // erase next block if we just saved data into a fresh block
    // at this point in time dfBuf is empty, so a lengthy erase cycle is ok
    if (dfLastPage % DF_BLOCK_SIZE == 0)
        df_erase(df_wrap(dfLastPage + DF_BLOCK_SIZE));
}

static void df_append (const void* buf, byte len) {
    //FIXME the current logic can't append incoming packets during a save!

    // fill in page time stamp when appending to a fresh page
    if (dfFill == 0)
        dfBuf.timestamp = now();
    
    long offset = now() - dfBuf.timestamp;
    if (offset >= 255 || dfFill + 1 + len > sizeof dfBuf.data) {
        df_saveBuf();

        dfBuf.timestamp = now();
        offset = 0;
    }

    // append new entry to flash buffer
    dfBuf.data[dfFill++] = offset;
    memcpy(dfBuf.data + dfFill, buf, len);
    dfFill += len;
}

// go through entire log buffer to figure out which page was last saved
static void scanForLastSave () {
    dfBuf.seqnum = 0;
    dfLastPage = DF_LOG_LIMIT - 1;
    // look for last page before an empty page
    for (word page = DF_LOG_BEGIN; page < DF_LOG_LIMIT; ++page) {
        word currseq;
        df_read(page, sizeof dfBuf.data, &currseq, sizeof currseq);
        if (currseq != 0xFFFF) {
            dfLastPage = page;
            dfBuf.seqnum = currseq + 1;
        } else if (dfLastPage == page - 1)
            break; // careful with empty-filled-empty case, i.e. after wrap
    }
}

static void df_initialize () {
    // assumes SPI has already been initialized for the RFM12B
    df_disable();
    pinMode(DF_ENABLE_PIN, OUTPUT);
    df_command(0x9F); // Read Manufacturer and Device ID
    word info = df_xfer(0) << 8;
    info |= df_xfer(0);
    df_deselect();

    if (info == DF_DEVICE_ID) {
        df_writeCmd(0x01);  // Write Status Register ...
        df_xfer(0);         // ... Global Unprotect
        df_deselect();

        scanForLastSave();
        
        Serial.print("DF I ");
        Serial.print(dfLastPage);
        Serial.print(' ');
        Serial.println(dfBuf.seqnum);
    
        // df_wipe();
        df_saveBuf(); //XXX
    }
}

static void discardInput () {
    while (Serial.read() >= 0)
        ;
}

static void df_dump () {
  struct { 
    word seqnum; 
    long timestamp; 
    word crc; 
  } 
  curr;
  discardInput();
  for (word page = DF_LOG_BEGIN; page < DF_LOG_LIMIT; ++page) {
    if (Serial.read() >= 0)
      break;
    // read marker from page in flash
    df_read(page, sizeof dfBuf.data, &curr, sizeof curr);
    if (curr.seqnum == 0xFFFF)
      continue; // page never written to
    Serial.print(" df# ");
    Serial.print(page);
    Serial.print(" : ");
    Serial.print(curr.seqnum);
    Serial.print(' ');
    Serial.print(curr.timestamp);
    Serial.print(' ');
    Serial.println(curr.crc);
  }
}

static word scanForMarker (word seqnum, long asof) {
  word lastPage = 0;
  struct { 
    word seqnum; 
    long timestamp; 
  } 
  last, curr;
  last.seqnum = 0xFFFF;
  // go through all the pages in log area of flash
  for (word page = DF_LOG_BEGIN; page < DF_LOG_LIMIT; ++page) {
    // read seqnum and timestamp from page in flash
    df_read(page, sizeof dfBuf.data, &curr, sizeof curr);
    if (curr.seqnum == 0xFFFF)
      continue; // page never written to
    if (curr.seqnum >= seqnum && curr.seqnum < last.seqnum) {
      last = curr;
      lastPage = page;
    }
    if (curr.seqnum == last.seqnum && curr.timestamp <= asof)
      lastPage = page;
  }
  return lastPage;
}

static void df_replay (word seqnum, long asof) {
    word page = scanForMarker(seqnum, asof);
    Serial.print("r: page ");
    Serial.print(page);
    Serial.print(' ');
    Serial.println(dfLastPage);
    discardInput();
    word savedSeqnum = dfBuf.seqnum;
    while (page != dfLastPage) {
        if (Serial.read() >= 0)
            break;
        page = df_wrap(page + 1);
        df_read(page, 0, &dfBuf, sizeof dfBuf); // overwrites ram buffer!
        if (dfBuf.seqnum == 0xFFFF)
            continue; // page never written to
        // skip and report bad pages
        word crc = ~0;
        for (word i = 0; i < sizeof dfBuf; ++i)
            crc = _crc16_update(crc, dfBuf.data[i]);
        if (crc != 0) {
            Serial.print("DF C? ");
            Serial.print(page);
            Serial.print(' ');
            Serial.println(crc);
            continue;
        }
        // report each entry as "R seqnum time <data...>"
        byte i = 0;
        while (i < sizeof dfBuf.data && dfBuf.data[i] < 255) {
            if (Serial.available())
                break;
            Serial.print("R ");
            Serial.print(dfBuf.seqnum);
            Serial.print(' ');
            Serial.print(dfBuf.timestamp + dfBuf.data[i++]);
            Serial.print(' ');
            Serial.print((int) dfBuf.data[i++]);
            byte n = dfBuf.data[i++];
            while (n-- > 0) {
                Serial.print(' ');
                Serial.print((int) dfBuf.data[i++]);
            }
            Serial.println();
        }
        // at end of each page, report a "DF R" marker, to allow re-starting
        Serial.print("DF R ");
        Serial.print(page);
        Serial.print(' ');
        Serial.print(dfBuf.seqnum);
        Serial.print(' ');
        Serial.println(dfBuf.timestamp);
    }
    dfFill = 0; // ram buffer is no longer valid
    dfBuf.seqnum = savedSeqnum + 1; // so next replay will start at a new value
    Serial.print("DF E ");
    Serial.print(dfLastPage);
    Serial.print(' ');
    Serial.print(dfBuf.seqnum);
    Serial.print(' ');
    Serial.println(millis());
}

#else // DATAFLASH

#define df_present() 0
#define df_initialize()
#define df_dump()
#define df_replay(x,y)
#define df_erase(x)

#endif

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

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

char helpText1[] PROGMEM = 
    "\n"
    "Available commands:" "\n"
    "  <nn> i     - set node ID (standard node ids are 1..26)" "\n"
    "               (or enter an uppercase 'A'..'Z' to set id)" "\n"
    "  <n> b      - set MHz band (4 = 433, 8 = 868, 9 = 915)" "\n"
    "  <nnn> g    - set network group (RFM12 only allows 212, 0 = any)" "\n"
    "  <n> c      - set collect mode (advanced, normally 0)" "\n"
    "  t          - broadcast max-size test packet, with ack" "\n"
    "  ...,<nn> a - send data packet to node <nn>, with ack" "\n"
    "  ...,<nn> s - send data packet to node <nn>, no ack" "\n"
    "  <n> l      - turn activity LED on PB1 on or off" "\n"
    "  <n> q      - set quiet mode (1 = don't report bad packets)" "\n"
;

static void showString (PGM_P s) {
    for (;;) {
        char c = pgm_read_byte(s++);
        if (c == 0)
            break;
        if (c == '\n')
            Serial.print('\r');
        Serial.print(c);
    }
}

static void showHelp () {
    showString(helpText1);
    Serial.println("Current configuration:");
    rf12_config();
}

static void handleSerialInput (char c) {
    if ('0' <= c && c <= '9')
        value = 10 * value + c - '0';
    else if (c == ',') {
        if (top < sizeof stack)
            stack[top++] = value;
        value = 0;
  }
  else if ('a' <= c && c <='z') {
        Serial.print("> ");
        Serial.print((int) value);
        Serial.println(c);
        switch (c) {
            case 'i': // set node id
                config.nodeId = (config.nodeId & 0xE0) + (value & 0x1F);
                saveConfig();
                break;
            case 'b': // set band: 4 = 433, 8 = 868, 9 = 915
                value = value == 8 ? RF12_868MHZ :
                        value == 9 ? RF12_915MHZ : RF12_433MHZ;
                config.nodeId = (value << 6) + (config.nodeId & 0x3F);
                saveConfig();
                break;
            case 'g': // set network group
                config.group = value;
                saveConfig();
                break;
            case 'c': // set collect mode (off = 0, on = 1)
                if (value)
                    config.nodeId |= COLLECT;
                else
                    config.nodeId &= ~COLLECT;
                saveConfig();
                break;
            case 't': // broadcast a maximum size test packet, request an ack
                cmd = 'a';
                sendLen = RF12_MAXDATA;
                dest = 0;
                for (byte i = 0; i < RF12_MAXDATA; ++i)
                    databuffer[i] = i + testCounter;
                Serial.print("test ");
                Serial.println((int) testCounter); // first byte in test buffer
                ++testCounter;
                break;
            case 'a': // send packet to node ID N, request an ack
            case 's': // send packet to node ID N, no ack
                cmd = c;
                sendLen = top;
                dest = value;
                memcpy(databuffer, stack, top);
                break;
            case 'l': // turn activity LED on or off
                activityLed(value);
                break;
            case 'q': // turn quiet mode on or off (don't report bad packets)
                quiet = value;
                break;
            case 'f': // send FS20 command: <hchi>,<hclo>,<addr>,<cmd>f
            case 'k': // send KAKU command: <addr>,<dev>,<on>k
            case 'd': // dump all log markers
            case 'r': // replay from specified seqnum/time marker
            case 'e': // erase specified 4Kb block
            case 'w': // wipe entire flash memory
            case 'z': // broadcast RGB LED Strip pattern
            case 'h':
            case 'j':
            case 'm':
            case 'n':
            case 'o':
            case 'p':
            case 'u':
            case 'v':
            case 'x':
            case 'y':
            default:
                showHelp();
                break;
        }
        value = top = 0;
        memset(stack, 0, sizeof stack);
  } 
  else if ('A' <= c && c <= 'Z') {
        config.nodeId = (config.nodeId & 0xE0) + (c & 0x1F);
        saveConfig();
  } 
  else if (c > ' ')
        showHelp();
}

/******************************************************************************  
 * debounce.c  
 * written by Kenneth A. Kuhn  
 * version 1.00  
 * 
 * This is an algorithm that debounces or removes random or spurious  
 * transistions of a digital signal read as an input by a computer.  This is  
 * particularly applicable when the input is from a mechanical contact.  An  
 * integrator is used to perform a time hysterisis so that the signal must  
 * persistantly be in a logical state (0 or 1) in order for the output to change  
 * to that state.  Random transitions of the input will not affect the output  
 * except in the rare case where statistical clustering is longer than the  
 * specified integration time.
 * 
 * The following example illustrates how this algorithm works.  The sequence  
 * labeled, real signal, represents the real intended signal with no noise.  The  
 * sequence labeled, corrupted, has significant random transitions added to the  
 * real signal.  The sequence labled, integrator, represents the algorithm  
 * integrator which is constrained to be between 0 and 3.  The sequence labeled,  
 * output, only makes a transition when the integrator reaches either 0 or 3.  
 * Note that the output signal lags the input signal by the integration time but  
 * is free of spurious transitions.  
 * 
 * real signal 0000111111110000000111111100000000011111111110000000000111111100000  
 * corrupted   0100111011011001000011011010001001011100101111000100010111011100010  
 * integrator  0100123233233212100012123232101001012321212333210100010123233321010  
 * output      0000001111111111100000001111100000000111111111110000000001111111000  
 * I have been using this algorithm for years and I show it here as a code  
 * fragment in C.  The algorithm has been around for many years but does not seem  
 * to be widely known.  Once in a rare while it is published in a tech note.  It  
 * is notable that the algorithm uses integration as opposed to edge logic  
 * (differentiation).  It is the integration that makes this algorithm so robust  
 * in the presence of noise.  
 * 
 ******************************************************************************/
/* The following parameters tune the algorithm to fit the particular  
 application.  SAMPLE_FREQUENCY indicates how many times per second a computer
 samples a  mechanical contact and DEBOUNCE_TIME indicates the integration time  used to remove bounce.
 
 Note: DEBOUNCE_TIME is in seconds and SAMPLE_FREQUENCY is in Hertz */

//#define DEBOUNCE_TIME       0.3  
//#define SAMPLE_FREQUENCY    5
//#define MAXIMUM             (DEBOUNCE_TIME * SAMPLE_FREQUENCY)
#define MAXIMUM         4
#define MAXIMUM_C       2

//unsigned int input;       /* 0 or 1 depending on the input signal */ 
static unsigned int integrator[3];  /* Will range from 0 to the specified MAXIMUM */

static unsigned int output[3];      /* Cleaned-up version of the input signal */

static unsigned int maximum[3] = {
  4,4,2}; 

unsigned int debounce(uint8_t input, char button) {

  // determine which button, a, b or c.
  if( button < 97 || button > 99 )
    return 0;
  int i = button-97; // 0, 1, 2

  /* Step 1: Update the integrator based on the input signal.  Note that the  
   integrator follows the input, decreasing or increasing towards the limits as  
   determined by the input state (0 or 1). */

  if (!input) 
  {  
    if (integrator[i] > 0)  
      integrator[i]--;  
  }  
  else if (integrator[i] < maximum[i])  
    integrator[i]++;


  /* Step 2: Update the output state based on the integrator.  Note that the  
   output will only change states if the integrator has reached a limit, either  
   0 or MAXIMUM. */

  if (integrator[i] == 0) {
    output[i] = 0;
    return 1;
  } 
  else if (integrator[i] >= maximum[i])  {  
    output[i] = 1;  
    integrator[i] = maximum[i];  /* defensive code if integrator got corrupted */

    return 1;
  }
  return 0;
}

// [modified] Example to control LPD6803-based RGB LED Modules in a strand
// Original code by Bliptronics.com Ben Moyes 2009
//Use this as you wish, but please give credit, or at least buy some of my LEDs!

// Code cleaned up and Object-ified by ladyada, should be a bit easier to use

/*****************************************************************************/

// Choose which 2 pins you will use for output.
// Can be any valid output pins.
static int dataPin = 5;       // 'yellow' wire
static int clockPin = 6;      // 'green' wire
// Don't forget to connect 'blue' to ground and 'red' to +5V

static int buttonA = 4; // digital 4 or DIO1
static int buttonB = 7; // digital 7 or DIO4
static int switchT = 14; // analog 0 or AI01

// Timer 1 is also used by the strip to send pixel clocks

#define PATTERN_OFF		0
#define PATTERN_TWINKLE		1
#define PATTERN_FIREFLY         2
#define PATTERN_CLOCKSYNC	3
#define PATTERN_LINEARWIPE	4
#define PATTERN_VERTICALWIPE	5
#define PATTERN_RADIALWIPE	6
#define PATTERN_RADARSWEEP	7
#define PATTERN_RAINBOW		8
#define PATTERN_RAINBOWCYCLE	9
#define PATTERN_COMBJELLIES	10
#define PATTERN_FIREWORKS	11
#define PATTERN_STAINS		12
#define PATTERN_PAINT		13
#define PATTERN_ADHOC		14
#define PATTERN_MODESELECT	15
#define PATTERN_IDENTIFICATION  16
#define PATTERN_LAST	12 // last available in menu selection; do not pass lantern

// define the pixel(s) that correspond to the deep sea diver's lantern
#define PIXEL_LANTERN  13
#define PIXEL_LANTERN_BACK 22
#define PIXEL_FRONT_LAST	17

// the lantern should be amber in color
#define COLOR_LANTERN Color(31,16,0)
#define COLOR_JELLY   Color(31,31,0)

#define HANDLEINPUTS_TIME  22 //microseconds
#define AUTONOMOUS_HANDLEINPUTS_TIME  30 //microseconds

static uint8_t pattern = 0;
static int autonomous = 0;
static int inputA = 0;
static int inputB = 0;
static int inputC = 0;
static int stopChooseAnother = 0;
static unsigned long wait = 50;
static int patternAvailable = 0;

#define PIX_COUNT		30
#define RF12_BUFFER_SIZE	66

static uint8_t my_data[RF12_BUFFER_SIZE];

static LPD6803 strip = LPD6803(PIX_COUNT, dataPin, clockPin);

#define FLAG_IGNORE_LANTERN 1
#define FLAG_IGNORE_FRONT   2
#define FLAG_IGNORE_BACK    4

#define LANTERN(X) (X == PIXEL_LANTERN || X == PIXEL_LANTERN_BACK)
#define IGNORE_LANTERN(X) ((X & FLAG_IGNORE_LANTERN) == FLAG_IGNORE_LANTERN)
#define IGNORE_FRONT(X) ((X & FLAG_IGNORE_FRONT) == FLAG_IGNORE_FRONT)
#define IGNORE_BACK(X) ((X & FLAG_IGNORE_BACK) == FLAG_IGNORE_BACK)

#define FRONT_PIXEL(X) (X <= PIXEL_FRONT_LAST)

// No color value, clear LEDs
void off(byte opts = 0) {
  int startIndex = (IGNORE_FRONT(opts)?PIXEL_FRONT_LAST+1:0);
  int endIndex = (IGNORE_BACK(opts)?PIXEL_FRONT_LAST+1:strip.numPixels());
  for(uint8_t p=startIndex; p<endIndex; p++) {
    debounceInputs();
    strip.setPixelColor(p,0);
  }
  strip.show();
  delay(wait);
  debounceInputs();
}

void FireFly(byte opts = 0){
  //initialize
  long FadeStatus; // one bit flag for each LED 1 = fade in, 0 = fade out
  long EnabledStatus; // one bit flag for each LED 1 = enabled, 0 = disabled
  uint8_t c = 0;
  for (uint8_t p=0; p < strip.numPixels(); p++) {
    debounceInputs();
    bitWrite(FadeStatus,p, random(0,2)); //randomly set the fade in/out status for each LED
    if (random(0,2)){//randomly set the enabled status
      bitWrite(EnabledStatus,p,1);
      c = random(0,32); // randomly set each enabled pixel to a random yellow-green brightness
      strip.setPixelColor(p,Color(c,c,0));
    }
  }

  int i = 64; // adjusts probability that an LED will come back on...
  while(1){
    for (uint8_t p=0; p < strip.numPixels(); p++) {
      debounceInputs();
      if (bitRead(EnabledStatus,p)){ //only do things if the LED is enabled

        //get pixel's red or green value
        //maybe use pointer to LED's 2byte color
        //and use a mask and maybe a << or >>
        // using extern static uint16_t *pixlels from ladyada's code

        c = (uint8_t)((strip.pixelColorData(p) >> 5) & 0x001F);
        if (bitRead(FadeStatus,p)) { //fade in
          if (c == 31){ //it's all the way on
            c = 30;
            bitWrite(FadeStatus,p, 0);
          } 
          else {
            c++; //make it fade in
          }
        } 
        else { //fade out
          if (c == 0){ //all the way out
            c = random(0,2);
            bitWrite(FadeStatus,p,1);
            bitWrite(EnabledStatus,p,c); //does this LED get turned off?
          } 
          else{
            c--; //make it fade out (maybe fade out faster than fade in? closer to life like)
          }
        }
        strip.setPixelColor(p,Color(c,c,0));
      } 
      else{ //pixel not enabled
        if (random(0,i)==1){ //randomly turn disabled LEDs on, but only do it once every cycle. adjust random max range to change probablilities
          bitWrite(EnabledStatus,p,1);
          if (bitRead(EnabledStatus,p)){
            bitWrite(FadeStatus,p,1);
          }
        }
      }
    }//end of for each LED loop
    strip.show();
    if( handleInputs() )
      return;
    debounceInputs();
    delay(wait);
    debounceInputs();
  }//end of while

}


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

void RadarSweep( uint16_t c = COLOR_JELLY, byte opts = FLAG_IGNORE_LANTERN|FLAG_IGNORE_FRONT){
  uint8_t pix[36] = 
  {
    0,1,0,1, 12,2,2,3, 3,12,5,4, 5,4,6,7, 7,6,8,8, 9,9,13,11, 10,10,11,14, 14,15,15,13, 16,16,17,17
  };
  uint16_t col[36] = 
  {
    c,c,0,0, c,c,0,c, 0,0,c,c, 0,0,c,c, 0,0,c,0, c,0,c,c, c,0,0,c, 0,c,0,0, c,0,c,0
  };
  uint8_t tim[36] = 
  {
    51,43,61,38, 70,67,37,67, 34,42,31,64, 42,48,32,61, 46,70,61,295, 41,59,64,47, 39,36,43,52, 45,32,52,46, 45,42,53,120
  };
  off(opts);
  while( 1 ) {
    off(opts);    
    for(int i=0; i<36; i++) {
      debounceInputs();
      strip.setPixelColor(pix[i],col[i]);
      strip.show();
      if( handleInputs() )
        return;
      debounceInputs();
      delay(wait+tim[i]);
      debounceInputs();
    }
  }
}

void RadialWipe( uint16_t c = COLOR_JELLY, byte opts = FLAG_IGNORE_LANTERN|FLAG_IGNORE_FRONT){
  uint8_t pix[36] = 
  {
    0,1,12,2,3,5,4,7,6,8,9,11,10,13,14,15,16,17
  };
  uint8_t tim[18] = 
  {
    110,51,82,70,74,72,42,95,32,117,326,41,93,47,39,36,104,61
  };
  off(opts);
  while( 1 ) {
    for( uint16_t offon = 0; offon<=c; offon+=c ) {
      for(uint8_t i=0; i<=PIXEL_FRONT_LAST; i++) {
        debounceInputs();
        strip.setPixelColor(pix[i], offon);
        strip.show();
        if( handleInputs() )
          return;
        delay(wait+tim[i]);
        debounceInputs();
      }
    }
  }
}

void VerticalWipe(uint16_t c = COLOR_JELLY, byte opts = FLAG_IGNORE_LANTERN|FLAG_IGNORE_FRONT){
  uint8_t pix[18] = 
  {
    9,8,10,7,6,11,4,5,14,3,12,13,2,16,15,1,17,0
  };
  uint8_t tim[18] = 
  {
    85,187,37,128,71,80,102,44,103,70,58,91,92,59,109,120,75,30
  };
  off(opts);
  while( 1 ) {
    for( uint16_t offon = 0; offon<=c; offon+=c ) {
      for(uint8_t i=0; i<=PIXEL_FRONT_LAST; i++) {
        debounceInputs();
        strip.setPixelColor(pix[i],offon);
        strip.show();
        if( handleInputs() )
          return;
        delay(wait+tim[i]);
        debounceInputs();
      }
    }
  }
}
void combJellies(byte opts = 0) {
  randomSeed(analogRead(0));

  /// assuming numPixels() is constrained by RF12_BUFFER_SIZE !!
  int pp = strip.numPixels();
  char sequence[30] = {
    0,22,10,29,13,23,4,18,1,28,8,17,19,12,25,15,24,2,11,7,21,16,6,26,9,3,27,14,20,5       };
  uint8_t *sequence_data = my_data+1;
  memcpy(sequence_data,sequence,pp);

  off(opts);

  while( 1 ) {
    //int color_size = 33;
    for (uint16_t c=0; c<96; c++) {     // cycle all 96 colors in the wheel
      for (uint8_t i=0; i<pp; i++) {
        debounceInputs();
        // have pixels independently cycle through color wheel
        uint8_t p = sequence_data[i];
        // tricky math! use each pixel as a fraction of the full 96-color wheel
        // (thats the i / strip.numPixels() part)
        // Then add in j which makes the colors go around per pixel
        // the % 96 is to make the wheel cycle around
        strip.setPixelColor(p, Wheel( ((i * 96 / strip.numPixels()) + c) % 96) );
      }
      strip.show();   // write all the pixels out
      debounceInputs();
      delay(wait);
      debounceInputs();
      if( handleInputs() )
        return;
    }
  }
}

void rainbow(byte opts = 0) {

  while( 1 ) {
    for(uint16_t c=0; c < 96; c++) {     // cycle all 96 colors in the wheel
      for (uint8_t p=0; p < strip.numPixels(); p++) {
        debounceInputs();
        strip.setPixelColor(p, Wheel( (p + c) % 96));
      }
      strip.show();   // write all the pixels out
      debounceInputs();
      delay(wait);
      debounceInputs();
      if( handleInputs() )
        return;
    }
  }
}

// Slightly different, this one makes the rainbow wheel equally distributed 
// along the chain
void rainbowCycle(byte opts = 0) {

  while( 1 ) {
    for (uint16_t c=0; c < 96; c++) {     // cycle all 96 colors in the wheel
      for (uint8_t p=0; p < strip.numPixels(); p++) {
        debounceInputs();
        // tricky math! we use each pixel as a fraction of the full 96-color wheel
        // (thats the i / strip.numPixels() part)
        // Then add in j which makes the colors go around per pixel
        // the % 96 is to make the wheel cycle around
        strip.setPixelColor(p, Wheel( ((p * 96 / strip.numPixels()) + c) % 96) );
      }  
      strip.show();   // write all the pixels out
      debounceInputs();
      delay(wait);
      debounceInputs();
      if( handleInputs() )
        return;
    }
  }
}


// fill the dots all at same time with said color
void colorDoubleBuffer16(uint16_t c, byte opts = 0) {
  for( uint8_t p=0; p < strip.numPixels(); p++) {
    debounceInputs();
    strip.setPixelColor(p, c);
  }
  strip.show();
  debounceInputs();
  delay(wait);
  debounceInputs();
}

// only turn on the specified LED
void modeSelect(uint8_t modeValue) {
  if( modeValue < 1 )
    modeValue = 1;
  if( modeValue > PATTERN_LAST )
    modeValue = PATTERN_LAST;
  strip.setPixelColor(modeValue-1, COLOR_JELLY);
  strip.show();
  debounceInputs();
  delay(wait);
  debounceInputs();
}

void identify( uint16_t c = COLOR_JELLY ) {
  uint8_t nodeid = (config.nodeId & 0x1F);
  for( uint8_t p=0; p < strip.numPixels(); p++) {
    debounceInputs();
    if( p < nodeid )
      strip.setPixelColor(p,c);
    else
      strip.setPixelColor(p,0);
  }
  strip.show();
  debounceInputs();
  delay(wait);
  debounceInputs();
}

// fill the dots all at same time with said color
void colorDoubleBuffer8(uint8_t *data, byte opts = 0) {
  // here's the trick -- incoming buffer (data) is a contiguous memory block of 60 bytes.
  // we will use a pointer to two-byte data type called uint16_t to treat the buffer as
  // a contiguous block of 30 two-byte ints. This also causes the array notation [i] to increment
  // through the array correctly as a two-byte int. Use reinterpret_cast only when you are absolutely
  // sure its okay to do so, else bad buffer overflows and data corruption can occur.
  uint16_t *color_ptr = reinterpret_cast<uint16_t*>(data); // point to elements in buffer as a two-byte int
  colorDoubleBuffer16( *color_ptr, opts );
}

void morseCode(char *buffer, byte opts = 0) {
  //blink out morse code for given message on the lantern
  for(char *c = buffer; *c>='a' && *c<='z'; c++) {
    debounceInputs();
    switch(*c) {
    case 'o':
      // dah dah dah
      colorDoubleBuffer16(COLOR_LANTERN,opts);
      delay(wait+700);
      debounceInputs();
      off(opts);
      colorDoubleBuffer16(COLOR_LANTERN,opts);
      delay(wait+700);
      debounceInputs();
      off(opts);
      colorDoubleBuffer16(COLOR_LANTERN,opts);
      delay(wait+700);
      debounceInputs();
      break;
    case 's':
      // dit dit dit
      colorDoubleBuffer16(COLOR_LANTERN,opts);
      delay(wait+200);
      debounceInputs();
      off(opts);
      colorDoubleBuffer16(COLOR_LANTERN,opts);
      delay(wait+200);
      debounceInputs();
      off(opts);
      colorDoubleBuffer16(COLOR_LANTERN,opts);
      delay(wait+200);
      debounceInputs();
      break;
    }
    off(opts);
  }
}

void sos() {
  char buffer[4] = "sos";
  morseCode(buffer);
}



void adHoc(uint8_t *data, byte opts = 0) {
  //adHoc iterates through the received payload combining 2 bytes to make a "Color"
  //the sender must send each pixel's "Color" as 2 bytes, which get recombined in this function

  // here's the trick -- incoming buffer (data) is a contiguous memory block of 60 bytes.
  // we will use a pointer to two-byte data type called uint16_t to treat the buffer as
  // a contiguous block of 30 two-byte ints. This also causes the array notation [i] to increment
  // through the array correctly as a two-byte int. Use reinterpret_cast only when you are absolutely
  // sure its okay to do so, else bad buffer overflows and data corruption can occur.
  uint16_t *color_ptr = reinterpret_cast<uint16_t*>(data); // point to elements in buffer as a two-byte int

  for (uint8_t p=0; p < strip.numPixels(); p++) { //need to iterate through each pixel
    debounceInputs();
    strip.setPixelColor(p, color_ptr[p]); //set the appropriate pixel to our "Color"
  }
  strip.show();
  debounceInputs();
  delay(wait);
  debounceInputs();
}

// fill the dots one after the other with said color
// good for testing purposes
void colorWipe(uint16_t c, byte opts = 1) {
  while(1) {
    for( uint16_t offon = 0; offon<=c; offon+=c ) {
      for( uint8_t p=0; p < strip.numPixels(); p++) {
        debounceInputs();
        strip.setPixelColor(p, offon);
        strip.show();
        debounceInputs();
        delay(wait);
        debounceInputs();

        if( handleInputs() )
          return;
      }
    }
  }
}

void fireworks(byte opts = 0) {
  uint8_t randRed,randGreen,randBlue,lucky;
  off(true);
  randomSeed(analogRead(0));
  while( !handleInputs() ) {
    debounceInputs();
    lucky = random(0,strip.numPixels());
    if(LANTERN(lucky))
      continue;
    randRed = random(0,32);
    randGreen = random(0,32);
    randBlue = random(0,32);
    off(opts);
    strip.setPixelColor(lucky,Color(randRed,randGreen,randBlue));
    strip.show();
    debounceInputs();
    delay(wait);
    debounceInputs();
  }
}

static void stains(byte opts = 0) {
  uint8_t randRed,randGreen,randBlue,lucky;
  randomSeed(analogRead(0));
  off();
  while( !handleInputs() ) {
    debounceInputs();
    lucky = random(0,strip.numPixels());
    if( !IGNORE_LANTERN(opts) && LANTERN(lucky) )
      continue; 
    randRed = random(0,32);
    randGreen = random(0,32);
    randBlue = random(0,32);
    strip.setPixelColor(lucky,Color(randRed,randGreen,randBlue));
    strip.show();
    debounceInputs();
    delay(wait);
    debounceInputs();
  }
}

// randomly turn on at most maxPixels
static void twinkle(uint8_t maxOn, uint8_t maxSwap, byte opts = 0) {
  // four bytes (long)

  if( maxOn > 12 )
    maxOn = 12;

  if( maxSwap > maxOn )
    maxSwap = maxOn; 

  uint8_t randStart = (IGNORE_FRONT(opts)?PIXEL_FRONT_LAST+1:0);
  uint8_t randEnd = (IGNORE_BACK(opts)?PIXEL_FRONT_LAST+1:strip.numPixels());

  uint8_t rpix = 0;

  randomSeed(analogRead(0));
  memset(my_data, 0, sizeof(my_data));

  // clear all pixels
  off(opts);

  // setup initial on/off state
  strip.setPixelColor(PIXEL_LANTERN, COLOR_LANTERN);
  for( uint8_t i=0; i<maxOn; i++) {
    do {
      debounceInputs();
      rpix = (uint8_t)random(randStart,randEnd);
    } 
    while( !IGNORE_LANTERN(opts) && LANTERN(rpix) );
    strip.setPixelColor(rpix, Color(31,31,31));
    my_data[i] = rpix; // store 'on' pixel for later swap out
  }
  strip.show();
  debounceInputs();
  delay(wait);
  debounceInputs();

  // repeatedly swap out at most maxSwap pixels from being in the 'on' state
  while( !handleInputs() ) {
    for( uint8_t i=0; i<maxSwap; i++) {
      debounceInputs();

      // turn off a random pixel
      uint8_t temp = (uint8_t)random(0,maxOn);
      strip.setPixelColor(my_data[temp], 0);

      // turn on a random pixel
      do {
        debounceInputs();
        rpix = (uint8_t)random(randStart,randEnd);
      } 
      while( !IGNORE_LANTERN(opts) && LANTERN(rpix) );
      uint16_t color = (uint16_t)random(32000,32767);
      strip.setPixelColor(rpix, color);

      // assign new 'on' pixel
      my_data[temp] = rpix;
      strip.show();
      debounceInputs();
      delay(wait);
      debounceInputs();
    }
  }
}

/* Helper functions */

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

void runPattern(int patternToRun = 0) {
  activityLed(1);

  if( !autonomous )
    memcpy(my_data+1, const_cast<uint8_t*>(rf12_data+1), RF12_BUFFER_SIZE-1);

  int same = (pattern == patternToRun);
  pattern = patternToRun;

  switch( pattern ) {
  default:
  case PATTERN_OFF:
    off( 1 );    // all off including the Lantern
    break;
  case PATTERN_LINEARWIPE:
    colorWipe( COLOR_JELLY );  // wipe the given color from the first LED to the last
    break;
  case PATTERN_RADARSWEEP:
    RadarSweep( COLOR_JELLY ); // radar sweep in the given color
    break;
  case PATTERN_RADIALWIPE:
    RadialWipe( COLOR_JELLY );  //radial wipe the given color
    break;
  case PATTERN_VERTICALWIPE:
    VerticalWipe( COLOR_JELLY );  // vertically wipe the given color
    break;
  case PATTERN_COMBJELLIES:
    combJellies();
    break;
  case PATTERN_RAINBOW:
    rainbow();
    break;
  case PATTERN_RAINBOWCYCLE:
    rainbowCycle();
    break;
  case PATTERN_TWINKLE:
    twinkle( 15, 10 );   // twinkle LEDs
    break;
  case PATTERN_FIREWORKS:
    fireworks();
    break;
  case PATTERN_STAINS:
    stains();
    break;
  case PATTERN_ADHOC:
    if( !patternAvailable )
      adHoc( my_data+1 ); // pass starting address of 60-byte array color data
    break;
  case PATTERN_PAINT:
    if( !patternAvailable )
      colorDoubleBuffer8( my_data+1 ); // set all LEDs as the given color
    break;
  case PATTERN_CLOCKSYNC:
    clockSync( COLOR_JELLY );  // synchronizing firelies in the given color
    break;
  case PATTERN_MODESELECT:
    /*if( !same ) {
      off(1);*/
      modeSelect( my_data[0] ); // show the given pattern numeric code
    /*}*/
    break;
  case PATTERN_IDENTIFICATION:
    /*if( !same ) {
      off(1);*/
      identify( COLOR_JELLY ); // show the given pattern numeric code
    /*}*/
    break;
  case PATTERN_FIREFLY:
    FireFly(); // fade in, out
    break;
  }

  activityLed(0);
}

// don't waste cycles with delay(); poll for new inputs
int my_delay_with_break(unsigned long wait_time) {
  unsigned long t0 = millis();
  while( (millis() - t0) < wait_time )
    if( handleInputs() )
      return 1;
  return 0;
}
// don't waste cycles with delay(); poll for new inputs
void my_delay(unsigned long wait_time) {
  unsigned long t0 = millis();
  while( millis() - t0 < wait_time )
    handleInputs();
}

int my_recvDone() {
  // rf12_recvDone() needs to be constantly called in order to recieve new transmissions
  // but we only care if CRC is ok
  // if a new transmission came in and CRC is ok, don't poll recv state again or else
  // rf12_crc, rf12_len, and rf12_data will be reset
  // once data has been processed (in loop()) you should reset the patternAvailable flag

  if( !patternAvailable ) {

#ifdef DEBUG
    if( rf12_recvDone() ) {
      byte n = rf12_len;
      if (rf12_crc == 0) {
        Serial.print("OK");
      } 
      else {
        if (quiet)
          return 0;
        Serial.println("bad crc: ");
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
    }
#endif

    if (rf12_crc == 0) {
      // in radio mode, tell clock sync to piss off
      pattern = ((rf12_len>0&&rf12_data[0]!=PATTERN_CLOCKSYNC)?rf12_data[0]:pattern);

      if (RF12_WANTS_ACK && (config.nodeId & COLLECT) == 0) {
#ifdef DEBUG
        Serial.println(" -> ack");
#endif
        rf12_sendStart(RF12_ACK_REPLY, 0, 0);
      }
    }

    patternAvailable = !rf12_crc;
  }

  return( patternAvailable );
}

void debounceInputs() {

  static byte prevC = 0;

  inputA = !digitalRead(buttonA);
  inputB = !digitalRead(buttonB);
  inputC = (debounce(inputA&inputB, 'c')?output[2]:prevC);
  if( inputC & prevC ) 
    inputC = 0;
  prevC = inputC;
  inputA = (debounce(inputA, 'a')?output[0]:0);
  inputB = (debounce(inputB, 'b')?output[1]:0);
}

// returns 1 if captured input should trigger a break in a loop
int handleInputs() {

  int trigger = 0;

#ifdef DEBUG
  digitalWrite(A1, LOW);
  if(Serial.available())
    handleSerialInput(Serial.read());
#endif

  debounceInputs();

  // update autonomous value
  if( autonomous != !digitalRead(switchT) ) {
    trigger = 1;
    autonomous = !digitalRead(switchT);
    off(); // cut off lights
#ifdef DEBUG
    if( autonomous )
      Serial.println("entering autonomous mode");
    else
      Serial.println("message from the big, giant head!");
#endif
  }

  if( autonomous ) {
    if( inputC ) {
      trigger = 1;
      off();
      stopChooseAnother = !stopChooseAnother;
      if( stopChooseAnother ) {
#ifdef DEBUG
        Serial.println("a and b -- menu entered");
#endif
        off(0);
        my_data[0] = pattern;
        pattern = PATTERN_MODESELECT;
      }
      else {
#ifdef DEBUG
        Serial.println("a and b -- menu exited");
#endif
        off(0);
        pattern = my_data[0];
      }

    } 
    else if (inputA) {
      if( stopChooseAnother ) {
#ifdef DEBUG
        Serial.println("a -- cycle mode -1");
#endif
        off();
        if( my_data[0] > 1 )
          my_data[0]--;
        else
          my_data[0] = 1;
      }
      else {
#ifdef DEBUG
        Serial.println("a -- speed up!");
#endif
        wait = (wait>0?(wait-10):50); // speed up then wrap around back to default speed
      }
    } 
    else if (inputB) {
      if( stopChooseAnother ) {
#ifdef DEBUG
        Serial.println("a -- cycle mode +1");
#endif
        off();
        if( my_data[0] < PATTERN_LAST )
          my_data[0]++;
        else
          my_data[0] = PATTERN_LAST;
      }
      else
#ifdef DEBUG
        Serial.println("b -- flash LEDs");
#endif
      off(1); // simulate flash by turning off LEDs before continuing with pattern
    }

  } 
  else if( my_recvDone() && !rf12_crc ) {
    trigger = 1;
  } 
  else if( inputC ) {
    wait = 50; // reset speed
  } 
  else if( inputA ) {
#ifdef DEBUG
    Serial.println("a -- speed up!");
#endif
    wait = (wait>0?(wait-10):50); // speed up then wrap around back to default speed
  } 
  else if( inputB ) {
#ifdef DEBUG
    Serial.println("b -- flash LEDs");
#endif
    off(1); // simulate flash by turning off LEDs before continuing with pattern
  }

#ifdef DEBUG
  digitalWrite(A1, LOW);
#endif

  return(trigger);
}

void setup() {

#ifdef DEBUG
  pinMode(A1, OUTPUT);
#endif

  // setup on-board inputs
  pinMode(switchT, INPUT);
  pinMode(buttonA, INPUT);
  pinMode(buttonB, INPUT);
  // set pull-up resisters on AVR
  digitalWrite(switchT, HIGH);
  digitalWrite(buttonA, HIGH);
  digitalWrite(buttonB, HIGH);


    if (rf12_config()) {
      config.nodeId = eeprom_read_byte(RF12_EEPROM_ADDR);
      config.group = eeprom_read_byte(RF12_EEPROM_ADDR + 1);
    } 
    else {
      config.nodeId = 0x41; // node A1 @ 433 MHz
      config.group = 0xD4;  // group 212 (valid values: 0-212)
      saveConfig();
    }

  // The Arduino needs to clock out the data to the pixels
  // this happens in interrupt timer 1, we can change how often
  // to call the interrupt. setting CPUmax to 100 will take nearly all all the
  // time to do the pixel updates and a nicer/faster display, 
  // especially with strands of over 100 dots.
  // (Note that the max is 'pessimistic', its probably 10% or 20% less in reality)

  strip.setCPUmax(55);  // start with 50% CPU usage. up this if the strand flickers or is slow

  // Start up the LED counter
  strip.begin();

  // Update the strip, to start they are all 'off'
  strip.show();

  df_initialize();

  memset(my_data,0,sizeof(my_data));

  patternAvailable = 0;
  pattern = PATTERN_RAINBOWCYCLE;

#ifdef DEBUG
  Serial.begin(57600);
  Serial.print("\n[Radio LED Client Program -- Tron]");
  showHelp();
#endif
}

void loop() {
  handleInputs();
  runPattern(pattern);
  patternAvailable = 0;
}
