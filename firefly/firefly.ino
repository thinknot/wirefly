/// Wireless Firefly
///==============================
/// Based on:
/// radio_led_client.pde from 10bitworks - Luminaria 2012 project
///
/// @dir pingPong Demo of a sketch which sends and receives packets.
/// 2010-05-17 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php
/// @see http://jeelabs.org/2009/02/14/ports-library-for-arduino/
/// 2009-02-13 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

/// with thanks to Peter G for creating a test sketch and pointing out the issue
/// see http://jeelabs.org/2010/05/20/a-subtle-rf12-detail/

#include <JeeLib.h>
#include <util/crc16.h>
#include <avr/eeprom.h>

// comment out DEBUG before compiling production codez!
#define DEBUG 1

#define COLLECT 0x20 // collect mode, i.e. pass incoming without sending acks

// EEPROM support
#define DATAFLASH   1   // check for presence of DataFlash memory on JeeLink
#define FLASH_MBIT  16  // support for various dataflash sizes: 4/8/16 Mbit

MilliTimer sendTimer;
byte needToSend;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Network activity LED
#define LED_PIN     9   // activity LED, comment out to disable

static void activityLed (byte on) {
#ifdef LED_PIN
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, !on);
#endif
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// PORTS and I/O
Port port_two (2);
Port port_three (3);

static void sendLed (byte on) {
    port_two.mode(OUTPUT);
    port_two.digiWrite(on);
}

static void receiveLed (byte on) {
    port_two.mode2(OUTPUT);
    port_two.digiWrite2(!on); // inverse, because LED is tied to VCC
}

static void blinkerLed (byte on) {
    port_three.mode(OUTPUT);
    port_three.digiWrite(on);
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// RF12 configuration data
typedef struct {
  byte nodeId;
  byte group;
  char msg[RF12_EEPROM_SIZE-4];
  word crc;
} 
RF12Config;

static RF12Config config;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Real time: involatile clock, based on millis
unsigned long clock_current  = 0; //this variable will be overwritten by millis() each iteration of loop
unsigned long clock_recent     = 0; //no time has passed yet
// state machine for LED control
int wait = 0; //interval when the blinker LED is on
unsigned long clock_elapsed = 0;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Pseduo time: volatile, based on network synchronization with other nodes
long timestamp_offset = 0;
unsigned long timestamp_lastblink = 0; // last lighting timestamp when the LED was blinked

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// SETUP
void setup () {
    port_two.mode(OUTPUT);
    port_three.mode(OUTPUT);
#ifdef DEBUG
    Serial.begin(57600);
    Serial.println("Wireless Firefly");    
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
    randomSeed(analogRead(0));
/*    
    //make sure everyone starts at a somewhat different time
    int start_delay = random(5000, 21001);
    Serial.print("Start delay: ");
    Serial.println(start_delay);
    delay(start_delay);
*/
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// MAIN LOOP
void loop () {
  // pseudo time, network time
    unsigned long timestamp_now = now();

  // clock time, real time, lighting time
    clock_recent = clock_current; //clock_current is from the previous loop() so save to clock_recent
    clock_current = millis(); 
    //this is roll-over proof, if clock_current is small, and clock_recent large, the result rolls over to a small positive value
    clock_elapsed = clock_elapsed + (clock_current - clock_recent);
    
#ifdef DEBUG
//    digitalWrite(A1, LOW); TODO: find out what is this??
    if (Serial.available())
        handleSerialInput(Serial.read());
#endif
    if (wait == 0) // if wait is zero, the light is off
    {
        // Decide if to turn on the blinker light
        if (timestamp_now > timestamp_lastblink + 3000)
        {      
            timestamp_lastblink = timestamp_now; // record the clock time for this blink
            blinkerLed(HIGH);
            wait = 500;     //wait 500 milliseconds before turning off the LED
            clock_elapsed = 0; // clear the elapsed time value, just for this loop iteration
    #ifdef DEBUG
            Serial.print("Blinker on: ");
            Serial.println(timestamp_now);
    #endif
        }
    }
    else // wait is not zero, so we must have previously turned the light on
    {        
        // if the LED was previously turned on, and enough time has passed, turn the LED off
        if (clock_elapsed >= wait) 
        {
            blinkerLed(LOW);
            wait = 0; //tell the next loop that the light is off
#ifdef DEBUG
            Serial.print("Blinker off: ");
            Serial.println(timestamp_now);
#endif                
        }
    }
    
    // receive a network message if one exists; keep the RF12 library happy
    if (rf12_recvDone()) {
        if (rf12_crc == 0) {
          // the message is good, unpack the timestamp data:
            unsigned long timestamp_received = receive();
          // accept feedback and self-regulate (compute a new timestamp_now):
            sync_lighting_time(timestamp_received);
        } 
    }
    
    // check the timer for sending a network message
    if (sendTimer.poll(30000)) // 30 seconds
        needToSend = 1;
        
    //if rf12_canSend returns 1, then you must subsequently call rf12_sendStart.
    if (needToSend && rf12_canSend()) {
        needToSend = 0;
        send(timestamp_now);   //do yo thang
    }
}

void send (unsigned long timestamp_now) {
    rf12_sendStart(0, &timestamp_now, sizeof timestamp_now);
#ifdef DEBUG
    Serial.print("Transmit time: ");
    Serial.println(timestamp_now);
    sendLed(1);
    delay(400); // otherwise TX led blinking isn't visible
    sendLed(0);
#endif
}

unsigned long receive() {
    byte n = rf12_len;
    // unpack the rf12_data into our long
    unsigned long timestamp_rx = (
      ((long) rf12_data[3] << 24) +
      ((long) rf12_data[2] << 16) +
      ((long) rf12_data[1] <<  8) +
      ((long) rf12_data[0])
    );
#ifdef DEBUG
/*
    Serial.print("RX ");
    Serial.print((int) rf12_hdr);
    for (byte i = 0; i < n; ++i) {
      Serial.print(' ');
      Serial.print((int) rf12_data[i]);
    }
    Serial.println();
*/
    receiveLed(1);
    delay(400); // otherwise RX led blinking isn't visible
    receiveLed(0);
#endif      
    return timestamp_rx;
}

// "now" for light timing: calculated by tracking an offset against "real time"
static unsigned long now () {
  return millis() + timestamp_offset;
}

void sync_lighting_time(unsigned long timestamp_received) {
    unsigned long timestamp_now = now();
    unsigned long timestamp_estimate = (timestamp_now + timestamp_received) / 2;
#ifdef DEBUG    
    Serial.print("Overheard time: ");
    Serial.println(timestamp_received);
    Serial.print("Old time: ");
    Serial.println(timestamp_now);
    Serial.print("New time: ");
    Serial.println(timestamp_estimate);
#endif    
    timestamp_offset = timestamp_estimate - timestamp_now;
}
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// RF12 configuration setup code
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
  static word bands[4] = {
    315, 433, 868, 915    };
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

