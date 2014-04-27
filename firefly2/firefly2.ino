#include <TimerOne.h>

#include <Ports.h>
#include <RF12.h>
#include <util/crc16.h>
#include <util/parity.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>

// comment out below before compiling production codez!
#define DEBUG 1

//#define UPSIDE_DOWN_LEDS 1

// Configure some values in EEPROM for easy config of the RF12 later on.
// 2009-05-06 <jcw@equi4.com> http://opensource.org/licenses/mit-license.php
// $Id: RF12demo.pde 7754 2011-08-22 11:38:59Z jcw $

// this version adds flash memory support, 2009-11-19

#define DATAFLASH   1   // check for presence of DataFlash memory on JeeLink
#define FLASH_MBIT  16  // support for various dataflash sizes: 4/8/16 Mbit

//#define LED_PIN     9   // on-board activity LED, comment out to disable

#define COLLECT 0x20 // collect mode, i.e. pass incoming without sending acks

MilliTimer g_sendTimer;
byte g_needToSend;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// RF12 configuration setup code

typedef struct {
  byte nodeId;
  byte group;
  char msg[RF12_EEPROM_SIZE-4];
  word crc;
} 
RF12Config;

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

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

static unsigned long now () {
  // FIXME 49-day overflow
  return millis() / 1000;
}

//Port port_two (2);
Port port_three (3);

static void activityLed (byte on) {
#ifdef LED_PIN
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, !on);
#endif
}

static void blinkerLed (byte on) {
    port_three.mode(OUTPUT);
    port_three.digiWrite(on);
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

#define PATTERN_OFF		0
#define PATTERN_TWINKLE		1
#define PATTERN_FIREFLY         2
#define PATTERN_CLOCKSYNC	3
#define PATTERN_MODESELECT	15
#define PATTERN_IDENTIFICATION  16

#define HANDLEINPUTS_TIME  22 //microseconds

static uint8_t g_pattern = 0;
static int g_stopChooseAnother = 0;
static unsigned long g_wait = 50;
static int g_patternReceived = 0;

#define PIX_COUNT		30
#define RF12_BUFFER_SIZE	66

static uint8_t my_data[RF12_BUFFER_SIZE];

//this function is intended only to run in debug mode!!
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
                cmd = c;
                sendLen = top;
                dest = value;
                memcpy(databuffer, stack, top);
                break;   
    case 'h':
    case 'j':
    case 'm':
    case 'n':
    case 'o':
    case 'p':
    case 'u':
    case 'v':
    case 'x': //switch to clocksync
        g_pattern = PATTERN_CLOCKSYNC;
        g_patternReceived = 1;
        break;
    case 'y': //switch to twinkle
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

//=============================
// PATTERN functions begin here
//============================= 

// PATTERN_OFF: No color value, clear LEDs
void off(byte opts = 0) {
/*
  int startIndex = (IGNORE_FRONT(opts)?PIXEL_FRONT_LAST+1:0);
  int endIndex = (IGNORE_BACK(opts)?PIXEL_FRONT_LAST+1:strip.numPixels());
  for(uint8_t p=startIndex; p<endIndex; p++) {
    debounceInputs();
    strip.setPixelColor(p,0);
  }
  strip.show();
*/
  blinkerLed(LOW);
  
  delay(g_wait);
//  debounceInputs();
}

// PATTERN_TWINKLE:
void randomTwinkle( ) {
#ifdef DEBUG
    Serial.println("Begin randomTwinkle");
#endif                
    int lantern_on = 0; // LED 1 = enabled, 0 = disabled 
    int wait_millis = 0; // time to stay either on or off
    unsigned long clock_elapsed = 0; // counting how long it's been on or off

    unsigned long clock_current = 0; //this variable will be updated each iteration of loop
    unsigned long clock_recent = 0; //no time has passed yet
    
    while(true) {
        clock_recent = clock_current; //save clock_current from the previous loop() to clock_recent
        clock_current = millis(); //update to "now"
        //this is roll-over proof, if clock_current is small, and clock_recent large, 
        //the result rolls over to a small positive value:
        clock_elapsed = clock_elapsed + (clock_current - clock_recent);    

//        debounceInputs();
        
        if (clock_elapsed >= wait_millis) { //time to switch
#ifdef DEBUG
                Serial.print("Blinker: ");
                Serial.print(lantern_on);
                Serial.print("   Timer: ");
                Serial.println(clock_elapsed);
#endif                
            if (lantern_on == 1) { //the light was previously turned on
                wait_millis = random(1500, 3999);
                blinkerLed(LOW);
                lantern_on = 0;
            }
            else { //the light was/is off
                wait_millis = random(300, 1200);
                blinkerLed(HIGH);
                lantern_on = 1;
            }
            clock_elapsed = 0;
        }
        if ( handleInputs() ) {
            //we got a trigger, probably a network msg
            if (g_pattern != PATTERN_TWINKLE)
                return;
        }
    }
}

// PATTERN_FIREFLY:
void teamFirefly( ) {

}

// PATTERN_CLOCKSYNC:
void clockSync( byte opts = 0){
#ifdef DEBUG
    Serial.println("Begin clockSync");
#endif                    
  // milliseconds, about the time it takes to send a packet
  unsigned long time_on = 750; 
  unsigned long time_cycle = 3*750;
  
  unsigned long time_start = 0;
  unsigned long time_end = 0;
  unsigned long sum_ON = 0;
  unsigned long sum_OFF = 0;
  
  int ON_longer = 0;
  int OFF_longer = 0;
  int ON_count = 0;
  int OFF_count = 0;

  off(opts);

//  debounceInputs();
  delay(random(0,2001)); //make sure everyone starts at a somewhat different time
//  debounceInputs();

  while(true)
  {
    ON_count = 0;
    sum_ON = 0;
    OFF_count = 0;
    blinkerLed(HIGH); //turn LED on
    
    //transmit a packet
    if (rf12_canSend()) {
      uint8_t send_data = PATTERN_CLOCKSYNC;
      rf12_sendStart(0, &send_data, 1);
    }
    time_start = millis();
    time_end = time_start + time_on + ON_longer;

    while (millis() < time_end) { //how far are we from the start
//      debounceInputs();
      if (rf12_recvDone() && !rf12_crc) { //did we get a good packet?
        sum_ON += millis() - time_start;
        ON_count++;
      }
    }

    blinkerLed(LOW); //turn LED off
    time_start = millis();
    time_end = time_start + time_cycle + OFF_longer;

    while(millis() < time_end) {
//      debounceInputs();
      if (rf12_recvDone() && rf12_crc == 0){
        sum_OFF += time_end - millis(); //how far are we from the start of the next cycle
        OFF_count++;
      }
    }
  
    if (ON_count > OFF_count) //it was ON longer
    {
      ON_longer = 0;
      OFF_longer = (sum_ON/ON_count) >> 1;
    } 
    else if (ON_count < OFF_count) //it was OFF longer
    {
      OFF_longer = 0;
      ON_longer = (sum_ON/ON_count) >> 1;
    } 
    else if (ON_count == 0 && OFF_count == 0)
    {
      ON_longer = 0;
      OFF_longer = 0;
    } 
    else if (ON_count == OFF_count)
    {
      if (sum_ON < sum_OFF)
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

    if( handleInputs() ) {
      return;
    }

  }
}

// only turn on the specified LED
void modeSelect(uint8_t modeValue) {
  if( modeValue < 1 )
    modeValue = 1;
//  debounceInputs();
  delay(g_wait);
//  debounceInputs();
}

void identify( ) {
  uint8_t nodeid = (config.nodeId & 0x1F);
//  debounceInputs();
  delay(g_wait);
//  debounceInputs();
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
/*
  if( !autonomous )
    memcpy(my_data+1, const_cast<uint8_t*>(rf12_data+1), RF12_BUFFER_SIZE-1);
*/
  int same = (g_pattern == patternToRun);
  if (!same) {
      g_needToSend = 1;
  }
  g_pattern = patternToRun;

  switch( g_pattern ) {
  default:
  case PATTERN_OFF:
    off( 1 );  // all lights off including the lantern
    break;
  case PATTERN_TWINKLE:
    randomTwinkle( ); // just blink lantern erratically, minimal radio comm
    break;
  case PATTERN_FIREFLY:
    teamFirefly( ); // slowly beginning to blink together
    break;
  case PATTERN_CLOCKSYNC:
    clockSync( );  // synchronizing firefly lanterns, the end-game
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
      identify( ); // show the given pattern numeric code
    /*}*/
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

void debugRecv() {
    byte n = rf12_len;
    if (rf12_crc == 0) {
        Serial.print("OK");
      }
      else {
        if (quiet)
          return;
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

//=======================
// my_recvDone
// @returns: g_PatternReceived = 1 if a pattern was received successfully
//=======================
int my_recvDone() {
  //if a pattern has not already been received then we should check
  if ( !g_patternReceived ) {
    // (receive a network message if one exists; keep the RF12 library happy)
    // rf12_recvDone() needs to be constantly called in order to recieve new transmissions
    // it checks to see if a packet has been received, returns true if it has:
    if ( rf12_recvDone() ) {
#ifdef DEBUG
        debugRecv();
#endif        
        // If a new transmission comes in and CRC is ok, don't poll recv state again,
        // or else rf12_crc, rf12_len, and rf12_data will be reset.
        if (rf12_crc == 0) {
          // assign the pattern that we recieve, unless...
          // in radio mode, tell clock sync to piss off (ignore clocksync because she is crazy)
          g_pattern = ((rf12_len>0 && rf12_data[0]!=PATTERN_CLOCKSYNC) ? rf12_data[0] : g_pattern);

          if (RF12_WANTS_ACK && (config.nodeId & COLLECT) == 0) {
#ifdef DEBUG
            Serial.println(" -> ack");
#endif
            //send an ack if some crazy out there wants it
            rf12_sendStart(RF12_ACK_REPLY, 0, 0);
          }
        }
        // if we got a bad crc, then no pattern received. (this is a noop)
        g_patternReceived = !rf12_crc;
      }
  }
  return g_patternReceived;
}

//=======================
// Send
//=======================
int my_send() {
/*    
    // check the timer for sending a network message
    if (g_sendTimer.poll(3000)) // 3 seconds
        g_needToSend = 1;
*/        
    //if rf12_canSend returns 1, then you must subsequently call rf12_sendStart.
    if (g_needToSend && rf12_canSend()) {
        g_needToSend = 0;
        activityLed(1);
        //do yo thang:
#ifdef DEBUG
        Serial.print(" -> ");
        Serial.print((int) sendLen);
        Serial.println(" b");
#endif
        byte header = cmd == 'a' ? RF12_HDR_ACK : 0;
        if (dest)
            header |= RF12_HDR_DST | dest;
#ifdef DEBUG
        Serial.print("Header: "); Serial.print((int)header);
        Serial.print(" Data: ");
        for( int d=0; d<sendLen; d++)
          Serial.print((int) databuffer[d]);
        Serial.print(" Length: "); Serial.print((int) sendLen);
        Serial.println();
#endif
        rf12_sendStart(header, databuffer, sendLen);
        cmd = 0;

        activityLed(0);
    }
}

// handleInputs()
// returns 1 if captured input should trigger a break in a loop
//
int handleInputs() {

  int trigger = 0;

#ifdef DEBUG
  if(Serial.available())
    handleSerialInput(Serial.read());
#endif

//  debounceInputs();

  if ( my_recvDone() && !rf12_crc ) {
      //my_recvDone alters g_pattern, alters/returns g_patternReceived
    trigger = 1;
  } 

  return trigger;
}

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

  g_pattern = PATTERN_TWINKLE;   // Wake up and twinkle
  g_patternReceived = 1;

#ifdef DEBUG
  Serial.begin(57600);
  Serial.print("\n[Firefly2 04-2014]");
  showHelp();
#endif
    randomSeed(analogRead(0));
}

// A pattern should call this function (or some equivalent of sub-functions)
// whenever reasonably possible to keep network comm and serial IO going. 
void my_interrupt()
{
    handleInputs();  //check for serial input, call my_recvDone()
    my_send(); //send the football if g_needToSend is true
}

void loop() {
    runPattern(g_pattern);  // switches control to the current pattern
    g_patternReceived = 0; // if we got here, the pattern has returned control.
    my_interrupt();
}
