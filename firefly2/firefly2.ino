#include <JeeLib.h>
#include <util/crc16.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <util/parity.h>

#define MAJOR_VERSION RF12_EEPROM_VERSION // bump when EEPROM layout changes
#define VERSION "[Wirefly 05-2014]"

/// Save a few bytes of flash by declaring const if used more than once.
const char INVALID1[] PROGMEM = "\rInvalid\n";
const char INITFAIL[] PROGMEM = "config save failed\n";

// comment out below before compiling production codez!
#define DEBUG 1

// Configure some values in EEPROM for easy config of the RF12 later on.
// 2009-05-06 <jcw@equi4.com> http://opensource.org/licenses/mit-license.php
// $Id: RF12demo.pde 7754 2011-08-22 11:38:59Z jcw $

#define SERIAL_BAUD 57600   // adjust as needed
#define DATAFLASH   0       // set to 0 for non-JeeLinks, else 4/8/16 (Mbit)
#define LED_PIN     9       // activity LED, comment out to disable

#define COLLECT 0x20 // collect mode, i.e. pass incoming without sending acks


Port port_three (3);

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

static void blinkerLed (byte on) {
    port_three.mode(OUTPUT);
    port_three.digiWrite(on);
}

static void printOneChar (char c) {
    Serial.print(c);
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
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
static char cmd;
static word value;
static byte stack[RF12_MAXDATA+4], top, sendLen, dest;
static byte testCounter; //number of test packets sent

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
    config.crc = calcCrc(&config, sizeof config - 2);
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
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

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
    showString(PSTR("Current configuration:\n"));
    rf12_configDump();
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Pattern control variables
#define PATTERN_OFF		0
#define PATTERN_TWINKLE		1
#define PATTERN_FIREFLY         2
#define PATTERN_CLOCKSYNC	10
#define PATTERN_CLOCKSYNC_PING	11

static uint8_t g_pattern = 0;
static int g_stopChooseAnother = 0;
static unsigned long g_wait = 50;

#define RF12_BUFFER_SIZE	66

static uint8_t my_data[RF12_BUFFER_SIZE];

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// serial I/O

//this function is intended only to run in debug mode!!
static void handleSerialInput (char c) {
    if ('0' <= c && c <= '9') {
        value = 10 * value + c - '0';
        return;
    }

    if (c == ',') {
        if (top < sizeof stack)
            stack[top++] = value; // truncated to 8 bits
        value = 0;
        return;
    }

    if ('a' <= c && c <= 'z') {
        showString(PSTR("> "));
        for (byte i = 0; i < top; ++i) {
            Serial.print((word) stack[i]);
            printOneChar(',');
        }
        Serial.print(value);
        Serial.println(c);
    }
    
    // keeping this out of the switch reduces code size (smaller branch table)
    if (c == '>') {
        // special case, send to specific band and group, and don't echo cmd
        // input: band,group,node,header,data...
        stack[top++] = value;
        // TODO: frequency offset is taken from global config, is that ok?
        rf12_initialize(stack[2], bandToFreq(stack[0]), stack[1],
                            config.frequency_offset);
        rf12_sendNow(stack[3], stack + 4, top - 4);
        rf12_sendWait(2);
        rf12_configSilent();
    } else if (c > ' ') {
        switch (c) {

        case 'i': // set node id
            config.nodeId = (config.nodeId & 0xE0) + (value & 0x1F);
            saveConfig();
            break;

        case 'b': // set band: 4 = 433, 8 = 868, 9 = 915
            value = bandToFreq(value);
            if (value) {
                config.nodeId = (value << 6) + (config.nodeId & 0x3F);
                config.frequency_offset = 1600;
                saveConfig();
            }
            break;
        case 'o': { // Increment frequency within band
// Stay within your country's ISM spectrum management guidelines, i.e.
// allowable frequencies and their use when selecting operating frequencies.
            if ((value > 95) && (value < 3904)) { // supported by RFM12B
                config.frequency_offset = value;
                saveConfig();
            }
            break;
        }
            
        case 'g': // set network group
            config.group = value;
            saveConfig();
            break;

        case 'c': // set collect mode (off = 0, on = 1)
            config.collect_mode = value;
            saveConfig();
            break;

        case 't': // broadcast a maximum size test packet, request an ack
            cmd = 'a';
            sendLen = RF12_MAXDATA;
            dest = 0;
            for (byte i = 0; i < RF12_MAXDATA; ++i)
                stack[i] = i + testCounter;
            showString(PSTR("test "));
            showByte(testCounter); // first byte in test buffer
            ++testCounter;
            break;

        case 'a': // send packet to node ID N, request an ack
        case 's': // send packet to node ID N, no ack
            cmd = c;
            sendLen = top;
            dest = value;
            break;
/*
        case 'f': // send FS20 command: <hchi>,<hclo>,<addr>,<cmd>f
            rf12_initialize(0, RF12_868MHZ, 0);
            activityLed(1);
            fs20cmd(256 * stack[0] + stack[1], stack[2], value);
            activityLed(0);
            rf12_configSilent();
            break;

        case 'k': // send KAKU command: <addr>,<dev>,<on>k
            rf12_initialize(0, RF12_433MHZ, 0);
            activityLed(1);
            kakuSend(stack[0], stack[1], value);
            activityLed(0);
            rf12_configSilent();
            break;
*/
        case 'z': // put the ATmega in ultra-low power mode (reset needed)
            if (value == 123) {
                showString(PSTR(" Zzz...\n"));
                Serial.flush();
                rf12_sleep(RF12_SLEEP);
                cli();
                Sleepy::powerDown();
            }
            break;

        case 'q': // turn quiet mode on or off (don't report bad packets)
            config.quiet_mode = value;
            saveConfig();
            break;

        case 'x': // set reporting mode to decimal (0), hex (1), hex+ascii (2)
            config.hex_output = value;
            saveConfig();
            break;

        case 'v': //display the interpreter version and configuration
            displayVersion();
            rf12_configDump();

        case 'l': // turn activity LED on or off
            activityLed(value);
            break;
/*
        case 'd': // dump all log markers
            if (df_present())
                df_dump();
            break;

        case 'r': // replay from specified seqnum/time marker
            if (df_present()) {
                word seqnum = (stack[0] << 8) | stack[1];
                long asof = (stack[2] << 8) | stack[3];
                asof = (asof << 16) | ((stack[4] << 8) | value);
                df_replay(seqnum, asof);
            }
            break;

        case 'e': // erase specified 4Kb block
            if (df_present() && stack[0] == 123) {
                word block = (stack[1] << 8) | value;
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
        case 'p': // broadcast a pattern command
            cmd = c;
            stack[0] = value;
            sendLen = 1;
            dest = 0;
            
            g_pattern = value; //sorta redundant
            break;
            
        default:
            showHelp();
        }
    }

    value = top = 0;
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
// PATTERN functions begin here

// = = = = = = = = = = = = = = = = = = = = = = = = = = =
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
  
  delay(g_wait);  //TODO: need to go into low power mode here instead of delay
//  debounceInputs();
}

// = = = = = = = = = = = = = = = = = = = = = = = = = = =
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
    
    while (true) {
        clock_recent = clock_current; //save clock_current from the previous loop() to clock_recent
        clock_current = millis(); //update to "now"
        //this is roll-over proof, if clock_current is small, and clock_recent large, 
        //the result rolls over to a small positive value:
        clock_elapsed = clock_elapsed + (clock_current - clock_recent);    

//        debounceInputs();
        
        if (clock_elapsed >= wait_millis) { //time to switch
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
        if ( my_interrupt() ) {
#ifdef DEBUG
            Serial.println("Twinkle: input received!");
#endif
            //we got a trigger, probably a network msg
            if (g_pattern != PATTERN_TWINKLE) {
#ifdef DEBUG
                Serial.print("old pattern: ");
                Serial.print(PATTERN_TWINKLE);
                Serial.print("   new pattern: ");
                Serial.println(g_pattern);
#endif
                return;
            }
        }
    }
}

// = = = = = = = = = = = = = = = = = = = = = = = = = = =
// PATTERN_FIREFLY:
void teamFirefly( ) {
/*    
MilliTimer g_sendTimer;
byte g_needToSend;
    // check the timer for sending a network message
    if (g_sendTimer.poll(3000)) // 3 seconds
        g_needToSend = 1;
*/        

}

// = = = = = = = = = = = = = = = = = = = = = = = = = = =
// PATTERN_CLOCKSYNC:
void clockSync( byte opts = 0){
#ifdef DEBUG
    Serial.println("Begin clockSync");
#endif                    
  // milliseconds, about the time it takes to send a packet
  unsigned long time_on = 750; 
  unsigned long time_cycle = 750;
  
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
      uint8_t send_data = PATTERN_CLOCKSYNC_PING;
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

    if( handleInputs() && (g_pattern <= PATTERN_CLOCKSYNC)) {
		return;
    }

  }
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
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

/*
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
*/

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
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
        debugRecv();
#endif        
		// if we got a bad crc, then no message received. (IOW this function becomes a noop)
		msgReceived = !rf12_crc;
		// If a new transmission comes in and CRC is ok, don't poll recv state again,
        // otherwise rf12_crc, rf12_len, and rf12_data will be reset.
        if (rf12_crc == 0) {
            activityLed(1);
			if (RF12_WANTS_ACK && (config.collect_mode) == 0) {
				showString(PSTR(" -> ack\n"));
				rf12_sendStart(RF12_ACK_REPLY, 0, 0);
			}
#ifdef DEBUG
			Serial.print("rf12_len "); Serial.println(rf12_len);
			Serial.print("rf12_data[0] "); Serial.println(rf12_data[0]);
#endif
            // assign the pattern that we recieve, unless...
            int new_pattern = ((rf12_len > 0) ? rf12_data[0] : g_pattern);

			// ...if we get a crazy clocksync ping msg, then just ignore it
			if (msgReceived = (new_pattern != PATTERN_CLOCKSYNC_PING)) {
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
    if (cmd) {
      if (rf12_canSend()) {
        //if rf12_canSend returns 1, then you must subsequently call rf12_sendStart.
        //do yo thang:
        activityLed(1);
#ifdef DEBUG
        showString(PSTR(" -> "));
        Serial.print((word) sendLen);
        showString(PSTR(" b\n"));
#endif
        byte header = cmd == 'a' ? RF12_HDR_ACK : 0;
        if (dest)
            header |= RF12_HDR_DST | dest;
        rf12_sendStart(header, stack, sendLen);
        cmd = 0;

        activityLed(0);
    }
  }
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// top-level functions 

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

    return trigger;
}

// = = = = = = = = = = = = = = = = = = = = = = = = = = =
// my_interrupt()
// A pattern should call this function (or some equivalent of sub-functions)
// whenever reasonably possible to keep network comm and serial IO going. 
// returns the output of handleInputs()
int my_interrupt()
{
    int inputReceived = handleInputs();  //check for serial input, call my_recvDone()
    my_send(); //try to send, if (cmd != 0)

    return inputReceived;
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

  g_pattern = PATTERN_TWINKLE;   // Wake up and twinkle

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

    randomSeed(analogRead(0));
}

// = = = = = = = = = = = = = = = = = = = = = = = = = = =
// Pattern control switch!
void runPattern() {
  activityLed(1);
#ifdef DEBUG
  Serial.print("Run pattern "); Serial.println(g_pattern);
#endif

  switch( g_pattern ) {
      default:
      case PATTERN_OFF:
        off( 1 );  // all lights off, including/especially the lantern
        break;
      case PATTERN_TWINKLE:
        randomTwinkle( ); // just blink lantern erratically, minimal radio comm
        break;
      case PATTERN_FIREFLY:
        teamFirefly( ); // slowly beginning to blink together, timed tx/rx
        break;
      case PATTERN_CLOCKSYNC:
        clockSync( );  // synchronizing firefly lanterns; the end-game
        break;
  }

  activityLed(0);
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Main loop
// = = = = = = = = = = = = = = = = = = = = = = = = = = =
void loop() {
    my_interrupt(); // note that patterns should also call interrupts, on their own time
    runPattern();  // switches control to the active pattern
}
