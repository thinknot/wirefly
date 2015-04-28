#include <JeeLib.h>
#include <util/crc16.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <util/parity.h>

#include "Arduino.h"
#include "firefly.h"

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


static void showHelp () {
    showString(helpText1);
    if (df_present())
        showString(helpText2);
    showString(PSTR("Current configuration:\n"));
    rf12_configDump();
}

static void printOneChar (char c) {
    Serial.print(c);
}

void showString (PGM_P s) {
    for (;;) {
        char c = pgm_read_byte(s++);
        if (c == 0)
            break;
        if (c == '\n')
            printOneChar('\r');
        printOneChar(c);  
    }
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

void displayVersion () {
    showString(PSTR(VERSION));
    showString(PSTR("\nBlue pin: "));
    Serial.println(BLUEPIN);
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// = = = = = = = = = = = = = = = = = = = = = = = = = = =
// serial I/O
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

//this function is intended only to run in debug mode!!
void handleSerialInput (char c) {
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
