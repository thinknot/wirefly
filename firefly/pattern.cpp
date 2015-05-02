#include <JeeLib.h>
#include <util/crc16.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <util/parity.h>

#include "Arduino.h"
#include "firefly.h"

/*
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
 */

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// = = = = = = = = = = = = = = = = = = = = = = = = = = =
// PATTERN functions begin here

uint8_t g_pattern = 0;

// = = = = = = = = = = = = = = = = = = = = = = = = = = =
// PATTERN_OFF: No color value, clear LEDs
void pattern_off() {
	rgbSet(MAX_RGB_VALUE, MAX_RGB_VALUE, MAX_RGB_VALUE);
	while (1)
		if (pattern_interrupt(PATTERN_OFF)) return;
}

// = = = = = = = = = = = = = = = = = = = = = = = = = = =
// PATTERN_LUXMETER: low-power periodic monitoring 
/*
void pattern_luxMeter() {
    byte highGain;
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
 */
// = = = = = = = = = = = = = = = = = = = = = = = = = = =
// PATTERN_TWINKLE:
void pattern_randomTwinkle() {
#ifdef DEBUG
	Serial.println("pattern_randomTwinkle()");
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
	Serial.println("pattern_teamFirefly()");
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
 RGB LED - Automatic Smooth Color Cycling
 Marco Colli
 April 2012 see http://forum.arduino.cc/index.php?topic=102040.0

 Uses the properties of the RGB Colour Cube
 The RGB colour space can be viewed as a cube of colour. If we assume a cube of dimension 1, then the 
 coordinates of the vertices for the cubve will range from (0,0,0) to (1,1,1) (all black to all white).
 The transitions between each vertex will be a smooth colour flow and we can exploit this by using the 
 path coordinates as the LED transition effect. 
 */

// Slowing things down we need...
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

		if (pattern_interrupt(PATTERN_FADER))
			return;
	}

	delay(FADE_WAIT_DELAY);
	// give it an extra rest at the end of the traverse

	if (pattern_interrupt(PATTERN_FADER))
		return;
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
	Serial.println("pattern_rgbPulse()");
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
void pattern_clockSync( ) {
#ifdef DEBUG
	Serial.println("pattern_clockSync()");
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
	pattern_off();

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
#ifdef DEBUG
				//        Serial.print("rf12_len "); Serial.println(rf12_len);
				//        Serial.print("rf12_data[0] "); Serial.println(rf12_data[0]);
#endif
				// grab any pattern that we recieve, unless no data...
				int new_pattern = ((rf12_len > 0) ? rf12_data[0] : g_pattern);
				if (new_pattern != PATTERN_CLOCKSYNC_PING) {
					//set the new pattern and bail out
					g_pattern = new_pattern;
					return;
				}
				// ...if we get a clocksync ping msg, then calculate
				else
				{
					//this means somebody else is on at the same time as me, keep track
					sum_ON += millis() - time_start;
					ON_count++;
				}
			}
		}

		//Phase2
		rgbSet(MAX_RGB_VALUE, MAX_RGB_VALUE, MAX_RGB_VALUE); //turn LED off

		time_start = millis(); //reset start time to now
		time_end = time_start + time_cycle + OFF_longer; //how long to listen w/ LED off

		while (millis() < time_end) { //listen for pings until the time is up
			//      debounceInputs();
			if (rf12_recvDone() && rf12_crc == 0){ //if we get a good packet
#ifdef DEBUG
				//        Serial.print("rf12_len "); Serial.println(rf12_len);
				//        Serial.print("rf12_data[0] "); Serial.println(rf12_data[0]);
#endif
				// grab any pattern that we recieve, unless no data...
				int new_pattern = ((rf12_len > 0) ? rf12_data[0] : g_pattern);
				if (new_pattern != PATTERN_CLOCKSYNC_PING) {
					//set the new pattern and bail out
					g_pattern = new_pattern;
					return;
				}
				// ...if we get a clocksync ping msg, then calculate
				else
				{
					//this means somebody else is on when I am off, keep track
					sum_OFF += time_end - millis(); 
					OFF_count++;
				}
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

		if (Serial.available()) {
			handleSerialInput(Serial.read());
			my_send();
		}

	}
}
