
#define REDPIN 5
#define GREENPIN 6
#define BLUEPIN 9

void setup () {
}

#define FADESPEED 25     // make this higher to slow down

word counter;

void loop () {
/*
    byte level = ++counter;
    analogWrite(5, bitRead(counter, 8) ? level : 0);
    analogWrite(6, bitRead(counter, 8) ? level : 0);
    analogWrite(9, bitRead(counter, 8) ? level : 0);
    delay(5);
*/

  int r, g, b;
// see http://arduino.cc/en/Reference/analogWrite
 
  // fade from blue to violet
  for (r = 0; r < 256; r++) { 
    analogWrite(REDPIN, r);
    delay(FADESPEED);
  } 
  // fade from violet to red
  for (b = 255; b > 0; b--) { 
    analogWrite(BLUEPIN, b);
    delay(FADESPEED);
  } 
  // fade from red to yellow
  for (g = 0; g < 256; g++) { 
    analogWrite(GREENPIN, g);
    delay(FADESPEED);
  } 
  // fade from yellow to green
  for (r = 255; r > 0; r--) { 
    analogWrite(REDPIN, r);
    delay(FADESPEED);
  } 
  // fade from green to teal
  for (b = 0; b < 256; b++) { 
    analogWrite(BLUEPIN, b);
    delay(FADESPEED);
  } 
  // fade from teal to blue
  for (g = 255; g > 0; g--) { 
    analogWrite(GREENPIN, g);
    delay(FADESPEED);
  } 
}

