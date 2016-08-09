/*****************************************************************************
MIT License

Copyright (c) 2016, CAROBOTIX INC.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*****************************************************************************/

/***
  Basic input test for MCP23017 I2C port expander
  Written by Jacky Lau, CAROBOT, August 8, 2016
 ***/

#include <Wire.h>
#include "CAROBOT_MCP23017.h"

/*
 * Connect a button to pin 21, I/O 0
 * 
 * Pin 1-8: GPB0-GPB7 (I/O pin 8-15)
 * Pin 21-28: GPA0-GPA7 (I/O pin 0-7)
 * 
 * Pin 9: 5V
 * Pin 10: Ground
 * Pin 12: SCL (I2C Clock)
 * Pin 13: SDA (I2C Data)
 * Pin 15-17: Address Selection (Connect to Ground recommended)
 *   See datasheet for details
 * Pin 18: Reset (connect to 5V through 10k resistor, LOW to reset)
 */

CAROBOT_MCP23017 MCP23017;

void setup() {
  MCP23017.begin();

  // pinMode options: OUTPUT, INPUT, INPUT_PULLUP
  // INPUT_PULLUP will activate weak 100k internal resistor
  MCP23017.pinMode(0, INPUT_PULLUP);

  pinMode(13, OUTPUT); // Pin 13 LED for testing
}

void loop() {
  digitalWrite(13, !MCP23017.digitalRead(0)); // LED will turn ON when button is pressed
}
