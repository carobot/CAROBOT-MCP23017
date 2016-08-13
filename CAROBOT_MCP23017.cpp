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
  Library for MCP23017 I2C port expander
  Written by Jacky Lau, CAROBOT, August 8, 2016
 ***/

#if defined(ARDUINO) && ARDUINO >= 100
    #include <Arduino.h>
#else
    #include <WProgram.h>
#endif

// for ATtiny I2C
#ifdef __AVR_ATtiny85__
    #include <TinyWireM.h>
    #define Wire TinyWireM
#else
    #include <Wire.h>
#endif

#include "CAROBOT_MCP23017.h"

/***
 * Helper functions to keep backward compatibility
 ***/
static inline void wire_write(uint8_t x) {
#if ARDUINO >= 100
    Wire.write((uint8_t) x);
#else
    Wire.send(x);
#endif
}

static inline uint8_t wire_read(void) {
#if ARDUINO >= 100
    return Wire.read();
#else
    return Wire.receive();
#endif
}

/***
 * Initialize the MCP23017 with 0 as the default I2C address
 ***/
void CAROBOT_MCP23017::begin(void) {
	begin(0);
}

/***
 * Initialize the MCP23017 with the given I2C address
 ***/
void CAROBOT_MCP23017::begin(uint8_t i2cAddress) {
    i2cAddress = i2cAddress & 0x07;

	Wire.begin();

	// set all I/O to input
	writeRegister(MCP23017_IODIRA, 0xFF);
	writeRegister(MCP23017_IODIRB, 0xFF);

	// set all I/O to default value
	writeRegister(MCP23017_GPIOA, 0x00);
	writeRegister(MCP23017_GPIOB, 0x00);

	// set all pull-up to disable
	writeRegister(MCP23017_GPPUA, 0x00);
	writeRegister(MCP23017_GPPUB, 0x00);
}

/***
 * Write to a given register
 ***/
void CAROBOT_MCP23017::writeRegister(uint8_t regAddress, uint8_t regValue) {
	Wire.beginTransmission(MCP23017_ADDRESS | i2cAddress);
	wire_write(regAddress);
	wire_write(regValue);
	Wire.endTransmission();
}

/***
 * Read from a given register
 ***/
uint8_t CAROBOT_MCP23017::readRegister(uint8_t regAddress) {
	Wire.beginTransmission(MCP23017_ADDRESS | i2cAddress);
	wire_write(regAddress);
	Wire.endTransmission();

	Wire.requestFrom(MCP23017_ADDRESS | i2cAddress, 1);

	return wire_read();
}

/***
 * Write to a given bit in a register
 ***/
void CAROBOT_MCP23017::writeRegisterBit(uint8_t regAddress, uint8_t bit, uint8_t pinValue){
	uint8_t regValue = readRegister(regAddress);

	regValue = bitWrite(regValue, bit, pinValue);

	writeRegister(regAddress, regValue);
}

/***
 * Write to all GPIO
 ***/
void CAROBOT_MCP23017::writeGPIO(uint16_t regValue) {
	Wire.beginTransmission(MCP23017_ADDRESS | i2cAddress);
	wire_write(MCP23017_GPIOA);
	wire_write(regValue & 0xFF);
	wire_write((regValue & 0xFF) >> 8);
	Wire.endTransmission();
}

/***
 * Read from all GPIO
 ***/
uint16_t CAROBOT_MCP23017::readGPIO() {
	uint16_t gpioValue = 0;

	Wire.beginTransmission(MCP23017_ADDRESS | i2cAddress);
	wire_write(MCP23017_GPIOA);
	Wire.endTransmission();

	Wire.requestFrom(MCP23017_ADDRESS | i2cAddress, 2);
	gpioValue = wire_read();
	gpioValue |= wire_read() << 8;

	return gpioValue;
}

/***
 * Set the pinmode of one GPIO
 ***/
void CAROBOT_MCP23017::pinMode(uint8_t pin, uint8_t mode) {

    uint8_t regIODIR = MCP23017_IODIRA;
    uint8_t regGPPU = MCP23017_GPPUA;
    if (pin > 7) {
        regIODIR = MCP23017_IODIRB;
        regGPPU = MCP23017_GPPUB;
    }

    // For MCP23017, output is 0, input is 1
    // For weak 100k pull-up, enable is 1, disable is 0
    if (mode == OUTPUT) {
        writeRegisterBit(regGPPU, pin % 8, 0);
        writeRegisterBit(regIODIR, pin % 8, 0);
    } else if (mode == INPUT) {
        writeRegisterBit(regIODIR, pin % 8, 1);
        writeRegisterBit(regGPPU, pin % 8, 0);
    } else if (mode == INPUT_PULLUP) {
        writeRegisterBit(regIODIR, pin % 8, 1);
        writeRegisterBit(regGPPU, pin % 8, 1);
    }
}

/***
 * Write to a GPIO
 ***/
void CAROBOT_MCP23017::digitalWrite(uint8_t pin, uint8_t state) {

    uint8_t regIODIR = MCP23017_IODIRA;
    uint8_t regGPPU = MCP23017_GPPUA;
    uint8_t regGPIO = MCP23017_GPIOA;
    if (pin > 7) {
        regIODIR = MCP23017_IODIRB;
        regGPPU = MCP23017_GPPUB;
        regGPIO = MCP23017_GPIOB;
    }

    writeRegisterBit(regGPPU, pin % 8, 0);
    writeRegisterBit(regIODIR, pin % 8, 0);
    writeRegisterBit(regGPIO, pin % 8, state);
}

/***
 * Read from a GPIO
 ***/
uint8_t CAROBOT_MCP23017::digitalRead(uint8_t pin) {
    if (pin > 7) {
        return (readRegister(MCP23017_GPIOB) >> pin % 8 & 0x01);
    } else {
        return (readRegister(MCP23017_GPIOA) >> pin % 8 & 0x01);
    }
}
