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

#ifndef CAROBOT_MCP23017_H

#define CAROBOT_MCP23017_H

// for ATtiny I2C
#ifdef __AVR_ATtiny85__
    #include <TinyWireM.h>
#else
    #include <Wire.h>
#endif

#define MCP23017_ADDRESS 0x20

// registers for port A
#define MCP23017_IODIRA 0x00
#define MCP23017_IPOLA 0x02
#define MCP23017_GPINTENA 0x04
#define MCP23017_DEFVALA 0x06
#define MCP23017_INTCONA 0x08
#define MCP23017_IOCONA 0x0A
#define MCP23017_GPPUA 0x0C
#define MCP23017_INTFA 0x0E
#define MCP23017_INTCAPA 0x10
#define MCP23017_GPIOA 0x12
#define MCP23017_OLATA 0x14

// registers for port B
#define MCP23017_IODIRB 0x01
#define MCP23017_IPOLB 0x03
#define MCP23017_GPINTENB 0x05
#define MCP23017_DEFVALB 0x07
#define MCP23017_INTCONB 0x09
#define MCP23017_IOCONB 0x0B
#define MCP23017_GPPUB 0x0D
#define MCP23017_INTFB 0x0F
#define MCP23017_INTCAPB 0x11
#define MCP23017_GPIOB 0x13
#define MCP23017_OLATB 0x15

class CAROBOT_MCP23017 {

public:
    void begin(void);
    void begin(uint8_t i2cAddress);

    void writeRegister(uint8_t regAddress, uint8_t regValue);
    uint8_t readRegister(uint8_t regAddress);
    void writeGPIO(uint16_t regValue);
    uint16_t readGPIO();

    void pinMode(uint8_t pin, uint8_t mode);
    void digitalWrite(uint8_t pin, uint8_t state);
    uint8_t digitalRead(uint8_t pin);

private:
    uint8_t i2cAddress;

    void writeRegisterBit(uint8_t regAddress, uint8_t bit, uint8_t pinValue);

};

#endif // CAROBOT_MCP23017_H

/*****************************************************************************/
