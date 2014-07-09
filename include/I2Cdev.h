// Modified (i.e. reduced) on 3/23/13 by Parker Stebbins
// (Severed FastWire and NBWire; also killed a few non-MPU functions.)

// I2Cdev library collection - Main I2C device class header file
// Abstracts bit and byte I2C R/W functions into a convenient class
// 6/9/2012 by Jeff Rowberg <jeff@rowberg.net>

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
=============================================== */


#ifndef _I2CDEV_H_
	#define _I2CDEV_H_

	#include "Arduino.h"
	#include <Wire.h>

	class I2Cdev {
		public:
			I2Cdev() {};
			
			static int8_t readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data, uint16_t timeout) { 
				uint8_t b;
				uint8_t count = readByte(devAddr, regAddr, &b, timeout);
				*data = b & (1 << bitNum);
				return count;
			}

			static int8_t readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data, uint16_t timeout=I2Cdev::readTimeout) {
				uint8_t count, b;
				if ((count = readByte(devAddr, regAddr, &b, timeout)) != 0) {
					uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
					b &= mask;
					b >>= (bitStart - length + 1);
					*data = b;
				}
				return count;
			}

			static int8_t readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t timeout=I2Cdev::readTimeout) {
				return readBytes(devAddr, regAddr, 1, data, timeout);
			}

			static int8_t readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout=I2Cdev::readTimeout) {
				int8_t count = 0;
				uint32_t t1 = millis();
				for (uint8_t k = 0; k < length; k += min(length, BUFFER_LENGTH)) {
					Wire.beginTransmission(devAddr);
					Wire.write(regAddr);
					Wire.endTransmission();
					Wire.beginTransmission(devAddr);
					Wire.requestFrom(devAddr, (uint8_t)min(length - k, BUFFER_LENGTH));

					for (; Wire.available() && (timeout == 0 || millis() - t1 < timeout); count++) {
						data[count] = Wire.read();
					}

					Wire.endTransmission();
				}
				// check for timeout
				if (timeout > 0 && millis() - t1 >= timeout && count < length) count = -1; // timeout
				return count;
			}

			static bool writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data) {
				uint8_t b;
				readByte(devAddr, regAddr, &b);
				b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
				return writeByte(devAddr, regAddr, b);
			}

			static bool writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) {
				uint8_t b;
				if (readByte(devAddr, regAddr, &b) != 0) {
					uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
					data <<= (bitStart - length + 1); // shift data into correct position
					data &= mask; // zero all non-important bits in data
					b &= ~(mask); // zero all important bits in existing byte
					b |= data; // combine data with existing byte
					return writeByte(devAddr, regAddr, b);
				} else {
					return false;
				}
			}

			static bool writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data) {
				return writeBytes(devAddr, regAddr, 1, &data);
			}

			static bool writeWord(uint8_t devAddr, uint8_t regAddr, uint16_t *data) {
				uint8_t status = 0;
				Wire.beginTransmission(devAddr);
				Wire.write(regAddr); // send address
				for (uint8_t i = 0; i < 1 * 2; i++) {
						Wire.write((uint8_t)(data[i++] >> 8)); // send MSB
						Wire.write((uint8_t)data[i]); // send LSB
				}
				status = Wire.endTransmission();
				return status == 0;
			}

			static bool writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data) {
				uint8_t status = 0;
					Wire.beginTransmission(devAddr);
					Wire.write((uint8_t) regAddr); // send address
				for (uint8_t i = 0; i < length; i++) {
					Wire.write((uint8_t) data[i]);
				}
				status = Wire.endTransmission();
				return status == 0;
			}


			static uint16_t readTimeout = 1000;
	};
#endif