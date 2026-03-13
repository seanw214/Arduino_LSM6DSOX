/*
  This file is part of the Arduino_LSM6DSOX library.
  Copyright (c) 2021 Arduino SA. All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "../../WirePort/Wire.h"
#include <math.h>
#include <iostream>

#define LSM6DSOX_ADDRESS 0x6A

#define LSM6DSOX_WHO_AM_I_REG 0X0F
#define LSM6DSOX_CTRL1_XL 0X10
#define LSM6DSOX_CTRL2_G 0X11

#define LSM6DSOX_STATUS_REG 0X1E

#define LSM6DSOX_CTRL6_C 0X15
#define LSM6DSOX_CTRL7_G 0X16
#define LSM6DSOX_CTRL8_XL 0X17

#define LSM6DSOX_OUT_TEMP_L 0X20
#define LSM6DSOX_OUT_TEMP_H 0X21

#define LSM6DSOX_OUTX_L_G 0X22
#define LSM6DSOX_OUTX_H_G 0X23
#define LSM6DSOX_OUTY_L_G 0X24
#define LSM6DSOX_OUTY_H_G 0X25
#define LSM6DSOX_OUTZ_L_G 0X26
#define LSM6DSOX_OUTZ_H_G 0X27

#define LSM6DSOX_OUTX_L_XL 0X28
#define LSM6DSOX_OUTX_H_XL 0X29
#define LSM6DSOX_OUTY_L_XL 0X2A
#define LSM6DSOX_OUTY_H_XL 0X2B
#define LSM6DSOX_OUTZ_L_XL 0X2C
#define LSM6DSOX_OUTZ_H_XL 0X2D

#define LSM6DSOX_FIFO_CTRL1 0x07
#define LSM6DSOX_FIFO_CTRL2 0x08
#define LSM6DSOX_FIFO_CTRL3 0x09
#define LSM6DSOX_FIFO_CTRL4 0x0A
#define LSM6DSOX_INT1_CTRL 0x0D
#define LSM6DSOX_FIFO_STATUS1 0x3A
#define LSM6DSOX_FIFO_STATUS2 0x3B
#define LSM6DSOX_FIFO_DATA_OUT_TAG 0x78

#define LSM6DSOX_FIFO_DATA_OUT_X_L            0x79
#define LSM6DSOX_FIFO_DATA_OUT_X_H            0x7A
#define LSM6DSOX_FIFO_DATA_OUT_Y_L            0x7B
#define LSM6DSOX_FIFO_DATA_OUT_Y_H            0x7C
#define LSM6DSOX_FIFO_DATA_OUT_Z_L            0x7D
#define LSM6DSOX_FIFO_DATA_OUT_Z_H            0x7E

class LSM6DSOXClass
{
public:
  int fd;

  LSM6DSOXClass(TwoWire &wire, uint8_t slaveAddress);
  ~LSM6DSOXClass();

  int begin();
  void end();

  // Accelerometer
  int readAcceleration(float &x, float &y, float &z); // Results are in g (earth gravity).
  float accelerationSampleRate();                     // Sampling rate of the sensor.
  int accelerationAvailable();                        // Check for available data from accelerometer
  int readFifoAcceleration(float& x, float& y, float& z);

  // Gyroscope
  int readGyroscope(float &x, float &y, float &z); // Results are in degrees/second.
  float gyroscopeSampleRate();                     // Sampling rate of the sensor.
  int gyroscopeAvailable();                        // Check for available data from gyroscope
  int readFifoGyroscope(float& x, float& y, float& z);

  // Temperature
  int readTemperature(int &temperature_deg);
  int readTemperatureFloat(float &temperature_deg);
  int temperatureAvailable();

  int readRegister(uint8_t address);
  int readRegister(uint8_t address, uint8_t* buf);
  int readRegisters(uint8_t address, uint8_t* data, size_t length);

private:
  int writeRegister(uint8_t address, uint8_t value);
  TwoWire *_wire;
  uint8_t _slaveAddress;
};

extern LSM6DSOXClass IMU_LSM6DSOX;
#undef IMU
#define IMU IMU_LSM6DSOX