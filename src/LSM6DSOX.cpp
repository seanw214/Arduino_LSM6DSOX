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

#include "LSM6DSOX.h"

LSM6DSOXClass::LSM6DSOXClass(TwoWire& wire, uint8_t slaveAddress) :
  _wire(&wire),
  _slaveAddress(slaveAddress)
{
}

LSM6DSOXClass::~LSM6DSOXClass()
{
}

int LSM6DSOXClass::begin()
{
  int ret = _wire->begin();
  if (ret != 0)
  {
    printf("_wire->begin() error! %d\n", ret);
    return ret;
  }

  _wire->getFileDescriptor(&fd);

  if (!(readRegister(LSM6DSOX_WHO_AM_I_REG) == 0x6C || readRegister(LSM6DSOX_WHO_AM_I_REG) == 0x69)) {
    end();
    return 0;
  }

  //set the gyroscope control register to work at 833Hz, 1000 dps and in bypass mode
  writeRegister(LSM6DSOX_CTRL2_G, 0x78); // 0x68 417Hz | 0x78 833Hz

  /*
  odr/4
  hpf reference mode disabled
  fast settle mode disabled
  high pass slope disabled
  accelerometer full scale mode disabled 
  lpf on 6d disabled
  */
  writeRegister(LSM6DSOX_CTRL8_XL, 0x00); // full scale mode enabled 0x02

  // Set the Accelerometer control register to work at 833Hz, 8g, and in bypass mode and enable ODR/4
  // low pass filter (check figure9 of LSM6DSOX's datasheet)
  writeRegister(LSM6DSOX_CTRL1_XL, 0x7C); // 0x6C 417Hz | 0x7C 833Hz

  // set gyroscope power mode to high performance and bandwidth to 16 MHz
  writeRegister(LSM6DSOX_CTRL7_G, 0x00);


  // set fifo gyro/accel data rate to 417Hz
  writeRegister(LSM6DSOX_FIFO_CTRL3, 0x77); // 0x66 417Hz | 0x77 833Hz

  // set FIFO to bypass mode to flush data
  //writeRegister(LSM6DSOX_FIFO_CTRL4, 0x00);

  // set INT1 to trigger when FIFO threshold is reached
  writeRegister(LSM6DSOX_INT1_CTRL, 0x08);

  // set FIFO watermark level
  /*
  1 sensor sample (x y z) = 3 16-bit words

  16 gyro samples * 3 axes = 48 16-bit words
  16 accel samples * 3 axes = 48 16-bit words

  32 total samples = FIFO watermark level 96 words
  */
  writeRegister(LSM6DSOX_FIFO_CTRL1, 0x60);

  // stop FIFO at watermark threshold level 0x80 | do not stop FIFO 0x00
  writeRegister(LSM6DSOX_FIFO_CTRL2, 0x80);

  /** Set FIFO operation mode. Available values are:
   * 0x00 LSM6DSOX_BYPASS_MODE: FIFO is not used, the buffer content is cleared
   * LSM6DSOX_FIFO_MODE: bufer continues filling until it becomes full. Then it stops collecting data.
   * 0x06 LSM6DSOX_STREAM_MODE: continuous mode. Older data are replaced by the new data.
   * LSM6DSOX_STREAM_TO_FIFO_MODE: FIFO buffer starts operating in Continuous mode and switches to FIFO mode when an event condition occurs.
   * LSM6DSOX_BYPASS_TO_STREAM_MODE: FIFO buffer starts operating in Bypass mode and switches to Continuous mode when an event condition occurs.
  **/
  writeRegister(LSM6DSOX_FIFO_CTRL4, 0x06);

  return 1;
}

void LSM6DSOXClass::end()
{
  writeRegister(LSM6DSOX_CTRL2_G, 0x00);
  writeRegister(LSM6DSOX_CTRL1_XL, 0x00);
  _wire->end();
}

int LSM6DSOXClass::readAcceleration(float& x, float& y, float& z)
{
  int16_t data[3];

  if (!readRegisters(LSM6DSOX_OUTX_L_XL, (uint8_t*)data, sizeof(data))) {
    x = NAN;
    y = NAN;
    z = NAN;

    return 0;
  }

  // TODO: set g programmatically during init
  const float g = 8.0f;

  x = data[0] * g / 32768.0;
  y = data[1] * g / 32768.0;
  z = data[2] * g / 32768.0;

  return 1;
}

int LSM6DSOXClass::readFifoAcceleration(float& x, float& y, float& z)
{
  int16_t data_raw[3];
  uint8_t data[6];

  if (!readRegisters(LSM6DSOX_FIFO_DATA_OUT_X_L, data, 6)) {
    x = NAN;
    y = NAN;
    z = NAN;
    printf("readFifoAcceleration() readRegisters() error!\n");
    return 0;
  }

  data_raw[0] = ((int16_t)data[1] << 8) | data[0];
  data_raw[1] = ((int16_t)data[3] << 8) | data[2];
  data_raw[2] = ((int16_t)data[5] << 8) | data[4];

  // TODO: set g programmatically during init
  const float g = 8.0f;

  x = data_raw[0] * g / 32768.0;
  y = data_raw[1] * g / 32768.0;
  z = data_raw[2] * g / 32768.0;

  return 1;
}

int LSM6DSOXClass::accelerationAvailable()
{
  if (readRegister(LSM6DSOX_STATUS_REG) & 0x01) {
    return 1;
  }

  return 0;
}

float LSM6DSOXClass::accelerationSampleRate()
{
  // TODO: set samplerate programmatically during init
  return 833.0F;
}

int LSM6DSOXClass::readGyroscope(float& x, float& y, float& z)
{
  int16_t data[3];

  if (!readRegisters(LSM6DSOX_OUTX_L_G, (uint8_t*)data, sizeof(data))) {
    x = NAN;
    y = NAN;
    z = NAN;

    return 0;
  }

  // TODO: set dps programmatically during init
  const float dps = 1000.0f;

  x = data[0] * dps / 32768.0;
  y = data[1] * dps / 32768.0;
  z = data[2] * dps / 32768.0;

  return 1;
}

int LSM6DSOXClass::readFifoGyroscope(float& x, float& y, float& z)
{
  int16_t data_raw[3];
  uint8_t data[6];

  if (!readRegisters(LSM6DSOX_FIFO_DATA_OUT_X_L, data, 6))
  {
    x = NAN;
    y = NAN;
    z = NAN;
    printf("readFifoGyroscope() readRegisters() error!\n");
    return 0;
  }

  data_raw[0] = ((int16_t)data[1] << 8) | data[0];
  data_raw[1] = ((int16_t)data[3] << 8) | data[2];
  data_raw[2] = ((int16_t)data[5] << 8) | data[4];

  // TODO: set dps programmatically during init
  const float dps = 1000.0f;

  x = data_raw[0] * dps / 32768.0;
  y = data_raw[1] * dps / 32768.0;
  z = data_raw[2] * dps / 32768.0;

  return 1;
}

int LSM6DSOXClass::gyroscopeAvailable()
{
  if (readRegister(LSM6DSOX_STATUS_REG) & 0x02) {
    return 1;
  }

  return 0;
}

int LSM6DSOXClass::readTemperature(int& temperature_deg)
{
  float temperature_float = 0;
  readTemperatureFloat(temperature_float);

  temperature_deg = static_cast<int>(temperature_float);

  return 1;
}

int LSM6DSOXClass::readTemperatureFloat(float& temperature_deg)
{
  /* Read the raw temperature from the sensor. */
  int16_t temperature_raw = 0;

  if (readRegisters(LSM6DSOX_OUT_TEMP_L, reinterpret_cast<uint8_t*>(&temperature_raw), sizeof(temperature_raw)) != 1) {
    return 0;
  }

  /* Convert to °C. */
  static int const TEMPERATURE_LSB_per_DEG = 256;
  static int const TEMPERATURE_OFFSET_DEG = 25;

  temperature_deg = (static_cast<float>(temperature_raw) / TEMPERATURE_LSB_per_DEG) + TEMPERATURE_OFFSET_DEG;

  return 1;
}

int LSM6DSOXClass::temperatureAvailable()
{
  if (readRegister(LSM6DSOX_STATUS_REG) & 0x04) {
    return 1;
  }

  return 0;
}

float LSM6DSOXClass::gyroscopeSampleRate()
{
  // TODO: set samplerate programmatically during init
  return 833.0F;
}

int LSM6DSOXClass::readRegister(uint8_t address)
{
  uint8_t value;
  
  if (readRegisters(address, &value, sizeof(value)) != 1) {
    return -1;
  }
  
  return value;
}

int LSM6DSOXClass::readRegister(uint8_t address, uint8_t* buf)
{
  
  if (readRegisters(address, buf, sizeof(buf)) != 1) {
    return -1;
  }
  
  return 0;
}

int LSM6DSOXClass::readRegisters(uint8_t address, uint8_t* data, size_t length)
{
  _wire->beginTransmission(_slaveAddress);
  _wire->write(address);

  if (_wire->endTransmission(false) != 0) {
    return -1;
  }

  if (_wire->requestFrom(_slaveAddress, length) != length) {
    return 0;
  }

  for (size_t i = 0; i < length; i++) {
    *data++ = _wire->read();
  }
  return 1;
}

int LSM6DSOXClass::writeRegister(uint8_t address, uint8_t value)
{
  
  _wire->beginTransmission(_slaveAddress);
  _wire->write(address);
  _wire->write(value);
  if (_wire->endTransmission() != 0) {
    return 0;
  }

  return 1;
}

#ifdef LSM6DS_DEFAULT_SPI
LSM6DSOXClass IMU_LSM6DSOX(LSM6DS_DEFAULT_SPI, PIN_SPI_SS1, LSM6DS_INT);
#else
LSM6DSOXClass IMU_LSM6DSOX(Wire, LSM6DSOX_ADDRESS);
#endif