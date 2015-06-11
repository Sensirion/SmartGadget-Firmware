/*Copyright (c) 2015, Sensirion AG
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of Sensirion AG nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.*/

#include "Sht31.h"

Sht31::Sht31(I2c* i2c):
  _i2c(i2c), _ShtAddr(SHT31_ADDRESS_ARD_PIN_FLOATING), _availability(SHT31_UNKNOWN)
{
}

bool Sht31::isAvailable()
{
  if (SHT31_AVAILABLE == _availability)
  {
    return true;
  }
  if (SHT31_NOT_AVAILABLE == _availability)
  {
    return false;
  }
  _availability = pollSensor(SHT31_ADDRESS_ARD_PIN_FLOATING)?SHT31_AVAILABLE:SHT31_NOT_AVAILABLE;
  if(SHT31_NOT_AVAILABLE == _availability)
  {
    _availability = pollSensor(SHT31_ADDRESS_ADR_PIN_LOW)?SHT31_AVAILABLE:SHT31_NOT_AVAILABLE;
    _ShtAddr = SHT31_ADDRESS_ADR_PIN_LOW;
  }
  return (SHT31_AVAILABLE == _availability)?true:false;
}


bool Sht31::startMeasurementLowResolution()
{
  uint32_t error;

  uint8_t tx[2] = {(SHT31_CMD_MEAS_POLLING_L >> 8), SHT31_CMD_MEAS_POLLING_L & 0xFF};
  error = _i2c->write(_ShtAddr << 1, reinterpret_cast<char*>(tx), 2, true);

  return error;
}

bool Sht31::startMeasurementHighResolution()
{
  uint32_t error;

  uint8_t tx[2] = {(SHT31_CMD_MEAS_POLLING_H >> 8), SHT31_CMD_MEAS_POLLING_H & 0xFF};
  error = _i2c->write(_ShtAddr << 1, reinterpret_cast<char*>(tx), 2, true);

  return error;
}

bool Sht31::readMeasurement_ft(float* humi, float* temp)
{
  uint32_t error;
  int16_t temp_int16_t;
  int16_t humi_int16_t;

  error = readMeasurement_uint16_t_scale100(&humi_int16_t, &temp_int16_t);

  if (0 != error)
  {
    return true;
  }

  *temp = static_cast<float>(temp_int16_t) / 100.0f;
  *humi = static_cast<float>(humi_int16_t) / 100.0f;

  return false;
}

bool Sht31::readMeasurement_uint16_t_scale100(int16_t* humi, int16_t* temp)
{
  uint8_t rx[6];
  uint32_t error;
  uint32_t rawTemp;
  uint32_t rawHumi;

  //[TEMP_LSB][TEMP_MSB][TEMP_CRC][HUMI_LSB][HUMI_MSB][HUMI_CRC]
  error = _i2c->read(_ShtAddr << 1 | 1, reinterpret_cast<char*>(rx), 6);

  if (0 != error)
  {
    return true;
  }
  //filter Out of Spec (<2.4V) Measurements
  if (0x00 == rx[0] && 0x00 == rx[1] && 0xFF == rx[3] && 0xFF == rx[4])
  {
    return true;
  }

  rawTemp = rx[1] | (rx[0] << 8);
  rawTemp = (100 * rawTemp * 175) / 65535 - 45 * 100;
  rawHumi = rx[4] | (rx[3] << 8);
  rawHumi = (100 * rawHumi * 100) / 65535;

  *temp = static_cast<int16_t>(rawTemp);
  *humi = static_cast<int16_t>(rawHumi);

  return false;
}

bool Sht31::pollSensor(uint8_t addr)
{
  uint32_t error;

  uint8_t tx[2] = {(SHT31_CMD_READ_STATUS >> 8), SHT31_CMD_READ_STATUS & 0xFF};
  uint8_t rx[3];
  error = _i2c->write(addr << 1, reinterpret_cast<char*>(tx), 2, true);
  error |= _i2c->read(addr << 1 | 1, reinterpret_cast<char*>(rx), 3);

  if ((0 != error) || (false == _i2c->checkIfBusOk()))
  {
    return false;
  }
  return true;
}
