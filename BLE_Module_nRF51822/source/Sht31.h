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

#ifndef SHT31_H
#define SHT31_H

#include "mbed.h"
#include "I2c.h"

/**
* @class Sht31
* @brief A Sensirion SHT31 Humidity & Temperature Sensor Abstraction class.<br>
**/
class Sht31
{
public:
  /** Create a Sht31
     *
     *  @param i2c reference of the i2c interface the sensor is attached to
     */
  Sht31(I2c* i2c);
  /** Check if a sensor is available. After an initial (physical) check, 
     * the presence is stored within a member and subsequent calls return 
     * that members value.
     *
     *  @returns
     *    an boolean representing the availability of a Sht31
     *        0 no sensor is available
     *    non-0 a sensor is available
     */
  bool isAvailable();
  /** Start a low resolution humidity and temperature measurement on the 
     * Sht31 sensor
     *
     *  @returns
     *    an boolean representing the success of the command
     *        0 command executed successfully
     *    non-0 there was an i2c error during communication
     */
  bool startMeasurementLowResolution();
  /** Start a high resolution humidity and temperature measurement on the 
     * Sht31 sensor
     *
     *  @returns
     *    an boolean representing the success of the command
     *        0 command executed successfully
     *    non-0 there was an i2c error during communication
     */
  bool startMeasurementHighResolution();
  /** Read out the measurement data of a previously started measurement
     *
     *  @param humi reference of a float where the humidity will be stored 
     *  in case of success
     *  @param temp reference of a float where the temperature will be 
     *  stored in case of success
     *  @returns
     *    an boolean representing the success of the command
     *        0 command executed successfully
     *    non-0 there was an i2c error during communication
     */
  bool readMeasurement_ft(float* humi, float* temp);

private:
  typedef enum
  {
    SHT31_UNKNOWN,
    SHT31_AVAILABLE,
    SHT31_NOT_AVAILABLE,
  } Sht31Availability;

  enum privateConstants
  {
    SHT31_ADDRESS_ADR_PIN_LOW = 0x45,
    SHT31_ADDRESS_ARD_PIN_FLOATING = 0x44,
    SHT31_CMD_READ_STATUS = 0xF32D,
    SHT31_CMD_MEAS_POLLING_H = 0x2400,
    SHT31_CMD_MEAS_POLLING_L = 0x2416,
  };

  Sht31();
  bool readMeasurement_uint16_t_scale100(int16_t* humi, int16_t* temp);
  bool pollSensor(uint8_t addr);

  I2c* _i2c;
  uint8_t _ShtAddr;
  Sht31Availability _availability;
};

#endif
