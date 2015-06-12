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

#ifndef I2C_H
#define I2C_H

#include "nrf_gpio.h"
#include "mbed.h"

/**
* @class I2c
* @brief An I2C Master, used for communicating with I2C slave devices using 
* normal IO's<br>
**/
class I2c
{
public:
  /** Create an I2c Master interface, connected to the specified pins
     *
     *  @param sda I2c data line pin
     *  @param scl I2c clock line pin
     */
  I2c(PinName sda, PinName scl);

  /** Read from an I2c slave
     *
     * Performs a complete read transaction. The bottom bit of
     * the address is forced to 1 to indicate a read.
     *
     *  @param address 8-bit I2c slave address [ addr | 1 ]
     *  @param data Pointer to the byte-array to read data in to
     *  @param length Number of bytes to read
     *  @param repeated Repeated start, true - don't send stop at end
     *
     *  @returns
     *       0 on success (ack),
     *   non-0 on failure (nack)
     */
  int read(int address, char* data, int length, bool repeated = false);

  /** Write to an I2c slave
     *
     * Performs a complete write transaction. The bottom bit of
     * the address is forced to 0 to indicate a write.
     *
     *  @param address 8-bit I2c slave address [ addr | 0 ]
     *  @param data Pointer to the byte-array data to send
     *  @param length Number of bytes to send
     *  @param repeated Repeated start, true - do not send stop at end
     *
     *  @returns
     *       0 on success (ack),
     *   non-0 on failure (nack)
     */
  int write(int address, const char* data, int length, bool repeated = false);
  /** Checks if the bus is OK
     *
     * Probes the SDA/ SCL line and expects a high level on bothat end
     *
     *  @returns
     *   non-0 on success (ack),
     *       0 on failure (nack)
     */
  bool checkIfBusOk();

private:

  int read(int ack);
  int write(int data);
  void start(void);
  void release(void);
  void stop(void);
  inline void delay1Nop(void);
  inline void delay2Nop(void);

  PinName _sda;
  PinName _scl;
  volatile uint32_t* _sclPinRegAddr;
  volatile uint32_t* _sdaPinRegAddr;
  //configuration register value for I2C output pin
  static const uint32_t _regValOut = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
  | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
  | (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
  | (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
  | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
  //configuration register value for I2C input pin
  static const uint32_t _regValIn = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
  | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
  | (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
  | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
  | (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);
};

#endif
