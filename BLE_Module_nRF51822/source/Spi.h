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

#ifndef SPI_H
#define SPI_H

#include "mbed.h"
#include "nrf_gpio.h"

/**
* @class ShtLcd
* @brief This is a custom SPI API using the mbed SPI HAL to provide an SPI 
* interface on any Pin.<br>
**/
class Spi
{
public:

  /** Create a Spi Interface
     *
     *  @param mosi MOSI Pin (from PinName)
     *  @param miso MISO Pin (from PinName)
     *  @param sck SCK Pin (from PinName)
     */
  Spi(PinName mosi, PinName miso, PinName sck);
  /** Set the SPI mode. This method is NOT implemented and has no effect!
     *
     *  @param bits Number of bits per SPI frame (4 - 16)
     *  @param mode Clock polarity and phase mode (0 - 3)
     *
     * @code
     * mode | POL PHA
     * -----+--------
     *   0  |  0   0
     *   1  |  0   1
     *   2  |  1   0
     *   3  |  1   1
     * @endcode
     */
  void format(int bits, int mode = 0);

  /** Write to the SPI Slave and return the response
     *
     *  @param value Data to be sent to the SPI slave
     *
     *  @returns
     *    Response from the SPI slave
    */
  int write(int value);
  /** Enables the SPI Interface
     *
    */
  void enable();
  /** Disables the SPI Interface. Useful before Sleep.
     *
    */
  void disable();

private:
  Spi();
  PinName _mosiPin;
  PinName _misoPin;
  PinName _sckPin;
  uint32_t _mosiPinPos;
  uint32_t _misoPinPos;
  uint32_t _sckPinPos;
};

#endif
