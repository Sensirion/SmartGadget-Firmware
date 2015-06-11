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

#include "I2c.h"

I2c::I2c(PinName sda, PinName scl): _sda(sda), _scl(scl)
{
  _sclPinRegAddr = &NRF_GPIO->PIN_CNF[static_cast<uint32_t>(scl)];
  _sdaPinRegAddr = &NRF_GPIO->PIN_CNF[static_cast<uint32_t>(sda)];
  *_sclPinRegAddr = _regValIn;
  *_sdaPinRegAddr = _regValIn;
}

int I2c::write(int address, const char* data, int length, bool repeated)
{
  bool err;
  start();
  err = write(address & 0xFE);
  for (int i = 0; i < length; i++)
  {
    err |= write(data[i]);
  }
  if (repeated == false)
  {
    stop();
  }
  else
  {
    release();
  }
  return err;
}

int I2c::write(int data)
{
  bool nack;
  for (int i = 7; i >= 0; i--)
  {
    *_sclPinRegAddr = _regValOut;
    ((data >> i & 0x01) == 0x00) ? *_sdaPinRegAddr = _regValOut : *_sdaPinRegAddr = _regValIn;
    delay1Nop();
    *_sclPinRegAddr = _regValIn;
    delay1Nop();
    delay2Nop();
  }
  *_sclPinRegAddr = _regValOut;
  *_sdaPinRegAddr = _regValIn;
  delay2Nop();
  *_sclPinRegAddr = _regValIn;
  nack = nrf_gpio_pin_read(static_cast<unsigned long>(_sda));
  *_sclPinRegAddr = _regValOut;

  return nack;
}

int I2c::read(int address, char* data, int length, bool repeated)
{
  bool err;
  start();
  err = write(address | 1);
  for (int i = 0; i < (length - 1); i++)
  {
    data[i] = read(true);
  }
  data[length - 1] = read(false);

  if (repeated == false)
  {
    stop();
  }
  else
  {
    release();
  }
  return err;
}

int I2c::read(int ack)
{
  uint8_t data = 0;
  *_sdaPinRegAddr = _regValIn;
  for (int i = 7; i >= 0; i--)
  {
    delay1Nop();
    delay1Nop();
    *_sclPinRegAddr = _regValIn;
    data |= nrf_gpio_pin_read(static_cast<unsigned long>(_sda)) << i;
    *_sclPinRegAddr = _regValOut;
  }
  (ack != 0x00) ? *_sdaPinRegAddr = _regValOut : *_sdaPinRegAddr = _regValIn;
  delay1Nop();
  *_sclPinRegAddr = _regValIn;
  delay2Nop();
  delay2Nop();
  delay2Nop();
  *_sclPinRegAddr = _regValOut;
  *_sdaPinRegAddr = _regValIn;

  return data;
}

void I2c::start(void)
{
  *_sdaPinRegAddr = _regValOut;
  delay1Nop();
  *_sclPinRegAddr = _regValOut;
  delay1Nop();
}

void I2c::stop(void)
{
  *_sdaPinRegAddr = _regValOut;
  delay1Nop();
  *_sclPinRegAddr = _regValIn;
  delay1Nop();
  *_sdaPinRegAddr = _regValIn;
  delay1Nop();
}

void I2c::release(void)
{
  *_sdaPinRegAddr = _regValIn;
  delay1Nop();
  *_sclPinRegAddr = _regValIn;
  delay1Nop();
}

void I2c::delay1Nop()
{
  __asm("NOP");
}

void I2c::delay2Nop()
{
  __asm("NOP");
  __asm("NOP");
}

bool I2c::checkIfBusOk()
{
  if (0 == nrf_gpio_pin_read(static_cast<unsigned long>(_sda)) || 0 == nrf_gpio_pin_read(static_cast<unsigned long>(_scl)))
  {
    return false;
  }
  return true;
}
