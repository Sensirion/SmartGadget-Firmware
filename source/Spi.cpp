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

#include "Spi.h"

Spi::Spi(PinName mosi, PinName miso, PinName sck):
  _mosiPin(mosi), _misoPin(miso), _sckPin(sck), _mosiPinPos(1 << mosi), _misoPinPos(1 << miso), _sckPinPos(1 << sck)
{
  nrf_gpio_cfg_output(_mosiPin);
  nrf_gpio_cfg_output(_sckPin);
  nrf_gpio_cfg_input(_misoPin, NRF_GPIO_PIN_NOPULL);
}

void Spi::format(int bits, int mode)
{
}

int Spi::write(int value)
{
  int retval = 0;
  NRF_GPIO->OUTCLR = _sckPinPos;
  //value & 0x80 = most significant bit
  (value & 0x80) ? NRF_GPIO->OUTSET = _mosiPinPos : NRF_GPIO->OUTCLR = _mosiPinPos;;
  value = value << 1;
  NRF_GPIO->OUTSET = _sckPinPos;
  retval |= nrf_gpio_pin_read(static_cast<unsigned long>(_misoPin));
  retval = retval << 1;

  NRF_GPIO->OUTCLR = _sckPinPos;
  (value & 0x80) ? NRF_GPIO->OUTSET = _mosiPinPos : NRF_GPIO->OUTCLR = _mosiPinPos;;
  value = value << 1;
  NRF_GPIO->OUTSET = _sckPinPos;
  retval |= nrf_gpio_pin_read(static_cast<unsigned long>(_misoPin));
  retval = retval << 1;

  NRF_GPIO->OUTCLR = _sckPinPos;
  (value & 0x80) ? NRF_GPIO->OUTSET = _mosiPinPos : NRF_GPIO->OUTCLR = _mosiPinPos;;
  value = value << 1;
  NRF_GPIO->OUTSET = _sckPinPos;
  retval |= nrf_gpio_pin_read(static_cast<unsigned long>(_misoPin));
  retval = retval << 1;

  NRF_GPIO->OUTCLR = _sckPinPos;
  (value & 0x80) ? NRF_GPIO->OUTSET = _mosiPinPos : NRF_GPIO->OUTCLR = _mosiPinPos;;
  value = value << 1;
  NRF_GPIO->OUTSET = _sckPinPos;
  retval |= nrf_gpio_pin_read(static_cast<unsigned long>(_misoPin));
  retval = retval << 1;

  NRF_GPIO->OUTCLR = _sckPinPos;
  (value & 0x80) ? NRF_GPIO->OUTSET = _mosiPinPos : NRF_GPIO->OUTCLR = _mosiPinPos;;
  value = value << 1;
  NRF_GPIO->OUTSET = _sckPinPos;
  retval |= nrf_gpio_pin_read(static_cast<unsigned long>(_misoPin));
  retval = retval << 1;

  NRF_GPIO->OUTCLR = _sckPinPos;
  (value & 0x80) ? NRF_GPIO->OUTSET = _mosiPinPos : NRF_GPIO->OUTCLR = _mosiPinPos;;
  value = value << 1;
  NRF_GPIO->OUTSET = _sckPinPos;
  retval |= nrf_gpio_pin_read(static_cast<unsigned long>(_misoPin));
  retval = retval << 1;

  NRF_GPIO->OUTCLR = _sckPinPos;
  (value & 0x80) ? NRF_GPIO->OUTSET = _mosiPinPos : NRF_GPIO->OUTCLR = _mosiPinPos;;
  value = value << 1;
  NRF_GPIO->OUTSET = _sckPinPos;
  retval |= nrf_gpio_pin_read(static_cast<unsigned long>(_misoPin));
  retval = retval << 1;

  NRF_GPIO->OUTCLR = _sckPinPos;
  (value & 0x80) ? NRF_GPIO->OUTSET = _mosiPinPos : NRF_GPIO->OUTCLR = _mosiPinPos;;
  value = value << 1;
  NRF_GPIO->OUTSET = _sckPinPos;
  retval |= nrf_gpio_pin_read(static_cast<unsigned long>(_misoPin));

  return retval;
}

void Spi::enable()
{
}

void Spi::disable()
{
}
