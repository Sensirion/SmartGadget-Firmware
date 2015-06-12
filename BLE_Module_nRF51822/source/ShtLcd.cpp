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

#include "ShtLcd.h"
#include <stdlib.h>
#include <string.h>

const uint8_t ShtLcd::digitLookup[] = {0xAF, // 0 
  0xA0, // 1
  0xCB, // 2
  0xE9, // 3
  0xE4, // 4
  0x6D, // 5
  0x6F, // 6
  0xA8, // 7
  0xEF, // 8
  0xED, // 9
  0xEE, // A
  0x67, // b
  0x0F, // c
  0xE3, // d
  0x4F, // e
  0x4E, // f
  0x42, // 16 r
  0x63, // 17 o
  0x62, // 18 n
  0x40, // 19 for neg digit
  0x07, // 20 L
};

ShtLcd::ShtLcd(ShtLcdPins* pPins)
{
  _spi = new Spi(pPins->MOSI, pPins->MISO, pPins->SCK);
  _spi->format(8, 3);
  _spi->disable();
  _cs = new DigitalOut(pPins->CS);
  _rdy = new DigitalIn(pPins->RDY);
  _rdy->mode(PullNone);
  *_cs = 1; // init CS
}

void ShtLcd::clear()
{
  for (uint8_t i = 0; i < 10; i++)
    _ShtLcdData[i] = 0x00;
}

void ShtLcd::setLine1(uint8_t aDecs, int32_t aNumber)
{
  setLine(0, aDecs, aNumber);
  
}

void ShtLcd::setLine2(uint8_t aDecs, int32_t aNumber)
{
  setLine(1, aDecs, aNumber);
}

void ShtLcd::setDigit(uint8_t aPosition, uint8_t aNumber)
{
  _ShtLcdData[aPosition] |= digitLookup[aNumber];
}

void ShtLcd::setDecimalPoint(uint8_t aPosition)
{
  _ShtLcdData[aPosition] |= 0x10;
}

void ShtLcd::setSymbol(uint8_t aSymbol)
{
  switch (aSymbol)
  {
  case SYMBOL_BLE: _ShtLcdData[8] |= 0x08;
    break;
  case SYMBOL_SENSI: _ShtLcdData[8] |= 0x04;
    break;
  case SYMBOL_NEG1: _ShtLcdData[8] |= 0x02;
    break;
  case SYMBOL_NEG2: _ShtLcdData[8] |= 0x01;
    break;
  case SYMBOL_LOWBATT: _ShtLcdData[7] |= 0x10;
    break;
  case SYMBOL_DEWPOINT: _ShtLcdData[3] |= 0x10;
    break;
  case SYMBOL_LPMODE: _ShtLcdData[9] |= 0x80;
    break;
  case SYMBOL_RH: _ShtLcdData[9] |= 0x40;
    break;
  case SYMBOL_C1: _ShtLcdData[9] |= 0x20;
    break;
  case SYMBOL_F1: _ShtLcdData[9] |= 0x08;
    break;
  case SYMBOL_C2: _ShtLcdData[9] |= 0x10;
    break;
  case SYMBOL_F2: _ShtLcdData[8] |= 0x10;
    break;
  case SYMBOL_BLEBLINK: _ShtLcdData[8] |= 0x88;
    break;
  default: break;
  }
}

void ShtLcd::update()
{
  _spi->enable();
  *_cs = 0;
  while (*_rdy == 0);
  _version[0] = _spi->write(_ShtLcdData[0]);
  _version[1] = _spi->write(_ShtLcdData[1]);
  for (uint8_t i = 2; i < 10; i++)
    _spi->write(_ShtLcdData[i]);
  *_cs = 1;
  _spi->disable();
}

void ShtLcd::getVersion(uint8_t* version)
{
  version[0] = _version[0];
  version[1] = _version[1];
}
void ShtLcd::setLine(uint8_t aLine, uint8_t aDecs, int32_t aNumber)
{
  uint8_t temp;
  uint8_t EnableZeros = 0;
  uint8_t xIsNeg = 0;
  uint8_t LineOffset = 4 * aLine;

  // set DP
  if (aDecs != 0) setDecimalPoint(3 + LineOffset - aDecs);

  // limit Number
  if (aNumber > 9999)aNumber = 9999;
  if (aNumber < -9999)aNumber = -9999;

  if (aNumber < 0)// is number negative?
  {
    xIsNeg = 1;
    aNumber = 0 - aNumber; // make positive number
  }

  // Digit 0
  temp = aNumber / 1000;
  if ((temp != 0) || EnableZeros || (aDecs >= 3))
  {
    if (xIsNeg)// write Neg once
    {
      setSymbol(aLine == 0?SYMBOL_NEG1:SYMBOL_NEG2);
      xIsNeg = 0;
    }
    setDigit(0 + LineOffset, temp);
    EnableZeros = 1;
  }

  // Digit 1
  aNumber = aNumber % 1000;
  temp = aNumber / 100;
  if ((temp != 0) || EnableZeros || (aDecs >= 2))
  {
    if (xIsNeg)// write Neg once
    {
      setDigit(0 + LineOffset, DIGITNEGATIV);
      xIsNeg = 0;
    }
    setDigit(1 + LineOffset, temp);
    EnableZeros = 1;
  }

  // Digit 2
  aNumber = aNumber % 100;
  temp = aNumber / 10;
  if ((temp != 0) || EnableZeros || (aDecs >= 1))
  {
    if (xIsNeg)// write Neg once
    {
      setDigit(1 + LineOffset, DIGITNEGATIV);
      xIsNeg = 0;
    }
    setDigit(2 + LineOffset, temp);
    EnableZeros = 1;
  }

  // Digit 3
  temp = aNumber % 10;
  if (xIsNeg)// write Neg
  {
    setDigit(2 + LineOffset, DIGITNEGATIV);
  }
  setDigit(3 + LineOffset, temp);
}
