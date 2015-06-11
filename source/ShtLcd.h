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

#ifndef DISPLAY_H
#define DISPLAY_H

//---------- Includes ----------------------------------------------------------
#include "mbed.h"
#include "Spi.h"

/**
* @class ShtLcd
* @brief This class handles the communication over spi with the LDC Driver 
* (custom MCU implementation)<br>
**/
class ShtLcd
{
public:
  enum publicConstants
  {
    DIGITR = 16,
    DIGITO = 17,
    DIGITN = 18,
    DIGITNEGATIV = 19,
    DIGITL = 20,
    SYMBOL_SENSI = 0,
    SYMBOL_NEG1 = 1,
    SYMBOL_NEG2 = 2,
    SYMBOL_LOWBATT = 3,
    SYMBOL_DEWPOINT = 4,
    SYMBOL_LPMODE = 5,
    SYMBOL_RH = 6,
    SYMBOL_C1 = 7,
    SYMBOL_F1 = 8,
    SYMBOL_C2 = 9,
    SYMBOL_F2 =10,
    SYMBOL_BLE =11,
    SYMBOL_BLEBLINK =12
  };

  // Connection to the display																		
  typedef struct
  {
    PinName MOSI;
    PinName MISO;
    PinName SCK;
    PinName CS;
    PinName RDY;
  } ShtLcdPins;

  /** Create a ShtLcd. This Function initialize the Pins for SPI, CS and 
     * RDY signals
     *
     *  @param pPins reference to the ShtLcdpins
     */
  ShtLcd(ShtLcdPins* pPins);

  /** Set number and decimalpoint on Line 1 in Buffer and manage the zeros 
     * before and after decimalpoint.<br>
     * examples: aDecs = 2, aNumber = 3			->  _0.03<br>
     * 					aDecs = 1, aNumber = 3			->  __0.3<br>
     *					aDecs = 2, aNumber = -30  	->  -0.30<br>
     *					aDecs = 2, aNumber = -1234  -> -12.34
     *
     *  @param aDecs Number of digits after decimalpoint
     *  @param aNumber Number for display, Range: -9999...9999
     */
  void setLine1(uint8_t aDecs, int32_t aNumber);

  /** Set number and decimalpoint on Line 2 in Buffer and manage the zeros 
     * before and after decimalpoint.<br>
     * examples: aDecs = 2, aNumber = 3			->  _0.03<br>
     * 					aDecs = 1, aNumber = 3			->  __0.3<br>
     *					aDecs = 2, aNumber = -30  	->  -0.30<br>
     *					aDecs = 2, aNumber = -1234  -> -12.34
     *
     *  @param aDecs Number of digits after decimalpoint
     *  @param aNumber Number for display, Range: -9999...9999
     */
  void setLine2(uint8_t aDecs, int32_t aNumber);

  /** set one digit on the given position in Buffer:<br>
     *  0 1 2 3<br>
     * 	4 5 6 7
     *
     *  @param aPosition position of the digit
     *  @param aDigit  Number or Charakter for set<br>
     *  Range: 0-9, A, b, C, d, E, F, <br>
     *  DIGITR:       r<br>
     *  DIGITO:       o<br>
     *  DIGITN:       n<br>
     *  DIGITNEGATIV: - <br>
     *  DIGITL:       L<br>
     */
  void setDigit(uint8_t aPosition, uint8_t aDigit);

  /** set the decimal point on given position:<br>
     *  0 1 2 (3)<br>
     *  4 5 6 (7)
     *
     *  @param aPosition Valid Range: 0-2, 4-6
     */
  void setDecimalPoint(uint8_t aPosition);

  /** set the given Symbol on display
     *
     *  @param aSymbol Enable Symbol 	
     */
  void setSymbol(uint8_t aSymbol);

  /** Write Data over the SPI Interface to ShtLcd, make all modification before 
     * this function
     * 	
     */
  void update();

  /** Clears the ShtLcd buffer
     * 	
     */
  void clear();

  /** This Function returns the LCD Driver's FW Version, is updated with 
     * Update()
     *
     *  @param version reference to the version (size = 2)
     */
  void getVersion(uint8_t* version);

private:
  void setLine(uint8_t aLine, uint8_t aDecs, int32_t aNumber);
  // lookup table for segments, bit mapping: BGCPAFED
  const static uint8_t digitLookup[];
  Spi* _spi;
  DigitalOut* _cs;
  DigitalIn* _rdy;
  uint8_t _ShtLcdData[10]; // Buffer for ShtLcd
  uint8_t _version[2];
};

#endif
