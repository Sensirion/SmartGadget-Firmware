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

#include "LCD.h"

void LCD_Init(void)
{
  
  // init LCD
  //Enable all LCD Pins 
  LCDPEN0 = 0xFF;
  LCDPEN1 = 0xFF;
  LCDPEN2 = 0xFF;
  LCDPEN3 = 0xFF;

  //Enable Back Plane Pins
  LCDBPEN0 = 0x03;   // LCD0+1 as Backplane
  LCDBPEN1 = 0x30;   // LCD12+13 as Backplane
  LCDBPEN2 = 0x00;
  LCDBPEN3 = 0x00;

  //Set Back Planes
  LCDWF0  = 0x88;  // LCD0  as COM3
  LCDWF1  = 0x11;  // LCD1  as COM0
  LCDWF12 = 0x22;  // LCD12 as COM1
  LCDWF13 = 0x44;  // LCD13 as COM2
  
  // Enable all segment for Startup
  LCDWF31 = 0xFF;  
  LCDWF30 = 0xFF;
  LCDWF29 = 0xFF;
  LCDWF28 = 0xFF;
  LCDWF27 = 0xFF;
  LCDWF26 = 0xFF;
  LCDWF25 = 0xFF;
  LCDWF24 = 0xFF;
  LCDWF15 = 0xFF;
  LCDWF14 = 0xFF;
  LCDWF11 = 0xFF;
  LCDWF10 = 0xFF;
  LCDWF9  = 0xFF;
  LCDWF8  = 0xFF;
  LCDWF7  = 0xFF;
  LCDWF6  = 0xFF;
  LCDWF5  = 0xFF;
  LCDWF4  = 0xFF;
  LCDWF3  = 0xFF;
  LCDWF2  = 0xFF;
 
  // LCD Control registers
  LCDRVC    = 0x8b;
  LCDSUPPLY = 0x83;
  LCDC0     = 0xA3;
  
  // Init Blink option
  LCDBCTL = 0b00001100; // Blink Frequency = 0.5Hz

}

void LCD_Update(u8t aData[10])
{

  if(aData[8]&0x80)   // check for blink status
  {
    aData[8]&=~0x80;  // clear Blink Status in Data
    
    // copy Data to LCD register
    LCDWF14  =aData[0]&0x0F;
    LCDWF15  =aData[0]>>4;
    LCDWF24  =aData[1]&0x0F;
    LCDWF25  =aData[1]>>4;
    LCDWF26  =aData[2]&0x0F;
    LCDWF27  =aData[2]>>4;
    LCDWF28  =aData[3]&0x0F;
    LCDWF29  =aData[3]>>4;
    LCDWF10  =aData[4]&0x0F;
    LCDWF9   =aData[4]>>4;
    LCDWF8   =aData[5]&0x0F;
    LCDWF7   =aData[5]>>4;
    LCDWF6   =aData[6]&0x0F;
    LCDWF5   =aData[6]>>4;
    LCDWF4   =aData[7]&0x0F;
    LCDWF3   =aData[7]>>4;
    LCDWF11  =aData[8]&0x0F;
    LCDWF2   =aData[8]>>4;
    LCDWF31  =aData[9]&0x0F;
    LCDWF30  =aData[9]>>4;
    
    // copy Blink State
    LCDWF14   |= aData[0] << 4;
    LCDWF15   |= aData[0] &0xF0;
    LCDWF24   |= aData[1] << 4;
    LCDWF25   |= aData[1] &0xF0;
    LCDWF26   |= aData[2] << 4;
    LCDWF27   |= aData[2] &0xF0;
    LCDWF28   |= aData[3] << 4;
    LCDWF29   |= aData[3] &0xF0;
    LCDWF10   |= aData[4] << 4;
    LCDWF9    |= aData[4] &0xF0;
    LCDWF8    |= aData[5] << 4;
    LCDWF7    |= aData[5] &0xF0;
    LCDWF6    |= aData[6]  << 4;
    LCDWF5    |= aData[6] &0xF0;
    LCDWF4    |= aData[7]  << 4;
    LCDWF3    |= aData[7] &0xF0;
    LCDWF11   |= aData[8]  << 4;
    LCDWF2    |= aData[8] &0xF0;
    LCDWF31   |= aData[9] << 4;
    LCDWF30   |= aData[9]&0xF0;
    
    LCDWF11 &=~0b10000000; // Clear BLE symbol in Blinkstate

    LCDBCTL_BLINK = 1;    // Enable Blink
  }
  else
  {    
    LCDBCTL_BLINK = 0;    // Disable Blink
    
    // copy Data to LCD register
    LCDWF14  = aData[0];
    LCDWF15  = aData[0]>>4;
    LCDWF24  = aData[1];
    LCDWF25  = aData[1]>>4;
    LCDWF26  = aData[2];
    LCDWF27  = aData[2]>>4;
    LCDWF28  = aData[3];
    LCDWF29  = aData[3]>>4;
    LCDWF10  = aData[4];
    LCDWF9   = aData[4]>>4;
    LCDWF8   = aData[5];
    LCDWF7   = aData[5]>>4;
    LCDWF6   = aData[6];
    LCDWF5   = aData[6]>>4;
    LCDWF4   = aData[7];
    LCDWF3   = aData[7]>>4;
    LCDWF11  = aData[8];
    LCDWF2   = aData[8]>>4;
    LCDWF31  = aData[9];
    LCDWF30  = aData[9]>>4;
  }
}

