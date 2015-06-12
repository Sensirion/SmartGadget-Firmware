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

#include "SPI.h"

void SPI_Init ()
{
  SCGC2_SPI = 1;        // enable SPI CLK
  SPIC1 = 0b00001100;   // SPI disabled, Slave Mode, CPOL=1, CPHA=1, MSB first
  SPIC2 = 0b00000000;
  SPIBR = 0b00000000; 
  KBISC_KBACK = 1;      // clear interrupt from CS
}

void SPI_Receive(u8t pData[10])
{
  SPIC1_SPE = 1;  // enable SPI
  
  while (SPIS_SPTEF==0);  // wait for empty buffer
  SPID = SW_VERSION_MAJOR;// send Byte 0
  
  PTCD_PTCD7 = 1;         // send Ready Signal
  
  while (SPIS_SPTEF==0);  // wait for empty buffer
  SPID = SW_VERSION_MINOR;// send Byte 1
  
  
  while (SPIS_SPRF==0);   // wait for full read Buffer
  pData[0] = SPID;        // read Byte 0
  
  while (SPIS_SPRF==0);   // wait for full read Buffer
  pData[1] = SPID;        // read Byte 1
  
  while (SPIS_SPRF==0);   // wait for full read Buffer
  pData[2] = SPID;        // read Byte 2
  
  while (SPIS_SPRF==0);   // wait for full read Buffer
  pData[3] = SPID;        // read Byte 3
  
  while (SPIS_SPRF==0);   // wait for full read Buffer
  pData[4] = SPID;        // read Byte 4
  
  while (SPIS_SPRF==0);   // wait for full read Buffer
  pData[5] = SPID;        // read Byte 5
  
  while (SPIS_SPRF==0);   // wait for full read Buffer
  pData[6] = SPID;        // read Byte 6
  
  while (SPIS_SPRF==0);   // wait for full read Buffer
  pData[7] = SPID;        // read Byte 7
  
  while (SPIS_SPRF==0);   // wait for full read Buffer
  pData[8] = SPID;        // read Byte 8
  
  while (SPIS_SPRF==0);   // wait for full read Buffer
  pData[9] = SPID;        // read Byte 9
  
  SPIC1_SPE = 0;          // Disable SPI
  
  return;
}
