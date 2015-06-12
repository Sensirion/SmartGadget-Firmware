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

#include "system.h"

void SYS_Init (void)
{
  // Disable pins for debugging if not used
  #ifdef DEBUG           
    SOPT1 =  0b01100011; //disable cop, enable stop, enabled Reset pin and Background pin
  #else
    SOPT1 =  0b01100000; //disable cop, enable stop, disable Reset pin and Background pin
  #endif

  // Init System  
  ICSC2 = 0b00000111; // 8MHz Clk, ext clock
  ICSC1 = 0b00000000; // FLL, ext clock
  SPMSC1 = 0x00;      // LVD disabled
  SPMSC2 = 0x01;      // LVD disabled
  SCGC1 = 0b00000000; // disable clock from all unused modules
  SCGC2 = 0b00010010;
  
  // initialization of ports
  PTAD  = 0b00000000; // Set all Ports to low level
  PTBD  = 0b00000000; // Set all Ports to low level
  PTCD  = 0b00000000; // Set all Ports to low level
  
  PTADD = 0b11110100; // Port A as Output, PTA0/1/3 as Input
  PTBDD = 0b11111111; // Port B as Output
  PTCDD = 0b11111111; // Port C as Output
  
  // init CS pin
  PTADD_PTADD0 = 0;   // set port as input
  KBIPE_KBIPE0 = 1;   // enable KBIP0
  KBIES_KBEDG0 = 0;   // enable falling edge
  KBISC_KBACK = 1;    // clear interrupts
  KBISC_KBIE = 1;     // enable interrupt
}

