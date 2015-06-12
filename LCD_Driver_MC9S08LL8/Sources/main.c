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

#include "System.h"
#include "LCD.h"
#include "SPI.h"

void main(void) 
{
  // initialize all components 
  SYS_Init(); 
  LCD_Init();
  SPI_Init();
  
  EnableInterrupts; // enable interrupts
    
  while(1)          // loop forever
  {
  	__asm STOP;     // Stop for saving Power
  } 
}


// CS_ISR: uC is waked up with neg. slope on CS
interrupt VectorNumber_Vkeyboard void CS_ISR(void)
{
  u8t ReceivedByte[10];     // received Bytes
  
  if(PTAD_PTAD0==0)// check for CS low
  {
    // get new Data for display
    SPI_Receive(ReceivedByte);
    
    // update LCD with new Data
    LCD_Update(ReceivedByte);
    
    KBISC_KBACK = 1;  // clear interrupt
    PTCD_PTCD7 = 0;   // send Ready Signal
  }
  else
  {
    KBISC_KBACK = 1;  // clear interrupt
  }
}

