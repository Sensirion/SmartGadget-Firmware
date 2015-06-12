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

#ifndef FLASHCMDQUEUE_H
#define FLASHCMDQUEUE_H

#include "nrf_soc.h"
#include "nrf_delay.h"
#include "mbed.h"

/**
* @class FlashCmdQueue
* @brief This static class handles the embedded flash operations, which are 
* executed async by the softdevice. Multiple Flash commands can be queued and 
* are executed one after an other (FIFO)<br>
* @note important note on sd_flash_write command: The value that has to be 
* written to flash is passed as a pointer. Make sure that the values remain 
* valid for at least 1ms after(!) the NRF_EVT_FLASH_OPERATION_SUCCESS Event 
* is throwed by the softdevice.<br>
**/
class FlashCmdQueue
{
public:
  /** Initialize the flash command queue
     *
     */
  static void init();

  /** push a new flash command to the queue (page erase)
     *
     *  @param pageToErase The flash page that needs to be erased
     */
  static inline void push(uint32_t pageToErase)
  {
    pushInternal(pageToErase, NULL, 0);
  };

  /** push a new flash command to the queue (overloaded: write float to flash)
     *
     *  @param location The flash location
     *  @param val the value to be written to this location
     */
  static inline void push(uint32_t* location, float val)
  {
    pushInternal(0, location, val);
  };

  /** Pops a command from the queue and executes it
     *
     */
  static void popAndExecute();

private:
  enum privateConstants
  {
    COMMAND_PAGE_ERASE = 0, //command enum value 1
    COMMAND_FLASH_WRITE = 1,//command enum value 2
    COMMAND_NEXT_NONE = 0xFF,//command enum none/empty
    COMMAND_EMPTY = 0xFF,
    MAX_QUEUED_COMMANDS = 5 //can be increased if needed (e.g. if many sensors attached)
  };

  typedef struct
  {
    uint8_t next;
    uint8_t command;
    uint32_t pageToErase;
    uint32_t* location;
    float value;
  } FlashCommand;

  static void pushInternal(uint32_t pageToErase, uint32_t* location, float val);
  static void addFlashCommandToQueue(uint8_t idx);

  static bool _flashOperationPending;
  static FlashCommand _flashCommands[MAX_QUEUED_COMMANDS];
  static uint8_t _tail;
  static uint8_t _lastIndexUsed;
};

#endif
