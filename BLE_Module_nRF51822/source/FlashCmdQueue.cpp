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

#include "FlashCmdQueue.h"

bool FlashCmdQueue::_flashOperationPending = false;
uint8_t FlashCmdQueue::_tail = 0;
uint8_t FlashCmdQueue::_lastIndexUsed = 0;
FlashCmdQueue::FlashCommand FlashCmdQueue::_flashCommands[] = {0};

void FlashCmdQueue::init()
{
  for (int i = 0; i < MAX_QUEUED_COMMANDS; i++)
  {
    _flashCommands[i].command = COMMAND_EMPTY;
  }
  _tail = COMMAND_NEXT_NONE;
  _flashOperationPending = false;
}

void FlashCmdQueue::addFlashCommandToQueue(uint8_t idx)
{
  if (_tail == COMMAND_NEXT_NONE)
  {
    _tail = idx;
  }
  else
  {
    uint8_t nextIndex = _tail;
    while (true)
    {
      if (COMMAND_NEXT_NONE == _flashCommands[nextIndex].next)
      {
        break;
      }
      nextIndex = _flashCommands[nextIndex].next;
    }
    _flashCommands[nextIndex].next = idx;
  }
}

void FlashCmdQueue::pushInternal(uint32_t pageToErase, uint32_t* location, float val)
{
  uint8_t idx = (_lastIndexUsed == (MAX_QUEUED_COMMANDS - 1)) ? 0 : _lastIndexUsed + 1;
  uint8_t cnt = 0;
  while (true)
  {
    if (COMMAND_EMPTY == _flashCommands[idx].command)
    {
      _lastIndexUsed = idx;
      _flashCommands[idx].command = (NULL == location) ? COMMAND_PAGE_ERASE : COMMAND_FLASH_WRITE;
      _flashCommands[idx].location = location;
      _flashCommands[idx].pageToErase = pageToErase;
      _flashCommands[idx].value = val;
      _flashCommands[idx].next = COMMAND_NEXT_NONE;
      addFlashCommandToQueue(idx);
      break;
    }
    idx = (idx == (MAX_QUEUED_COMMANDS - 1)) ? 0 : idx + 1;
    cnt++;
    if (cnt == MAX_QUEUED_COMMANDS)
    {
      while (true);
    }
  }
  //directly execute if no ongoing flash operation pending
  if (false == _flashOperationPending)
  {
    popAndExecute();
  }
}

void FlashCmdQueue::popAndExecute(void)
{
  if (COMMAND_NEXT_NONE != _tail)
  {
    _flashOperationPending = true;
    switch (_flashCommands[_tail].command)
    {
    case COMMAND_FLASH_WRITE:
      while (NRF_ERROR_BUSY == sd_flash_write(_flashCommands[_tail].location, reinterpret_cast<uint32_t*>(&_flashCommands[_tail].value), 1));
      break;
    case COMMAND_PAGE_ERASE:
      while (NRF_ERROR_BUSY == sd_flash_page_erase(_flashCommands[_tail].pageToErase));
      break;
    default: //should never reach
      break;
    }
    //clean up executed command
    _flashCommands[_tail].command = COMMAND_EMPTY;
    //set new next pointer
    _tail = _flashCommands[_tail].next;
  }
  else
  {
    _flashOperationPending = false;
  }
}
