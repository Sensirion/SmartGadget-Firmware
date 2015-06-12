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

#include "Logger.h"

Logger::Logger():_loggerDownloadFinishedCbPtr(NULL), _amountOfRegisteredServices(0), _lock(false), _downloadStarted(false)
{
}

void Logger::setLoggerDownloadFinishedCallback(FunctionPointer* fp)
{
  _loggerDownloadFinishedCbPtr = fp;
}

void Logger::addServiceToLogger(ISensorValueServiceLogger* service)
{
  if (!_lock)
  {
    _registeredServices[_amountOfRegisteredServices] = service;
    _amountOfRegisteredServices++;
  }
}

void Logger::initSingleServiceMemory(uint32_t pagesPerService, int serviceNumber)
{
  uint32_t startPage;
  uint32_t endPage;
  startPage = NRF51822_FLASH_PAGES_USED + 1 + serviceNumber * pagesPerService;
  endPage = startPage + (pagesPerService - 1);
  _ringBuffers[serviceNumber] = new RingBuffer(startPage, endPage);
  _loggedValues[serviceNumber] = 0;
  _returnedValues[serviceNumber] = 0;
  _valuesToReturn[serviceNumber] = 0;
  _startValueMaxOffset[serviceNumber] = 0;
  _startValueMax[serviceNumber] = 0;
  _startValue[serviceNumber] = 0;
}

void Logger::initMemory()
{
  if (_amountOfRegisteredServices > 0 && !_lock)
  {
    uint32_t pagesPerService = (NRF51822_NUMBER_OF_FLASH_PAGES - NRF51822_FLASH_PAGES_USED) / _amountOfRegisteredServices;
    for (int i = 0; i < _amountOfRegisteredServices; i++)
    {
      initSingleServiceMemory(pagesPerService, i);
    }
  }
  _lock = true;
}

void Logger::logValue(ISensorValueServiceLogger* service, float value)
{
  uint32_t index = getServiceIndex(service);
  if (index != INDEXING_ERROR)
  {
    _loggedValues[index] = _ringBuffers[index]->write(value);
  }
}

void Logger::setStartValuePosition()
{
  for (int i = 0; i < _amountOfRegisteredServices; i++)
  {
    _startValueMax[i] = _loggedValues[i]; //remember current position
    _startValueMaxOffset[i] = 0; //reset start value offset
    _valuesToReturn[i] = 0; //reset values to return to zero
  }
}

void Logger::setStartValuePositionOffset(uint32_t offset)
{
  for (int i = 0; i < _amountOfRegisteredServices; i++)
  {
    _startValueMaxOffset[i] = offset; //remember current position
  }
}

uint32_t Logger::getAmountOfLoggedValues()
{
  uint32_t maxAmountAmongServices = 0;
  uint32_t amountSingleService = 0;
  for (int i = 0; i < _amountOfRegisteredServices; i++)
  {
    amountSingleService = (_ringBuffers[i]->getRingBufferMinSizeFt() < _loggedValues[i]) ? _ringBuffers[i]->getRingBufferMinSizeFt() : _loggedValues[i];
    if (amountSingleService > maxAmountAmongServices)
    {
      maxAmountAmongServices = amountSingleService;
    }
  }
  return maxAmountAmongServices;
}

void Logger::setAmountOfValuesToReturn(uint64_t amount)
{
  for (int i = 0; i < _amountOfRegisteredServices; i++)
  {
    _valuesToReturn[i] = amount;
  }
}

void Logger::startDownload()
{
  if (false == _downloadStarted)
  {
    for (int i = 0; i < _amountOfRegisteredServices; i++)
    {
      _startValue[i] = _startValueMax[i] - _startValueMaxOffset[i];
    }
    _downloadStarted = true;
    downloadLoggedValues(); //Force First Notification
  }
}


void Logger::cancelDownload()
{
  if (true == _downloadStarted)
  {
    for (int i = 0; i < _amountOfRegisteredServices; i++)
    {
      _returnedValues[i] = 0;
      _valuesToReturn[i] = 0;
      _startValueMaxOffset[i] = 0;
      _startValueMax[i] = 0;
      _startValue[i] = 0;
    }
    _downloadStarted = false;
  }
}

void Logger::singleLogValueNotification(uint8_t Index)
{
  ISensorValueServiceLogger::LoggedValueNotification log;
  uint8_t len = 0;
  for (int i = 0; i < (sizeof(log.values) / sizeof(log.values[0])); i++)
  {
    if (false == _ringBuffers[Index]->read(_startValue[Index] - _returnedValues[Index], &log.values[i]))
    {
      // RingBuffer Read returned false -> No data available -> end of data reached
      break;
    }
    len++;
    _returnedValues[Index]++;
    if (_valuesToReturn[Index] == _returnedValues[Index])
    {
      break;
    }
  }
  if (len > 0)
  {
    log.sequenceNumber = _returnedValues[Index] - len + 1;

    _registeredServices[Index]->notifyLoggedValue(log, len * sizeof(log.values[0]) + sizeof(log.sequenceNumber));
  }
  else //end of data reached but more requested
  {
    _returnedValues[Index] = _valuesToReturn[Index];
  }
}

void Logger::downloadLoggedValues()
{
  if (false == _downloadStarted)
  {
    return;
  }
  uint8_t nextIdx = 0;
  //find next value index to send
  for (int i = 0; i < _amountOfRegisteredServices; i++)
  {
    nextIdx = (_returnedValues[i] < _returnedValues[nextIdx]) ? i : nextIdx;
  }
  if (_valuesToReturn[nextIdx] > _returnedValues[nextIdx])
  {
    singleLogValueNotification(nextIdx);
  }
  else //download finished, clean up
  {
    _downloadStarted = false;
    for (int i = 0; i < _amountOfRegisteredServices; i++)
    {
      _returnedValues[i] = 0;
    }
    _loggerDownloadFinishedCbPtr->call();
  }
}

uint32_t Logger::getServiceIndex(ISensorValueServiceLogger* service)
{
  for (int i = 0; i < _amountOfRegisteredServices; i++)
  {
    if (service == _registeredServices[i])
    {
      return i;
    }
  }
  return INDEXING_ERROR;
}

void Logger::reset()
{
  for (int i = 0; i < _amountOfRegisteredServices; i++)
  {
    _loggedValues[i] = 0;
    _valuesToReturn[i] = 0;
    _returnedValues[i] = 0;
    _startValue[i] = 0;
    _startValueMax[i] = 0;
    _startValueMaxOffset[i] = 0;
    _ringBuffers[i]->reset();
  }
}
