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

#ifndef LOGGER_H
#define LOGGER_H

#include "mbed.h"
#include "RingBuffer.h"
#include "ISensorValueServiceLogger.h"

/**
* @class Logger
* @brief A Logger class, designed to work with Bluetooth Low Energy Services 
* as Data Provider and Ring Buffers for storage<br>
* @details Usage:<br>
* ->Add Services where you want to log a characteristic to the logger with addServiceToLogger<br>
* ->(optional) Register a callback that is called after the logger is read out completely
* with setLoggerDownloadFinishedCallback<br>
* ->Initialize the available flash memory with initMemory (distributed equally)<br>
* ->Log values using logValue<br>
* ->To initialize a download, several settings have to be made:<br>
*     ->setStartValuePosition makes the logger remember the current position as "latest" value
*     ->(optional)setStartValuePositionOffset adds an offset (to treat a older value as "latest")
*     ->(optional)getAmountOfLoggedValues get the amount of available values to return
*     ->setAmountOfValuesToReturn how many values have to be returned ("latest" is returned first)
*     ->startDownload 
**/
class Logger
{
public:
  /** Create a Logger
     *
     */
  Logger();
  /** Register a Bluetooth Low Energy service to the logger.
     *
     *  @param service Bluetooth Low Energy service reference. Has to implement
     *  ISensorValueServiceLogger Interface
     */
  void addServiceToLogger(ISensorValueServiceLogger* service);
  /** Set a callback that will be called after the logger was read out 
     * completely.
     *
     *  @param fp FunctionPointer can be static or member
     */
  void setLoggerDownloadFinishedCallback(FunctionPointer* fp);
  /** Initialize the memory (equally shared amongs the previously registered 
     * services). After this call, no other service can be added to the logger.
     *
     */
  void initMemory();
  /** Log a value of a registered service
     *
     *  @param service service reference. needed to store the value in the 
     *  right area
     *  @param value value to be stored
     */
  void logValue(ISensorValueServiceLogger* service, float value);
  /** Remember the current position within the logged values and take it 
     * as newest available for the next download (applies to all registered 
     * services)
     *
     */
  void setStartValuePosition();
  /** Set an offset to the previously marked position (setStartValuePos). 
     * (applies to all registered services)
     *
     *  @param offset The offset means the user does not request the most 
     *  recent data, but some data in the past. This amount of values are 
     *  skipped
     */
  void setStartValuePositionOffset(uint32_t offset);
  /** Get the amount of logged data. if there are two services registered and 
     * each contains 35 Values, 35 is returned.
     *
     *  @returns
     *    an integer representing the amount of values logged by the 
     *    registered services
     */
  uint32_t getAmountOfLoggedValues();
  /** Set the amount of Logged values to return. The newest (set by 
     * setStartValuePos and optional setStartValuePosOffset) values are 
     * returned first.
     *
     *  @param amount The amount of values to return for each registered 
     *  service
     */
  void setAmountOfValuesToReturn(uint64_t amount);
  /** Start the Logger Download and commit the logged values to the service 
     * data handler, which generates notification within the softdevice. Issues
     * the first call of downloadLoggedValues
     *
     */
  void startDownload();
  /** Cancels the current download. Typically used when connection is lost 
     * during download
     *
     */
  void cancelDownload();
  /** Download the next value. This function should be registered with the 
     * service data handler as "notification fill-up function"
     *
     */
  void downloadLoggedValues();
  /** reset the whole logger and delete all logged values. The registered 
     * services remain
     *
     */
  void reset();

  /** Check if there is a download already running
     *
     *  @returns
     *    0     no download in progress
          non-0 download in progress
     */
  bool isDownloadInProgress()
  {
    return _downloadStarted;
  };

private:
  enum privateConstants
  {
    NRF51822_NUMBER_OF_FLASH_PAGES = 256, //nRF51822-xxAA
#if (0 == NEED_CONSOLE_OUTPUT)
    NRF51822_FLASH_PAGES_USED = 127, //Adjust if application code + softdevice size exceeds 128kB
#else
    NRF51822_FLASH_PAGES_USED = 145,//140,//135, //Adjust if application code + softdevice size exceeds 128kB
#endif
    MAX_REGISTERED_SERVICES = 2,
    INDEXING_ERROR = 0xFFFF
  };

  uint32_t getServiceIndex(ISensorValueServiceLogger* service);
  void initSingleServiceMemory(uint32_t pagesPerService, int i);
  void singleLogValueNotification(uint8_t nextIdx);

  FunctionPointer* _loggerDownloadFinishedCbPtr;
  ISensorValueServiceLogger* _registeredServices[MAX_REGISTERED_SERVICES];
  RingBuffer* _ringBuffers[MAX_REGISTERED_SERVICES];
  uint32_t _loggedValues[MAX_REGISTERED_SERVICES];
  uint32_t _startValue[MAX_REGISTERED_SERVICES];
  uint32_t _startValueMax[MAX_REGISTERED_SERVICES];
  uint32_t _startValueMaxOffset[MAX_REGISTERED_SERVICES];
  uint32_t _valuesToReturn[MAX_REGISTERED_SERVICES];
  uint32_t _returnedValues[MAX_REGISTERED_SERVICES];
  uint8_t _amountOfRegisteredServices;
  ISensorValueServiceLogger::LoggedValueNotification** _pValueBuffer;
  bool _lock;
  bool _downloadStarted;
};

#endif
