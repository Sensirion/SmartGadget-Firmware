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

#ifndef __BLE_LOGGER_SERVICE_H__
#define __BLE_LOGGER_SERVICE_H__

#include "BLEDevice.h"
#include "ServiceDataHandler.h"
#include "Logger.h"

extern const uint8_t LoggerServiceSyncTimeMsCharacteristicUUID[];
extern const uint8_t LoggerServiceOldestSampleTimeMsCharacteristicUUID[];
extern const uint8_t LoggerServiceNewestSampleTimeMsCharacteristicUUID[];
extern const uint8_t LoggerServiceStartLoggerDownloadCharacteristicUUID[];
extern const uint8_t LoggerServiceLoggerIntervalMsCharacteristicUUID[];
extern const uint8_t LoggerServiceUUID[];

/**
* @class LoggerService
* @brief Logger Service. This is used to read out buffered measurement data using timestamps in 
* milliseconds. After Syncing Time a time-range can be given and the after setting the start 
* download characteristic, the data is returned as notifications (fast but unsafe).<br>
* Service:  Proprietary, 128bit UUID 0000F234-B38D-4985-720E-0F993A68EE41 <br>
* Syncronize Timestamp Char: Proprietary, 128bit UUID 0000F235-B38D-4985-720E-0F993A68EE41 <br>
* Oldest Sample Timestamp Char: Proprietary, 128bit UUID 0000F236-B38D-4985-720E-0F993A68EE41 <br>
* Newest Sample Timestamp Char: Proprietary, 128bit UUID 0000F237-B38D-4985-720E-0F993A68EE41 <br>
* Start Logger Download Char: Proprietary, 128bit UUID 0000F238-B38D-4985-720E-0F993A68EE41 <br>
* Logger Interval Char: Proprietary, 128bit UUID 0000F239-B38D-4985-720E-0F993A68EE41 <br>
*/
class LoggerService
{
public:
  /** Create a LoggerService
     *
     *  @param ble Bluetooth Low Energy device from mbed BLE_API
     *  @param serviceDataHandler reference to the serviceDataHandler that is used by 
     *  the application
     */
  LoggerService(BLEDevice& ble, ServiceDataHandler* pServiceDataHandler) :
    _ble(ble),
    _pServiceDataHandler(pServiceDataHandler),
    _loggerServiceSyncTimeMsCharacteristic(LoggerServiceSyncTimeMsCharacteristicUUID, (uint8_t*)&_initValZero, SIZE_SYNC_TIME_MS_CHAR_VAL_BYTES,
                                           SIZE_SYNC_TIME_MS_CHAR_VAL_BYTES, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE, NULL, 0, (uint8_t*)("Sync Time Ms"), 12, 12),
    _loggerServiceOldestSampleTimeMsCharacteristic(LoggerServiceOldestSampleTimeMsCharacteristicUUID, (uint8_t*)&_initValZero, SIZE_READ_BACK_TO_TIME_MS_CHAR_VAL_BYTES,
                                                   SIZE_READ_BACK_TO_TIME_MS_CHAR_VAL_BYTES, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ,
                                                   NULL, 0, (uint8_t*)("Oldest Sample Time Ms"), 20, 20),
    _loggerServiceNewestSampleTimeMsCharacteristic(LoggerServiceNewestSampleTimeMsCharacteristicUUID, (uint8_t*)&_initValZero, SIZE_NEWEST_SAMPLE_TIME_MS_CHAR_VAL_BYTES,
                                                   SIZE_NEWEST_SAMPLE_TIME_MS_CHAR_VAL_BYTES, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE,
                                                   NULL, 0, (uint8_t*)("Newest Sample Time Ms"), 21, 21),
    _loggerServiceStartLoggerDownloadCharacteristic(LoggerServiceStartLoggerDownloadCharacteristicUUID, (uint8_t*)&_initValZero, SIZE_START_LOGGER_DOWNLAOD_CHAR_VAL_BYTES,
                                                    SIZE_START_LOGGER_DOWNLAOD_CHAR_VAL_BYTES, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE,
                                                    NULL, 0, (uint8_t*)("Start Logger Download"), 21, 21),
    _loggerServiceLoggerIntervalMsCharacteristic(LoggerServiceLoggerIntervalMsCharacteristicUUID, (uint8_t*)&_initValInterval, SIZE_LOGGER_INTERVAL_MS_CHAR_VAL_BYTES,
                                                 SIZE_LOGGER_INTERVAL_MS_CHAR_VAL_BYTES, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE,
                                                 NULL, 0, (uint8_t*)("Logger Interval Ms"), 18, 18)
  {
    GattCharacteristic* charTable[] = {&_loggerServiceSyncTimeMsCharacteristic,
      &_loggerServiceOldestSampleTimeMsCharacteristic,
      &_loggerServiceNewestSampleTimeMsCharacteristic,
      &_loggerServiceStartLoggerDownloadCharacteristic,
      &_loggerServiceLoggerIntervalMsCharacteristic};
    GattService loggerService(LoggerServiceUUID, charTable, sizeof(charTable) / sizeof(GattCharacteristic *));

    ble.addService(loggerService);
  }

  /** Get the handle of the SyncTimeMsCharacteristic
     *
     *  @returns
     *    the handle
     */
  uint16_t getSyncTimeMsCharacteristicHandle(void)
  {
    return _loggerServiceSyncTimeMsCharacteristic.getValueHandle();
  }

  /** Get the handle of the OldestSampleTimeMsCharacteristic
     *
     *  @returns
     *    the handle
     */
  uint16_t getOldestSampleTimeMsCharacteristicHandle(void)
  {
    return _loggerServiceOldestSampleTimeMsCharacteristic.getValueHandle();
  }

  /** Get the handle of the NewestSampleTimeMsCharacteristic
     *
     *  @returns
     *    the handle
     */
  uint16_t getNewestSampleTimeMsCharacteristicHandle(void)
  {
    return _loggerServiceNewestSampleTimeMsCharacteristic.getValueHandle();
  }

  /** Get the handle of the LoggerIntervalMsCharacteristic
     *
     *  @returns
     *    the handle
     */
  uint16_t getLoggerIntervalMsCharacteristicHandle(void)
  {
    return _loggerServiceLoggerIntervalMsCharacteristic.getValueHandle();
  }

  /** Get the handle of the StartLoggerDownloadCharacteristic
     *
     *  @returns
     *    the handle
     */
  uint16_t getStartLoggerDownloadCharacteristicHandle(void)
  {
    return _loggerServiceStartLoggerDownloadCharacteristic.getValueHandle();
  }

  /** Set the value of the NewestSampleTimeMsCharacteristic
     *
     *  @param value The value to set
     */
  void setNewestSampleTimeMsCharacteristic(uint64_t value)
  {
    _ble.updateCharacteristicValue(_loggerServiceNewestSampleTimeMsCharacteristic.getValueHandle(), reinterpret_cast<uint8_t*>(&value), SIZE_NEWEST_SAMPLE_TIME_MS_CHAR_VAL_BYTES);
  }

  /** Set the value of the OldestSampleTimeMsCharacteristic
     *
     *  @param value The value to set
     */
  void setOldestSampleTimeMsCharacteristic(uint64_t value)
  {
    _ble.updateCharacteristicValue(_loggerServiceOldestSampleTimeMsCharacteristic.getValueHandle(), reinterpret_cast<uint8_t*>(&value), SIZE_OLDEST_SAMPLE_TIME_MS_CHAR_VAL_BYTES);
  }

  /** Set the value of the StartLoggerDownloadCharacteristic
     *
     *  @param value The value to set
     */
  void setStartLoggerDownloadCharacteristic(uint8_t value)
  {
    _pServiceDataHandler->push(_loggerServiceStartLoggerDownloadCharacteristic.getValueAttribute().getHandle(), static_cast<uint8_t*>(&value), SIZE_START_LOGGER_DOWNLAOD_CHAR_VAL_BYTES);
  }

  /** Set the value of the LoggerIntervalMsCharacteristic
     *
     *  @param value The value to set
     */
  void setLoggerIntervalMsCharacteristic(uint32_t value)
  {
    _ble.updateCharacteristicValue(_loggerServiceLoggerIntervalMsCharacteristic.getValueHandle(), reinterpret_cast<uint8_t*>(&value), SIZE_LOGGER_INTERVAL_MS_CHAR_VAL_BYTES);
  }

  /** Get the value of the SyncTimeMsCharacteristic
     *
     *  @returns
     *    the currently set value
     */
  uint64_t getSyncTimeMsCharacteristicValue()
  {
    uint64_t retval = 0;
    uint16_t size = SIZE_SYNC_TIME_MS_CHAR_VAL_BYTES;
    _ble.readCharacteristicValue(_loggerServiceSyncTimeMsCharacteristic.getValueHandle(), reinterpret_cast<uint8_t*>(&retval), &size);
    return retval;
  }

  /** Get the value of the OldestSampleTimeMsCharacteristic
     *
     *  @returns
     *    the currently set value
     */
  uint64_t getOldestSampleTimeMsCharacteristicValue()
  {
    uint64_t retval = 0;
    uint16_t size = SIZE_READ_BACK_TO_TIME_MS_CHAR_VAL_BYTES;
    _ble.readCharacteristicValue(_loggerServiceOldestSampleTimeMsCharacteristic.getValueHandle(), reinterpret_cast<uint8_t*>(&retval), &size);
    return retval;
  }

  /** Get the value of the NewestSampleTimeMsCharacteristic
     *
     *  @returns
     *    the currently set value
     */
  uint64_t getNewestSampleTimeMsCharacteristicValue(void)
  {
    uint64_t retval = 0;
    uint16_t size = SIZE_NEWEST_SAMPLE_TIME_MS_CHAR_VAL_BYTES;
    _ble.readCharacteristicValue(_loggerServiceNewestSampleTimeMsCharacteristic.getValueHandle(), reinterpret_cast<uint8_t*>(&retval), &size);
    return retval;
  }

  /** Get the value of the StartLoggerDownloadCharacteristic
     *
     *  @returns
     *    the currently set value
     */
  uint8_t getStartLoggerDownloadCharacteristicValue(void)
  {
    uint8_t retval = 0;
    uint16_t size = SIZE_START_LOGGER_DOWNLAOD_CHAR_VAL_BYTES;
    _ble.readCharacteristicValue(_loggerServiceStartLoggerDownloadCharacteristic.getValueHandle(), static_cast<uint8_t*>(&retval), &size);
    return retval;
  }

  /** Get the value of the LoggerIntervalMsCharacteristic
     *
     *  @returns
     *    the currently set value
     */
  uint32_t getLoggerIntervalMsCharacteristicValue()
  {
    uint64_t retval = 0;
    uint16_t size = SIZE_LOGGER_INTERVAL_MS_CHAR_VAL_BYTES;
    _ble.readCharacteristicValue(_loggerServiceLoggerIntervalMsCharacteristic.getValueHandle(), reinterpret_cast<uint8_t*>(&retval), &size);
    return retval;
  }

  /** Resets all characteristics.
     *
     */
  void resetCharacteristics()
  {
    uint64_t zeroval = _initValZero; //source value has to be located in RAM
    _ble.updateCharacteristicValue(_loggerServiceSyncTimeMsCharacteristic.getValueAttribute().getHandle(), reinterpret_cast<uint8_t*>(&zeroval), SIZE_SYNC_TIME_MS_CHAR_VAL_BYTES);
    _ble.updateCharacteristicValue(_loggerServiceOldestSampleTimeMsCharacteristic.getValueAttribute().getHandle(), reinterpret_cast<uint8_t*>(&zeroval), SIZE_OLDEST_SAMPLE_TIME_MS_CHAR_VAL_BYTES);
    _ble.updateCharacteristicValue(_loggerServiceNewestSampleTimeMsCharacteristic.getValueAttribute().getHandle(), reinterpret_cast<uint8_t*>(&zeroval), SIZE_NEWEST_SAMPLE_TIME_MS_CHAR_VAL_BYTES);
    _ble.updateCharacteristicValue(_loggerServiceStartLoggerDownloadCharacteristic.getValueAttribute().getHandle(), reinterpret_cast<uint8_t*>(&zeroval), SIZE_START_LOGGER_DOWNLAOD_CHAR_VAL_BYTES);
  }

private:
  enum privateConstants
  {
    DEFAULT_LOGGER_INTERVAL_MS = 600000, //->10min
    SIZE_SYNC_TIME_MS_CHAR_VAL_BYTES = 8,
    SIZE_READ_BACK_TO_TIME_MS_CHAR_VAL_BYTES = 8,
    SIZE_NEWEST_SAMPLE_TIME_MS_CHAR_VAL_BYTES = 8,
    SIZE_OLDEST_SAMPLE_TIME_MS_CHAR_VAL_BYTES = 8,
    SIZE_START_LOGGER_DOWNLAOD_CHAR_VAL_BYTES = 1,
    SIZE_LOGGER_INTERVAL_MS_CHAR_VAL_BYTES = 4,
  };

  BLEDevice& _ble;
  ServiceDataHandler* _pServiceDataHandler;
  static const uint64_t _initValZero;
  static const uint32_t _initValInterval;
  GattCharacteristic _loggerServiceSyncTimeMsCharacteristic;
  GattCharacteristic _loggerServiceOldestSampleTimeMsCharacteristic;
  GattCharacteristic _loggerServiceNewestSampleTimeMsCharacteristic;
  GattCharacteristic _loggerServiceStartLoggerDownloadCharacteristic;
  GattCharacteristic _loggerServiceLoggerIntervalMsCharacteristic;
};

#endif /* #ifndef __BLE_LOGGER_SERVICE_H__*/
