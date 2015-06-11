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

#ifndef SMARTGADGET_H
#define SMARTGADGET_H

#include "mbed.h"
#include "BLEDevice.h"
#include "ServiceDataHandler.h"
#include "DeviceInformationService.h"
#include "HumidityService.h"
#include "TemperatureService.h"
#include "LoggerService.h"
#include "BatteryService.h"
#include "DeviceInformationService.h"
#include "ServiceDataHandler.h"
#include "Logger.h"
#include "I2c.h"
#include "Sht31.h"
#include "SmartGadgetPeripheral.h"
#include "FunctionPointer.h"
#include "app_timer.h"

/**
* @class SmartGadget
* @brief The current application class. This class setups the whole system and 
* runs the application code within a periodic callback that is called by the 
* softdevice. This class is static.<br>
**/
class SmartGadget
{
public:
  /** Create a SmartGadget
     *
     */
  SmartGadget();
  /** Initialize the SmartGadget. Initializes the BLE/SoftDevice, 
     * all peripherals, classes and timers needed to run the application.
     *
     */
  static void init(void);
  /** Processes Application/BLE Events. Has to be called in a while loop.
     *
     */
  static void process(void);

private:
  enum privateConstants
  {
    ADVERTISING_INTERVAL_0MS625 = 3200, /* 2000ms; multiple of 0.625ms. */
    PERIODIC_CALLBACK_INTERVAL_MS = 1000,
    BATTERY_CHECK_INTERVAL_MS = 30000,
    DOWNLOAD_LIVE_VALUES_LATENCY = 4,
    MAX_APPLICATION_NOTIFICATIONS_DURING_DOWNLOAD = 3,
    LOWBATT_THRESHHOLD = 10,
    CONNECTION_SPEED_SWITCH_THRESHOLD = 25,
    CONNECTION_SPEED_DELAY_TO_SLOW_S = 60,
    CONNECTION_SPEED_DELAY_TO_FAST_S = 2
  };

  static void periodicCallback(void* p_context);
  static void disconnectionCallback(Gap::Handle_t handle, Gap::DisconnectionReason_t reason);
  static void connectionCallback(Gap::Handle_t handle, Gap::addr_type_t peerAddrType, const Gap::address_t peerAddr, const Gap::ConnectionParams_t* param);
  static void propagateSensorValuesToBleService(float humidity, float temperature);
  static void updateAndHandleBatteryLevel();
  static void handleAdvertising(void);
  static void handleConnectionSpeed(bool forceToFast = false);
  static void batteryLevelWasRead(GattCharacteristicReadAuthCBParams* para);
  static void onDataSent(unsigned count);
  static void onDataWritten(const GattCharacteristicWriteCBParams* eventDataP);
  static void loggerDownloadFinished();
  static void getSystemId(uint8_t* systemId);
  static void syncTimeMsValueUpdated(uint32_t loggerIntervalMs);
  static void oldestSampleTimeMsValueUpdated(uint32_t loggerIntervalMs);
  static void newestSampleTimeMsValueUpdated(uint32_t loggerIntervalMs);
  static void loggerIntervalMsValueUpdated(uint32_t loggerIntervalMs);
  static void startLoggerDownloadValueUpdated(void);
  static void updateHumidityAndTemperatureCharacteristics(float humidity, float temperature);
  static void shutdownSystem(void);
  static void processSensorDataAndStartNewMeasurement(void);
  static void switchBleOnOff(void);
  static void decreaseAndHandleConnectionSpeedCounter(void);
  static void switchToSlowConnectionSpeed(void);
  static void increaseAndHandleConnectionSpeedCounter(bool force);
  static void switchToFastConnectionSpeed(void);

  static BLEDevice* _pBle;
  static ServiceDataHandler* _pServiceDataHandler;
  static Logger* _pLogger;
  static DeviceInformationService* _pDeviceInformationService;
  static HumidityService* _pHumidityService;
  static TemperatureService* _pTemperatureService;
  static LoggerService* _pLoggerService;
  static BatteryService* _pBatteryService;
  static I2c* _pI2c;
  static Sht31* _pSht31;
  static SmartGadgetPeripheral* _pSmartGadgetPeripheral;

  static bool _bleIsTurnedOn;
  static uint64_t _timeMs;
  static uint64_t _timeMsSynced;
  static int64_t _oldestPossibleLogTimeMs;
  static int64_t _newestPossibleLogTimeMs;
  static bool _lastMeasLowRes;
  static int8_t _connSpeedCounter;
  static Gap::Handle_t _currentHandle;

  static const char* STR_SHT31_T;
  static const char* STR_SHT31_RH;
  static const char STR_FW_VERSION[3];
};

#endif
