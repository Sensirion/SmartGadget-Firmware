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

#include "SmartGadget.h"

#define FIRMWARE_VERSION_MAJ 1
#define FIRMWARE_VERSION_MIN 3

BLEDevice* SmartGadget::_pBle = NULL;
ServiceDataHandler* SmartGadget::_pServiceDataHandler = NULL;
Logger* SmartGadget::_pLogger = NULL;
DeviceInformationService* SmartGadget::_pDeviceInformationService = NULL;
HumidityService* SmartGadget::_pHumidityService = NULL;
TemperatureService* SmartGadget::_pTemperatureService = NULL;
LoggerService* SmartGadget::_pLoggerService = NULL;
BatteryService* SmartGadget::_pBatteryService = NULL;
I2c* SmartGadget::_pI2c = NULL;
Sht31* SmartGadget::_pSht31 = NULL;
SmartGadgetPeripheral* SmartGadget::_pSmartGadgetPeripheral = NULL;
bool SmartGadget::_bleIsTurnedOn = false;
uint64_t SmartGadget::_timeMs = 0;
uint64_t SmartGadget::_timeMsSynced = 0;
int64_t SmartGadget::_oldestPossibleLogTimeMs = 0;
int64_t SmartGadget::_newestPossibleLogTimeMs = 0;
bool SmartGadget::_lastMeasLowRes = false;
int8_t SmartGadget::_connSpeedCounter = CONNECTION_SPEED_DELAY_TO_SLOW_S;
int SmartGadget::_loggerDownloadFinishedDelay = 0;
Gap::Handle_t SmartGadget::_currentHandle = 0;
const char* SmartGadget::STR_SHT31_T = "SHT31 Temperature in \xc2\xb0\x43";
const char* SmartGadget::STR_SHT31_RH = "SHT31 Humidity in %RH";
const char SmartGadget::STR_FW_VERSION[] = {FIRMWARE_VERSION_MAJ + '0','.',FIRMWARE_VERSION_MIN + '0'};

SmartGadget::SmartGadget()
{
  _pBle = new BLEDevice();
  _pBle->init();
  _pLogger = new Logger();
  _pLogger->setLoggerDownloadFinishedCallback(new FunctionPointer(loggerDownloadFinished));
  _pServiceDataHandler = new ServiceDataHandler(*_pBle);
  _pServiceDataHandler->setDataFillUpCallback(new FunctionPointer(_pLogger, &Logger::downloadLoggedValues));
  _pSmartGadgetPeripheral = new SmartGadgetPeripheral(_pBle);
  _pSmartGadgetPeripheral->setFirmwareVersion(static_cast<uint8_t>(FIRMWARE_VERSION_MAJ), static_cast<uint8_t>(FIRMWARE_VERSION_MIN));
  _pI2c = new I2c(p11, p12);
  _pSht31 = new Sht31(_pI2c);
  uint8_t systemId[8];
  getSystemId(systemId);
  _pDeviceInformationService = new DeviceInformationService(*_pBle,
                                                            systemId,
                                                            "Sensirion AG", //manufacturersName
                                                            "SmartGadget", //modelNumber
                                                            "00000000", //serialNumber
                                                            "1", //hardwareRevision
                                                            STR_FW_VERSION, //firmwareRevision
                                                            "0.1"); //softwareRevision
  _pBatteryService = new BatteryService(*_pBle, _pServiceDataHandler, 100, &SmartGadget::batteryLevelWasRead);

  if (_pSht31->isAvailable())
  {
    _pLoggerService = new LoggerService(*_pBle, _pServiceDataHandler);
    _pHumidityService = new HumidityService(*_pBle, _pServiceDataHandler, reinterpret_cast<const uint8_t*>(STR_SHT31_RH));
    _pTemperatureService = new TemperatureService(*_pBle, _pServiceDataHandler, reinterpret_cast<const uint8_t*>(STR_SHT31_T));

    _pLogger->addServiceToLogger(_pHumidityService);
    _pLogger->addServiceToLogger(_pTemperatureService);

    _pSht31->startMeasurementHighResolution();
  }
}

void SmartGadget::init(void)
{
  _pBle->onDisconnection(disconnectionCallback);
  _pBle->onConnection(connectionCallback);
  _pBle->onDataSent(onDataSent);
  _pBle->onDataWritten(onDataWritten);

  // setup advertising and prefered connection parameters
  _pBle->accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED | GapAdvertisingData::LE_GENERAL_DISCOVERABLE);
  _pBle->setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);
  _pBle->setAdvertisingInterval(ADVERTISING_INTERVAL_0MS625);
  _pBle->setDeviceName(reinterpret_cast<const uint8_t*>("Smart Humigadget"));
  _pBle->accumulateScanResponse(GapAdvertisingData::COMPLETE_LOCAL_NAME, reinterpret_cast<const uint8_t*>("Smart Humigadget"), 16);

  Gap::ConnectionParams_t connectionParams;
  connectionParams.minConnectionInterval = 792; // Minimum Connection Interval in 1.25 ms units, see @ref BLE_GAP_CP_LIMITS.
  connectionParams.maxConnectionInterval = 808; // Maximum Connection Interval in 1.25 ms units, see @ref BLE_GAP_CP_LIMITS.
  connectionParams.slaveLatency = 0; // Slave Latency in number of connection events, see @ref BLE_GAP_CP_LIMITS.
  connectionParams.connectionSupervisionTimeout = 500; // Connection Supervision Timeout in 10 ms units, see @ref BLE_GAP_CP_LIMITS.
  _pBle->setPreferredConnectionParams(&connectionParams);
  _pLogger->initMemory();

  // Use the nordic App Timer instead of the mbed Ticker class due to interrupt disabling within the mbed Ticker
  static app_timer_id_t timer_ID;
  APP_TIMER_INIT(255, 1, 1, false);
  app_timer_create(&timer_ID, APP_TIMER_MODE_REPEATED, static_cast<app_timer_timeout_handler_t>(periodicCallback));
  app_timer_start(timer_ID, 128, NULL);

  //Show Versions on LCD and Error if no Sensor was found
  if (false == _pSmartGadgetPeripheral->lcdShowVersion() || (false == _pSht31->isAvailable()))
  {
    _pSmartGadgetPeripheral->lcdError();
    if (_pBle->getGapState().advertising)
    {
      _pBle->stopAdvertising();
    }
    if (_pBle->getGapState().connected)
    {
      _pBle->onDisconnection(NULL);
      _pBle->disconnect(Gap::REMOTE_USER_TERMINATED_CONNECTION);
    }
  }

  //initial Battery level check -> Shutdown if battery too low
  uint8_t newBatt = _pSmartGadgetPeripheral->getBatteryLevel();
  if (newBatt == 0) //shutdown all
  {
    _pSmartGadgetPeripheral->lcdShowLowBatteryOnly();
    if (_pBle->getGapState().advertising)
    {
      _pBle->stopAdvertising();
    }
    if (_pBle->getGapState().connected)
    {
      _pBle->onDisconnection(NULL);
      _pBle->disconnect(Gap::REMOTE_USER_TERMINATED_CONNECTION);
    }
  }
}

void SmartGadget::onDataSent(unsigned count) // called after notifications have been sent
{
  _pServiceDataHandler->notificationHandler(ServiceDataHandler::CONTEXT_CB);
}

void SmartGadget::onDataWritten(const GattCharacteristicWriteCBParams* eventDataP)// called after characteristics have been updated by client
{
  uint32_t currentLoggerIntervalMs = _pLoggerService->getLoggerIntervalMsCharacteristicValue();
  if (eventDataP->charHandle == _pLoggerService->getSyncTimeMsCharacteristicHandle())
  {
    syncTimeMsValueUpdated(currentLoggerIntervalMs);
  }
  if (eventDataP->charHandle == _pLoggerService->getOldestSampleTimeMsCharacteristicHandle())
  {
    oldestSampleTimeMsValueUpdated(currentLoggerIntervalMs);
  }
  if (eventDataP->charHandle == _pLoggerService->getNewestSampleTimeMsCharacteristicHandle())
  {
    newestSampleTimeMsValueUpdated(currentLoggerIntervalMs);
  }
  if (eventDataP->charHandle == _pLoggerService->getLoggerIntervalMsCharacteristicHandle())
  {
    loggerIntervalMsValueUpdated(currentLoggerIntervalMs);
  }
  if (eventDataP->charHandle == _pLoggerService->getStartLoggerDownloadCharacteristicHandle())
  {
    startLoggerDownloadValueUpdated();
  }
}

void SmartGadget::disconnectionCallback(Gap::Handle_t handle, Gap::DisconnectionReason_t reason)
{
  _pLoggerService->resetCharacteristics();
  _pLogger->cancelDownload();
  if (_bleIsTurnedOn)
  {
    _pBle->startAdvertising();
  }
  _pSmartGadgetPeripheral->lcdUpdate(); //force display update
}

void SmartGadget::connectionCallback(Gap::Handle_t handle, Gap::addr_type_t peerAddrType, const Gap::address_t peerAddr, const Gap::ConnectionParams_t* param)
{
  _currentHandle = handle;
  _connSpeedCounter = CONNECTION_SPEED_DELAY_TO_SLOW_S;
  _pSmartGadgetPeripheral->lcdUpdate(); //force display update
}

void SmartGadget::propagateSensorValuesToBleService(float humidity, float temperature)
{
  if (_pBle->getGapState().connected)
  {
    if ((false == _pLogger->isDownloadInProgress()) || (0 == _timeMs % (DOWNLOAD_LIVE_VALUES_LATENCY * PERIODIC_CALLBACK_INTERVAL_MS)))
    {
      updateHumidityAndTemperatureCharacteristics(humidity, temperature);     
    }
    _pServiceDataHandler->setNumbersOfApplicationNotifications((PERIODIC_CALLBACK_INTERVAL_MS == _timeMs % (DOWNLOAD_LIVE_VALUES_LATENCY * PERIODIC_CALLBACK_INTERVAL_MS)) ? MAX_APPLICATION_NOTIFICATIONS_DURING_DOWNLOAD : 1);
  }
}

void SmartGadget::updateAndHandleBatteryLevel()
{
  if ((0 == _timeMs % BATTERY_CHECK_INTERVAL_MS) && (false == _pLogger->isDownloadInProgress()))
  {
    static uint8_t batt = 100;
    uint8_t newBatt = _pSmartGadgetPeripheral->getBatteryLevel();
    newBatt = newBatt * 0.1 + batt * 0.9;
    // Battery Level should always decrease
    if (newBatt < batt)
    {
      batt = newBatt;
      if (_pBle->getGapState().connected)
      {
        _pBatteryService->updateBatteryLevel(batt);
      }
    }
    if (LOWBATT_THRESHHOLD >= batt) // turn on Low Batt sign
    {
      _pSmartGadgetPeripheral->setLowBatteryFlag();
    }
    if (0 == batt) // battery empty -> shutdown everything
    {
      shutdownSystem();
    }
  }
}

/**
 * Runs once a second in interrupt context triggered by the 'ticker';
 */
void SmartGadget::periodicCallback(void* p_context)
{
  handleAdvertising();
  if (_pSht31->isAvailable())
  {
    processSensorDataAndStartNewMeasurement();
  }

  updateAndHandleBatteryLevel();
  if (_pBle->getGapState().connected)
  {
    _pServiceDataHandler->notificationHandler(ServiceDataHandler::CONTEXT_APP);
    handleConnectionSpeed();
    if(_loggerDownloadFinishedDelay > 0)
    {
      _loggerDownloadFinishedDelay--;
      if(_loggerDownloadFinishedDelay == 0)
        _pLoggerService->setStartLoggerDownloadCharacteristic(0);
    }
  }

  _timeMs += PERIODIC_CALLBACK_INTERVAL_MS;
}

void SmartGadget::handleAdvertising()
{
  if (_pSmartGadgetPeripheral->wasButtonPressedLongAndReset())
  {
    switchBleOnOff();
  }
}

void SmartGadget::loggerDownloadFinished()
{
  _loggerDownloadFinishedDelay = 2;
}

void SmartGadget::batteryLevelWasRead(GattCharacteristicReadAuthCBParams* para)
{
  if (_connSpeedCounter > 0)
  {
    //extend fast mode again
    _connSpeedCounter = CONNECTION_SPEED_DELAY_TO_SLOW_S;
  }
  else
  {
    // force to fast mode
    handleConnectionSpeed(true);
  }
}

void SmartGadget::handleConnectionSpeed(bool forceToFast)
{
  if (_pLogger->isDownloadInProgress() && 0 < _connSpeedCounter)
  {
    _connSpeedCounter = CONNECTION_SPEED_DELAY_TO_SLOW_S; //reset countdown to slow speed
  }
  if ((0 < _connSpeedCounter) && (false == _pLogger->isDownloadInProgress()) && (false == forceToFast))
  {
    decreaseAndHandleConnectionSpeedCounter();
  }
  if (((0 > _connSpeedCounter) && (true == _pLogger->isDownloadInProgress())) || (true == forceToFast))
  {
    increaseAndHandleConnectionSpeedCounter(forceToFast);   
  }
}

void SmartGadget::getSystemId(uint8_t* systemId)
{  
  Gap::addr_type_t addrType;
  Gap::address_t addr;
  _pBle->getAddress(&addrType, addr);
  systemId[0] = addr[0];
  systemId[1] = addr[1];
  systemId[2] = addr[2];
  // set middle bytes to zero
  systemId[4] = 0x00;
  systemId[3] = 0x00;
  // shift three bytes up
  systemId[7] = addr[5];
  systemId[6] = addr[4];
  systemId[5] = addr[3];
}

void SmartGadget::syncTimeMsValueUpdated(uint32_t currentLoggerIntervalMs)
{
   _timeMsSynced = _timeMs;
    _newestPossibleLogTimeMs = _pLoggerService->getSyncTimeMsCharacteristicValue() - _timeMsSynced % currentLoggerIntervalMs;
    _newestPossibleLogTimeMs = (_newestPossibleLogTimeMs < 0) ? 0 : _newestPossibleLogTimeMs;//limit to 0
    _oldestPossibleLogTimeMs = _newestPossibleLogTimeMs - _pLogger->getAmountOfLoggedValues() * currentLoggerIntervalMs;
    _oldestPossibleLogTimeMs = (_oldestPossibleLogTimeMs < 0) ? 0 : _oldestPossibleLogTimeMs;//limit to 0
    _pLoggerService->setNewestSampleTimeMsCharacteristic(_newestPossibleLogTimeMs);
    _pLoggerService->setOldestSampleTimeMsCharacteristic(_oldestPossibleLogTimeMs);
    _pLogger->setStartValuePosition();
    _pLogger->setAmountOfValuesToReturn((_newestPossibleLogTimeMs - _oldestPossibleLogTimeMs) / currentLoggerIntervalMs);
}

void SmartGadget::oldestSampleTimeMsValueUpdated(uint32_t currentLoggerIntervalMs)
{
  uint64_t newOldestTimeMs = _pLoggerService->getOldestSampleTimeMsCharacteristicValue();
  uint64_t currentNewestTimeMs = _pLoggerService->getNewestSampleTimeMsCharacteristicValue();
  if (_oldestPossibleLogTimeMs > newOldestTimeMs) //prevent out of range
  {
    newOldestTimeMs = _oldestPossibleLogTimeMs;
  }
  if (newOldestTimeMs > currentNewestTimeMs) //prevent out of range
  {
    newOldestTimeMs = currentNewestTimeMs;
  }
  uint64_t offset = currentLoggerIntervalMs - (newOldestTimeMs - _oldestPossibleLogTimeMs) % currentLoggerIntervalMs;
  newOldestTimeMs += (offset == currentLoggerIntervalMs) ? 0 : offset;

  _pLoggerService->setOldestSampleTimeMsCharacteristic(newOldestTimeMs);
  _pLogger->setAmountOfValuesToReturn((currentNewestTimeMs - newOldestTimeMs) / currentLoggerIntervalMs);
}

void SmartGadget::newestSampleTimeMsValueUpdated(uint32_t currentLoggerIntervalMs)
{
  uint64_t newNewestTimeMs = _pLoggerService->getNewestSampleTimeMsCharacteristicValue();
  uint64_t currentOldestTimeMs = _pLoggerService->getOldestSampleTimeMsCharacteristicValue();
  if (_newestPossibleLogTimeMs < newNewestTimeMs) //prevent out of range
  {
    newNewestTimeMs = _newestPossibleLogTimeMs;
  }
  if (newNewestTimeMs < currentOldestTimeMs) //prevent out of range
  {
    newNewestTimeMs = currentOldestTimeMs;
  }
  uint64_t offset = (currentLoggerIntervalMs - (_newestPossibleLogTimeMs - newNewestTimeMs) % currentLoggerIntervalMs);
  newNewestTimeMs -= (offset == currentLoggerIntervalMs) ? 0 : offset;

  _pLoggerService->setNewestSampleTimeMsCharacteristic(newNewestTimeMs);
  _pLogger->setStartValuePositionOffset((_newestPossibleLogTimeMs - newNewestTimeMs) / currentLoggerIntervalMs);
  _pLogger->setAmountOfValuesToReturn((newNewestTimeMs - currentOldestTimeMs) / currentLoggerIntervalMs);
}

void SmartGadget::loggerIntervalMsValueUpdated(uint32_t currentLoggerIntervalMs)
{
  uint32_t interval = _pLoggerService->getLoggerIntervalMsCharacteristicValue();
  interval -= interval % PERIODIC_CALLBACK_INTERVAL_MS; //round down
  if (0 == interval)
  {
    interval = PERIODIC_CALLBACK_INTERVAL_MS;
  }
  _pLoggerService->setLoggerIntervalMsCharacteristic(interval);
  _pLogger->reset();
  _timeMs = 0;
  _timeMsSynced = 0;
}
void SmartGadget::startLoggerDownloadValueUpdated()
{
  if (0x00 != _pLoggerService->getStartLoggerDownloadCharacteristicValue())
  {
    _pLogger->startDownload();
  }
}

void SmartGadget::updateHumidityAndTemperatureCharacteristics(float humidity, float temperature)
{
  if (_lastMeasLowRes)
    {
      _pHumidityService->updateSensorValue(_pSmartGadgetPeripheral->getDisplayedHumidity());
      _pTemperatureService->updateSensorValue(_pSmartGadgetPeripheral->getDisplayedTemperature());
    }
    else
    {
      _pHumidityService->updateSensorValue(humidity);
      _pTemperatureService->updateSensorValue(temperature);
    }
}

void SmartGadget::shutdownSystem()
{
  _pSmartGadgetPeripheral->lcdShowLowBatteryOnly();
  if (_pBle->getGapState().advertising)
  {
    _pBle->stopAdvertising();
  }
  if (_pBle->getGapState().connected)
  {
    _pBle->onDisconnection(NULL);
    _pBle->disconnect(Gap::REMOTE_USER_TERMINATED_CONNECTION);
  }
}

void SmartGadget::processSensorDataAndStartNewMeasurement()
{
  float humidity, temperature;
  bool lcdLowPower = false;
  if (0 == _pSht31->readMeasurement_ft(&humidity, &temperature))
    {
      if (_timeMs % _pLoggerService->getLoggerIntervalMsCharacteristicValue() == 0)
      {
        _pLogger->logValue(_pHumidityService, humidity);
        _pLogger->logValue(_pTemperatureService, temperature);
      }
      propagateSensorValuesToBleService(humidity, temperature);
      lcdLowPower = _pSmartGadgetPeripheral->lcdUpdate(humidity, temperature);
    }
    if (PERIODIC_CALLBACK_INTERVAL_MS == (_pLoggerService->getLoggerIntervalMsCharacteristicValue() - _timeMs % _pLoggerService->getLoggerIntervalMsCharacteristicValue()) || (false == lcdLowPower))
    {
      _pSht31->startMeasurementHighResolution();
      _lastMeasLowRes = false;
    }
    else
    {
      _pSht31->startMeasurementLowResolution();
      _lastMeasLowRes = true;
    }
}

void SmartGadget::switchBleOnOff()
{
  if (false == _bleIsTurnedOn)
  {
    _bleIsTurnedOn = true;
    _pSmartGadgetPeripheral->lcdShowRfOnAndId();
    _pBle->startAdvertising();
  }
  else
  {
    _bleIsTurnedOn = false;
    _pSmartGadgetPeripheral->lcdShowRfOff();
    if (_pBle->getGapState().advertising)
    {
      _pBle->stopAdvertising();
    }
    if (_pBle->getGapState().connected)
    {
      _pBle->disconnect(Gap::REMOTE_USER_TERMINATED_CONNECTION);
    }
  }
}

void SmartGadget::decreaseAndHandleConnectionSpeedCounter()
{
  _connSpeedCounter --;
  if (0 == _connSpeedCounter)
  {
    switchToSlowConnectionSpeed();
    _connSpeedCounter = -1 * CONNECTION_SPEED_DELAY_TO_FAST_S;
  }
}

void SmartGadget::switchToSlowConnectionSpeed()
{  
    Gap::ConnectionParams_t para;
    para.slaveLatency = 0;
    para.maxConnectionInterval = 800;
    para.minConnectionInterval = 780;
    para.connectionSupervisionTimeout = 500;
    _pBle->updateConnectionParams(_currentHandle, &para);
}

void SmartGadget::increaseAndHandleConnectionSpeedCounter(bool force)
{
   _connSpeedCounter ++;
    if ((0 == _connSpeedCounter) || (true == force))
    {
      switchToFastConnectionSpeed();
    }
}

void SmartGadget::switchToFastConnectionSpeed()
{  
  Gap::ConnectionParams_t para;
  para.slaveLatency = 0;
  para.maxConnectionInterval = 32;
  para.minConnectionInterval = 16;
  para.connectionSupervisionTimeout = 500;
  _pBle->updateConnectionParams(_currentHandle, &para);
  _connSpeedCounter = CONNECTION_SPEED_DELAY_TO_SLOW_S;
}

void SmartGadget::process()
{
  _pBle->waitForEvent();
}
