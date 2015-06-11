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

#ifndef __BLE_HUMIDITY_SERVICE_H__
#define __BLE_HUMIDITY_SERVICE_H__

#include "BLEDevice.h"
#include "ServiceDataHandler.h"
#include "ISensorValueServiceLogger.h"

extern const uint8_t HumidityServiceHumidityValueCharacteristicUUID[];
extern const uint8_t HumidityServiceUUID[];

/**
* @class HumidityService
* @brief Ambient Humidity Service. This service displays the humidity level from 0%->100% 
* Relative Humidity represented by a float32T.<br>
* Service:  Proprietary, 128bit UUID 00001234-B38D-4985-720E-0F993A68EE41 <br>
* Humidity Level Char: Proprietary, 128bit UUID 00001235-B38D-4985-720E-0F993A68EE41 <br>
*/
class HumidityService : public ISensorValueServiceLogger
{
public:
  /** Create a HumidityService
     *
     *  @param ble Bluetooth Low Energy device from mbed BLE_API
     *  @param serviceDataHandler reference to the serviceDataHandler that is used by the application
     *  @param sensorName String that is used as the humidity characteristic user description string
     */
  HumidityService(BLEDevice& ble, ServiceDataHandler* serviceDataHandler, const uint8_t* sensorName) :
    _ble(ble),
    _pServiceDataHandler(serviceDataHandler),
    _humidityValueCharacteristic(HumidityServiceHumidityValueCharacteristicUUID, static_cast<uint8_t*>(NULL), 0, sizeof(LoggedValueNotification),
                                 GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_EXTENDED_PROPERTIES,
                                 NULL, 0, const_cast<uint8_t*>(sensorName), strlen(reinterpret_cast<const char*>(sensorName)), strlen(reinterpret_cast<const char*>(sensorName)))
  {
    GattCharacteristic* charTable[] = {&_humidityValueCharacteristic};
    GattService humidityService(HumidityServiceUUID, charTable, sizeof(charTable) / sizeof(GattCharacteristic *));

    ble.addService(humidityService);
  }

  /** Update the humidity level with a new value.
     * 
     * @param newLevel New humidity level.
     */
  void updateSensorValue(float newLevel)
  {
    _pServiceDataHandler->push(_humidityValueCharacteristic.getValueAttribute().getHandle(), reinterpret_cast<uint8_t*>(&newLevel), 4);
  }

  /** Update a characteristic supporting notifications. Inherited virtual function implementation. 
     *
     *  @param loggedValue Logged Value Notification containing all the information needed
     *  for the serviceDataHandler
     *  @param len size of the data provided
     */
  virtual void notifyLoggedValue(LoggedValueNotification loggedValue, uint8_t len)
  {
    _pServiceDataHandler->push(_humidityValueCharacteristic.getValueAttribute().getHandle(), reinterpret_cast<uint8_t*>(&loggedValue.sequenceNumber), len);
  }

private:
  BLEDevice& _ble;
  ServiceDataHandler* _pServiceDataHandler;
  GattCharacteristic _humidityValueCharacteristic;
};

#endif /* #ifndef __BLE_HUMIDITY_SERVICE_H__*/
