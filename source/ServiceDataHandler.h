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

#ifndef SERVICEDATAHANDLER_H
#define SERVICEDATAHANDLER_H

#include "mbed.h"
#include "BLEDevice.h"
#include "FunctionPointer.h"

/* Holds all characteristic values that the services want to update queued and sends as 
much value updates as possible each connection interval to the ble stack. 
ATTENTION: values are passed as reference -> caller has to hold at least n values where
n equals MAX_NOTIFICATIONS_PER_CONN_EVT. Only applies if the caller has a large
amount of values to pass to the service data handler (i.e. reading out logged data).*/

/**
* @class ServiceDataHandler
* @brief A handler for the notifications. This class keeps an overview of how 
* many notifications are sent in the current connection event. Requires that 
* all characteristic values supporting notifications are updated over this 
* Service Data handler.<br>
**/
class ServiceDataHandler
{
public:
  typedef enum
  {
    CONTEXT_APP = 0,
    CONTEXT_CB = 1,
  } Context;

  /** Create a ServiceDataHandler
    *
    *  @param ble Bluetooth Low Energy device from mbed BLE_API
    */
  ServiceDataHandler(BLEDevice& ble);

  /** push Data to the handler that has to be updated to a certain BLE service
    *
    *  @param handle Bluetooth Low Energy Characteristic handle (from BLE_API)
    *  @param value reference to the data that has to be updated
    *  @param size of the data in bytes
    *  @returns
    *    an integer representing the success of the push operation
    *        0 for failure (queue full)
         non-0 for success
    */
  bool push(uint16_t handle, const uint8_t* value, uint16_t size);

  /** Set the callback pointer that is called every connection event when there 
    * were less than the maximum amount of notifications sent.
    *
    *  @param fp Function Pointer
    */
  void setDataFillUpCallback(FunctionPointer* fp);
  /** This handler should be called at the end of the periodic callback 
    * (application code) and can also be called within the onDataSent 
    * callback
    *
    *  @param context is either CONTEXT_APP when called from the periodic 
    *  callback or CONTEXT_CB if called within onDataSent callback
    */
  void notificationHandler(Context context);
  /** Set the amount of maximum possible Application-Notifications. 
    * The handler fills the notification queue up to the level of 
    * maximum-overall minus the amount set with this function (=safety margin) 
    * when called from onDataSent callback. This is needed due to the lack of 
    * synchronization between the connection event and the periodic callback.
    *
    *  @param amount of maximum possible notifications
    */
  void setNumbersOfApplicationNotifications(uint8_t amount);

private:
  enum privateConstants
  {
    DEFAULT_CONN_SPEED_MS = 1000,
    MAX_NOTIFICATIONS_PER_CONN_EVT = 7,
    QUEUE_SIZE = MAX_NOTIFICATIONS_PER_CONN_EVT,
  };

  ServiceDataHandler();

  BLEDevice& _pBle;
  FunctionPointer* _dataFillUpFp;
  uint8_t _queuedItems;
  uint8_t _maxNoti;
};

#endif
