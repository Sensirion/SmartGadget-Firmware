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

#ifndef SMARTGADGETPERIPHERAL_H
#define SMARTGADGETPERIPHERAL_H

#include "mbed.h"
#include "app_util.h"
#include "BLEDevice.h"
#include "ShtLcd.h"
#include "DewPoint.h"
#include "nrf_delay.h"

/**
* @class ShtLcd
* @brief This class handles the peripherals like Button, LCD (and its displayed 
* values/ modes) and the battery level<br>
**/
class SmartGadgetPeripheral
{
public:

  /** Create a SmartGadgetPeripheral
     *
     *  @param ble Bluetooth Low Energy device from mbed BLE_API
     */
  SmartGadgetPeripheral(BLEDevice* ble);

  /** Update the SHT LCD with new sensor values. The LcdUpdate has a low power 
     * mode where the displayed values remain when almost no change is detected.
     *
     *  @param humidity ambient humidity (%RH) level as float
     *  @param temperature ambient temperature (°C) level as float
     *  @param force force the update of the LCD and override the low power 
     *  (steady) display feature.
     *
     *  @returns
     *    an boolean representing if the next displayed value will be updated or not
     *        0 next measurement should be accurate (display will be updated)
     *    non-0 next measurement does not have to be accurate (display will not be updated)
     */
  static bool lcdUpdate(float humidity, float temperature, bool force = false);
  /** Update the SHT LCD with the current sensor values. Useful when switching display 
     * mode, calls LcdUpdate(float,float,bool) with force = true and buffered display values
     *
     */
  static void lcdUpdate();
  /** Polls if the button was pressed long (>1s) and resets the flag
     *
     *  @returns
     *    an boolean representing the fact if the button was pressed long or not
     *        0 button was not pressed long
     *    non-0 button was pressed long
     */
  static bool wasButtonPressedLongAndReset(void);
  /** Show the "RF ON" string and parts of the MAC Address
     *
     */
  static void lcdShowRfOnAndId(void);
  /** Show the "RF OFF" string
     *
     */
  static void lcdShowRfOff(void);
  /** Show the FW Versions (Application and LCD Driver) on the LCD
     *
     */
  static bool lcdShowVersion(void);
  /** Clear the display and show low batt symbol only
     *
     */
  static void lcdShowLowBatteryOnly(void);
  /** Show the "Err" string on the LCD and detach the button pin interrupt
     *
     */
  static void lcdError(void);
  /** Set the Low Batt Flag. After each LcdUpdate the low Batt sign is added 
     * if this function was called before.
     *
     */
  static void setLowBatteryFlag(void);
  /** Get the battery level in percent
     *
     *  @returns
     *    an integer representing the battery level in percent [0...100]
     */
  static uint8_t getBatteryLevel(void);
  /** Set the Application Firmware Version that has to be displayed when 
     * LcdShowVersion is called
     *
     *  @param humidity ambient humidity (%RH) level as float
     *  @param temperature ambient temperature (°C) level as float
     *  @param force force the update of the LCD and override the low 
     *  power (steady) display feature.
     */
  static void setFirmwareVersion(uint8_t major, uint8_t minor);

  /** Get the displayed humidity value
     *
     *  @returns
     *    an float representing the displayed humidity value
     */
  static float getDisplayedHumidity(void)
  {
    return _humidityLcdCopy;
  };

  /** Get the displayed temperature value
     *
     *  @returns
     *    an float representing the displayed temperature value
     */
  static float getDisplayedTemperature(void)
  {
    return _temperatureLcdCopy;
  };

private:
  enum privateConstants
  {
    ADC_REF_VOLTAGE_IN_MILLIVOLTS = 1200,
    ADC_PRE_SCALING_COMPENSATION = 3,
    BUTTON_TIMEOUT_S = 1,
    BUTTON_TIMER_THRESHOLD_MS = 900,
    BUTTON_DEBOUNCE_MS = 50,
    THRESHOLD_LCD_UPDATE_RH = 1,
    THRESHOLD_LCD_UPDATE_DC = 1,
    MAX_LCD_UPDATE_S = 15,
    BUTTON_PIN = p9
  };

  SmartGadgetPeripheral();
  static bool checkIfLcdUpdateNeeded(float humidity, float temperature);
  static void setSegmentsRfOnAndId();
  static void setSegmentsRfOff();
  static bool setSegmentsVersion();
  static void setSegmentsData();
  static void buttonPressedISR(void);
  static void buttonReleasedISR(void);
  static bool checkSwitchToHighPowerLcd(float humidity, float temperature);
  static bool checkSwitchToLowPowerLcd(float humidity, float temperature);
  static bool lcdVersionIsPlausible(uint8_t* version);
  static void displayTemperatureCelsius(void);
  static void displayTemperatureFahrenheit(void);
  static void displayDevPointCelsius(void);
  static void displayDewPointFahrenheit(void);
  static void displayHumidity(void);
  static void displayFirstLcdLine(void);
  static void displaySecondLcdLine(void);

  static ShtLcd* _pShtLcd;
  static Timer* _pTimer;
  static Timeout* _pTimeout;
  static InterruptIn* _pButtonInt;
  static BLEDevice* _pBle;

  static bool _bouncingButton;
  static float _humidityLcdCopy;
  static float _temperatureLcdCopy;
  static bool _isCelsius;
  static bool _isDewPoint;
  static uint8_t _buttonPressedS;
  static uint8_t _LcdPowerModeSeconds;
  static bool _LcdLowPowerMode;
  static uint8_t _LcdFreezeSeconds;
  static uint8_t _FwVersionMaj;
  static uint8_t _FwVersionMin;
  static bool _LowBatt;
};

#endif
