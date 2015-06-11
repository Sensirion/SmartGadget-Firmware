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

#include "SmartGadgetPeripheral.h"

BLEDevice* SmartGadgetPeripheral::_pBle = NULL;
Timer* SmartGadgetPeripheral::_pTimer = NULL;
Timeout* SmartGadgetPeripheral::_pTimeout = NULL;
ShtLcd* SmartGadgetPeripheral::_pShtLcd = NULL;
InterruptIn* SmartGadgetPeripheral::_pButtonInt = NULL;
bool SmartGadgetPeripheral::_bouncingButton = false;
float SmartGadgetPeripheral::_humidityLcdCopy = 0.0f;
float SmartGadgetPeripheral::_temperatureLcdCopy = 0.0f;
bool SmartGadgetPeripheral::_isCelsius = false;
bool SmartGadgetPeripheral::_isDewPoint = false;
uint8_t SmartGadgetPeripheral::_buttonPressedS = false;
uint8_t SmartGadgetPeripheral::_LcdPowerModeSeconds = 0;
bool SmartGadgetPeripheral::_LcdLowPowerMode = false;
uint8_t SmartGadgetPeripheral::_LcdFreezeSeconds = 5;
uint8_t SmartGadgetPeripheral::_FwVersionMaj = 0;
uint8_t SmartGadgetPeripheral::_FwVersionMin = 0;
bool SmartGadgetPeripheral::_LowBatt = false;

SmartGadgetPeripheral::SmartGadgetPeripheral(BLEDevice* ble)
{
  _pBle = ble;
  InterruptIn* _pButtonInt = new InterruptIn(static_cast<PinName>(BUTTON_PIN));
  _pButtonInt->mode(PullUp);
  _isCelsius = _pButtonInt->read();
  _pTimer = new Timer();
  _pTimeout = new Timeout();
  _pButtonInt->fall(SmartGadgetPeripheral::buttonPressedISR);
  _pButtonInt->rise(SmartGadgetPeripheral::buttonReleasedISR);

  ShtLcd::ShtLcdPins lcdPins;
  lcdPins.CS = p2;
  lcdPins.MISO = p3;
  lcdPins.MOSI = p5;
  lcdPins.RDY = p0;
  lcdPins.SCK = p1;
  _pShtLcd = new ShtLcd(&lcdPins);
}

void SmartGadgetPeripheral::lcdUpdate()
{
  lcdUpdate(_humidityLcdCopy, _temperatureLcdCopy, true);
}

bool SmartGadgetPeripheral::checkIfLcdUpdateNeeded(float humidity, float temperature)
{
  bool needsLcdUpdateNow = false;
  if (_LcdLowPowerMode) // current state Low Power
  {
    needsLcdUpdateNow |= checkSwitchToHighPowerLcd(humidity, temperature);
  }
  else // current state High Power
  {
    needsLcdUpdateNow |= checkSwitchToLowPowerLcd(humidity, temperature);
  }
  return needsLcdUpdateNow;
}

bool SmartGadgetPeripheral::lcdUpdate(float humidity, float temperature, bool force)
{
  if (_LcdFreezeSeconds > 0) //version, rf on or something else is displayed
  {
    _LcdFreezeSeconds --;
    if (_LcdFreezeSeconds == 0)
    {
      force = true;
    }
    else
    {
      return false;
    }
  }
  //check if update needed
  force |= checkIfLcdUpdateNeeded(humidity, temperature);
  if (force)
  {
    _humidityLcdCopy = humidity;
    _temperatureLcdCopy = temperature;

    _pShtLcd->clear();
    setSegmentsData();
    _pShtLcd->update();
  }

  return (_LcdLowPowerMode & ((MAX_LCD_UPDATE_S - 1) != _LcdPowerModeSeconds));
}

void SmartGadgetPeripheral::lcdShowRfOnAndId()
{
  _LcdFreezeSeconds = 6;
  _pShtLcd->clear();
  setSegmentsRfOnAndId();
  _pShtLcd->update();
}

void SmartGadgetPeripheral::lcdShowRfOff()
{
  _LcdFreezeSeconds = 3;
  _pShtLcd->clear();
  setSegmentsRfOff();
  _pShtLcd->update();
}

bool SmartGadgetPeripheral::lcdShowVersion()
{
  //update Version
  _pShtLcd->clear();
  _pShtLcd->update();
  //display
  _LcdFreezeSeconds = 1;
  _pShtLcd->clear();
  bool success = setSegmentsVersion();
  _pShtLcd->update();
  return success;
}

void SmartGadgetPeripheral::lcdShowLowBatteryOnly()
{
  InterruptIn tmp(static_cast<PinName>(BUTTON_PIN)); //overwrite and reset button interrupt
  _pShtLcd->clear();
  _pShtLcd->setSymbol(ShtLcd::SYMBOL_SENSI);
  _pShtLcd->setSymbol(ShtLcd::SYMBOL_LOWBATT);
  _pShtLcd->update();
}

void SmartGadgetPeripheral::lcdError()
{
  _pShtLcd->clear();
  _pShtLcd->setSymbol(ShtLcd::SYMBOL_SENSI);
  _pShtLcd->setDigit(1, 0xe);
  _pShtLcd->setDigit(2, ShtLcd::DIGITR);
  _pShtLcd->setDigit(3, ShtLcd::DIGITR);
  _pShtLcd->update();
}

void SmartGadgetPeripheral::setSegmentsRfOnAndId()
{
  Gap::addr_type_t addrType;
  Gap::address_t addr;
  _pBle->getAddress(&addrType, addr);
  _pShtLcd->setSymbol(ShtLcd::SYMBOL_SENSI);
  _pShtLcd->setDigit(0, ShtLcd::DIGITR);
  _pShtLcd->setDigit(1, 0xf);
  _pShtLcd->setDigit(2, ShtLcd::DIGITO);
  _pShtLcd->setDigit(3, ShtLcd::DIGITN);
  _pShtLcd->setDigit(4, addr[1] >> 4);
  _pShtLcd->setDigit(5, addr[1] & 0x0f);
  _pShtLcd->setDigit(6, addr[0] >> 4);
  _pShtLcd->setDigit(7, addr[0] & 0x0f);
}

void SmartGadgetPeripheral::setSegmentsRfOff()
{
  _pShtLcd->setSymbol(ShtLcd::SYMBOL_SENSI);
  _pShtLcd->setDigit(0, ShtLcd::DIGITR);
  _pShtLcd->setDigit(1, 0xf);
  _pShtLcd->setDigit(4, ShtLcd::DIGITO);
  _pShtLcd->setDigit(5, 0xf);
  _pShtLcd->setDigit(6, 0xf);
}

bool SmartGadgetPeripheral::setSegmentsVersion()
{
  uint8_t version[2];
  _pShtLcd->setSymbol(ShtLcd::SYMBOL_SENSI);
  _pShtLcd->getVersion(version);
  _pShtLcd->setDigit(1, _FwVersionMaj);
  _pShtLcd->setDigit(2, _FwVersionMin);
  _pShtLcd->setDigit(5, version[0]);
  _pShtLcd->setDigit(6, version[1]);
  _pShtLcd->setDecimalPoint(1);
  _pShtLcd->setDecimalPoint(5);
  return lcdVersionIsPlausible(version);
}

void SmartGadgetPeripheral::setSegmentsData()
{
  _pShtLcd->setSymbol(ShtLcd::SYMBOL_SENSI);
  if (_LowBatt)
  {
    _pShtLcd->setSymbol(ShtLcd::SYMBOL_LOWBATT);
  }
  displayFirstLcdLine();
  displaySecondLcdLine();
  if (_pBle->getGapState().connected)
  {
    _pShtLcd->setSymbol(ShtLcd::SYMBOL_BLE);
  }
  else if (_pBle->getGapState().advertising)
  {
    _pShtLcd->setSymbol(ShtLcd::SYMBOL_BLEBLINK);
  }
}

void SmartGadgetPeripheral::setFirmwareVersion(uint8_t major, uint8_t minor)
{
  _FwVersionMaj = major;
  _FwVersionMin = minor;
}

void SmartGadgetPeripheral::buttonPressedISR()
{
  nrf_delay_us(10000); //debounce
  if (0 == ((NRF_GPIO->IN >> 9) & 0x1))
  {
    _buttonPressedS = 1;
  }
}

void SmartGadgetPeripheral::buttonReleasedISR()
{
  if (0 != _buttonPressedS)
  {
    _buttonPressedS = 0;
    _isDewPoint = !_isDewPoint;
    if (0 == _LcdFreezeSeconds)
    {
      lcdUpdate();
    }
  }
}

bool SmartGadgetPeripheral::wasButtonPressedLongAndReset()
{
  if (0 != _buttonPressedS)
  {
    _buttonPressedS++;
  }
  if (3 == _buttonPressedS)
  {
    _buttonPressedS = 0;
    return true;
  }
  return false;
}

void SmartGadgetPeripheral::setLowBatteryFlag(void)
{
  _LowBatt = true;
}

uint8_t SmartGadgetPeripheral::getBatteryLevel(void)
{
  static bool once = true; //do the configuration only once
  if (once)
  {
    once = false;
    NRF_ADC->CONFIG = (ADC_CONFIG_RES_8bit << ADC_CONFIG_RES_Pos) |
      (ADC_CONFIG_INPSEL_SupplyOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos) |
      (ADC_CONFIG_REFSEL_VBG << ADC_CONFIG_REFSEL_Pos) |
      (ADC_CONFIG_PSEL_Disabled << ADC_CONFIG_PSEL_Pos) |
      (ADC_CONFIG_EXTREFSEL_None << ADC_CONFIG_EXTREFSEL_Pos);
  }
  NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Enabled ;
  NRF_ADC->TASKS_START = 1;
  while (0 == NRF_ADC->EVENTS_END)
  {
  }
  NRF_ADC->EVENTS_END = 0;
  uint16_t result = static_cast<uint16_t>(NRF_ADC->RESULT);
  //calculate voltage in millivolts
  result = ((((result) * ADC_REF_VOLTAGE_IN_MILLIVOLTS) / 255) * ADC_PRE_SCALING_COMPENSATION);
  NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Disabled ;
  NRF_ADC->TASKS_STOP = 1;

  //calculate level in percent [0...100]
  //2.4V means empty battery (0%)
  //3.0V means full battery (100%)
  uint16_t level = (result < 2400) ? 0 : (result - 2400);
  level /= 6;
  return static_cast<uint8_t>(level);
}

bool SmartGadgetPeripheral::checkSwitchToHighPowerLcd(float humidity, float temperature)
{
  bool switchToHighPowerNow = false;
  _LcdPowerModeSeconds++;
  if (_LcdPowerModeSeconds == MAX_LCD_UPDATE_S)
  {
    switchToHighPowerNow = true;
    _LcdPowerModeSeconds = 0;
  }
  if ((abs(humidity - _humidityLcdCopy) > THRESHOLD_LCD_UPDATE_RH)
    || (abs(temperature - _temperatureLcdCopy) > THRESHOLD_LCD_UPDATE_DC))
  {
    //go high power next time
    _LcdLowPowerMode = false;
    _LcdPowerModeSeconds = 0;
  }
  return switchToHighPowerNow;
}

bool SmartGadgetPeripheral::checkSwitchToLowPowerLcd(float humidity, float temperature)
{  
  if (_LcdPowerModeSeconds == MAX_LCD_UPDATE_S) //go low power if long enough less than threshold
  {
    _LcdLowPowerMode = true;
    _LcdPowerModeSeconds = 0;
  }
  if ((abs(humidity - _humidityLcdCopy) <= THRESHOLD_LCD_UPDATE_RH)
    || (abs(temperature - _temperatureLcdCopy) <= THRESHOLD_LCD_UPDATE_DC))
  {
    _LcdPowerModeSeconds++;
  }
  else
  {
    _LcdPowerModeSeconds = 0;
  }
  return true;
}

bool SmartGadgetPeripheral::lcdVersionIsPlausible(uint8_t* version)
{
  return (((version[0] == 0x00) && (version[1] == 0x00)) ||
           ((version[0] == 0xFF) && (version[1] == 0xFF))) ? false : true;
}

void SmartGadgetPeripheral::displayTemperatureCelsius(void)
{
  _pShtLcd->setLine2(2, static_cast<int32_t>(100.0f * _temperatureLcdCopy));
  _pShtLcd->setSymbol(ShtLcd::SYMBOL_C2);
}
void SmartGadgetPeripheral::displayTemperatureFahrenheit(void)
{
  float fahrenheit = (_temperatureLcdCopy * 9 / 5 + 32);
  if (fahrenheit >= 100)
  {
    _pShtLcd->setLine2(1, static_cast<int32_t>(10.0f * fahrenheit));
  }
  else
  {
    _pShtLcd->setLine2(2, static_cast<int32_t>(100.0f * fahrenheit));
  }
  _pShtLcd->setSymbol(ShtLcd::SYMBOL_F2);
}
void SmartGadgetPeripheral::displayDevPointCelsius(void)
{
  _pShtLcd->setLine1(2, static_cast<int32_t>(100.0f * DewPoint::calculateDewPoint(_humidityLcdCopy, _temperatureLcdCopy)));
  _pShtLcd->setSymbol(ShtLcd::SYMBOL_C1);
}
void SmartGadgetPeripheral::displayDewPointFahrenheit(void)
{
  float fahrenheit = (DewPoint::calculateDewPoint(_humidityLcdCopy, _temperatureLcdCopy) * 9 / 5 + 32);
  if (fahrenheit >= 100)
  {
    _pShtLcd->setLine1(1, static_cast<int32_t>(10.0f * fahrenheit));
  }
  else
  {
    _pShtLcd->setLine1(2, static_cast<int32_t>(100.0f * fahrenheit));
  }
  _pShtLcd->setSymbol(ShtLcd::SYMBOL_F1);
}
void SmartGadgetPeripheral::displayHumidity(void)
{
  _pShtLcd->setLine1(2, static_cast<int32_t>(_humidityLcdCopy * 100.0f));
  _pShtLcd->setSymbol(ShtLcd::SYMBOL_RH);
}

void SmartGadgetPeripheral::displaySecondLcdLine()
{
  if (_isCelsius)
  {
    displayTemperatureCelsius();
  }
  else
  {
    displayTemperatureFahrenheit();    
  }
}

void SmartGadgetPeripheral::displayFirstLcdLine()
{
  if (_isDewPoint)
  {
    if (_isCelsius)
    {
      displayDevPointCelsius();      
    }
    else
    {
      displayDewPointFahrenheit();
    }
    _pShtLcd->setSymbol(ShtLcd::SYMBOL_DEWPOINT);
  }
  else
  {
    displayHumidity();
  }
}
