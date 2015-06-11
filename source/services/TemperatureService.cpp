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

#include "TemperatureService.h"
//BASE UUID: 00000000-B38D-4985-720E-0F993A68EE41

const uint16_t TemperatureServiceShortUUID = 0x2234;
const uint16_t TemperatureServiceTemperatureValueCharacteristicShortUUID = 0x2235;

const uint8_t TemperatureServiceUUID[] = {
  0x00, 0x00, static_cast<uint8_t>(TemperatureServiceShortUUID >> 8), static_cast<uint8_t>(TemperatureServiceShortUUID & 0xFF), 0xb3, 0x8d, 0x49, 0x85,
  0x72, 0x0e, 0x0f, 0x99, 0x3a, 0x68, 0xee, 0x41,
};
const uint8_t TemperatureServiceTemperatureValueCharacteristicUUID[] = {
  0x00, 0x00, static_cast<uint8_t>(TemperatureServiceTemperatureValueCharacteristicShortUUID >> 8), static_cast<uint8_t>(TemperatureServiceTemperatureValueCharacteristicShortUUID & 0xFF), 0xb3, 0x8d, 0x49, 0x85,
  0x72, 0x0e, 0x0f, 0x99, 0x3a, 0x68, 0xee, 0x41,
};
