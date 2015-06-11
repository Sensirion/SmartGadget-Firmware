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

#ifndef RINGBUFFER_H
#define RINGBUFFER_H

#include "mbed.h"
#include "FlashCmdQueue.h"

/**
* @class RingBuffer
* @brief A Ring Buffer (FIFO) class that uses the on-chip flash. The size is 
* given in Pages (NRF51822: 1kB per page)<br>
**/
class RingBuffer
{
public:
  /** Create a RingBuffer.
     *
     *  @param firstPage that can be used
     *  @param lastPage to use. lastPage has to be higher than firstPage.
     */
  RingBuffer(uint32_t firstPage, uint32_t lastPage);
  /** Write a value (float) to the ring buffer
     *
     *  @param val value to write
     */
  uint64_t write(float val);
  /** Read a value (float) from the ring buffer
     *
     *  @param pos position of the value (always increases when buffering new 
     *  values!)
     *  @param val location where to write the read value
     *  @returns
     *    non-0 success.
     *        0 failure. Position requested lies in the future or is already 
     *        overwritten
     */
  bool read(uint64_t pos, float* val);
  /** Get the minimal available buffer size, which is number of pages - 1
     *  @returns
     *    an integer representing the available space in "units" 
     *    (here: float/4Bytes)
     */
  uint32_t getRingBufferMinSizeFt();
  /** reset the Buffer pointers and start over
     *
     */
  void reset();

private:
  enum privateConstants
  {
    NRF51822_PAGESIZE_BYTES = 1024,
    BYTES_PER_VALUE = 4
  };

  uint32_t _pageNumberFirst;
  uint32_t _pageNumberLast;
  uint32_t _nextPointer;
  uint64_t _counter;
};

#endif
