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

#include "RingBuffer.h"

RingBuffer::RingBuffer(uint32_t firstPage, uint32_t lastPage):_pageNumberFirst(firstPage), _pageNumberLast(lastPage), _nextPointer(0), _counter(0)
{
  FlashCmdQueue::init();
}

uint64_t RingBuffer::write(float val)
{
  if (_nextPointer % NRF51822_PAGESIZE_BYTES == 0) //new Flash Page reached -> erase next
  {
    if (_nextPointer / NRF51822_PAGESIZE_BYTES + _pageNumberFirst > _pageNumberLast) // last page reached, start over
    {
      _nextPointer = 0;
    }
    FlashCmdQueue::push(_pageNumberFirst + _nextPointer / NRF51822_PAGESIZE_BYTES);
  }
  // write value to the destination (offset+base)
  FlashCmdQueue::push(reinterpret_cast<uint32_t*>(_nextPointer + NRF51822_PAGESIZE_BYTES * _pageNumberFirst), val);
  _nextPointer += BYTES_PER_VALUE;

  return _counter++;
}

bool RingBuffer::read(uint64_t pos, float* val)
{
  if (pos >= _counter) //can not read future values
  {
    return false;
  }
  if ((_counter - pos) > ((NRF51822_PAGESIZE_BYTES * (_pageNumberLast - _pageNumberFirst)) / BYTES_PER_VALUE)) //can not read (possibly) overwritten values
  {
    return false;
  }
  // calculate address and read float value
  uint64_t addr = (pos * BYTES_PER_VALUE) % (NRF51822_PAGESIZE_BYTES * (_pageNumberLast - _pageNumberFirst + 1));
  addr += _pageNumberFirst * NRF51822_PAGESIZE_BYTES;
  *val = *reinterpret_cast<float*>((uint32_t)addr);
  return true;
}

uint32_t RingBuffer::getRingBufferMinSizeFt()
{
  // Amount of flash pages minus one gives the size you have at least available for values
  // minus one because if a flash page is full, the next is erased (ring buffer style)
  return ((_pageNumberLast - _pageNumberFirst - 1) * NRF51822_PAGESIZE_BYTES / BYTES_PER_VALUE);
}

void RingBuffer::reset()
{
  _counter = 0;
  _nextPointer = 0;
}
