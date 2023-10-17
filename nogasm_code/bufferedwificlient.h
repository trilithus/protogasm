#pragma once

#include <WiFiS3.h>
#include <CircularBuffer.h>

template<uint8_t TScratchSz>
class BufferedWiFiClient : public WiFiClient
{
private:
    CircularBuffer<uint8_t, 4096> _buffer;
    uint8_t isFlushing=0;
    uint64_t writeCount=0;
    bool _errorState=false;
public:
  bool hasError() const { return _errorState; }
  void clearError() { _errorState = false; }
  void clear() { _buffer.clear(); }

  virtual size_t write(uint8_t c)
  {
    if (isFlushing==0)
    {
      if (_buffer.isFull())
      {
        Serial.println("BufferedWiFiClient buffer is full during write, flushing...");
        Serial.print("Flushed ");
        Serial.print(flushOut(-1));
        Serial.println(" bytes.");
      }

      if (!_buffer.isFull())
      {
          _buffer.push(c);
          return 1;
      }
      return 0;
    }
    else
    {
      return WiFiClient::write(c);
    }
  }

  virtual size_t write(const uint8_t *buf, size_t size)
  {
    if (isFlushing==0)
    {
      uint32_t written = 0;
      for(int i=0; i < size; i++)
      {
        written += write(buf[i]);
      }
      //Serial.print("Wrote "); Serial.print(written); Serial.println(" bytes to buffer");
      return written;
    }
    else
    {
      return WiFiClient::write(buf,size);
    }
  }

  size_t flushOut(int8_t maxBytes=TScratchSz)
  {
    isFlushing++;

    uint8_t scratchBuffer[TScratchSz];
    uint8_t* scratchBufferPtr = &scratchBuffer[0];

    const uint32_t bytesToWrite = (maxBytes <= -1) ? _buffer.size() : std::min(uint32_t(_buffer.size()), uint32_t(maxBytes));
    uint32_t bytesWritten=0;

    uint8_t scratchBufferWritten=0;
    
    for(int i=0; i < bytesToWrite; i++)
    {
      *scratchBufferPtr = _buffer.shift();
      scratchBufferPtr++;
      scratchBufferWritten++;

      if (scratchBufferWritten == std::size(scratchBuffer) || i+1 == bytesToWrite)
      { /*flush*/
        bytesWritten += WiFiClient::write(&scratchBuffer[0], scratchBufferWritten);
        scratchBufferPtr = &scratchBuffer[0];
        scratchBufferWritten = 0;
      }
    }

    if (bytesToWrite>0 && bytesWritten==0)
    {
      _errorState = true;
    }
    isFlushing--;
    return bytesWritten;
  }

  bool HasDataToWriteOut() const { return _buffer.size(); }
};