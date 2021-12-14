#pragma once
#include "commandmanager.h"
////////////////////////////////////////////////////////////////////////////

enum class CommandID : uint8_t
{
  UNDEFINED,
  BeginSession,
  EndSession,
  Print,
  PrintLn,
};

class PrintCmd : public Command
{
public:
  PrintCmd() { cmdId = static_cast<uint8_t>(CommandID::Print); };
  void  SetString(const char* pValue) 
  {
    const uint32_t strLen = strlen(pValue);
    if (strLen > 0)
    {
      AllocateBuffer(strLen);
      memcpy(ptr, pValue, strLen);
    }
    else
    {
      AllocateBuffer(1);
      ptr[0] = 0;
    }
  }
  const char* GetString() const
  {
    return ptr;
  }

  virtual void Execute() const 
  {
    #if defined(CMD_DEBUG) && CMD_DEBUG==1
    Serial.println("print cmd execute");
    #endif
    Serial.write(ptr, GetBufferSize());
  }
private:
  char* &ptr = reinterpret_cast<char*&>(_buffer);
};

struct
{
  struct Inner
  {
    Inner()
    {
      CommandAssembler::RegisterCommand<static_cast<uint8_t>(CommandID::Print)>([]()->Command*{ return new PrintCmd(); });
    }
  } _innerMe;
} _registerMe;