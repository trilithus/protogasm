#pragma once
#include "commandmanager.h"
enum class CommandID : uint8_t
{
  UNDEFINED,
  Print,
  CSVLog,
  SessionBeginEnd,
};

#include "csvlogcommand.h"
////////////////////////////////////////////////////////////////////////////
//DO NOT DEOPTIMIZE HERE!!!
//#pragma GCC push_options
//#pragma GCC optimize ("O0")

#if 0
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
    Serial.flush();
  }
private:
  char* &ptr = reinterpret_cast<char*&>(_buffer);
};
#endif

class SessionBeginEndCmd : public Command
{public:
  SessionBeginEndCmd() { cmdId = static_cast<uint8_t>(CommandID::SessionBeginEnd); };
  void SetValue(bool value) 
  {
    AllocateBuffer(sizeof(uint32_t));
    ptr[0]=value?1:0; 
  }

  virtual void Execute() const 
  {
  }
protected:
  uint32_t* &ptr = reinterpret_cast<uint32_t*&>(_buffer);
};

struct
{
  struct Inner
  {
    Inner()
    {
      //CommandAssembler::RegisterCommand<static_cast<uint8_t>(CommandID::Print)>([]()->Command*{ return new PrintCmd(); });
      CommandAssembler::RegisterCommand<static_cast<uint8_t>(CommandID::CSVLog)>([]()->Command*{ return new CSVLogCommand(); });
      CommandAssembler::RegisterCommand<static_cast<uint8_t>(CommandID::SessionBeginEnd)>([]()->Command*{ return new SessionBeginEndCmd(); });
    }
  } _innerMe;
} _registerMe;
