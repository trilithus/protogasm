#pragma once

#ifndef DEBUG_MSG
#ifdef DEBUG_ESP_PORT
#define DEBUG_MSG(msg) DEBUG_ESP_PORT.print((const char*)msg)
#define DEBUG_MSGF(fmt, ...) DEBUG_ESP_PORT.printf((const char*)fmt,  __VA_ARGS__ )
#else
#define DEBUG_MSG(...)
#define DEBUG_MSGF(...)
#endif
#endif

//command system:
class Command
{
  friend class CommandAssembler;
public:
  uint8_t cmdId = 0;
  ~Command() { ReleaseBuffer(); }
protected:
  uint32_t _bufferSz{};
  unsigned long _delay=0;
  void* _buffer = nullptr;
  void* _rawbuffer = nullptr;

  void AllocateBuffer(const uint32_t bufferSize) { 
    if (_buffer != nullptr) ReleaseBuffer();
    _rawbuffer = malloc(bufferSize+3);
    if ((uintptr_t)_rawbuffer%4 != 0)
    {
      DEBUG_MSG("UNALLIGNED!!\n");
      _buffer = reinterpret_cast<void*>((uintptr_t)_rawbuffer + (4-(uintptr_t)_rawbuffer%4) );
    }
    else
    {
      _buffer = _rawbuffer;
    }
    _bufferSz = bufferSize;
  }
  void ReleaseBuffer()
  {
    if (_buffer != nullptr) free(_rawbuffer);
    _rawbuffer = nullptr;
    _buffer = nullptr;
  }
public:
  void SetDelay(unsigned long delay) { _delay = delay; }
  unsigned long GetDelay() const { return _delay; }
  void* GetBuffer() { return _buffer; }
  uint32_t GetBufferSize() const { return _bufferSz; }

  virtual void Execute() const { return; }
};

class CommandAssembler 
{
  private:
    enum State
    {
      stateINVALID,
      stateWaitMagic,
      stateGetCmdId,
      stateGetCmdLen,
      stateGetCmdContent,
      stateCmdDone,
    } _state = stateINVALID;

    uint8_t _cmdIdRecv = 0;
    uint8_t _cmdId = 0;

    uint8_t _cmdLenRecv = 0;
    uint32_t _cmdLen = 0;

    uint8_t _cmdContentRecv = 0;
    Command* _newCmd = nullptr;

    uint8_t _magic[4]{};
  public:
    void StartCommand() 
    { 
        #if defined(CMD_DEBUG) && CMD_DEBUG==1
        DEBUG_MSG(F("Begin CommandAssembler cmd\n"));
        #endif
        DeleteCommand(); 
        _state = stateGetCmdId; 
        _cmdIdRecv = 0;
        _cmdLenRecv = 0;
        _cmdContentRecv = 0;
        #if defined(CMD_DEBUG) && CMD_DEBUG==1
        DEBUG_MSG(F("Begin CommandAssembler cmd\n"));
        #endif
    }
    void ListenForCommand()
    {
        DeleteCommand(); 
        _state = stateWaitMagic; 
        _cmdIdRecv = 0;
        _cmdLenRecv = 0;
        _cmdContentRecv = 0;
    }
    bool IsDone() const { return _state == stateCmdDone; }
    const Command* GetCommand() const { return _newCmd; }

    typedef Command* (*CreateCommandT)();
    static CreateCommandT _commands[5];
    template<uint8_t CID>
    static void RegisterCommand(CreateCommandT arg)
    {
      if (CID >= 0 && CID < (sizeof(_commands)/sizeof(_commands[0])))
      {
        _commands[CID] = arg;
      }
    }

    void DeleteCommand() {
        if (_newCmd != nullptr)
        {
            #if defined(CMD_DEBUG) && CMD_DEBUG==1
            DEBUG_MSG(F("Deleting CommandAssembler cmd\n"));
            #endif
            delete(_newCmd);
            _newCmd = nullptr;
        }
    }
    void receiveByte(uint8_t byteValue)
    {
      if (_state == stateWaitMagic)
      {
        _magic[0] = _magic[1];
        _magic[1] = _magic[2];
        _magic[2] = _magic[3];
        _magic[3] = byteValue;

        if (_magic[0] == 0xDE &&
            _magic[1] == 0xAD  && 
            _magic[2] == 0xBE &&
            _magic[3] == 0xEF)
            {
              StartCommand();
            }

      } else if (_state == stateGetCmdId)
      {
        _cmdId  = byteValue;
        _cmdIdRecv++;
        _state = stateGetCmdLen;
        #if defined(CMD_DEBUG) && CMD_DEBUG==1
        DEBUG_MSGF(F("stateGetCmdId: %d\n"), _cmdId);
        #endif
      }
      else if (_state == stateGetCmdLen)
      {
        #if 1
        #if defined(CMD_DEBUG) && CMD_DEBUG==1
        DEBUG_MSG(F("stateGetCmdLen\n"));
        #endif
        uint8_t* buffer = reinterpret_cast<uint8_t*>(&_cmdLen);
        buffer[_cmdLenRecv] = byteValue;

        if (++_cmdLenRecv == sizeof(_cmdLen))
        {
          //Got length, assemble command:
          #if defined(CMD_DEBUG) && CMD_DEBUG==1
          DEBUG_MSGF("cmdlen: %d\n", _cmdLen);
          #endif
          _state = stateGetCmdContent;
          this->CreateCommand(_cmdId, _cmdLen);
        }
        #endif
      } else if (_state == stateGetCmdContent)
      {
        #if defined(CMD_DEBUG) && CMD_DEBUG==1
        DEBUG_MSG(F("stateGetCmdContent\n"));
        #endif
        if (_newCmd == nullptr || _newCmd->GetBufferSize() == 0)
        {
          #if defined(CMD_DEBUG) && CMD_DEBUG==1
          DEBUG_MSG(F("Done\n"));
          #endif
          _state = stateCmdDone;
          return;  
        }
        uint8_t* buffer = reinterpret_cast<uint8_t*>(_newCmd->GetBuffer());
        buffer[_cmdContentRecv] = byteValue;
#if defined(CMD_DEBUG) && CMD_DEBUG == 1
        DEBUG_MSGF(F("stateGetCmdContent recv: %d/%d\n"), _cmdContentRecv+1, _newCmd->GetBufferSize());
#endif
        if (++_cmdContentRecv == _newCmd->GetBufferSize())
        {
          _state = stateCmdDone;
          return;
        }
      }
    }
private:
    Command* CreateCommand(uint8_t cmdId, const uint32_t bufferSz)
    {
      #if defined(CMD_DEBUG) && CMD_DEBUG==1
      DEBUG_MSGF(F("cmdID: %d\n"), cmdId);
      #endif
      if (_commands[cmdId] == nullptr)
      {
        Serial.println(F("Command not registered"));
        while(true) {};
      }
      if (_newCmd != nullptr)
      {
        Serial.println(F("ERROR - Command not freed"));
      }

      _newCmd = _commands[cmdId]();
      if (_newCmd != nullptr)
      {
        if (bufferSz > 0)
        {
          _newCmd->AllocateBuffer(bufferSz);
        }
      }
    }
};

CommandAssembler::CreateCommandT CommandAssembler::_commands[5]{};