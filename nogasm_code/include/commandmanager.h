#pragma once

//#define CMD_DEBUG 1

//command system:
class Command
{
  friend class CommandAssembler;
public:
  uint8_t cmdId = 0;
  ~Command() { ReleaseBuffer(); }
protected:
  uint32_t _bufferSz{};
  void* _buffer = nullptr;

  void AllocateBuffer(const uint32_t bufferSize) { 
    if (_buffer != nullptr) ReleaseBuffer();
    _buffer = malloc(bufferSize);
    _bufferSz = bufferSize;
  }
  void ReleaseBuffer()
  {
    if (_buffer != nullptr) free(_buffer);
    _buffer = nullptr;
  }
public:
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
  public:
    void StartCommand() 
    { 
        DeleteCommand(); 
        _state = stateGetCmdId; 
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
            Serial.println("Deleting CommandAssembler cmd");
            #endif
            delete(_newCmd);
            _newCmd = nullptr;
        }
    }
    void receiveByte(uint8_t byteValue)
    {
      #if defined(CMD_DEBUG) && CMD_DEBUG==1
      Serial.print("State: ");
      #endif

      if (_state == stateGetCmdId)
      {
        #if defined(CMD_DEBUG) && CMD_DEBUG==1
        Serial.println("stateGetCmdId");
        #endif
        _cmdId  = byteValue;
        _cmdIdRecv++;
        _state = stateGetCmdLen;
      }
      else if (_state == stateGetCmdLen)
      {
        #if 1
        #if defined(CMD_DEBUG) && CMD_DEBUG==1
        Serial.println("stateGetCmdLen");
        #endif
        uint8_t* buffer = reinterpret_cast<uint8_t*>(&_cmdLen);
        buffer[_cmdLenRecv] = byteValue;

        if (++_cmdLenRecv == sizeof(_cmdLen))
        {
          //Got length, assemble command:
          _state = stateGetCmdContent;
          this->CreateCommand(_cmdId, _cmdLen);
        }
        #endif
      } else if (_state == stateGetCmdContent)
      {
        #if 0
        #if defined(CMD_DEBUG) && CMD_DEBUG==1
        Serial.println("stateGetCmdContent");
        #endif
        #endif
        if (_newCmd == nullptr || _newCmd->GetBufferSize() == 0)
        {
          _state = stateCmdDone;
          return;  
        }
        uint8_t* buffer = reinterpret_cast<uint8_t*>(_newCmd->GetBuffer());
        buffer[_cmdContentRecv] = byteValue;
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
      Serial.print("cmdID: "); Serial.println(static_cast<uint8_t>(cmdId));
      #endif
      if (_commands[cmdId] == nullptr)
      {
        Serial.println("Command not registered");
        while(true) {};
      }
      _newCmd = _commands[cmdId]();
      if (_newCmd != nullptr && bufferSz>0)
      {
        _newCmd->AllocateBuffer(bufferSz);
      }
      else
      {
        _newCmd = nullptr;
      }
    }
};

CommandAssembler::CreateCommandT CommandAssembler::_commands[5]{};