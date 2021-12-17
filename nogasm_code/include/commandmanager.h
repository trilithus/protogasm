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
          Serial.println("Begin CommandAssembler cmd");
        #endif
        DeleteCommand(); 
        _state = stateGetCmdId; 
        _cmdIdRecv = 0;
        _cmdLenRecv = 0;
        _cmdContentRecv = 0;
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
            Serial.println("Deleting CommandAssembler cmd");
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
        Serial.print("stateGetCmdId: ");
        Serial.println(_cmdId);
        #endif
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
          #if defined(CMD_DEBUG) && CMD_DEBUG==1
          Serial.print("cmdlen:");
          Serial.println(_cmdLen);
          #endif
          _state = stateGetCmdContent;
          this->CreateCommand(_cmdId, _cmdLen);
        }
        #endif
      } else if (_state == stateGetCmdContent)
      {
        #if defined(CMD_DEBUG) && CMD_DEBUG==1
        Serial.println("stateGetCmdContent");
        #endif
        if (_newCmd == nullptr || _newCmd->GetBufferSize() == 0)
        {
          #if defined(CMD_DEBUG) && CMD_DEBUG==1
          Serial.println("Done");
          #endif
          _state = stateCmdDone;
          return;  
        }
        uint8_t* buffer = reinterpret_cast<uint8_t*>(_newCmd->GetBuffer());
        buffer[_cmdContentRecv] = byteValue;
#if defined(CMD_DEBUG) && CMD_DEBUG == 1
        Serial.print("stateGetCmdContent recv: ");
        Serial.print(_cmdContentRecv+1);
        Serial.print("/");
        Serial.println(_newCmd->GetBufferSize());
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