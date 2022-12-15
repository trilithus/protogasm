#pragma once
#include "commandmanager.h"

////////////////////////////////////////////////////////////////////////////
//DO NOT DEOPTIMIZE HERE!!!
//#pragma GCC push_options
//#pragma GCC optimize ("O0")

class CSVLogCommand : public Command
{
public:
  enum class TypeDescription : uint8_t 
  {
    type_int32,
    type_uint32,
    type_int,
    type_float,
  };
  CSVLogCommand() { cmdId = static_cast<uint8_t>(CommandID::CSVLog); };

  template <typename... Types>
  void LogCSV(Types... args) {
    constexpr uint8_t numberOfTypes = sizeof...(args);
    constexpr uint8_t alignment = ((4-(numberOfTypes%4))%4);
    const uint32_t typeBufferSz = getTypeSizes(args...);

    TypeDescription typeDescriptor[numberOfTypes+alignment];
    uint8_t sizeDescriptor[numberOfTypes+alignment];

    AllocateBuffer(4/*type desc length*/ + sizeof(typeDescriptor) + sizeof(sizeDescriptor) + typeBufferSz);
    describeType(&typeDescriptor[0], args...);
    describeSize(&sizeDescriptor[0], args...);

    uint8_t* ptrBuffer = reinterpret_cast<uint8_t*>(_buffer);
    ptrBuffer[0] = numberOfTypes;
    ptrBuffer[1] = alignment;
    ptrBuffer[2] = 0;
    ptrBuffer[3] = 0;
    ptrBuffer += 4;

    memcpy(ptrBuffer, &typeDescriptor[0], sizeof(typeDescriptor));
    ptrBuffer+=sizeof(typeDescriptor);

    memcpy(ptrBuffer, &sizeDescriptor[0], sizeof(sizeDescriptor));
    ptrBuffer+=sizeof(sizeDescriptor);

    writeToBuffer(ptrBuffer, args...);
  };
private:
  template <typename TType, typename... Types>
  constexpr uint32_t getTypeSizes(TType _type, Types... remainderArgs)
  {
    if constexpr(sizeof...(remainderArgs) > 0)
    {
      return sizeof(_type) + getTypeSizes(remainderArgs...);
    } else {
      return sizeof(_type);
    }
  }

  template <typename TType, typename... Types>
  constexpr uint32_t writeToBuffer(uint8_t* _buffer, TType _type, Types... remainderArgs)
  {
    memcpy(_buffer, &_type, sizeof(_type));
    _buffer += sizeof(_type);

    if constexpr(sizeof...(remainderArgs) > 0)
    {
      writeToBuffer(_buffer, remainderArgs...);
    }
  }

  template <typename T, typename... Types>
  constexpr void describeSize(uint8_t* sizeArray, T _param, Types... remainderArgs)
  {
    *sizeArray = sizeof(_param);
    sizeArray++;
    if constexpr(sizeof...(remainderArgs) > 0)
    {
      describeSize(sizeArray, remainderArgs...);
    } 
  }

  template <typename... Types>
  constexpr void describeType(TypeDescription* descriptionArray, int32_t _param, Types... remainderArgs)
  {
    *descriptionArray = TypeDescription::type_int32;
    descriptionArray++;
    if constexpr(sizeof...(remainderArgs) > 0)
    {
      describeType(descriptionArray, remainderArgs...);
    } 
  }

  template <typename... Types>
  constexpr void describeType(TypeDescription* descriptionArray, uint32_t _param, Types... remainderArgs)
  {
    *descriptionArray = TypeDescription::type_uint32;
    descriptionArray++;
    if constexpr(sizeof...(remainderArgs) > 0)
    {
      describeType(descriptionArray, remainderArgs...);
    } 
  }

  template <typename... Types>
  constexpr void describeType(TypeDescription* descriptionArray, float _param, Types... remainderArgs)
  {
    *descriptionArray = TypeDescription::type_float;
    descriptionArray++;
    if constexpr(sizeof...(remainderArgs) > 0)
    {
      describeType(descriptionArray, remainderArgs...);
    } 
  }
};