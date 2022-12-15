/*
#pragma GCC push_options
#pragma GCC optimize ("O0")
#define CMD_DEBUG 1
*/

#ifdef DEBUG_ESP_PORT
#define DEBUG_MSG(msg) DEBUG_ESP_PORT.print((const char*)msg)
#define DEBUG_MSGF(fmt, ...) DEBUG_ESP_PORT.printf((const char*)fmt,  __VA_ARGS__ )
#else
#define DEBUG_MSG(...)
#define DEBUG_MSGF(...)
#endif
#include <time.h>

#include <Wire.h>
#include "SdFat.h"
#include "sdios.h"
#include <Wire.h>
#include <CircularBuffer.h>

#include <NTPClient.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#if 0
#define FTP_SERVER_NETWORK_TYPE NETWORK_ESP8266
#define DEFAULT_FTP_SERVER_NETWORK_TYPE_ESP8266 NETWORK_ESP8266
#define DEFAULT_STORAGE_TYPE_ESP8266 				STORAGE_SDFAT2
#define STORAGE_TYPE STORAGE_SDFAT2
#define STORAGE_SPIFFS_FORCE_DISABLE
#include <SimpleFTPServer.h>
#endif

//Command manager:
#include "commandmanager.h"
#include "sharedcommands.h"

#define I2C_PROTOCOL 0

static int clamp(const int val, const int minVal, const int maxVal) { return (val<minVal) ? minVal : (val>maxVal ? maxVal : val); };

const uint8_t WIRE_PIN_SDA = D1;
const uint8_t WIRE_PIN_SCL = D2;
const uint8_t SD_CS_PIN = D8;

#define FIFO_SIZE_SECTORS 16
#define SPI_CLOCK SD_SCK_MHZ(15)
//#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SPI_CLOCK)
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SPI_CLOCK)

static CircularBuffer<uint8_t,1024*2> g_buffer;     // uses 538 bytes
static CommandAssembler g_assembler;

// Time:
bool g_hasTime = false;
WiFiUDP g_ntpUDP;
NTPClient g_timeClient(g_ntpUDP);

#if 0
// FTPd
FtpServer g_ftpd;
#endif

// Storage

#define SD_FAT_TYPE 1
#if SD_FAT_TYPE == 0
typedef SdFat sd_t;
typedef File file_t;
#elif SD_FAT_TYPE == 1
typedef SdFat32 sd_t;
typedef File32 file_t;
#elif SD_FAT_TYPE == 2
typedef SdExFat sd_t;
typedef ExFile file_t;
#elif SD_FAT_TYPE == 3
typedef SdFs sd_t;
typedef FsFile file_t;
#else  // SD_FAT_TYPE
#error Invalid SD_FAT_TYPE
#endif  // SD_FAT_TYPE

class SDWriter
{
  private:
    static sd_t _sd;
    file_t _sessionFile;
    uint32_t _sessionID = 0;
    uint32_t _written = 0;
  public:
    void Setup();
  
    void BeginSession();
    void EndSession();
    bool HasSession() const { return _sessionID != 0; }
    uint32_t GetSessionID() const { return _sessionID; }

    void Flush();
    void Write(const char* pString, uint32_t pLength=0);
    void WriteLn(const char* pString, uint32_t pLength=0);
} g_Storage;
sd_t SDWriter::_sd;
///////////////////////////////////////////////////////////////////
#define error(s) ::SDWriter::_sd.errorHalt(&Serial, F(s))
void SDWriter::BeginSession()
{
  if (_sessionID > 0)
  {
    Serial.println("Warning: Session was still running, ending.."); Serial.flush();
    EndSession();
  }

  constexpr const char* indexFilename = "index.txt";
  static constexpr auto bufferLength = (sizeof(uint32_t)*8+1);
  char buffer[bufferLength] = {0};
  uint32_t index=0;

#if 1
  file_t indexFile;
  if (indexFile.open(indexFilename, O_RDONLY)) 
  {
    DEBUG_MSGF(F("[SD] Opened %s\n"), indexFilename);
    const auto count = indexFile.read(&buffer[0], sizeof(buffer)-1);
    if (count >= 0 && count < sizeof(buffer))
    {
      buffer[count] = 0; //terminate string
      index = atoi(buffer);
    }
    indexFile.close();
  }
  else
  {
    DEBUG_MSGF(F("[SD] Failed to open %s\n"), indexFilename);
  }

  buffer[0] = 0;

  index++;
  itoa(index, buffer, 10);
  DEBUG_MSGF(F("[SD] Index now is %s\n"), buffer);

  if (indexFile.open(indexFilename, O_WRONLY | O_CREAT | O_TRUNC))
  {
    DEBUG_MSGF(F("[SD] Opened %s for writing\n"), indexFilename);
    const auto written = indexFile.write(buffer, strlen(buffer));
    indexFile.close();
  }
  else
  {
    DEBUG_MSGF(F("[SD] Failed to open %s for writing\n"), indexFilename);
    error("could not open file for writing..");
    return;
  }
#endif

  //Open session file:
  String filename = F("data_session_");
  filename += reinterpret_cast<const char*>(&buffer[0]);
  filename += F(".log");
  if (_sessionFile.open(filename.c_str(), O_WRONLY | O_CREAT | O_TRUNC))
  {
    _sessionFile.seekSet(0);
    DEBUG_MSGF(F("[SD] Opened %s for writing\n"), filename.c_str());
    _sessionID = index;
    _written = 0;
  }
  else
  {
    DEBUG_MSGF(F("[SD] Failed to open %s for writing\n"), filename.c_str());
    error("unable to open session file");
    return;
  }
}

void SDWriter::EndSession()
{
  if (_sessionID > 0)
  {
    const bool result = _sessionFile.close();
    DEBUG_MSG(result ? "[SD] Session file closed." : "[SD] Failed to close session file!!");
    if (_written <= 100)
    {
      Serial.println("Not enough data written, attempting to clean file."); Serial.flush();
      char buffer[32] = {0}; //should reuse buffer earlier
      itoa(_sessionID, buffer, 10);
      String filename = F("data_session_");
      filename += reinterpret_cast<const char*>(&buffer[0]);
      filename += F(".log");
      _sessionFile.remove(filename.c_str());
    }
    _sessionID = 0;
  }
}

void SDWriter::Setup()
{
  if (!_sd.begin(SD_CONFIG)) {
    _sd.initErrorHalt(&Serial);
  }
}

void SDWriter::Flush()
{
  if (HasSession())
  {
    DEBUG_MSG(F("[SD] Flushing...\n"));
    _sessionFile.flush();
  }
}

void SDWriter::Write(const char* pString, uint32_t pLength/*=0*/)
{
  if (HasSession())
  {
    const auto len = clamp((pLength>0 ? pLength : strlen(pString)), 0, 100);
    //DEBUG_MSGF(F("[SD] Writing %d bytes\n"), len);
    _sessionFile.write(pString, len);
    _written += len;
  }
}

void SDWriter::WriteLn(const char* pString, uint32_t pLength/*=0*/)
{
  size_t bytesWritten = 0;
  if (HasSession())
  {
    const auto len = (pLength>0 ? pLength : strlen(pString));
    if(len>0) 
    {
      bytesWritten += _sessionFile.write(pString, len);
    }

    bytesWritten += _sessionFile.write("\n", 1);

    if (bytesWritten==0)
    {
      //Handle error:
      Serial.print(F("Write error: ")); 
      Serial.print(_sessionFile.getWriteError());
      Serial.println("");
    }
  }
  _written += bytesWritten;
}
///////////////////////////////////////////////////////////////////
class CustomSessionBeginEnd : public SessionBeginEndCmd
{
public:
  virtual void Execute() const override;
};

void CustomSessionBeginEnd::Execute() const
{
  Serial.print(F("Session "));
  if (this->ptr[0] == 1)
  {
    Serial.println(F("begin"));
    g_Storage.BeginSession();
  } else {
    Serial.println(F("end"));
    g_Storage.EndSession();
  }
  Serial.flush();
}

#if 0
class CustomPrintCmd : public PrintCmd
{
public:
  virtual void Execute() const override;
};

void CustomPrintCmd::Execute() const
{
  Serial.write(this->GetString(), GetBufferSize());

  if (g_Storage.HasSession())
  {
    g_Storage.Write(this->GetString(), GetBufferSize());
    g_Storage.Flush();
  }
} 
#endif

class CustomCSVLog : public CSVLogCommand
{
public:
  virtual void Execute() const override
  {
    if (!g_Storage.HasSession())
    {
      return;
    }

    uint8_t* ptrBuffer = reinterpret_cast<uint8_t*>(_buffer);
    //for(int i=0; i < _bufferSz; i++)
    //{
    //  DEBUG_MSGF("%02X ", ptrBuffer[i]);
    //}

    uint8_t numberOfTypes = ptrBuffer[0]; 
    uint8_t alignment = ptrBuffer[1]; 
    ptrBuffer += 4;
    //DEBUG_MSGF("numberOfTypes = %d\n", numberOfTypes);
    //DEBUG_MSGF("alignment = %d\n", alignment); 

    TypeDescription* types = reinterpret_cast<TypeDescription*>(alloca(numberOfTypes * sizeof(TypeDescription)));
    memcpy(types, ptrBuffer, numberOfTypes * sizeof(TypeDescription));
    ptrBuffer+=numberOfTypes * sizeof(TypeDescription);
    ptrBuffer+=alignment;

    uint8_t* typeSizes = reinterpret_cast<uint8_t*>(alloca(numberOfTypes * sizeof(uint8_t)));
    memcpy(typeSizes, ptrBuffer, numberOfTypes * sizeof(uint8_t));
    ptrBuffer+=numberOfTypes * sizeof(uint8_t);
    ptrBuffer+=alignment;

    for(uint i=0; i < numberOfTypes; i++)
    {
      char buffer[32]{0};
      switch(types[i])
      {
        case TypeDescription::type_uint32: {
          //DEBUG_MSGF("[%d] uint32(%d) %02X %02X %02X %02X\n", i, typeSizes[i], ptrBuffer[0], ptrBuffer[1], ptrBuffer[2], ptrBuffer[3]); Serial.flush();
          sprintf(&buffer[0], "%u", *reinterpret_cast<uint32_t*>(ptrBuffer));
          ptrBuffer += typeSizes[i];
        } break;
        case TypeDescription::type_int32: {
          //DEBUG_MSGF("[%d] uint32(%d) %02X %02X %02X %02X\n", i, typeSizes[i], ptrBuffer[0], ptrBuffer[1], ptrBuffer[2], ptrBuffer[3]); Serial.flush();
          sprintf(&buffer[0], "%d", *reinterpret_cast<int32_t*>(ptrBuffer));
          ptrBuffer += typeSizes[i];
        } break;
        case TypeDescription::type_int: {
          //DEBUG_MSGF("[%d] int(%d) %02X %02X %02X %02X\n", i, typeSizes[i], ptrBuffer[0], ptrBuffer[1], ptrBuffer[2], ptrBuffer[3]); Serial.flush();
          sprintf(&buffer[0], "%d", *reinterpret_cast<short*>(ptrBuffer));
          ptrBuffer += typeSizes[i];
        } break;
        case TypeDescription::type_float: {
          //DEBUG_MSGF("[%d] float(%d) %02X %02X %02X %02X\n", i, typeSizes[i], ptrBuffer[0], ptrBuffer[1], ptrBuffer[2], ptrBuffer[3]); Serial.flush();
          sprintf(&buffer[0], "%f", *reinterpret_cast<float*>(ptrBuffer));
          ptrBuffer += typeSizes[i];
        } break;
      }

      //Serial.write(buffer);
      g_Storage.Write(buffer);
      DEBUG_MSG(buffer);

      if (i+1 == numberOfTypes) {
        g_Storage.WriteLn("");
        DEBUG_MSG("\n");
      } else {
        g_Storage.Write(",");
        DEBUG_MSG(",");
      }
    }
    g_Storage.Flush();
  }
};
//////////////////////////////////////////////////////////////////
void setup()
 {
  Serial.begin(230400); /* begin serial for debug */
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Initialize WIFI:
  WiFi.begin("EchelonIVDE", "DikkeBoss2005");
  Serial.print("Connecting");
  auto now = millis();
  bool timeout = false;
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
    if (millis() > now+(1000*30))
    {
      timeout = true;
      break;
    }
  }
  Serial.println();
  if (!timeout)
  {
    Serial.print("Connected, IP address: ");
    Serial.println(WiFi.localIP());
    Serial.print("hwaddress: ");
    Serial.println(WiFi.macAddress().c_str());
  } else {
    Serial.println("Wifi Timeout");
  }
  if (!timeout) 
  {
    g_hasTime = true;
    g_timeClient.begin();
    SdFile::dateTimeCallback([](uint16_t* date, uint16_t* time){
      time_t now = g_timeClient.getEpochTime();
      auto tm = localtime(&now);
      *date = FS_DATE(tm->tm_year+1900, tm->tm_mon, tm->tm_mday);
      *time = FS_TIME(tm->tm_hour, tm->tm_min, tm->tm_sec);
    });
  }

  //g_ftpd.begin();

  DEBUG_MSG("Hello low level print\n");
  Wire.begin(WIRE_PIN_SDA, WIRE_PIN_SCL, 0x43); /* join i2c bus with SDA=D1 and SCL=D2 of NodeMCU */
  Wire.onReceive(i2cReceiveEvent); /* register receive event */
  //#if defined(I2C_PROTOCOL) && I2C_PROTOCOL==0
  Wire.onRequest(i2cRequestEvent); /* register request event */
  //#endif
  //Wire.setClock(30);
  //Wire.setTimeout(1000);
  //Wire.setClockStretchLimit(1e6*10);

  {
    static constexpr uint8_t cid = static_cast<uint8_t>(CommandID::SessionBeginEnd);
    CommandAssembler::RegisterCommand<cid>([]()->Command*{
      return new CustomSessionBeginEnd();
    });
  }
  {
    static constexpr uint8_t cid = static_cast<uint8_t>(CommandID::CSVLog);
    CommandAssembler::RegisterCommand<cid>([]()->Command*{
      return new CustomCSVLog();
    });
  }
  #if 0
  {
    static constexpr uint8_t cid = static_cast<uint8_t>(CommandID::Print);
    CommandAssembler::RegisterCommand<cid>([]()->Command*{
      return new CustomPrintCmd();
    });
  }
  #endif

  // Initialize SD.
  g_Storage.Setup();
  Serial.println("Listening...");

  #if defined(I2C_PROTOCOL) && I2C_PROTOCOL==1
  g_assembler.ListenForCommand();
  #endif

  digitalWrite(LED_BUILTIN, HIGH);
}

uint32_t i = 0;
uint8_t _requestsPending = 0;
uint32_t recv = 0;
void i2cReceiveEvent(int howMany) 
{
  while (Wire.available()) 
  { 
    const uint8_t b = Wire.read();
    if (g_buffer.push(b) == false)
    {
      Serial.println("Warning! Circularbuffer went around!");
      Serial.flush();
    }
  }
  recv += howMany;
}

void i2cRequestEvent() 
{
  #if defined(I2C_PROTOCOL) && I2C_PROTOCOL==0
  _requestsPending++;
  if (!g_buffer.size()==0)
  {
    Serial.println(F("Processing too slow"));
  }
  g_buffer.clear();
  Wire.write(g_Storage.GetSessionID()); //reply
  #endif
}

void loop()
{
  if (g_hasTime)
  {
    g_timeClient.update();
  }
   //g_ftpd.handleFTP();

#if defined(I2C_PROTOCOL) && I2C_PROTOCOL==0
  static uint8_t lastRequests = 0;
  if (_requestsPending != lastRequests)
  {
    if (_requestsPending > 0)
    {
      g_assembler.StartCommand();
    }
    lastRequests = _requestsPending;
  }
#endif

  if (g_buffer.size() > 0)
  {
    const auto b = g_buffer.shift();
    #if defined(I2C_PROTOCOL) && I2C_PROTOCOL==0
    if (_requestsPending > 0)
    #endif
    {
      g_assembler.receiveByte(b);
      if (g_assembler.IsDone())
      {
        //pinMode(WIRE_PIN_SDA, INPUT);
        //digitalWrite(WIRE_PIN_SDA, LOW);
        _requestsPending = 0; //not multihost
        noInterrupts();
        g_assembler.GetCommand()->Execute();
        delay(1);
        interrupts();
        g_assembler.DeleteCommand();
        //pinMode(WIRE_PIN_SDA, OUTPUT);
        #if defined(I2C_PROTOCOL) && I2C_PROTOCOL==1
        g_assembler.ListenForCommand();
        #endif
      }
    }
  }
  yield();
}
