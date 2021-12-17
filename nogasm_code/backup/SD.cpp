/////////////////////////////////////////////////////
// Storage class definition
//Storage:
#define SD_FAT_TYPE 0
// SDCARD_SS_PIN is defined for the built-in SD on some boards.
#ifndef SDCARD_SS_PIN
const uint8_t SD_CS_PIN = SS;
#else  // SDCARD_SS_PIN
// Assume built-in SD is used.
const uint8_t SD_CS_PIN = SDCARD_SS_PIN;
#endif  // SDCARD_SS_PIN

// FIFO SIZE - 512 byte sectors.  Modify for your board.
#ifdef __AVR_ATmega328P__
// Use 512 bytes for 328 boards.
#define FIFO_SIZE_SECTORS 1
#elif defined(__AVR__)
// Use 2 KiB for other AVR boards.
#define FIFO_SIZE_SECTORS 4
#else  // __AVR_ATmega328P__
// Use 8 KiB for non-AVR boards.
#define FIFO_SIZE_SECTORS 16
#endif  // __AVR_ATmega328P__

// Try max SPI clock for an SD. Reduce SPI_CLOCK if errors occur.
#define SPI_CLOCK SD_SCK_MHZ(50)

// Try to select the best SD card configuration.
#if HAS_SDIO_CLASS
#define SD_CONFIG SdioConfig(FIFO_SDIO)
#elif  ENABLE_DEDICATED_SPI
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SPI_CLOCK)
#else  // HAS_SDIO_CLASS
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SPI_CLOCK)
#endif  // HAS_SDIO_CLASS

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

#if 1
class SDWriter
{
  private:
    static sd_t _sd;
    file_t _sessionFile;
    uint32_t _sessionID = 0;
  public:
    void Setup();
  
    void BeginSession();
    void EndSession();
    bool HasSession() const { return _sessionID != 0; }

    void Write(const char* pString);
    void WriteLn(const char* pString);
} g_Storage;
sd_t SDWriter::_sd;
#endif




#if 1
///////////////////////////////////////////////////////////////////
#define error(s) ::SDWriter::_sd.errorHalt(&Serial, F(s))
void SDWriter::BeginSession()
{
  if (_sessionID > 0)
  {
    EndSession();
  }

  constexpr const char* indexFilename = "index.txt";
  uint8_t buffer[32] = {0};
  uint32_t index=0;

  file_t indexFile;
  if (indexFile.open(indexFilename, O_RDONLY)) 
  {
    const auto count = indexFile.read(&buffer[0], sizeof(buffer)-1);
    if (count >= 0 && count < sizeof(buffer))
    {
      buffer[count] = 0; //terminate string
      index = atoi(buffer);
    }
    indexFile.close();
  }
  buffer[0] = 0;

  index++;
  itoa(index, buffer, 10);

  if (indexFile.open(indexFilename, O_WRONLY | O_CREAT | O_TRUNC))
  {
    indexFile.write(buffer, strlen(buffer));
    indexFile.flush();
    indexFile.close();
  }
  else
  {
    error("could not open file for writing..");
    return;
  }

  //Open session file:
  String filename = F("data_session_");
  filename += reinterpret_cast<const char*>(&buffer[0]);
  filename += F(".log");
  if (_sessionFile.open(filename.c_str(), O_WRONLY | O_CREAT | O_TRUNC))
  {
    _sessionID = index;
  }
  else
  {
    error("unable to open session file");
    return;
  }
}

void SDWriter::EndSession()
{
  if (_sessionID > 0)
  {
    _sessionID = 0;
    _sessionFile.flush();
    _sessionFile.close();
  }
}

void SDWriter::Setup()
{
  if (!_sd.begin(SD_CONFIG)) {
    _sd.initErrorHalt(&Serial);
  }
}

void SDWriter::Write(const char* pString)
{
  if (HasSession())
  {
    const auto len = strlen(pString);
    _sessionFile.write(pString, len);
  }
}

void SDWriter::WriteLn(const char* pString)
{
  if (HasSession())
  {
    const auto len = strlen(pString);
    _sessionFile.write(pString, len);
    _sessionFile.write("\n", 1);
  }
}
#endif