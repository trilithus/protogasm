//SD library:
#include "SdFat.h"
#include "sdios.h"
#include <Wire.h>

#pragma GCC push_options
#pragma GCC optimize ("O0")

//Command manager:
#include "commandmanager.h"
#include "sharedcommands.h"

//SD configuration:
const uint8_t SD_CS_PIN = D8;
const uint8_t WIRE_PIN_SDA = D1;
const uint8_t WIRE_PIN_SCL = D2;

////////////////////////////////////////////////////////
//SD library initialization:
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
#define SPI_CLOCK SD_SCK_MHZ(15)

// Try to select the best SD card configuration.
//#if HAS_SDIO_CLASS
//#define SD_CONFIG SdioConfig(FIFO_SDIO)

//#if  ENABLE_DEDICATED_SPI
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SPI_CLOCK)
//#else  // HAS_SDIO_CLASS
//#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SPI_CLOCK)
//#endif  // HAS_SDIO_CLASS

////////////////////////////////////////////////////////
// Global objects
typedef SdExFat sd_t;
typedef ExFile  file_t;
static sd_t g_sd;
CommandAssembler g_assembler;
/// helpers
#define sderror(s) g_sd.errorHalt(&Serial, F(s))

void setup() {
 Serial.begin(9600); /* begin serial for debug */

   // Initialize SD.
  if (!g_sd.begin(SD_CONFIG)) {
    g_sd.initErrorHalt(&Serial);
  }

   ////////////////////////////////////////////////////////
 // i2c initialization
 Wire.begin(WIRE_PIN_SDA, WIRE_PIN_SCL, 8); /* join i2c bus with SDA=D1 and SCL=D2 of NodeMCU */
 Wire.onReceive(i2cReceiveEvent); /* register receive event */
 Wire.onRequest(i2cRequestEvent); /* register request event */
 Wire.setClock(30);
 Wire.setTimeout(1e6*10);
 Serial.println("Initialized");
}

bool _assembleCommand = false;
void loop() {
  // put your main code here, to run repeatedly:
  testSD();
}

void i2cReceiveEvent(int howMany) 
{
  while (Wire.available()) 
  { 
    const uint8_t b = Wire.read();
    if (_assembleCommand)
    {
        g_assembler.receiveByte(b);
        if (g_assembler.IsDone())
        {
          _assembleCommand = false;
          g_assembler.GetCommand()->Execute();
          g_assembler.DeleteCommand();
        }  
    }
  }
}

void i2cRequestEvent() 
{
  if (_assembleCommand)
  {
    g_assembler.DeleteCommand();
  }
  g_assembler.StartCommand();
  _assembleCommand = true;
  Wire.write(1);
}

static auto loops = 0;
static auto _sessionID = 0;
static file_t _sessionFile;
void testSD() {
  loops++;
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(3000);

  constexpr const char* indexFilename = "index.txt";
  char buffer[32] = {0};
  uint32_t index=0;

  file_t indexFile;
  if (indexFile.open(indexFilename, O_RDONLY)) 
  {
    const auto count = indexFile.read(&buffer[0], sizeof(buffer)-1);
    if (count >= 0 && count < sizeof(buffer))
    {
      buffer[count] = 0; //terminate string
      index = atoi(buffer);
      Serial.print("index: ");
      Serial.println(index);
    }
    indexFile.close();
  }
  else
  {
    Serial.println("index file not read");
  }
  buffer[0] = 0;

  index++;
  itoa(index, &buffer[0], 10);
  

  if (indexFile.open(indexFilename, O_WRONLY | O_CREAT | O_TRUNC))
  {
    indexFile.write(buffer, strlen(buffer));
    indexFile.flush();
    indexFile.close();
  }
  else
  {
    sderror("could not open file for writing..");
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
    sderror("unable to open session file");
    return;
  }
  _sessionFile.flush();
  _sessionFile.close();
/*
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  if (dataFile)
  {
    dataFile.println(loops);
    dataFile.close();
  }
  */
}
