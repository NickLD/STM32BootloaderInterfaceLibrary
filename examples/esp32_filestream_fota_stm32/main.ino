#include <Arduino.h>
#include "SPIFFS.h"
#include <STM32BootloaderInterface.h>
#include <MD5Builder.h>

HardwareSerial stmSerial(2);

STM32BootloaderInterface stmBootloader;

// We will compute the MD5 hashes of the data we upload, and read it back to verify
MD5Builder writeMd5;
MD5Builder readMd5;

// Ensure these pins match your setup, along with RX/TX
#define RESET_PIN 18
#define BOOT0_PIN 32

void setup()
{
  // I have tested that we can talk upto 921600 baud to a Blue-Pill, but safe range ST says is 1200-115200.
  // We are using hardwareSerial for this example, but you can use SoftSerial aswell. Just make sure its 8E1 and speed is ok
  stmSerial.begin(57600, SERIAL_8E1, 16, 17); // Just swap over TX and RX if you don't get comms, but ensure BOOT and RST pins are connected!
  Serial.begin(115200);
  stmBootloader.SetDebugPort(&Serial);
  delay(1000);
  int count = 5;
  while (count > 0)
  {
    delay(1000);
    Serial.printf("auto starting test in %d sec\n", count);
    count--;
  }
  Serial.println("Starting STM32 Bootloader test");
  // Initlizes the lib with supplied params. Check constructor for all options.
  bool initResult = stmBootloader.Init(&stmSerial, RESET_PIN, BOOT0_PIN);

  Serial.println("Result of Lib Init: " + initResult ? "SUCCESS" : "FAILURE");
  // If we couldn't Init, we should just break.
  if (!initResult)
  {
    return;
  }

  Serial.println("Attempting Bootloader init...");
  // This will try and put the STM32 device in bootloader mode, and talk to it.
  auto bootloaderInitResult = stmBootloader.ResetToBootloaderModeAndInit();
  // Check if we were able to talk to the bootloader and init it nicely.
  if (bootloaderInitResult != STM32BootloaderInterface::CommandResult::RESULT_OK)
  {
    Serial.println("Attempting Bootloader init FAILURE!!");
    return;
  }

  Serial.println("Attempting Bootloader init SUCCESS!!");
  Serial.println("Reading in firmware to write from SPIFFS...");

  SPIFFS.begin();
  // /firmware.bin is NOT the same as firmware.bin ;)
  File fp = SPIFFS.open("/firmware.bin", "r");
  uint32_t fileSize = fp.size();
  Serial.println("File stream ready. Size:" + fileSize);

  // When we want to write to Flash, we need to call BeginFlashWrite() to setup our write-state.
  // Take a look at its constructor for more options.
  auto flashInitWriteResult = stmBootloader.BeginFlashWrite(fileSize);
  if (flashInitWriteResult != STM32BootloaderInterface::CommandResult::RESULT_OK)
  {
    Serial.println("Attempting flashWrite init FAILURE!!");
    return;
  }
  // Setup our MD5 builders
  writeMd5.begin();
  readMd5.begin();

  // This is a neat feature, which allows us to register a callback which will passs the data it just wrote
  // to easily preform checksuming/hashing of the data we write from a stream.
  stmBootloader.RegisterWriteStreamMiddleware([](uint8_t *data, size_t length) {
    writeMd5.add(data, length);
  });

  Serial.println("BeginFlashWrite() success.. Writing stream now.");
  // This method lets us just pass in a stream of the data you want to write to STM32
  size_t writtenSize = stmBootloader.WriteStreamToFlash(fp);
  Serial.println("Write Stream finished! Written size:");
  Serial.println(writtenSize);

  // To check for success, we can look at the returned writtenSize and/or the GetLastCommandStatus()
  if ((stmBootloader.GetLastCommandStatus() != STM32BootloaderInterface::CommandResult::RESULT_OK) || writtenSize != fileSize)
  {
    Serial.println("write stream fail! exit.");
    return;
  }
  Serial.println("Reading back flash to verify...");
  uint8_t flashReadBuffer[32];
  // API for reading bytes back from STM32 is pretty similar as Stream API.
  // ReadBytesFromFlash() will read up to the size BeginFlashWrite() and from same offset for ease of use.
  while (stmBootloader.available() > 0 && stmBootloader.GetLastCommandStatus() == STM32BootloaderInterface::CommandResult::RESULT_OK)
  {
    size_t readLen = stmBootloader.ReadBytesFromFlash(flashReadBuffer, sizeof(flashReadBuffer));
    readMd5.add(flashReadBuffer, readLen);
  }

  if (stmBootloader.GetLastCommandStatus() != STM32BootloaderInterface::CommandResult::RESULT_OK)
  {
    Serial.println("Read fail! exit.");
    return;
  }
  char writeHash[16];
  char readHash[16];

  writeMd5.calculate();
  readMd5.calculate();

  writeMd5.getChars(writeHash);
  readMd5.getChars(readHash);

  Serial.println("Computed MD5 hash of written:");
  Serial.println(writeMd5.toString());
  Serial.println("Computed MD5 hash of read:");
  Serial.println(readMd5.toString());

  // Using the calculated hashes of write and read, we can see if we successfully programmed the STM32 device
  if (memcmp(writeHash, readHash, 16) != 0)
  {
    Serial.println("Computed hashes do NOT match! write verification failure.");
    return;
  }
  Serial.println("Computed hashes match! Successfully programmed STM32 Device!");
  Serial.println("Telling STM32 to boot to written area.");

  stmBootloader.BootToWrittenMemory();
  // OR, we can toggle the reset pins to do that with:
  // stmBootloader.ResetToNormalBootMode();

  Serial.println("Completed tests :)");
}

void loop()
{
  delay(1000);
}