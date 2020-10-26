#ifndef STM32_BOOTLOADER_INTERFACE_H
#define STM32_BOOTLOADER_INTERFACE_H
// This controls the debug messages.
//#define STM32_BOOTLOADER_INTERFACE_DEBUGMODE_H
#include <Arduino.h>

// Section totally ripped from https://github.com/knolleary/pubsubclient/blob/master/src/PubSubClient.h
#if defined(ESP8266) || defined(ESP32)
#include <functional>
#define STM32BOOTLOADER_MIDDLEWARE_CALLBACK_SIGNATURE std::function<void(uint8_t *, size_t)> _writeMiddlewareCallback
#else
#define STM32BOOTLOADER_MIDDLEWARE_CALLBACK_SIGNATURE void (*_writeMiddlewareCallback)(uint8_t *, size_t)
#endif

class STM32BootloaderInterface
{
  // Defines how the library should interact with the STM32,
  // Such as what pins to use for resetting, and putting in bootloader
  // mode.
  struct STM32InterfaceConfig
  {
    // What pin to use for resetting the chip
    uint8_t ResetPin;
    // What pin controls the BOOT0 pin of STM32
    uint8_t BootPin;
    // Should The logic for the reset pin be inverted?
    bool InvertResetPin;
    // Should The logic for the BOOT0 pin be inverted?
    bool InvertBootPin;
    // How large the buffer should be for read/write in bytes
    uint8_t BlockBufferSize;
    // How long (ms) should the read timeout be
    uint16_t SerialTimeout;
    // How large each page in flash is
    uint16_t PageSize;
  };
public:
  enum CommandResult
  {
    // Returned when a command/response was sent and received/processed successfully (ack)
    RESULT_OK = 0,
    // Returned when a command/response wasn't received/processed successfully (nack)
    RESULT_BAD = 1,
    // Returned when a command response was never received within SerialTimeout window.
    // Comms at this point should be considered 'dead', and one should ResetToBootloaderModeAndInit() again..
    RESULT_DEVICE_TIMEOUT = 2,
    // Returned when the failure was a result of an incomplete write (written length < specified), and write-mode was ended
    RESULT_INCOMPLETE_WRITE = 3,
    // Returned when/if you try to 'keep writing' to STM32 flash after it's already completed write-mode operation, or if you try to write to flash without calling BeginFlashWrite() first.
    RESULT_NOT_IN_WRITE_MODE = 4,
    // Returned when/if you try to preform a read operation while currently in write-mode
    RESULT_CANNOT_READ_IN_WRITE_MODE = 5,
    // Returned when you try to BeginFlashWrite() before the current write-mode is completed
    RESULT_ALREADY_IN_WRITE_MODE = 6,
    // Returned when you try to write more bytes than the write-mode size was init to
    RESULT_WRITE_SIZE_TOO_LARGE = 7
  };

private:
  STM32InterfaceConfig _baseConfig;

  Stream *_serialPort = NULL;
  Stream *_debugPort = NULL;

  uint8_t *_blockBuffer = NULL;

  uint32_t _startAddress = 0;
  uint32_t _bytesReadWriteOffset = 0;
  uint32_t _targetWriteSize = 0;
  CommandResult _latestDeviceState = CommandResult::RESULT_DEVICE_TIMEOUT;

  boolean _deviceInWriteMode = false;

  STM32BOOTLOADER_MIDDLEWARE_CALLBACK_SIGNATURE;

  boolean _SetBlockBufferSize(uint8_t size);

  void _EncodeFlashAddress(uint32_t address, uint8_t *result);

  uint8_t _SerialPortWrite(uint8_t *buffer, uint8_t length);
  uint8_t _SerialPortRead(uint8_t *buffer, uint8_t length);
  uint8_t _FlushSerialPort();

  CommandResult _WaitForAck();
  CommandResult _SendGenericCommand(uint8_t command);

  CommandResult _ReadMemoryAtAddress(uint32_t address, uint8_t length, uint8_t *optBuffer = NULL);
  CommandResult _WriteMemoryAtAddress(uint32_t address, uint8_t length, uint8_t *optBuffer = NULL);
  CommandResult _EraseIndividualPage(uint8_t pageToErase);
  CommandResult _EraseAllMemory(uint32_t binSize, boolean yieldBetween = false);
  CommandResult _GetBootloaderInfo();
  CommandResult _GoToAddress(uint32_t address);
  

public:
  // A library that lets you easily interact with STM32 built-in
  // USART bootloader to (primarly) upload new firmware
  // to the chip.
  // IMPORTANT NOTES:
  // --1. Serial config should be SERIAL_8E1/SWSERIAL_8E1 (8 Bits, Even Parity, 1 Stop Bit)
  // --2. STM32 Bootloader baud range is 1200-115200
  // --3. If using SoftSerial, obviously go on slower-end of range
  STM32BootloaderInterface(void);

  // Sets the Serial port used to talk to STM32
  // @param port Reference to the serialport.
  void SetSerialPort(Stream *port);

  // Sets the debug port for debugging purposes
  // @param port Reference to the port.
  void SetDebugPort(Stream *port);

  // Inits the library's block buffer, serialport, etc. to specified config
  // Returns a boolean indicating success or failure.
  // @param port The serialport the lib will use to try and talk to bootloader
  // @param resetPin The pin tied to the RESET line of STM32
  // @param bootPin The pin tied to BOOT0 line of STM32
  // @param serialTimeout The timeout in milliseconds the SP will be configured to wait for response for
  // @param bufferSize The size of the internal buffer used by lib for read/writing data to STM32
  // @param pageSize The size of each page in STM32 flash
  // @param invertReset Determines if the logic for putting in bootloader mode should be inverted for RESET line
  // @param invertBoot Determines if the logic for putting in bootloader mode should be inverted for BOOT0 line
  boolean Init(Stream *port, uint8_t resetPin = 255, uint8_t bootPin = 255, uint16_t serialTimeout = 2000, uint8_t bufferSize = 32, uint16_t pageSize = 1024, boolean invertReset = false, boolean invertBoot = false);

  // Inits the STM32 hardware itself into bootloader mode, and attempts to talk to it
  // @param retryLimit The number of times the library will retry talking to bootloader to init.
  // @param delayBetweenRetryMs The number of milliseconds the library will delay() between retries.
  CommandResult ResetToBootloaderModeAndInit(int retryLimit = 15, int delayBetweenRetryMs = 50);

  // Inits the STM32 hardware itself into normal, user-code execution mode
  void ResetToNormalBootMode();

  // Inits the library to prepare for the subsequent write operations to the STM32 device.
  // Returns the CommandResult of the operation.
  // IMPORTANT NOTE:
  // -- This operation will, by default, pre-erase the area in memory to be written (aligned to page). This can take some seconds to complete.
  // @param size the Size of the data we are going to write contiguously to STM32.
  // @param beginAddress the Starting Address in flash where we will begin our writes.
  // @param eraseWriteAreaPages Should we erase the pages in flash where our written data will overlap/be written to?
  // @param yieldBetweenPageErase Should we call yield() after we erase a page?
  CommandResult BeginFlashWrite(uint32_t size, uint32_t beginAddress = 0x08000000, boolean eraseWriteAreaPages = true, boolean yieldBetweenPageErase = true);

  // Finishes a 'BeginFlashWrite' request. Used to prematurely end a(n incomplete) write operation.
  // Returns the CommandResult of the operation.
  // IMPORTANT NOTES:
  // --1. FlashWrite commands will intrinsically do this when they reach end of specified write size. Do not call unless you know what you are doing.
  // --2. The CommandResult of this operation will always be RESULT_INCOMPLETE_WRITE if ending prematurely. This is however, not reflected in the internal device state.
  CommandResult EndFlashWrite();

  // Returns the most recent 'CommandResult' from an operation preformed on the device.
  CommandResult GetLastCommandStatus();

  // Writes a buffer to the STM32 flash (automagically accounting for already written offset, and beginAddress)
  // Returns the size of bytes written.
  // IMPORTANT NOTES:
  // --1. Query 'GetLastCommandStatus()' after a write call to ensure it completed successfully
  // --2. Will automagically end 'WriteMode' when amount of data has reached specified amount in BeginFlashWrite()
  // --3. In event a write operation fails (timeout/nack), the lib will return the size of data it was able to write.
  // @param data the buffer of data to write.
  // @param size the size of the data to write.
  size_t WriteToFlash(uint8_t *data, size_t size);

  // Lets you write raw' into the flash of the STM32, at an address you specify.
  // Returns the CommandResult of the write operation.
  // IMPORTANT NOTE:
  // -- This method doesn't depend on BeginFlashWrite() being called first. As a result, it doesn't affect the internal memory offsets.
  // @param address The address in flash to write the byte to.
  // @param buffer The buffer to write directly into flash.
  // @param length The length of bytes to write to device.
  CommandResult WriteRawToFlash(uint32_t address, uint8_t *buffer, size_t length);

  // Writes a stream to the STM32 flash (automagically accounting for already written offset, and beginAddress)
  // Returns the size of bytes written.
  // IMPORTANT NOTES:
  // --1. Query 'GetLastCommandStatus()' after a writeStream call to ensure it completed successfully
  // --2. Should use this method in conjunction with RegisterWriteStreamMiddleware() for safety
  // --3. In event a write operation fails (timeout/nack), the lib will return the size of data it was able to write.
  // --4. Will automagically end 'WriteMode' when amount of data has reached specified amount in BeginFlashWrite(), and return size written.
  // @param data the stream you want to write to flash.
  size_t WriteStreamToFlash(Stream &data);

  // Registers a 'MiddlewareCallback', which is passed data & length in chunks size of config blockBufferSize.
  // Used in conjuntion with WriteStreamToFlash() to easily allow calculation of verification hashes/checksums
  // to ensure flash integrity.
  // @param _writeMiddlewareCallback the 'MiddlewareCallback' function, which is supplied (uint8_t *data, size_t length).
  void RegisterWriteStreamMiddleware(STM32BOOTLOADER_MIDDLEWARE_CALLBACK_SIGNATURE);

  // Reads a buffer-full (sizeof specified 'blockBufferSize' in Init(), and then remainer at final block) from the flash.
  // IMPORTANT NOTES:
  // --1. Will allow reading from previous BeginFlashWrite() start address, and length. Each subsequent call will be offset by length read from previous call.
  // --2. This CANNOT be called if currently in 'WriteMode'. Finish writing complete data of length specified in BeginFlashWrite(), or call EndFlashWrite()
  // @param buffer The buffer to read in the bytes from flash into.
  // @param length The size you would like to read into the buffer.
  size_t ReadBytesFromFlash(uint8_t *buffer, size_t length);

  // If there is bytes available to read from ReadBytesFromFlash()
  size_t available();

  // Lets you read 'raw' from the flash of the STM32, at an address you specify.
  // Returns the CommandResult of the read operation.
  // IMPORTANT NOTE:
  // -- This method doesn't interact with internal flash offsets. As a result, it can even be used before a complete 'Write' operation has completed, or outside write bounds.
  // @param address The address in flash to read the byte from.
  // @param buffer The pointer to read the data into from device.
  // @param length The amount of bytes to read from device.
  CommandResult ReadRawFromFlash(uint32_t address, uint8_t *buffer, size_t length);

  // Lets you tell the STM32 device to 'go' and boot from a particular point in flash. This is an alternative option to toggling the STM32's reset line.
  // Returns the CommandResult of the read operation.
  // WHAT IT DOES:
  // -- initializes the registers of the peripherals used by the bootloader to their default reset values
  // -- initializes the main stack pointer of the user application
  // -- jumps to the memory location programmed in the received ‘address + 4’
  // @param address The address in flash to 'boot to'
  CommandResult BootToAddress(uint32_t address);
  // Lets you tell the STM32 device to 'go' and boot from start address where you had initilized a BeginFlashWrite()
  CommandResult BootToWrittenMemory();
};
#endif

/*
Below part is taken directly from https://github.com/jerabaul29/ArduinoDebugMacros/blob/master/src/DebugMacros.h
Kept just what I needed from original, and modified to needs.
Debug messages could be on anything that implements Stream instead of just Serial which is nice modification.

 Written by Jean Rabault, jean.rblt@gmail.com, May 2018.
 Free to use as long as this header is retained.
 
 A set of macros to make it easier to perform debugging through Serial.
 Use a separate Parameters.h header file to decide if debug output or not,
 and the parameters used for debugging (which port, which baud rate).
*/
#ifdef STM32_BOOTLOADER_INTERFACE_DEBUGMODE_H
# define DEBPMSG(x) this->_debugPort->println(F("D " x));
# define DEBPVAR(x) this->_debugPort->print(F("D " #x " is ")); this->_debugPort->println(x);
# define DEBPWHERE this->_debugPort->println(F("D at " __FILE__ " l " DEBSHOW(__LINE__)));
# define DEBPMACRO(x) this->_debugPort->println(F("D macro " #x " is " DEBSHOW(x)));
#else  // not in debug mode: empty macros
# define DEBPMSG(x) // nothing
# define DEBPVAR(x) // nothing
# define DEBPWHERE // nothing
# define DEBPMACRO(x) // nothing
#endif
// add a layer of indirection to allow macro extension
#define DEBSHOW(x) DEBSHOW_(x)
#define DEBSHOW_(x) #x