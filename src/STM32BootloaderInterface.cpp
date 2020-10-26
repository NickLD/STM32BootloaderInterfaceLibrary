#include <STM32BootloaderInterface.h>

STM32BootloaderInterface::STM32BootloaderInterface()
{
}

void STM32BootloaderInterface::SetSerialPort(Stream *port)
{
    this->_serialPort = port;
}

void STM32BootloaderInterface::SetDebugPort(Stream *port)
{
    this->_debugPort = port;
}

void STM32BootloaderInterface::RegisterWriteStreamMiddleware(STM32BOOTLOADER_MIDDLEWARE_CALLBACK_SIGNATURE)
{
    this->_writeMiddlewareCallback = _writeMiddlewareCallback;
}
boolean STM32BootloaderInterface::Init(Stream *port, uint8_t resetPin, uint8_t bootPin, uint16_t serialTimeout, uint8_t bufferSize, uint16_t pageSize, boolean invertReset, boolean invertBoot)
{
    DEBPMSG("Init() Called.");
    // If SerialPort is NULL, we should assign it to what we were passed
    if (this->_serialPort == NULL)
    {
        DEBPMSG("SP is null, assigning passed.");
        this->_serialPort = port;
    }
    // Do the check again, to see if we were passed a null
    if (this->_serialPort == NULL)
    {
        DEBPMSG("SP is not valid. exit.");
        return false;
    }
    // Set the timeout for our SerialPort
    this->_serialPort->setTimeout(serialTimeout);

    // If either of the pins are 255, we can't continue (not set)
    if (resetPin == 255 || bootPin == 255)
    {
        DEBPMSG("Pins not valid. exit.");
        return false;
    }
    // Setup the Reset and BOOT0 pins as Inputs
    pinMode(resetPin, INPUT);
    pinMode(bootPin, INPUT);
    DEBPMSG("RST & BOOT pins init.");
    // Ensure we could allocate our block buffer
    DEBPMSG("Trying to alloc blockBuffer.");
    if (!this->_SetBlockBufferSize(bufferSize))
    {
        DEBPMSG("Blockbuffer alloc failed. exit.");
        return false;
    }
    DEBPMSG("Block buffer was allocated.");
    DEBPVAR(this->_baseConfig.BlockBufferSize);
    // Checks out :)
    this->_baseConfig.ResetPin = resetPin;
    this->_baseConfig.InvertResetPin = invertReset;
    this->_baseConfig.BootPin = bootPin;
    this->_baseConfig.InvertBootPin = invertBoot;
    this->_baseConfig.SerialTimeout = 5000;
    this->_baseConfig.PageSize = 1024;
    DEBPMSG("Init finished. success.");
    return true;
}

void STM32BootloaderInterface::ResetToNormalBootMode()
{
    pinMode(this->_baseConfig.ResetPin, OUTPUT);
    digitalWrite(this->_baseConfig.ResetPin, (this->_baseConfig.InvertResetPin) ? HIGH : LOW);
    delay(50);
    digitalWrite(this->_baseConfig.ResetPin, (this->_baseConfig.InvertBootPin) ? LOW : HIGH);
    pinMode(this->_baseConfig.ResetPin, INPUT);
}

STM32BootloaderInterface::CommandResult STM32BootloaderInterface::ResetToBootloaderModeAndInit(int retryLimit, int delayBetweenRetryMs)
{
    DEBPMSG("ResetToBootloaderModeAndInit() called.");
    // Put chip in Bootloader mode
    pinMode(this->_baseConfig.ResetPin, OUTPUT);
    pinMode(this->_baseConfig.BootPin, OUTPUT);

    digitalWrite(this->_baseConfig.BootPin, (this->_baseConfig.InvertBootPin) ? LOW : HIGH);
    delay(50);
    digitalWrite(this->_baseConfig.ResetPin, (this->_baseConfig.InvertResetPin) ? HIGH : LOW);
    delay(50);
    digitalWrite(this->_baseConfig.ResetPin, (this->_baseConfig.InvertBootPin) ? LOW : HIGH);
    delay(50);
    digitalWrite(this->_baseConfig.BootPin, (this->_baseConfig.InvertResetPin) ? HIGH : LOW);

    pinMode(this->_baseConfig.BootPin, INPUT);
    pinMode(this->_baseConfig.ResetPin, INPUT);
    DEBPMSG("Toggled pin into bootload mode.");

    // Begin attempts to try and talk to chip
    for (int i = 0; i < retryLimit; i++)
    {
        DEBPMSG("(re?)trying to talk to bootloader.");
        // Flush potential garbage from the RX buffer
        this->_FlushSerialPort();
        /*
        Once the system memory boot mode is entered and the STM32 microcontroller (based on
        on Arm®(a) cores) has been configured (for more details refer to AN2606) the bootloader
        code begins to scan the USARTx_RX line pin, waiting to receive the 0x7F data frame: a
        start bit, 0x7F data bits, even parity bit and a stop bit.
        The duration of this data frame is measured using the Systick timer. The count value of the
        timer is then used to calculate the corresponding baud rate factor with respect to the current
        system clock.
        Next, the code initializes the serial interface accordingly. Using this calculated baud rate, an
        acknowledge byte (0x79) [Mine sends a NACK though??] is returned to the host, which signals that
        the STM32 is ready to receive commands.
        */
        uint8_t cmd[1];
        cmd[0] = 0x7F;
        this->_SerialPortWrite(cmd, 1);

        CommandResult response = this->_WaitForAck();
        // Check if we get an ACK/NACK back from the device, which means we are talking correctly,
        // and STM32 has auto-bauded nicely. (From what I understand, it should only ever be ACK, but my STM32 will do a NACK on success?)
        if (response == CommandResult::RESULT_OK || response == CommandResult::RESULT_BAD)
        {
            DEBPMSG("Bootloader responded.");
            this->_FlushSerialPort();
            this->_latestDeviceState = CommandResult::RESULT_OK;
            return this->_latestDeviceState;
        }
        if (response == CommandResult::RESULT_DEVICE_TIMEOUT)
        {
            DEBPMSG("Bootloader didn't respond in time.");
        }
        delay(delayBetweenRetryMs);
        retryLimit--;
    }
    DEBPMSG("Timed out trying to talk to bootloader");
    return CommandResult::RESULT_DEVICE_TIMEOUT;
}

STM32BootloaderInterface::CommandResult STM32BootloaderInterface::GetLastCommandStatus()
{
    return this->_latestDeviceState;
}

STM32BootloaderInterface::CommandResult STM32BootloaderInterface::BootToAddress(uint32_t address)
{
    this->_latestDeviceState = this->_GoToAddress(address);
    return this->_latestDeviceState;
}

STM32BootloaderInterface::CommandResult STM32BootloaderInterface::BootToWrittenMemory()
{
    this->_latestDeviceState = this->_GoToAddress(this->_startAddress);
    return this->_latestDeviceState;
}

STM32BootloaderInterface::CommandResult STM32BootloaderInterface::BeginFlashWrite(uint32_t size, uint32_t beginAddress, boolean eraseWriteAreaPages, boolean yieldBetweenPageErase)
{
    DEBPMSG("BeginFlashWrite() called");
    // Do some sanity checks first

    // Is the size valid?
    if (size == 0)
    {
        DEBPMSG("Size is 0. exiting.");
        return CommandResult::RESULT_BAD;
    }
    // Are we already in write-mode?
    if (this->_deviceInWriteMode)
    {
        DEBPMSG("Already in write mode. exiting.");
        return CommandResult::RESULT_ALREADY_IN_WRITE_MODE;
    }
    // Were we able to last talk to the STM32's bootloader?
    if (this->_latestDeviceState != CommandResult::RESULT_OK)
    {
        DEBPMSG("Last bootloader resp wasn't success. exiting.");
        return CommandResult::RESULT_BAD;
    }
    // Everything looks good, lets try to proceed
    this->_startAddress = beginAddress;
    this->_bytesReadWriteOffset = 0;
    this->_targetWriteSize = size;
    this->_deviceInWriteMode = true;
    // Try to erase pages before we return if requested
    this->_latestDeviceState = (eraseWriteAreaPages) ? this->_EraseAllMemory(size, yieldBetweenPageErase) : CommandResult::RESULT_OK;
    DEBPMSG("Successful BeginFlashWrite()");
    return this->_latestDeviceState;
}

STM32BootloaderInterface::CommandResult STM32BootloaderInterface::EndFlashWrite() {
    this->_bytesReadWriteOffset = 0;
    this->_deviceInWriteMode = false;
    return CommandResult::RESULT_INCOMPLETE_WRITE;
}

STM32BootloaderInterface::CommandResult STM32BootloaderInterface::WriteRawToFlash(uint32_t address, uint8_t *buffer, size_t length)
{
    this->_latestDeviceState = this->_WriteMemoryAtAddress(address, length, buffer);
    return this->_latestDeviceState;
}

STM32BootloaderInterface::CommandResult STM32BootloaderInterface::ReadRawFromFlash(uint32_t address, uint8_t *buffer, size_t length)
{
    this->_latestDeviceState = this->_ReadMemoryAtAddress(address, length, buffer);
    return this->_latestDeviceState;
}

size_t STM32BootloaderInterface::available(){
  if (this->_deviceInWriteMode)
    {
        return 0;
    }
    return (this->_targetWriteSize - this->_bytesReadWriteOffset);
}

size_t STM32BootloaderInterface::ReadBytesFromFlash(uint8_t *buffer, size_t length){
    // Are we in write-mode?
    if (this->_deviceInWriteMode)
    {
        DEBPMSG("Cannot read while in write-mode. exiting.");
        this->_latestDeviceState = CommandResult::RESULT_CANNOT_READ_IN_WRITE_MODE;
        return 0;
    }
    DEBPMSG("Reading Bytes From device");
    uint32_t maxReadLength = (this->_targetWriteSize - this->_bytesReadWriteOffset);
    uint32_t readLength = (maxReadLength  > length) ? length : maxReadLength;
    DEBPVAR(this->_bytesReadWriteOffset);
    DEBPVAR(readLength);
    //delay(2000);
    this->_latestDeviceState = this->_ReadMemoryAtAddress(this->_startAddress + this->_bytesReadWriteOffset, readLength, buffer);
    // Check read result
    if(this->_latestDeviceState != CommandResult::RESULT_OK){
        return readLength;
    }
    // Command was good, increment offset by read amount
    this->_bytesReadWriteOffset += readLength;
    return readLength;
}

size_t STM32BootloaderInterface::WriteToFlash(uint8_t *data, size_t size)
{
    // Are we even in write-mode?
    if (!this->_deviceInWriteMode)
    {
        DEBPMSG("Not in write-mode. exiting.");
        this->_latestDeviceState = CommandResult::RESULT_NOT_IN_WRITE_MODE;
        return 0;
    }
    // is the requested size to write greater than the what we have remaining to write?
    if (size > (this->_targetWriteSize - this->_bytesReadWriteOffset))
    {
        DEBPMSG("Requested write size is < remaining to write. exiting.");
        this->_latestDeviceState = CommandResult::RESULT_WRITE_SIZE_TOO_LARGE;
        return 0;
    }

    size_t remainingByteSize = size;
    DEBPVAR(size);
    // Do while the bytes to write is larger than our internal buffer size
    while (remainingByteSize > this->_baseConfig.BlockBufferSize)
    {
        DEBPMSG("Copying data from source buffer to block buffer");
        // Copy from source buffer (at offset of (size - remainingByteSize)) to our blockBuffer by size of BlockBufferSize
        memcpy(this->_blockBuffer, data + (size - remainingByteSize), this->_baseConfig.BlockBufferSize);
        DEBPVAR(this->_baseConfig.BlockBufferSize);
        DEBPMSG("Copied bytes into block buffer. Attempting to write.");
        //this->_latestDeviceState = this->_WriteMemoryAtAddress(this->_startAddress + this->_bytesReadWriteOffset - this->_baseConfig.BlockBufferSize, this->_baseConfig.BlockBufferSize);
        this->_latestDeviceState = this->_WriteMemoryAtAddress(this->_startAddress + this->_bytesReadWriteOffset, this->_baseConfig.BlockBufferSize);
        // Check if the write failed or not. Return what we DID write if it did fail.
        if (this->_latestDeviceState != CommandResult::RESULT_OK)
        {
            DEBPMSG("Write operation failed.");
            return size - remainingByteSize;
        }
        DEBPMSG("Write operation success.");
        DEBPVAR(this->_bytesReadWriteOffset);
        // subtract from remainingSize by amount we pushed out, which is BlockBufferSize
        remainingByteSize -= this->_baseConfig.BlockBufferSize;
        // Increment our ReadWriteOffset by the amount we just wrote to STM32
        this->_bytesReadWriteOffset += this->_baseConfig.BlockBufferSize;
    }
    DEBPVAR(remainingByteSize);
    DEBPMSG("Completed bluk transfer bytes remaining to transfer.");
    // For remaining that that doesn't perfectly fit the BlockBufferSize, we need to do one last write
    memcpy(this->_blockBuffer, data + (size - remainingByteSize), remainingByteSize);
    DEBPMSG("Copied bytes into block buffer. Attempting to write.");
    //this->_latestDeviceState = this->_WriteMemoryAtAddress(this->_startAddress + this->_bytesReadWriteOffset - remainingByteSize, remainingByteSize);
    this->_latestDeviceState = this->_WriteMemoryAtAddress(this->_startAddress + this->_bytesReadWriteOffset, remainingByteSize);
    if (this->_latestDeviceState != CommandResult::RESULT_OK)
    {
        DEBPMSG("Write operation failed.");
        return size - remainingByteSize;
    }
    DEBPMSG("Write operation success");
    this->_bytesReadWriteOffset += remainingByteSize;
    remainingByteSize -= remainingByteSize;
    // Check if the entire targetWriteSize was satisfied by this operation. if it was, we should end write-mode.
    if ((this->_targetWriteSize - this->_bytesReadWriteOffset) == 0)
    {
        DEBPMSG("Completed write init size. Resetting mode and return.");
        // Reset the things we should reset
        this->_bytesReadWriteOffset = 0;
        this->_deviceInWriteMode = false;
    }
    // Completed operation :)
    return size;
}

size_t STM32BootloaderInterface::WriteStreamToFlash(Stream &data)
{
    DEBPMSG("WriteStreamToFlash() called");
    if (!this->_deviceInWriteMode)
    {
        DEBPMSG("Cancelling, not in write-mode");
        this->_latestDeviceState = CommandResult::RESULT_NOT_IN_WRITE_MODE;
    }
    // Go through entire stream
    while (data.available() > 0)
    {
        DEBPMSG("Reading from stream.");
        // Try to read in the data by size of the block buffer
        size_t readSize = data.readBytes(this->_blockBuffer, this->_baseConfig.BlockBufferSize);
        DEBPVAR(readSize);
        DEBPMSG("Read bytes in to blockBuffer. Trying to write.");
        // try to write that out to the flash
        this->_latestDeviceState = this->_WriteMemoryAtAddress(this->_startAddress + this->_bytesReadWriteOffset, readSize);
        if (this->_latestDeviceState != CommandResult::RESULT_OK)
        {
            DEBPMSG("Write operation failed.");
            return this->_bytesReadWriteOffset;
        }
        DEBPMSG("Write operation success. Offset:");
        DEBPVAR(this->_bytesReadWriteOffset);
        if (this->_writeMiddlewareCallback != NULL)
        {
            DEBPMSG("Calling registered strean Middleware.");
            this->_writeMiddlewareCallback(this->_blockBuffer, readSize);
            DEBPMSG("Middleware function finished. Continuing.");
        }
        this->_bytesReadWriteOffset += readSize;
        // Check if the entire targetWriteSize was satisfied by this operation. if it was, we should end write-mode.
        if ((this->_targetWriteSize - this->_bytesReadWriteOffset) == 0)
        {
            DEBPMSG("Completed write init size. Resetting mode and return.");
            // Reset the things we should reset
            this->_bytesReadWriteOffset = 0;
            this->_deviceInWriteMode = false;

            return this->_bytesReadWriteOffset;
        }
    }
    DEBPMSG("Finished writing stream");
    return this->_bytesReadWriteOffset;
}

// ------------ Private Method Declarations below ------------ //

// Allocates memory for the blockBuffer, which is defined in the config
boolean STM32BootloaderInterface::_SetBlockBufferSize(uint8_t size)
{
    // Allocate space for our BlockBuffer
    if (this->_baseConfig.BlockBufferSize == 0)
    {
        this->_blockBuffer = (uint8_t *)malloc(size);
        DEBPMSG("Blockbuffer allocated.");
    }
    else
    {
        DEBPMSG("Need to realloc blockbuffer.");
        uint8_t *newBuffer = (uint8_t *)realloc(this->_blockBuffer, size);
        // If the new buffer is NULL, we can't continue
        if (newBuffer == NULL)
        {
            DEBPMSG("Realloc failed. exit.");
            return false;
        }
        DEBPMSG("Realloc success.");
        this->_blockBuffer = newBuffer;
    }
    this->_baseConfig.BlockBufferSize = size;
    DEBPMSG("Blockbuffer allocation complete.");
    return true;
}

// Takes an Address in form of uint32, and produces the proper formatted address packet w/ checksum
void STM32BootloaderInterface::_EncodeFlashAddress(uint32_t address, uint8_t *result)
{
    uint8_t b3 = (uint8_t)((address >> 0) & 0xFF);
    uint8_t b2 = (uint8_t)((address >> 8) & 0xFF);
    uint8_t b1 = (uint8_t)((address >> 16) & 0xFF);
    uint8_t b0 = (uint8_t)((address >> 24) & 0xFF);

    uint8_t crc = (uint8_t)(b0 ^ b1 ^ b2 ^ b3);
    result[0] = b0;
    result[1] = b1;
    result[2] = b2;
    result[3] = b3;
    result[4] = crc;
}

uint8_t STM32BootloaderInterface::_SerialPortWrite(uint8_t *buffer, uint8_t length)
{
    return this->_serialPort->write(buffer, length);
}

uint8_t STM32BootloaderInterface::_SerialPortRead(uint8_t *buffer, uint8_t length)
{
    return this->_serialPort->readBytes(buffer, length);
}

uint8_t STM32BootloaderInterface::_FlushSerialPort()
{
    while (this->_serialPort->available() > 0)
    {
        this->_serialPort->read();
    }
    return 1;
}

// Used to await confirmation of commands from STM32 Bootloader
STM32BootloaderInterface::CommandResult STM32BootloaderInterface::_WaitForAck()
{
    uint8_t cmd[1] = {0};
    uint8_t nread = this->_SerialPortRead(cmd, 1);
    if (nread == 1)
    {
        if (cmd[0] == 0x79)
        {
            // ACK
            return CommandResult::RESULT_OK;
        }
        if (cmd[0] == 0x1F)
        {
            // NACK
            return CommandResult::RESULT_BAD;
        }
        // We didn't parse this correctly, we will just call it a timeout
        // as it could be garbage from an incompatable baud rate, or noise.
        return CommandResult::RESULT_DEVICE_TIMEOUT;
    }
    // We never got a response back within SerialTimeout we set.
    return CommandResult::RESULT_DEVICE_TIMEOUT;
}

// Wraps all commands we send, handling command checksuming, and waiting for device response.
STM32BootloaderInterface::CommandResult STM32BootloaderInterface::_SendGenericCommand(uint8_t command)
{
    // Ensure there isn't any garbage/unread & un-needed data in the RX buffer that would cause _WaitForAck() to read incorrectly.
    this->_FlushSerialPort();
    uint8_t cmd[2];
    cmd[0] = command;
    cmd[1] = command ^ 0xFF;
    this->_SerialPortWrite(cmd, 2);
    return this->_WaitForAck();
}

// Will read a number of bytes out of STM32 Flash at an address we specify
STM32BootloaderInterface::CommandResult STM32BootloaderInterface::_ReadMemoryAtAddress(uint32_t address, uint8_t length, uint8_t *optBuffer)
{
    // 0x11 is memory read command
    CommandResult response = this->_SendGenericCommand(0x11);
    // Check the response from Device
    if (response != CommandResult::RESULT_OK)
    {
        // Will pass-through error (Bad Response, or Timeout) to the caller
        return response;
    }
    // Device said it's ready to receive address to read. Prepare and send the response.
    uint8_t enc_address[5] = {0};
    this->_EncodeFlashAddress(address, enc_address);
    this->_SerialPortWrite(enc_address, 5);

    response = this->_WaitForAck();
    if (response != CommandResult::RESULT_OK)
    {
        return response;
    }

    // Device was ok with address. it Now wants to know how many bytes to read out, so compute that and send it.
    uint8_t buffer[2];
    buffer[0] = length - 1; // Bytes to read
    buffer[1] = buffer[0] ^ 0xFF;

    this->_SerialPortWrite(buffer, 2);

    response = this->_WaitForAck();
    if (response != CommandResult::RESULT_OK)
    {
        return response;
    }
    // Device has replied back with the data we requested.
    // Begin reading that into our blockBuffer
    uint8_t numberOfBytesRead = 0;
    while (numberOfBytesRead < length)
    {
        numberOfBytesRead += this->_SerialPortRead(((optBuffer == NULL) ? this->_blockBuffer : optBuffer) + numberOfBytesRead, length - numberOfBytesRead);
    }
    return CommandResult::RESULT_OK;
}

// Will write a number of bytes out to STM32 Flash at an address and length we specify from our blockBuffer
STM32BootloaderInterface::CommandResult STM32BootloaderInterface::_WriteMemoryAtAddress(uint32_t address, uint8_t length, uint8_t *optBuffer)
{
    // Can happen during write, where the amount to write was perfectly divisible by blockBufferSize
    if (length == 0)
    {
        return CommandResult::RESULT_OK;
    }
    CommandResult response = this->_SendGenericCommand(0x31);

    if (response != CommandResult::RESULT_OK)
    {
        return response;
    }

    uint8_t enc_address[5] = {0};
    this->_EncodeFlashAddress(address, enc_address);
    this->_SerialPortWrite(enc_address, 5);

    response = this->_WaitForAck();
    if (response != CommandResult::RESULT_OK)
    {
        return response;
    }
    // Tell STM32 how many bytes we want to write
    uint8_t buffer[1];
    buffer[0] = length - 1;
    this->_SerialPortWrite(buffer, 1);

    uint8_t crc = buffer[0];
    // We need to calculate the checksum for the number of bytes we will write from blockBuffer
    for (int i = 0; i < length; i++)
    {
        crc = crc ^ ((optBuffer == NULL) ? this->_blockBuffer : optBuffer)[i];
    }
    // Send the bytes to write to the STM32
    this->_SerialPortWrite(((optBuffer == NULL) ? this->_blockBuffer : optBuffer), length);
    buffer[0] = crc;
    // Send device the CRC for command + bytes to write
    this->_SerialPortWrite(buffer, 1);

    return this->_WaitForAck();
}

// Erases a page in STM32 Flash
STM32BootloaderInterface::CommandResult STM32BootloaderInterface::_EraseIndividualPage(uint8_t pageToErase)
{
    // send cmd that we want to erase single page
    // Send command to erase 'classic'/not extended mode
    CommandResult response = this->_SendGenericCommand(0x43);

    if (response != CommandResult::RESULT_OK)
    {
        return response;
    }

    uint8_t cmd[3];
    cmd[0] = 0;               // Erase 1 page
    cmd[1] = pageToErase;     // Page to erase
    cmd[2] = cmd[0] ^ cmd[1]; // Checksum

    this->_SerialPortWrite(cmd, 3);

    return this->_WaitForAck();
}

// Erases a section of flash memory, page by page, that fits specified bin size.
// TODO: this (incorrectly) assumes that we are starting at beginning of flash memory (0x08000000) which isn't necessarily correct.
// We need to adjust this so that it corectly erases from the correct page within our offset specified in BeginWriteFlash()
STM32BootloaderInterface::CommandResult STM32BootloaderInterface::_EraseAllMemory(uint32_t binSize, boolean yieldBetween)
{
    DEBPMSG("_EraseAllMemory() called.");
    // Go through the flash and erase the amount we need to store our data in
    for (uint32_t i = 0; i < binSize; i += this->_baseConfig.PageSize)
    {
        uint8_t pageToErase = i / this->_baseConfig.PageSize;
        DEBPMSG("Trying to erase page:");
        DEBPVAR(pageToErase);
        CommandResult result = this->_EraseIndividualPage(pageToErase);

        if (result != CommandResult::RESULT_OK)
        {
            DEBPMSG("Page erase fail");
            return result;
        }
        DEBPMSG("Page erase success");
        if (yieldBetween)
        {
            yield();
        }
    }
    DEBPMSG("Erased all pages requested.");
    return CommandResult::RESULT_OK;
}
// Will tell the bootloader where to boot from an address
STM32BootloaderInterface::CommandResult STM32BootloaderInterface::_GoToAddress(uint32_t address)
{
    CommandResult result = this->_SendGenericCommand(0x21);

    if (result != CommandResult::RESULT_OK)
    {
        return result;
    }
    uint8_t enc_address[5] = {0};
    this->_EncodeFlashAddress(address, enc_address);
    this->_SerialPortWrite(enc_address, 5);
    return this->_WaitForAck();
}