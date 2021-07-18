#ifndef SDCARD_h
#define SDCARD_h

#include <inttypes.h>
#include "SDINFO.h"

extern uint8_t _SPI3_SCK;   //SCLK
extern uint8_t _SPI3_MOSI;  //MOSI
extern uint8_t _SPI3_MISO;  //MISO
extern uint8_t CHIP_SELECT; //CS

uint8_t const SPI_FULL_SPEED = 0;
uint8_t const SPI_HALF_SPEED = 1;
uint8_t const SPI_QUARTER_SPEED = 2;
unsigned int const SD_INIT_TIMEOUT = 2000;
unsigned int const SD_ERASE_TIMEOUT = 10000;
unsigned int const SD_READ_TIMEOUT = 300;
unsigned int const SD_WRITE_TIMEOUT = 600;
uint8_t const SD_CARD_ERROR_CMD0 = 0X1;
uint8_t const SD_CARD_ERROR_CMD8 = 0X2;
uint8_t const SD_CARD_ERROR_CMD17 = 0X3;
uint8_t const SD_CARD_ERROR_CMD24 = 0X4;
uint8_t const SD_CARD_ERROR_CMD25 = 0X05;
uint8_t const SD_CARD_ERROR_CMD58 = 0X06;
uint8_t const SD_CARD_ERROR_ACMD23 = 0X07;
uint8_t const SD_CARD_ERROR_ACMD41 = 0X08;
uint8_t const SD_CARD_ERROR_BAD_CSD = 0X09;
uint8_t const SD_CARD_ERROR_ERASE = 0X0A;
uint8_t const SD_CARD_ERROR_ERASE_SINGLE_BLOCK = 0X0B;
uint8_t const SD_CARD_ERROR_ERASE_TIMEOUT = 0X0C;
uint8_t const SD_CARD_ERROR_READ = 0X0D;
uint8_t const SD_CARD_ERROR_READ_REG = 0X0E;
uint8_t const SD_CARD_ERROR_READ_TIMEOUT = 0X0F;
uint8_t const SD_CARD_ERROR_STOP_TRAN = 0X10;
uint8_t const SD_CARD_ERROR_WRITE = 0X11;
uint8_t const SD_CARD_ERROR_WRITE_BLOCK_ZERO = 0X12;
uint8_t const SD_CARD_ERROR_WRITE_MULTIPLE = 0X13;
uint8_t const SD_CARD_ERROR_WRITE_PROGRAMMING = 0X14;
uint8_t const SD_CARD_ERROR_WRITE_TIMEOUT = 0X15;
uint8_t const SD_CARD_ERROR_SCK_RATE = 0X16;
uint8_t const SD_CARD_TYPE_SD1 = 1;
uint8_t const SD_CARD_TYPE_SD2 = 2;
uint8_t const SD_CARD_TYPE_SDHC = 3;

class Sd2Card
{
public:
  Sd2Card(void) : errorCode_(0), inBlock_(0), partialBlockRead_(0), type_(0) {}
  uint32_t cardSize(void);
  uint8_t erase(uint32_t firstBlock, uint32_t lastBlock);
  uint8_t eraseSingleBlockEnable(void);
  uint8_t errorCode(void) const
  {
    return errorCode_;
  }
  uint8_t errorData(void) const
  {
    return status_;
  }
  uint8_t init(void)
  {
    return init(SPI_FULL_SPEED, CHIP_SELECT);
  }
  uint8_t init(uint8_t sckRateID)
  {
    return init(sckRateID, CHIP_SELECT);
  }
  uint8_t init(uint8_t sckRateID, uint8_t chipSelectPin);
  void partialBlockRead(uint8_t value);
  uint8_t partialBlockRead(void) const
  {
    return partialBlockRead_;
  }
  uint8_t readBlock(uint32_t block, uint8_t *dst);
  uint8_t readData(uint32_t block, uint16_t offset, uint16_t count, uint8_t *dst);
  uint8_t readCID(cid_t *cid)
  {
    return readRegister(CMD10, cid);
  }
  uint8_t readCSD(csd_t *csd)
  {
    return readRegister(CMD9, csd);
  }
  void readEnd(void);
  uint8_t setSpiClock(uint32_t clock);
  uint8_t type(void) const
  {
    return type_;
  }
  uint8_t writeBlock(uint32_t blockNumber, const uint8_t *src, uint8_t blocking = 1);
  uint8_t writeData(const uint8_t *src);
  uint8_t writeStart(uint32_t blockNumber, uint32_t eraseCount);
  uint8_t writeStop(void);
  uint8_t isBusy(void);

private:
  uint32_t block_;
  uint8_t chipSelectPin_;
  uint8_t errorCode_;
  uint8_t inBlock_;
  uint16_t offset_;
  uint8_t partialBlockRead_;
  uint8_t status_;
  uint8_t type_;
  uint8_t cardAcmd(uint8_t cmd, uint32_t arg)
  {
    cardCommand(CMD55, 0);
    return cardCommand(cmd, arg);
  }
  uint8_t cardCommand(uint8_t cmd, uint32_t arg);
  void error(uint8_t code)
  {
    errorCode_ = code;
  }
  uint8_t readRegister(uint8_t cmd, void *buf);
  uint8_t sendWriteCommand(uint32_t blockNumber, uint32_t eraseCount);
  void chipSelectHigh(void);
  void chipSelectLow(void);
  void type(uint8_t value)
  {
    type_ = value;
  }
  uint8_t waitNotBusy(unsigned int timeoutMillis);
  uint8_t writeData(uint8_t token, const uint8_t *src);
  uint8_t waitStartBlock(void);
};
#endif
