#ifndef __SD_H__
#define __SD_H__

#include <inttypes.h>
#include "Stream.h"
#include "SDFAT.h"

#define FILE_READ O_READ
#define FILE_WRITE (O_READ | O_WRITE | O_CREAT | O_APPEND)

namespace SDLib
{

  class File : public Stream
  {

  private:
    char _name[13];
    SdFile *_file;

  public:
    File(SdFile f, const char *name);
    File(void);
    virtual size_t write(uint8_t);
    virtual size_t write(const uint8_t *buf, size_t size);
    virtual int availableForWrite();
    virtual int read();
    virtual int peek();
    virtual int available();
    virtual void flush();
    int read(void *buf, uint16_t nbyte);
    bool seek(uint32_t pos);
    uint32_t position();
    uint32_t size();
    void close();
    operator bool();
    char *name();
    bool isDirectory(void);
    File openNextFile(uint8_t mode = O_RDONLY);
    void rewindDirectory(void);
    using Print::write;
  };

  class SDClass
  {

  private:
    Sd2Card card;
    SdVolume volume;
    SdFile root;
    SdFile getParentDir(const char *filepath, int *indx);

  public:
    bool begin(uint8_t csPin = CHIP_SELECT);
    bool begin(uint32_t clock, uint8_t csPin);
    void end();
    File open(const char *filename, uint8_t mode = FILE_READ);
    File open(const String &filename, uint8_t mode = FILE_READ)
    {
      return open(filename.c_str(), mode);
    }
    bool exists(const char *filepath);
    bool exists(const String &filepath)
    {
      return exists(filepath.c_str());
    }
    bool mkdir(const char *filepath);
    bool mkdir(const String &filepath)
    {
      return mkdir(filepath.c_str());
    }
    bool remove(const char *filepath);
    bool remove(const String &filepath)
    {
      return remove(filepath.c_str());
    }
    bool rmdir(const char *filepath);
    bool rmdir(const String &filepath)
    {
      return rmdir(filepath.c_str());
    }

  private:
    int fileOpenMode;
    friend class File;
    friend bool callback_openPath(SdFile &, const char *, bool, void *);
  };
  extern SDClass SD;
};
using namespace SDLib;
typedef SDLib::File SDFile;
typedef SDLib::SDClass SDFileSystemClass;
#define SDFileSystem SDLib::SD
#endif
