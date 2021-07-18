#include <inttypes.h>
#include "SD.h"

class BlackBox
{

public:
  BlackBox(uint8_t _SCK, uint8_t _MOSI, uint8_t _MISO, uint8_t _ChipSelect, const String _BlackBoxName, const String _Separator)
  {
    Separator = _Separator;
    BlackBoxName = _BlackBoxName;
    ChipSelect = _ChipSelect;
    SCK = _SCK;
    MOSI = _MOSI;
    MISO = _MISO;
  }
  void Init();
  void IMU(int16_t AngleRoll, int16_t AnglePitch, int16_t Heading);
  void RC(uint16_t Throttle_CH, uint16_t Yaw_CH, uint16_t Pitch_CH, uint16_t Roll_CH);
  void GPS(int32_t Latitude, int32_t Longitude, uint8_t GPS_Num_Sat);

private:
  uint8_t ChipSelect;
  uint8_t SCK;
  uint8_t MOSI;
  uint8_t MISO;
  String Directory;
  String Separator;
  String BlackBoxName;
};
