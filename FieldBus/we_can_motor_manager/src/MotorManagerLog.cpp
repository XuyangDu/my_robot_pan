#include "MotorManagerLog.h"

int MotorManagerLog::logCan(VCI_CAN_OBJ frame, bool Xmit, bool output)
{
  if( frame.ID == 0x080 ) return 0;
  if( frame.ID == 0x180 ) return 0;
  if( (frame.ID > 0x700) && (frame.ID < 0x780) ) return 0;

  int index;
  index = frame.Data[2];
  index <<= 8;
  index |= frame.Data[1];
  return logDirect(WE_LOG::WELog_Error, output, "CAN.%c: 0x%08X - 0x%02X | 0x%02X 0x%02X | 0x%02X || 0x%02X 0x%02X 0x%02X 0x%02X || 0x%04X\n", (Xmit? 'X':'R'), frame.ID,  \
                   frame.Data[0], frame.Data[1], frame.Data[2], frame.Data[3], frame.Data[4], frame.Data[5], frame.Data[6], frame.Data[7],         \
      index);
}


MotorManagerLog _log;
