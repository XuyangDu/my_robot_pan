#ifndef MOTORMANAGERLOG_H
#define MOTORMANAGERLOG_H
#include <we_log/WELog.h>
#include "CanOpen.h"
#include "SerialCom.h"

class MotorManagerLog:public WE_LOG::WELog
{
public:
  int logCan( VCI_CAN_OBJ frame, bool Xmit = true, bool output = false);

};

extern  MotorManagerLog _log;

#endif // MOTORMANAGERLOG_H
