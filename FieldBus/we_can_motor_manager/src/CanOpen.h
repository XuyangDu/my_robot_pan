#ifndef CANOPEN_H
#define CANOPEN_H
#include <map>
#include "ControlCan.h"
#include "AmpDef.h"
#include "DeviceInterface.h"
#include "MotorManagerLog.h"


class CanOpen:public DeviceInterface
{
public:
  CanOpen(int _canType, int _deviceID, int _canID);
  ~CanOpen();

  virtual bool openDevice();
  virtual void startRecive();

  unsigned int Xmit(uchar id, int index, int sub, int size,  BYTE* data);
  unsigned int Xmit(uint32 id, int size, BYTE* data);
  unsigned int Xmit(uchar id, int index, int sub, uint32 data);
  unsigned int Xmit(uchar id, int index, int sub, uint16 data);
  unsigned int Xmit(uchar id, int index, int sub, uint8 data);
  unsigned int Xmit(uchar id, int index, int sub, int32 data);
  unsigned int Xmit(uchar id, int index, int sub, int16 data);
  unsigned int Xmit(uchar id, int index, int sub, int8 data);


  void processData();
  typedef std::pair<double*, boost::recursive_mutex*> IntValMutex;
  bool registePDO(uint32 id, IntValMutex pair);
  void setPosePDOBaseID(uint32 id);

private:

  virtual bool closeDevice();
  int canType;
  int deviceID;
  int canID;
  std::map<uint32, IntValMutex> valMap;
  uint32 PosePDOBaseID;

  const static int dataBufferLen = 100;
  VCI_CAN_OBJ frame;
  VCI_CAN_OBJ dataBuffer[dataBufferLen];

};


#endif // CANOPEN_H
