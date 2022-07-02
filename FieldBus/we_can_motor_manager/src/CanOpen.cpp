#include <ros/ros.h>
#include "CanOpen.h"
using namespace WE_LOG;

uint32 bytes_to_uint32( byte *b )
{
    uint32 ret;
    ret  = (uint32)(ByteCast(b[0]));
    ret |= (uint32)(ByteCast(b[1])) << 8;
    ret |= (uint32)(ByteCast(b[2])) << 16;
    ret |= (uint32)(ByteCast(b[3])) << 24;
    return ret;
}

uint16 bytes_to_uint16( byte *b )
{
    uint16 ret;
    ret  = (uint16)(ByteCast(b[0]));
    ret |= (uint16)(ByteCast(b[1])) << 8;
    return ret;
}

int32 bytes_to_int32( byte *b )
{
    return (int32)bytes_to_uint32(b);
}

int16 bytes_to_int16( byte *b )
{
    return (int16)bytes_to_uint16(b);
}


CanOpen::CanOpen(int _canType, int _deviceID, int _canID)
{
  canType = _canType;
  deviceID = _deviceID;
  canID = _canID;
}

CanOpen::~CanOpen()
{
  stopRecive();
  closeDevice();
}

bool CanOpen::openDevice()
{
  if(canType< 0 || deviceID < 0 || canID < 0)
  {

    ROS_ERROR("CanID not set!\n");
    return false;
  }

  if(opened)
    return true;

  if(!VCI_OpenDevice(canType, deviceID, canID))
  {
    ROS_ERROR("Failed to open can!\n");
    return false;
  }


      VCI_INIT_CONFIG initCfg;
      memset(&initCfg, 0, sizeof(VCI_INIT_CONFIG));
      initCfg.AccMask = 0xffffffff;
      initCfg.Filter = 1;
      initCfg.Timing0 = 0x00;
      initCfg.Timing1 = 0x14;
      initCfg.Mode = 0;
      if(!VCI_InitCAN(canType,deviceID,canID,&initCfg))
      {
          ROS_ERROR("Init can failed!");
          VCI_CloseDevice(canType,deviceID);
          return false;
      }

      VCI_BOARD_INFO boardInfo;
      if(!VCI_ReadBoardInfo(canType,deviceID,&boardInfo))
      {
          ROS_ERROR("Read board info failed");
          VCI_CloseDevice(canType,deviceID);
          return false;
      }

      else
      {
          ROS_INFO("------------ Board Info ------------");
          ROS_INFO("         HW V: %d", boardInfo.hw_Version);
          ROS_INFO("         FW V: %d", boardInfo.fw_Version);
          ROS_INFO("         Dr V: %d", boardInfo.dr_Version);
          ROS_INFO("         Lb V: %d", boardInfo.in_Version);
          ROS_INFO("         CN V: %d", (int)boardInfo.can_Num);
          ROS_INFO("         Sn V: %s\n", boardInfo.str_Serial_Num);
          ROS_INFO("         Ty V: %s\n", boardInfo.str_hw_Type);
          ROS_INFO("------------------------------------");
      }



      if(!VCI_ClearBuffer(canType, deviceID, canID))
      {
          VCI_CloseDevice(canType,deviceID);
          ROS_ERROR("Clear can buffer failed");
          return false;
      }

      if(!VCI_StartCAN(canType, deviceID, canID))
      {
          VCI_CloseDevice(canType,deviceID);
          ROS_ERROR("Start can failed");
          return false;
      }
      ROS_INFO("Can opened!");
      opened = true;
      return opened;
}
bool CanOpen::closeDevice()
{
  if(!opened)
    return true;
  if(VCI_CloseDevice(canType, deviceID))
  {
    _log.logInfo(true, "Close can.\n");
    opened = false;
    return true;
  }
  else
  {
    ROS_ERROR("Close can error!\n");
    return false;
  }


}

unsigned int CanOpen::Xmit(uchar id, int index, int sub, int size, BYTE *data)
{
  if(size > 4)
  {
    ROS_ERROR("Xmit error! Data length must <= 4 !\n");
    return 0;
  }

  memset(&frame, 0, sizeof(VCI_CAN_OBJ));
  frame.ID = 0x000000600 + id;
  frame.SendType = 0;
  frame.RemoteFlag = 0;
  frame.ExternFlag = 0;
  frame.DataLen = 4 + size;
  frame.Data[0] = (size == 1? 47:(size == 2? 43 : 35));
  frame.Data[1] = ByteCast(index);
  frame.Data[2] = ByteCast(index >>8);
  frame.Data[3] = ByteCast(sub);
  for(int i = 0; i < size; i++)
      frame.Data[4+i] = data[i];

  deviceLock.lock();
  unsigned int e= VCI_Transmit(canType, deviceID, canID, &frame, 1);
  deviceLock.unlock();

  //_log.logCan(frame,true,true);

  if(e != 19)
      ROS_ERROR("Cam send msg error, send length %d.\n", e);

  return e;
}


unsigned int CanOpen::Xmit(uint32 id, int size, unsigned char *data)
{
  if(size > 8)
  {
    ROS_ERROR("Xmit error! Data length must <= 8 !\n");
    return 0;
  }
  memset(&frame, 0, sizeof(VCI_CAN_OBJ));
  frame.ID = id;
  frame.SendType = 0;
  frame.RemoteFlag = 0;
  frame.ExternFlag = 0;
  frame.DataLen = size;
  for(int i = 0; i < size; i++)
    frame.Data[i] = data[i];

  deviceLock.lock();
  unsigned int e= VCI_Transmit(canType, deviceID, canID, &frame, 1);
  deviceLock.unlock();
  //_log.logCan(frame,true,true);
  return e;
}


unsigned int CanOpen::Xmit(uchar id, int index, int sub, uint32 data)
{
  BYTE arr[4];
  arr[0] = ByteCast(data);
  arr[1] = ByteCast(data>>8);
  arr[2] = ByteCast(data>>16);
  arr[3] = ByteCast(data>>24);
  return Xmit(id, index, sub, 4, arr);
}

unsigned int CanOpen::Xmit(uchar id, int index, int sub, uint16 data)
{
  BYTE arr[2];
  arr[0] = ByteCast(data);
  arr[1] = ByteCast(data>>8);
  return Xmit(id, index, sub, 2, arr);
}

unsigned int CanOpen::Xmit(uchar id, int index, int sub, uint8 data)
{
   return Xmit(id, index, sub, 1, &data);
}

unsigned int CanOpen::Xmit(uchar id, int index, int sub, int32 data)
{
  return Xmit(id, index, sub, (uint32)(data));
}

unsigned int CanOpen::Xmit(uchar id, int index, int sub, int16 data)
{
  return Xmit(id, index, sub, (uint16)(data));
}

unsigned int CanOpen::Xmit(uchar id, int index, int sub, int8 data)
{
  return Xmit(id, index, sub, (uint8)(data));
}

bool CanOpen::registePDO(uint32 id, IntValMutex pair)
{
  if((id & 0xFFFFFF80) != PosePDOBaseID)
  {
    ROS_ERROR("pose PDO id 0x%08X  diff frome base PDO id 0x%08X ", id, PosePDOBaseID);
    return false;
  }

  valMap[id] = pair;
  return true;
}

void CanOpen::startRecive()
{
  runReceiveThread = true;
  receiveThread = new boost::thread(&CanOpen::processData, this);
}

void CanOpen::processData()
{
  ROS_INFO("Can Receive Thread start...");

  unsigned int num = 0;
  double val;
  ros::Time lastMsgTime = ros::Time::now();
  while(runReceiveThread)
  {
    memset(dataBuffer, 0, sizeof(VCI_CAN_OBJ)*dataBufferLen);

    deviceLock.lock();
    num = VCI_Receive(canType, deviceID, canID, dataBuffer, dataBufferLen, 5);
    deviceLock.unlock();

    if(num == 0xFFFFFFFF )
    {
      ROS_ERROR("Can receive data error! return val 0xFFFFFFFF\n");
    }


    if(num)
    {
      for(unsigned int i = 0; i < num; i++)
      {
        if((dataBuffer[i].ID & 0xFFFFFF80) == PosePDOBaseID)
        {
          val = (bytes_to_int32(dataBuffer[i].Data) / 1111.0);
          valMap[dataBuffer[i].ID].second->lock();
          *(valMap[dataBuffer[i].ID].first) = val;
          valMap[dataBuffer[i].ID].second->unlock();

          /*_log.logCan(dataBuffer[i], false, true);
          if(dataBuffer[i].ID == (PosePDOBaseID |12))
          {
              printf("0x%08X - angle: %.2f\r",dataBuffer[i].ID, val);
             fflush(stdout);
          }*/

        }
        else if(((dataBuffer[i].ID & 0xFFFFFF80) == 0x580) && dataBuffer[i].Data[0] == 0x80)
        {
          ROS_ERROR("******************************  Download Data Error  ******************************");
          _log.logCan(dataBuffer[i], false, true);
          ROS_ERROR("***********************************************************************************");
        }
      }
      lastMsgTime = ros::Time::now();
    }
    usleep(5000);
    if((ros::Time::now() - lastMsgTime).toSec() > 1)
        ROS_ERROR("Receive msg from can network time out!");
  }

  ROS_INFO("Can receive thread stop.");
}


void CanOpen::setPosePDOBaseID(uint32 id)
{
  PosePDOBaseID = id;
}
