#include<stdio.h>
#include "DeviceInterface.h"

DeviceInterface::DeviceInterface()
{
  opened = false;
  runReceiveThread = false;
  receiveThread = NULL;
}


bool DeviceInterface::isOpened()
{
  return opened;
}

void DeviceInterface::stopRecive()
{
  if(runReceiveThread == false)
    return;
  runReceiveThread = false;
  if(receiveThread)
  {
    receiveThread->join();
    delete receiveThread;
    receiveThread = NULL;
  }
}
