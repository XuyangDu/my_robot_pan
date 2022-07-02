/*
 * WEStopButton.cpp
 *
 *  Created on: Apr 27, 2012
 *      Author: mincheng
 */

#include <string>
#include "WEStopButton.h"

using namespace std;
WEStopButton::WEStopButton(const char *name, CSerialCom *com, onStopButtonType fun)
{


  this->com = com;
  onStopButtonFunc = fun;
  this->before_stopped = false;
  this->first_cmd = true;
  ros::NodeHandle private_nh("~/" + string(name));
  private_nh.param("normalChar", normalChar, 81);

}

WEStopButton::~WEStopButton()
{
}

void WEStopButton::mainLoop()
{
  int len = 0;bool temp;
  memset(revBuffer,'a',128);
  //revBuffer[127]=0;
  if(com->PushData(revBuffer,128) == 128)
  {
    char rec[128];
    memset(rec,0,128);
    usleep(50);
    len = com->ReceiveData(rec,128);
    if(len > 0)
    {
      stopped = false;
    }
    else
    {
        stopped = true;
    }
    if (stopped != this->before_stopped && this->first_cmd != true)
      onStopButtonFunc(stopped);
    if(this->first_cmd == true)
      onStopButtonFunc(stopped);
    this->before_stopped = stopped;
    this->first_cmd = false;
  }
  


}
