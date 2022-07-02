/*
 * WEStopButton.h
 *
 *  Created on: Apr 27, 2012
 *      Author: mincheng
 */

#ifndef WESTOPBUTTON_H_
#define WESTOPBUTTON_H_
#include"ros/ros.h"
#include <boost/thread/thread.hpp>
#include "SerialCom.h"


#define REV_BUFFER_LEN_ 128
#define MAX_SEND_COUNT 5

typedef boost::function<void (bool)> onStopButtonType;

class WEStopButton
{
public:
  WEStopButton(const char* name, CSerialCom* com, onStopButtonType fun);
  ~WEStopButton();
  void mainLoop();

private:

  onStopButtonType onStopButtonFunc;

  bool stopped;
  bool first_cmd;
  bool before_stopped;
//  bool firstCmd;
  CSerialCom* com;

//  int stopChar;
  int normalChar;
//  int noDatacount;

  char revBuffer[REV_BUFFER_LEN_];
};

#endif /* WESTOPBUTTON_H_ */
