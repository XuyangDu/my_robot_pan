/**
 @brief:serial port class
 @author:Min Chneg
 @date:2012.2.23
 */

/*
* async_serial.cpp
*
* Created on: Sep 10, 2020 13:00
* Description:
*
* Copyright (c) 2020 Ruixiang Du (rdu)
*/

#ifndef _SERIAL_COM_H_
#define _SERIAL_COM_H_

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <string>
#include <map>
#include "DeviceInterface.h"
#include"MotorManagerLog.h"
#define BYTE unsigned char
#define LIST_LEN 100

class CSerialCom:public DeviceInterface
{

public:

  CSerialCom(const char * _name, int _speed, int _databits, char _parity, int _stopbits);
  ~CSerialCom();

  virtual bool openDevice();
  virtual void startRecive();
  virtual void processData();

  void sendPanCmd(int pose, int speed);
  void sendTiltCmd(int pose, int speed);
  void sendPanTiltCmd(int panPose, int panSpeed, int tiltPose, int tiltSpeed);

  int PushData(const void * data, int length);
  int ReceiveData(void *buf, int length);

  typedef std::pair<double*, boost::recursive_mutex*> IntValMutex;
  void registPose(std::string name , IntValMutex pair);

private:

  virtual bool closeDevice();

  bool OpenPort(const char * portName);
  bool SetPortSpeed(int speed);
  bool SetParity(int databits, char parity, int stopbits);


  std::map<std::string, IntValMutex> valMap;

  std::string name;
  int speed;
  int databits;
  char parity;
  int stopbits;
  int m_port_file;

  int lastPanPose;
  int lastPanSpeed;
  int lastTiltPose;
  int lastTiltSpeed;

  char cmd[LIST_LEN];

};

#endif

