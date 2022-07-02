/*
 * WEMotor.h
 *
 *  Created on: Apr 11, 2012
 *      Author: Min Cheng
 */

#ifndef WEMOTOR_H_
#define WEMOTOR_H_
#include <string>
#include <sstream>
#include <vector>
#include <queue>
#include <boost/thread/pthread/recursive_mutex.hpp>
#include <ros/ros.h>
#include "SerialCom.h"
#include "Motor.h"

class ComMotorZW: public Motor
{
public:
  ComMotorZW(const char* _name, int _id, CSerialCom* _com);
  ~ComMotorZW();

  virtual bool init();

  virtual void setupPositionMove(const MotorProfile &p);
  virtual void setupPositionMove(double pose);
  void setupPositionMove(double pose, int speed);
  virtual void startPositionMove(bool relative = false);
  virtual void velocityMove(int speed);
  virtual void setSpeed(int val);
  virtual void resetProfile();
  virtual void motorInit();

protected:

  CSerialCom* com;
};

#endif /* WEMOTOR_H_ */
