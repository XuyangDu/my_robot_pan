/*
 * WEMotor.cpp
 *
 *  Created on: Apr 11, 2012
 *      Author: kiter
 */
#include "SerialComMotor.h"
#include "xform.h"
ComMotorZW::ComMotorZW(const char* _name, int _id, CSerialCom* _com):Motor(_name, _id)
{

  com = _com;
  if(!(_com->isOpened()))
  {
    enable = false;
    return;
  }
  enable = true;
  motorEcoderRates = 100;
}

ComMotorZW::~ComMotorZW()
{
  if(!enable)
    return;
  fflush(stdout);

}

bool ComMotorZW::init()
{
  com->registPose(name.c_str(), std::make_pair(&pose, &lock));
  return true;
}

void ComMotorZW::setupPositionMove(const MotorProfile &p)
{
  lastProfile = p;
  lastProfile.Position -= offset;
  lastProfile.Position = between(lastProfile.Position, lastProfile.minAngle, lastProfile.maxAngle);
  lastProfile.velocity = between(lastProfile.velocity, 0, lastProfile.maxSpeed);
  if(name == std::string("pan"))
      com->sendPanCmd(lastProfile.Position * motorEcoderRates * lastProfile.coefficient, lastProfile.velocity);
  else if(name == std::string("tilt"))
      com->sendTiltCmd(lastProfile.Position * motorEcoderRates * lastProfile.coefficient, lastProfile.velocity);

}

void ComMotorZW::setupPositionMove(double pose)
{
    lastProfile.Position = between(pose - offset, lastProfile.minAngle, lastProfile.maxAngle);
  if(name == std::string("pan"))
      com->sendPanCmd(lastProfile.Position * motorEcoderRates * lastProfile.coefficient, lastProfile.velocity);
  else if(name == std::string("tilt"))
      com->sendTiltCmd(lastProfile.Position * motorEcoderRates * lastProfile.coefficient, lastProfile.velocity);
}

void ComMotorZW::setupPositionMove(double pose, int speed)
{
    lastProfile.Position = between(pose - offset, lastProfile.minAngle, lastProfile.maxAngle);
    lastProfile.velocity = between(speed, 0, lastProfile.maxSpeed);

    if(name == std::string("pan"))
        com->sendPanCmd(lastProfile.Position * motorEcoderRates * lastProfile.coefficient, lastProfile.velocity);
    else if(name == std::string("tilt"))
        com->sendTiltCmd(lastProfile.Position * motorEcoderRates * lastProfile.coefficient, lastProfile.velocity);

}

void ComMotorZW::startPositionMove(bool relative)
{
  lastProfile.poseRelative = relative;
}
void ComMotorZW::velocityMove(int speed)
{
  lastProfile.velocity =speed;
  ROS_WARN("Velocity move is not supported by ComMotorZW");
}

void ComMotorZW::setSpeed(int val)
{
  if(val == -1)
    return;
  lastProfile.velocity = val;
}

void ComMotorZW::resetProfile()
{
  setSpeed(defaultProfile.velocity);
}

void ComMotorZW::motorInit()
{
    setupPositionMove(0);
}
