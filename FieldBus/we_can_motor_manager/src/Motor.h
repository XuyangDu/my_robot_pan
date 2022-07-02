#ifndef MOTOR_H
#define MOTOR_H
#include <boost/thread/recursive_mutex.hpp>
#include <string>
#include <ros/ros.h>
#include"AmpDef.h"
#include <stdio.h>

struct MotorProfile
{
  int moveType;

  double Position;
  int velocity;
  int acceleration;
  int deceleration;

  int jerk;
  int stopDeceleration;

  bool poseRelative;

  int maxSpeed;
  int maxASpeed;

  double maxAngle;
  double minAngle;

  int coefficient;
};


class Motor
{
public:
  Motor(const char* _name, int _id)
  {
    name = std::string(_name);
    ID = _id;
    isStop = false;
    enable = false;
    pose = 0;
    offset = 0;
    ros::NodeHandle nh("~/" + name);
    nh.param("moveType", lastProfile.moveType, 3);
    nh.param("Position", lastProfile.Position, 0.0);
    nh.param("velocity", lastProfile.velocity, 0);
    nh.param("acceleration", lastProfile.acceleration, 0);
    nh.param("deceleration", lastProfile.deceleration, 0);
    nh.param("jerk", lastProfile.jerk, 0);
    nh.param("stopDeceleration", lastProfile.stopDeceleration,0);
    nh.param("poseRelative", lastProfile.poseRelative, false);
    nh.param("maxSpeed", lastProfile.maxSpeed, 0);
    nh.param("maxASpeed", lastProfile.maxASpeed, 0);
    nh.param("maxAngle", lastProfile.maxAngle, 0.0);
    nh.param("minAngle", lastProfile.minAngle, 0.0);
    nh.param("coefficient", lastProfile.coefficient, 1);
    defaultProfile = lastProfile;
/*
    printf("================  %s  ================\n", name.c_str());
    printf("         moveType : %d\n", lastProfile.moveType);
    printf("         Position : %f\n", lastProfile.Position);
    printf("         velocity : %d\n", lastProfile.velocity);
    printf("     acceleration : %d\n", lastProfile.acceleration);
    printf("     deceleration : %d\n", lastProfile.deceleration);
    printf("             jerk : %d\n", lastProfile.jerk);
    printf(" stopDeceleration : %d\n", lastProfile.stopDeceleration);
    printf("     poseRelative : %d\n", lastProfile.poseRelative);
    printf("         maxSpeed : %d\n", lastProfile.maxSpeed);
    printf("         maxAngle : %f\n", lastProfile.maxAngle);
    printf("         minAngle : %f\n", lastProfile.minAngle);
    printf("      coefficient : %d\n", lastProfile.coefficient);
    printf("=========================================\n\n");
*/

  }


  virtual bool init() =0;

  virtual void setupPositionMove(const MotorProfile &p) =0;
  virtual void setupPositionMove(double pose) =0;
  virtual void startPositionMove(bool relative = false) =0;
  virtual void velocityMove(int speed) =0;
  virtual void setSpeed(int val)=0;
  virtual void resetProfile()=0;
  virtual void setSpeedAcc(int val){}
  virtual void setSpeedDec(int val){}


  virtual void motorStop(){isStop = true;}
  virtual void motorContinue(){isStop = false;}
  virtual void motorbreak(){}
  virtual void motorClear(){}
  virtual void motorInit(){}
  virtual void restart(){}
  virtual void setPower(){}

  MotorProfile* getProfilePtr()
  {
    return &lastProfile;
  }

  double getPose()
  {
    double val;
    lock.lock();
    val = pose * (double)lastProfile.coefficient + offset;
    lock.unlock();
    return val;

  }
  int getMotorEcoderRates()
  {
      return motorEcoderRates;
  }

  bool isEnable()
  {
    return enable;
  }

  void setOffset(double val)
  {
      offset = val;
  }

  std::string name;

protected:

  bool isStop;
  MotorProfile lastProfile;
  MotorProfile defaultProfile;
  double pose;
  boost::recursive_mutex lock;

  int ID;
  bool enable;
  int motorEcoderRates;
  double offset;
};

#endif // MOTOR_H
