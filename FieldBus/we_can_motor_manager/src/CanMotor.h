#ifndef __CAN_MOTOR_H__
#define __CAN_MOTOR_H__
#include"Motor.h"
#include"CanOpen.h"
class CanMotor:public Motor
{
public:
  CanMotor(const char* _name, int _id, CanOpen* _can);
  ~CanMotor();

  virtual bool init();

  virtual void setupPositionMove(const MotorProfile &p);
  virtual void setupPositionMove(double pose);
  virtual void startPositionMove(bool relative = false);
  virtual void velocityMove(int speed);
  virtual void setSpeed(int val);
  virtual void resetProfile();
  virtual void setSpeedAcc(int val);
  virtual void setSpeedDec(int val);

  virtual void motorStop();
  virtual void motorContinue();
  virtual void motorbreak();
  virtual void motorClear();
  virtual void motorInit();
  virtual void setPower();
  virtual void restart();


  void setSyncProducer(bool val);
  unsigned int getPosePDOBaseID();
  void clearPose();


private:

  bool isPositionMode();
  bool isVelocityMode();

  void setPosePDO();
  void stopPosePDO();
  void setSync();
  void stopSync();

  void setControlWord(uint16 val);
  void setHaltMove();
  void setContinueMove();
  void setEnable();
  void setDisable();

  void setMotionProfileType(int16 val);
  void setModeofOperation(int8 val);
  void SetTargetPos(double val);
  void SetProfileVel(int32 val);
  void SetProfileAcc(int32 val);
  void SetProfileDec(int32 val);
  void SetProfileJerk(uint32 val);
  void SetProfileStop(int32 val);
  void SetTargetVel(int32 val);

  void setProfile(const MotorProfile &p);

  CanOpen* can;
  bool syncProducer;

  uint32 pdoId;
  int syncPeriod;
  int PDOPeriod;

  const static unsigned int syncMessageID = 0x00000080;
  const static unsigned int posePDOBaseID = 0x00000380;
  const static int PDOIndex = 0x1802;
  const static int PDOMapIndex = 0x1A02;

};

#endif // __CAN_MOTOR_H__
