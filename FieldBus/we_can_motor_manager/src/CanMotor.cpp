#include "xform.h"
#include "CanMotor.h"
CanMotor::CanMotor(const char* _name, int _id, CanOpen *_can):Motor(_name, _id)
{
  this->can = _can;
  if(!can->isOpened())
  {
    enable = false;
    return;
  }

  if(ID < 0 || ID >127)
  {
    enable = false;
    ROS_ERROR("Wrong can node id %d.(id should >0 and <127)", _id);
    return;
  }
  motorEcoderRates = 1111;
  syncProducer = false;

  syncPeriod = 10000; // Microseconds
  PDOPeriod = 5;     // sync signal
  enable = true;
}

CanMotor::~CanMotor()
{
  if(!enable)
    return;

  stopSync();
  stopPosePDO();
  fflush(stdout);
}


 bool CanMotor::init()
{

    can->Xmit(ID, OBJID_QSTOP_MODE, 0, (int16)2);//stop mode
    can->Xmit(ID, OBJID_HALT_MODE, 0, (int16)1);//halt mode
    can->Xmit(ID, OBJID_AMP_MODE, 0, (int16)(0x1E00>>8)); //set amp mode

    unsigned char data[2];
    data[0] = 0x01;
    data[1] = ByteCast(ID);

    can->Xmit(0x00000000, 2, data);//start node
    can->Xmit(ID, OBJID_EVENT_STAT_LATCH, 0, (int32)0xFFFFFFFF); //clear event status
    can->Xmit(ID, OBJID_FAULTS, 0, (uint32)0);//clear faults

    if(isPositionMode())//set profile mode
    {
      setModeofOperation(lastProfile.moveType);
      setMotionProfileType(0);
      SetProfileVel(lastProfile.velocity);
    }
    else if(isVelocityMode())
    {
      setMotionProfileType(-1);
      setModeofOperation(lastProfile.moveType);
      SetTargetVel(lastProfile.velocity);
      setDisable();
      usleep(1);
      clearPose();
    }
    else
    {
      ROS_ERROR("Node [%s] Move Type undefined!");
      enable = false;
      return false;
    }

    //SetTargetPos(lastProfile.Position);
    SetProfileAcc(lastProfile.acceleration);
    SetProfileDec(lastProfile.deceleration);
    SetProfileJerk(lastProfile.jerk);
    SetProfileStop(lastProfile.stopDeceleration);
    setEnable();

    setSync();
    setPosePDO();
    enable = true;
    return true;
}

 void CanMotor::setupPositionMove(const MotorProfile &p)
{
  setProfile(p);
}

 void CanMotor::setupPositionMove(double pose)
 {
   SetTargetPos(pose);
 }

 void CanMotor::startPositionMove(bool relative)
{
  if(!isPositionMode() || isStop)
    return;

  setEnable();

  if(relative)
    can->Xmit(ID, OBJID_CONTROL, 0, (int16)0x007F);
  else
    can->Xmit(ID, OBJID_CONTROL, 0, (int16)0x003F);

  lastProfile.poseRelative = relative;
}

 void CanMotor::velocityMove(int speed)
{
  SetTargetVel(speed);
}

 void CanMotor::motorStop()
 {
   if(isPositionMode())
     setHaltMove();

   if(isVelocityMode())
     setDisable();

   Motor::motorStop();
 }

 void CanMotor::motorContinue()
 {
   if(isPositionMode())
     setContinueMove();

   if(isVelocityMode())
     setEnable();

   Motor::motorContinue();
 }

 void CanMotor::motorbreak()
 {
   if(isPositionMode())
     setDisable();
 }

 void CanMotor::motorClear()
 {
   if(isPositionMode())
   {
     setDisable();
     clearPose();
     SetTargetPos(0);
     setEnable();
   }
 }

 void CanMotor::motorInit()
 {
   if(isPositionMode())
   {
     can->Xmit(ID, OBJID_PROFILE_POS, 0, (int32)0);
     startPositionMove();
   }
 }


 ////////////////////////////////////////////////////////

 bool CanMotor::isPositionMode()
 {
   return lastProfile.moveType == AMPMODE_CAN_PROFILE;
 }

 bool CanMotor::isVelocityMode()
 {
   return lastProfile.moveType == AMPMODE_CAN_VELOCITY;
 }



void CanMotor::clearPose()
{
  can->Xmit(ID,0x2240, 0, (int32)0);
}

void CanMotor::setSpeed(int val)
{

  if(isPositionMode())
  {
    if(val == -1)
      return;
    lastProfile.velocity = val;
    SetProfileVel(val);
  }
  else if(isVelocityMode())
  {
    lastProfile.velocity = val;
    SetTargetVel(val);
  }
}

void CanMotor::resetProfile()
{
  setSpeed(defaultProfile.velocity);
  setSpeedAcc(defaultProfile.acceleration);
  setSpeedDec(defaultProfile.deceleration);
}

void CanMotor::setSpeedAcc(int val)
{
    SetProfileAcc(val);
}

void CanMotor::setSpeedDec(int val)
{
    SetProfileDec(val);
}

void CanMotor::setSyncProducer(bool val)
{
  syncProducer = val;
}

unsigned int CanMotor::getPosePDOBaseID()
{
    return posePDOBaseID;
}


void CanMotor::setPosePDO()
{
  pdoId = posePDOBaseID|ID;
  if(can->registePDO(pdoId, std::make_pair(&pose, &lock)))
  {
    can->Xmit(ID, PDOIndex, 1, (uint32)(0x80000000 | pdoId));
    can->Xmit(ID, PDOIndex, 2, (BYTE)PDOPeriod);
    can->Xmit(ID, PDOMapIndex, 0, (BYTE)0);
    can->Xmit(ID, PDOMapIndex, 1, (uint32)(0x22400020));
    can->Xmit(ID, PDOMapIndex, 0, (BYTE)1);
    can->Xmit(ID, PDOIndex, 1, (uint32)(pdoId));
  }
}

void CanMotor::stopPosePDO()
{
   can->Xmit(ID, PDOIndex, 1, (uint32)(0x80000000 | pdoId));
}

void CanMotor::setSync()
{
  can->Xmit(ID, 0x1006, 0, (uint32)syncPeriod);
  if(syncProducer)
    can->Xmit(ID, 0x1005, 0, (uint32)(0x40000000|syncMessageID));
  else
      can->Xmit(ID, 0x1005, 0, (uint32)syncMessageID);
}

void CanMotor::stopSync()
{
  if(!syncProducer)
    return;

  can->Xmit(ID, 0x1005, 0, (uint32)syncMessageID);

}




void CanMotor::setControlWord(uint16 val)
{
  can->Xmit(ID, OBJID_CONTROL, 0, val);
}

void CanMotor::setHaltMove()
{
  can->Xmit(ID, OBJID_CONTROL, 0, (int16)0x010F);
}

void CanMotor::setContinueMove()
{
    if(isPositionMode())
    {
        if(lastProfile.poseRelative)
          can->Xmit(ID, OBJID_CONTROL, 0, (int16)0x007F);
        else
          can->Xmit(ID, OBJID_CONTROL, 0, (int16)0x003F);

    }
    else if(isVelocityMode())
    {
        setEnable();
    }

}

void CanMotor::setEnable()
{
  can->Xmit(ID, OBJID_CONTROL, 0, (int16)0x000F);
}

void CanMotor::setDisable()
{
  can->Xmit(ID, OBJID_CONTROL, 0, (int16)0x0005);
}
void CanMotor::restart()
{
    can->Xmit(ID, OBJID_CONTROL, 0, (int16)0x008F);
}

void CanMotor::setPower()
{
    if(isPositionMode())
        setEnable();
}

void CanMotor::setMotionProfileType(int16 val)
{
  can->Xmit(ID, OBJID_PROFILE_TYPE, 0, val);
}

void CanMotor::setModeofOperation(int8 val)
{
  can->Xmit(ID, OBJID_OP_MODE, 0, val);
  lastProfile.moveType = val;

}



void CanMotor::SetTargetPos(double val)
{
  val = between(val, lastProfile.minAngle, lastProfile.maxAngle);
  can->Xmit(ID, OBJID_PROFILE_POS, 0, (int32)(val * lastProfile.coefficient * motorEcoderRates));
  lastProfile.Position = val;
}

void CanMotor::SetProfileVel(int32 val)
{
  can->Xmit(ID, OBJID_PROFILE_VEL, 0, val);
  lastProfile.velocity = val;
}


void CanMotor::SetProfileAcc(int32 val)
{
  can->Xmit(ID, OBJID_PROFILE_ACC, 0, val);
  lastProfile.acceleration = val;
}

void CanMotor::SetProfileDec(int32 val)
{
  can->Xmit(ID, OBJID_PROFILE_DEC, 0, val);
  lastProfile.deceleration = val;
}

void CanMotor::SetProfileJerk(uint32 val)
{
  can->Xmit(ID, OBJID_PROFILE_JRK, 0, val);
  lastProfile.jerk = val;
}
void CanMotor::SetProfileStop(int32 val)
{
  can->Xmit(ID, OBJID_PROFILE_QSTOP, 0, val);
  lastProfile.stopDeceleration = val;
}

void CanMotor::SetTargetVel(int32 val)
{
  val *= motorEcoderRates;
  val = between(val, -lastProfile.maxSpeed, lastProfile.maxSpeed);
  can->Xmit(ID, OBJID_TARGET_VEL, 0, val * lastProfile.coefficient);
  lastProfile.velocity = val;
}

void CanMotor::setProfile(const MotorProfile &p)
{
  if(p.moveType == AMPMODE_CAN_PROFILE)
  {
    if(p.moveType != lastProfile.moveType)
    {
      setModeofOperation(p.moveType);
      setMotionProfileType(0);
    }
    if(p.Position != lastProfile.Position)
    {
      SetTargetPos(p.Position);
    }

    if(p.velocity != lastProfile.velocity)
    {
      SetProfileVel(p.velocity);
    }
  }
  else if(p.moveType == AMPMODE_CAN_VELOCITY)
  {
    if(p.moveType != lastProfile.moveType)
    {
      setMotionProfileType(-1);
      setModeofOperation(p.moveType);
    }

    if(p.velocity != lastProfile.velocity)
    {
      SetTargetVel(p.velocity);
    }
  }
  else
  {
     ROS_ERROR("Node [%s] Move Type undefined!");
  }

  if(p.acceleration != lastProfile.acceleration)
  {
    SetProfileAcc(p.acceleration);
  }

  if(p.deceleration != lastProfile.deceleration)
  {
    SetProfileDec(p.deceleration);
  }
}
