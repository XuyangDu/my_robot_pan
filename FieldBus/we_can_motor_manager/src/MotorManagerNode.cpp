/*
 * motorManagerNode.cpp
 *
 *  Created on: May 4, 2013
 *      Author: Min Cheng
 */

#include <iostream>
#include <string>
#include <string.h>
#include <pthread.h>
#include "xform.h"
#include <we_msgs/MotorAngles.h>
#include "MotorManagerNode.h"
#include <we_can_motor_manager/MotorManagerParameterConfig.h>

using namespace std;

MotorManagerNode::MotorManagerNode() :
    WEConsole("base", boost::bind(&MotorManagerNode::processCmd, this, _1)),
    private_nh("~")
{
  drsv = NULL;
  poseX = poseY = angle = 0.0;
  wlPose = wrPose = wlLastPose = wrLastPose = 0;
  elevator = 0;
  lastOdomTime = ros::Time::now();
  can = NULL;
  panTiltCom = NULL;
  stopButtonCom = NULL;
   wl = wr = ev = sz = sy = el = wy = wz = paw = NULL;
   runBaseThread = runArmThread = runPanTiltThread = runStopButtonThread = false;
  pan = tilt = NULL;
  stopButton = NULL;

  cmdSub = nh.subscribe<std_msgs::String>("motor_manager_cmd", 100, &MotorManagerNode::onCmd, this);
  cmdVelSub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 20, &MotorManagerNode::onCmdVel, this);
  odomPub = nh.advertise<nav_msgs::Odometry>("odom", 100);
  motorAnglePub = nh.advertise<we_msgs::MotorAngles>("motor_angle", 100);
  stopButtonPub = nh.advertise<std_msgs::String>("stop_button", 10);

  tf_broadcaster = new tf::TransformBroadcaster;
  pantiltBroader.reset(new PantiltBroadcaster(private_nh, tf_broadcaster));
  drsv = new dynamic_reconfigure::Server<we_can_motor_manager::MotorManagerParameterConfig>;
  drsv->setCallback(boost::bind(&MotorManagerNode::reconfigureCB, this, _1, _2));

  private_nh.param("maxLinearSpeed", maxLinearSpeed, 0.5);
  private_nh.param("maxTurnSpeed", maxTurnSpeed, 0.4);

  private_nh.param("wheelDist", wheelDist, 0.556);
  private_nh.param("wheelPerimeter", wheelPerimeter, 0.606);
  private_nh.param("elevatorUnitHeight", elevatorUnitHeight, 0.15803);
  private_nh.param("maxElevatorHeight", maxElevatorHeight, 0.0);
  private_nh.param("minElevatorHeight", minElevatorHeight, 0.0);

  string keyName;
  if(private_nh.searchParam("elevatorInitFile", keyName))
  {
    string file;
    private_nh.param(keyName, file, string("/cfg/elevator.yaml"));
    elevatorConfig.setFileName(file);
    elevatorConfig.loadConfigs();
    if(elevatorConfig.loadConfigs())
    {
      if (!elevatorConfig.getValue("elevatorInitHeight", elevatorInitHeight))
      {
        ROS_WARN("%s", elevatorConfig.getErrorTex().c_str());
      }

    }
    else
    {
      ROS_WARN("%s", elevatorConfig.getErrorTex().c_str());
    }

    ROS_INFO("Elevator init height %.2f\n", elevatorInitHeight);
  }
  else
  {
    saveElevatorHeight = false;
  }


  initMotors();
  MotorPtrVector & motors = motorMap["all"];
  for(MotorPtrVector::iterator it = motors.begin(); it !=  motors.end(); it++)
    (*it)->init();

  double offset;
  private_nh.param("panOffset", offset, 0.0);
  if(pan)
      pan->setOffset(offset);
  private_nh.param("tiltOffset", offset, 0.0);
  ROS_INFO("tilt init %.2f\n", offset);
  if(tilt)
      tilt->setOffset(offset);

  if(can && can->isOpened())
    can->startRecive();
  if(panTiltCom && panTiltCom->isOpened())
    panTiltCom->startRecive();

  if(ev)
  {
      ev->clearPose();
      ev->setupPositionMove(0);
  }
}


MotorManagerNode::~MotorManagerNode()
{
  wheel(0,0);

  if(saveElevatorHeight && elevator >= minElevatorHeight)
  {

    if (!elevatorConfig.setValue("elevatorInitHeight", elevator))
      ROS_WARN("%s", elevatorConfig.getErrorTex().c_str());
    if (!elevatorConfig.saveConfigs())
      ROS_WARN("%s", elevatorConfig.getErrorTex().c_str());

  }

  if(can)
    can->stopRecive();

  if(panTiltCom)
    panTiltCom->stopRecive();

  runBaseThread = false;
  runArmThread = false;
  runStopButtonThread = false;
  runPanTiltThread = false;

  if (baseThread != NULL)
  {
    baseThread->join();
    delete baseThread;
  }

  if (armThread != NULL)
  {
    armThread->join();
    delete armThread;
  }

  if (panTiltThread != NULL)
  {
    panTiltThread->join();
    delete panTiltThread;
  }

  if (stopButtonThread != NULL)
  {
    stopButtonThread->join();
    delete stopButtonThread;
  }
  if(wl)
      delete wl;
  if(wr)
      delete wr;
  if(ev)
      delete ev;
  if(sz)
      delete sz;
  if(sy)
      delete sy;
  if(el)
      delete el;
  if(wy)
      delete wy;
  if(wz)
      delete wz;
  if(paw)
      delete paw;
  if(pan)
      delete pan;
  if(tilt)
      delete tilt;

  if(can != NULL)
    delete can;

  if(panTiltCom != NULL)
    delete panTiltCom;

  if(stopButton != NULL)
    delete stopButton;

  if(stopButtonCom != NULL)
    delete stopButtonCom;

  delete tf_broadcaster;
}

void MotorManagerNode::initMotors()
{

  private_nh.param("baseLoopRate", baseLoopRate, 10);
  private_nh.param("armLoopRate", armLoopRate, 10);
  private_nh.param("stopButtonLoopRate", stopButtonLoopRate, 10);
  private_nh.param("panTiltLoopRate", panTiltLoopRate, 10);

  private_nh.param("runBase", runBase, true);
  private_nh.param("runArm", runArm, true);
  private_nh.param("runPanTilt", runPanTilt, true);
  private_nh.param("runStopButton", runStopButton, true);

  if(runBase || runArm)
  {
    can = new CanOpen(VCI_USBCAN1, 0, 0);
    if(!can->openDevice())
    {
      ROS_ERROR("Open can failed, can network motors will not be started!");
      runBase = false;
      runArm = false;
    }
  }

  if (runBase)
  {
    wl = new CanMotor("wl", 12, can);
    wr = new CanMotor("wr", 13, can);
    can->setPosePDOBaseID(wl->getPosePDOBaseID());
    wl->setSyncProducer(true);
    registMotor((Motor*)wl);
    registMotor((Motor*)wr);
    registMotor("all",  (Motor*)wl);
    registMotor("all",  (Motor*)wr);

    runBaseThread = true;
    baseThread = new boost::thread(boost::bind(&MotorManagerNode::baseLoop, this));
  }
  else
  {
    runBaseThread = false;
    baseThread = NULL;
    ROS_WARN("Base thread didn't start up !");
  }

  if(runArm)
  {
      ev = new CanMotor("ev", 11, can);   //50w
      sz = new CanMotor("sz", 2, can);      //50w
      sy = new CanMotor("sy", 3, can);      //50w
      el = new CanMotor("el", 4, can);      //30w
      wy = new CanMotor("wy", 5, can);       //30w
      wz = new CanMotor("wz", 6, can);         //15w
      paw = new CanMotor("paw", 7, can);        //15w
    if(!runBase)
    {
      can->setPosePDOBaseID(sz->getPosePDOBaseID());
      sz->setSyncProducer(true);
    }
    registMotor((Motor*)ev);
    registMotor((Motor*)sz);
    registMotor((Motor*)sy);  //50w
    registMotor((Motor*)el);  //30w
    registMotor((Motor*)wy);  //30w
    registMotor((Motor*)wz);  //15w
    registMotor((Motor*)paw); //15w

    registMotor("all", (Motor*)ev);
    registMotor("all", (Motor*)sz);
    registMotor("all", (Motor*)sy);
    registMotor("all", (Motor*)el);
    registMotor("all", (Motor*)wy);
    registMotor("all", (Motor*)wz);
    registMotor("all", (Motor*)paw);

    registMotor("arm", (Motor*)sz);
    registMotor("arm", (Motor*)sy);
    registMotor("arm", (Motor*)el);
    registMotor("arm", (Motor*)wy);
    registMotor("arm", (Motor*)wz);
    registMotor("arm", (Motor*)paw);

    runArmThread = true;
    armThread = new boost::thread(boost::bind(&MotorManagerNode::armLoop, this));
  }
  else
  {
    runArmThread = false;
    armThread = NULL;
    ROS_WARN("Arm thread didn't start up !");
  }

  if(runPanTilt)
  {
    private_nh.param("PanTiltComName", PanTiltComName, string("/dev/ttyUSB0"));
    panTiltCom = new CSerialCom(PanTiltComName.c_str(),19200, 8, 'N', 1);
    if(!panTiltCom->openDevice())
    {
      ROS_ERROR("Open pan and tilt usbcom failed!");
      runPanTilt = false;
      runPanTiltThread = false;
      panTiltThread = NULL;
      ROS_WARN("Pan and Tilt thread didn't start up !");
    }
    else
    {
      pan = new ComMotorZW("pan", 76, panTiltCom);
      tilt = new ComMotorZW("tilt", 77, panTiltCom);

      registMotor((Motor*)pan);
      registMotor((Motor*)tilt);

      registMotor("all",  (Motor*)pan);
      registMotor("all",  (Motor*)tilt);

      registMotor("cam", (Motor*)pan);
      registMotor("cam", (Motor*)tilt);
      runPanTiltThread = true;
      panTiltThread = new boost::thread(boost::bind(&MotorManagerNode::panTiltLoop, this));
    }

  }
  else
  {
      runPanTiltThread = false;
      panTiltThread = NULL;
      ROS_WARN("Pan and Tilt thread didn't start up !");
  }


  if(runStopButton)
  {
    private_nh.param("StopButtonComName", StopButtonComName, string("/dev/ttyUSB0"));
    stopButtonCom = new CSerialCom(StopButtonComName.c_str(), 9600, 8, 'N', 1);
    if(!stopButtonCom->openDevice())
    {
      ROS_ERROR("Open stop button usbcom failed!");
      runStopButton = false;
      runStopButtonThread = false;
      stopButtonThread = NULL;
      ROS_WARN("Stop button thread didn't start up !");
    }
    else
    {
      stopButton = new WEStopButton("StopButton", stopButtonCom, boost::bind(&MotorManagerNode::onStopButton, this, _1));
      runStopButtonThread = true;
      stopButtonThread = new boost::thread(boost::bind(&MotorManagerNode::stopButtonLoop, this));
    }

  }
  else
  {
      runStopButtonThread = false;
      stopButtonThread = NULL;
      ROS_WARN("Stop button thread didn't start up !");
  }
}

void MotorManagerNode::baseLoop()
{
  ROS_INFO("Base Thread Start...");
  ros::Rate loopRate(baseLoopRate);
  sleep(2);
  while (runBaseThread)
  {
    baseFeed.stamp = ros::Time::now();
    wlPose = wl->getPose();
    wrPose = wr->getPose();

    baseFeed.leftWheelDist =  (wlPose - wlLastPose) * wheelPerimeter / 360.0;
    baseFeed.rightWheelDist = (wrPose - wrLastPose) * wheelPerimeter / 360.0;
    wlLastPose = wlPose;
    wrLastPose = wrPose;
    publishOdomAndTF(baseFeed);

    loopRate.sleep();
    /*
    if (loopRate.cycleTime() > ros::Duration(1.0 / baseLoopRate))
      ROS_WARN(
          "Base loop missed its desired rate of %dHz... the loop actually took %.5f seconds", baseLoopRate, loopRate.cycleTime().toSec());
      */
  }
}

void MotorManagerNode::armLoop()
{
  ROS_INFO("Arm Thread Start...");
  ros::Rate loopRate(armLoopRate);
  sleep(2);
  while (runArmThread)
  {
    we_msgs::MotorAngles angles;

    angles.stamp = ros::Time::now();
    angles.shoulderz = sz->getPose();
    angles.shouldery= sy->getPose();
    angles.elbow = el->getPose();
    angles.wristy= wy->getPose();
    angles.wristz= wz->getPose();
    angles.paw = paw->getPose();

    panTiltFeedLock.lock();
    angles.pan = (int)(panTiltFeed.panPose + (panTiltFeed.panPose > 0 ? 0.5 : -0.5));
    angles.tilt = (int)(panTiltFeed.tiltPose + (panTiltFeed.tiltPose > 0 ? 0.5 : -0.5));
    angles.elevator = panTiltFeed.elevatorPose;
    panTiltFeedLock.unlock();

    motorAnglePub.publish(angles);

    loopRate.sleep();
    /*
    if (loopRate.cycleTime() > ros::Duration(1.0 / armLoopRate))
      ROS_WARN(
          "Arm loop missed its desired rate of %dHz... the loop actually took %.4f seconds", armLoopRate, loopRate.cycleTime().toSec());
          */
  }
}

void MotorManagerNode::panTiltLoop()
{
  ROS_INFO("Pan Tilt Thread Start...");
  ros::Rate loopRate(panTiltLoopRate);
  sleep(2);
  while(runPanTiltThread)
  {
    panTiltFeedLock.lock();
    panTiltFeed.stamp = ros::Time::now();
    panTiltFeed.panPose = pan->getPose();
    panTiltFeed.tiltPose = tilt->getPose();

    if(ev)
        elevator = elevatorInitHeight + (ev->getPose() * elevatorUnitHeight / 360.0);

    panTiltFeed.elevatorPose = elevator;
    panTiltFeedLock.unlock();

    pantiltBroader->updateParams();
    pantiltBroader->updateTransform(panTiltFeed.panPose, panTiltFeed.tiltPose, panTiltFeed.elevatorPose, panTiltFeed.stamp);

    if(!runArmThread)
    {
        we_msgs::MotorAngles angles;

        angles.stamp = ros::Time::now();
        angles.shoulderz = 0;
        angles.shouldery= 0;
        angles.elbow = 0;
        angles.wristy= 0;
        angles.wristz= 0;

        panTiltFeedLock.lock();
        angles.pan = (int)(panTiltFeed.panPose + (panTiltFeed.panPose > 0 ? 0.5 : -0.5));
        angles.tilt = (int)(panTiltFeed.tiltPose + (panTiltFeed.tiltPose > 0 ? 0.5 : -0.5));
        angles.elevator = panTiltFeed.elevatorPose;
        panTiltFeedLock.unlock();

        motorAnglePub.publish(angles);

    }
    loopRate.sleep();
  }
}

void MotorManagerNode::stopButtonLoop()
{
  ROS_INFO("Stop Button Thread Start...");
  ros::Rate loopRate(stopButtonLoopRate);
  sleep(2);
  while (runStopButtonThread)
  {
    stopButton->mainLoop();
    loopRate.sleep();
  }
}


void MotorManagerNode::onCmd(const std_msgs::StringConstPtr & cmd)
{
  processCmd(cmd->data.c_str());

}
void MotorManagerNode::onCmdVel(const geometry_msgs::TwistConstPtr & twist)
{
  float rSpeed, lSpeed;
  float linear = between(twist->linear.x, -maxLinearSpeed, maxLinearSpeed);
  float turn = between(twist->angular.z,  -maxTurnSpeed, maxTurnSpeed);
  calSpeed(linear, turn, rSpeed, lSpeed);
  wheel(lSpeed, rSpeed);
}

void MotorManagerNode::onStopButton(bool stop)
{
  MotorPtrVector motors = motorMap["all"];
  for(MotorPtrVector::iterator it = motors.begin(); it != motors.end(); it++)
  {
    if(stop)
      (*it)->motorStop();
    else
      (*it)->motorContinue();
  }

  std_msgs::String str;
  if(stop)
    str.data = string("stop button on");
  else
      str.data = string("stop button off");
  stopButtonPub.publish(str);
}

void MotorManagerNode::processCmd(const char *tempCmd)
{
  if (strlen(tempCmd) < 1)
    return;
  const char *cmd;
  string temp(tempCmd);
  unsigned int start = temp.find_first_not_of(' ');
  unsigned int end = temp.find_last_not_of(' ');
  if(start != string::npos && end != string::npos)
  {
    temp = temp.substr(start, end - start + 1);
    cmd = temp.c_str();
  }
  else
      return;

  float f1, f2;
  int d1, d2, d3, d4, d5;

  if (PEEK_CMD_FF(cmd, "m", 1, f1, f2))
  {
    wheel(f1, f2);

  }
  else if (PEEK_CMD_F(cmd, "ev", 2, f1))
  {
    if(ev)
    {
      ev->setupPositionMove(generateElevatorPose(f1));
      ev->startPositionMove();
    }

  }
  else if (PEEK_CMD_F(cmd, "set ev", 6, f1))
  {
    if(ev)
    {
      elevatorInitHeight = f1;
      ev->clearPose();
    }
  }
  else if (PEEK_CMD_F(cmd, "add ev", 6, f1))
  {
    if(ev)
    {
      f1 += elevator;
      ev->setupPositionMove(generateElevatorPose(f1));
      ev->startPositionMove();
    }
  }
  else if (PEEK_CMD_D(cmd,"stop", 4, d1))
  {
    onStopButton(d1);
  }
  else if(PEEK_CMD_N(cmd, "stop", 4))
  {
    wheel(0, 0);
  }
  else if (PEEK_CMD_DDDDD(cmd, "arm", 3, d1, d2, d3, d4, d5))
  {
    arm(d1, d2, d3, d4,d5);
  }
  else if (PEEK_CMD_D(cmd, "tilt", 4, d1))
  {
    if(tilt)
    {
      tilt->setupPositionMove(d1);
      tilt->startPositionMove();
    }
  }
  else if (PEEK_CMD_D(cmd, "pan", 3, d1))
  {
    if(pan)
    {
      pan->setupPositionMove(d1);
      pan->startPositionMove();
    }
  }
  else if (PEEK_CMD_D(cmd, "paw", 3, d1))
  {
    if(paw)
    {
      paw->setupPositionMove(d1);
      paw->startPositionMove();
    }
  }
  else if (PEEK_CMD_D(cmd, "sz", 2, d1))
  {
    if(sz)
    {
      sz->setupPositionMove(d1);
      sz->startPositionMove();
    }
  }
  else if (PEEK_CMD_D(cmd, "sy", 2, d1))
  {
    if(sy)
    {
      sy->setupPositionMove(d1);
      sy->startPositionMove();
    }
  }
  else if (PEEK_CMD_D(cmd, "el", 2, d1))
  {
    if(el)
    {
      el->setupPositionMove(d1);
      el->startPositionMove();
    }
  }
  else if (PEEK_CMD_D(cmd, "wy", 2, d1))
  {
    if(wy)
    {
      wy->setupPositionMove(d1);
      wy->startPositionMove();
    }
  }
  else if (PEEK_CMD_D(cmd, "wz", 2, d1))
  {
    if(wz)
    {
      wz->setupPositionMove(d1);
      wz->startPositionMove();
    }
  }
  else if (PEEK_CMD_N(cmd, "add", 3))
  {
    string name(cmd);
    name = name.substr(name.find(' ') + 1);
    if (name.find("pan") != string::npos)
    {
      if(pan)
      {
        double inc = 0;
        sscanf(name.c_str(), "pan %lf", &inc);

        inc += pan->getPose();
        pan->setupPositionMove(inc);
        pan->startPositionMove();
      }
    }
    else if (name.find("tilt") != string::npos)
    {
        // add tilt
      if(tilt)
      {
        double inc = 0;
        sscanf(name.c_str(), "tilt %lf", &inc);
        inc += tilt->getPose();
        tilt->setupPositionMove(inc);
        tilt->startPositionMove();
      }
    }
    else
        cout << "receive unknown command [" << cmd << "]\n\n";
    return;
  }
  else if (PEEK_CMD_N(cmd, "break", 5))
  {
    string name(cmd);
    name = name.substr(name.find(' ') + 1);
    if(inMap(name))
    {
      MotorPtrVector motors;
      if(name == string("all"))
      {
        motors = motorMap["arm"];
        for(MotorPtrVector::iterator it = motors.begin(); it != motors.end(); it++)
          (*it)->motorbreak();

        motors = motorMap["cam"];
        for(MotorPtrVector::iterator it = motors.begin(); it != motors.end(); it++)
          (*it)->motorbreak();
      }
      else
      {
        motors = motorMap[name];
        for(MotorPtrVector::iterator it = motors.begin(); it != motors.end(); it++)
          (*it)->motorbreak();
      }

    }
    else
    {
      cout << "receive unknown command [" << cmd << "]\n\n";
      return;
    }

  }
  else if (PEEK_CMD_N(cmd, "clear", 5))
  {
    string name(cmd);
    name = name.substr(name.find(' ') + 1);
    if(inMap(name))
    {
      MotorPtrVector motors;
      if(name == string("all"))
      {
        motors = motorMap["arm"];
        for(MotorPtrVector::iterator it = motors.begin(); it != motors.end(); it++)
          (*it)->motorClear();

        motors = motorMap["cam"];
        for(MotorPtrVector::iterator it = motors.begin(); it != motors.end(); it++)
          (*it)->motorClear();
      }
      else
      {
        motors = motorMap[name];
        for(MotorPtrVector::iterator it = motors.begin(); it != motors.end(); it++)
          (*it)->motorClear();
      }
    }
    else
    {
      cout << "receive unknown command [" << cmd << "]\n\n";
      return;
    }
  }
  else if (PEEK_CMD_N(cmd, "init", 4))
  {
    string name(cmd);
    name = name.substr(name.find(' ') + 1);
    if(inMap(name))
    {
      MotorPtrVector motors;
      if(name == string("all"))
      {
        motors = motorMap["arm"];
        for(MotorPtrVector::iterator it = motors.begin(); it != motors.end(); it++)
          (*it)->motorInit();

        motors = motorMap["cam"];
        for(MotorPtrVector::iterator it = motors.begin(); it != motors.end(); it++)
          (*it)->motorInit();
      }
      else
      {
        motors = motorMap[name];
        for(MotorPtrVector::iterator it = motors.begin(); it != motors.end(); it++)
          (*it)->motorInit();
      }
    }
    else
    {
      cout << "receive unknown command [" << cmd << "]\n";
      return;
    }
  }
  else if (PEEK_CMD_N(cmd, "power", 5))
  {
    string name(cmd);
    name = name.substr(name.find(' ') + 1);
    if(inMap(name))
    {
      MotorPtrVector motors;
      if(name == string("all"))
      {
        motors = motorMap["arm"];
        for(MotorPtrVector::iterator it = motors.begin(); it != motors.end(); it++)
          (*it)->setPower();

        motors = motorMap["cam"];
        for(MotorPtrVector::iterator it = motors.begin(); it != motors.end(); it++)
          (*it)->setPower();
      }
      else
      {
        motors = motorMap[name];
        for(MotorPtrVector::iterator it = motors.begin(); it != motors.end(); it++)
          (*it)->setPower();
      }
    }
    else
    {
      cout << "receive unknown command [" << cmd << "]\n";
      return;
    }
  }
  else if (PEEK_CMD_N(cmd, "restart", 7))
  {
    string name(cmd);
    name = name.substr(name.find(' ') + 1);
    if(inMap(name) && name != string("all") && name != string("arm"))
    {
      MotorPtrVector motors;
      motors = motorMap[name];
      for(MotorPtrVector::iterator it = motors.begin(); it != motors.end(); it++)
          (*it)->restart();
    }
    else
    {
      cout << "receive unknown command [" << cmd << "]\n";
      return;
    }
  }
  else if (PEEK_CMD_DDDDD(cmd, "speed arm", 9, d1, d2, d3, d4, d5))
  {
    if(runArmThread)
    {
      sz->setSpeed(d1);
      sy->setSpeed(d2);
      el->setSpeed(d3);
      wy->setSpeed(d4);
      wz->setSpeed(d5);
    }
  }
  else if (PEEK_CMD_N(cmd, "speed acc", 9))
  {
    string name(cmd+9);
    int start = name.find_first_of(' ');
    int end = name.find_last_of(' ');
    string stringVal = name.substr(end + 1);
    name = name.substr(start + 1, end - start - 1);

    std::stringstream ss(stringVal);
    int val = -100;
    ss >> val;

    if(inMap(name))
    {
      MotorPtrVector motors;
      motors = motorMap[name];
      for(MotorPtrVector::iterator it = motors.begin(); it != motors.end(); it++)
        (*it)->setSpeedAcc(val);
    }
    else
    {
      cout << "receive unknown command [" << cmd << "]\n";
      return;
    }
  }
  else if (PEEK_CMD_N(cmd, "speed dec", 9))
  {
    string name(cmd+9);
    int start = name.find_first_of(' ');
    int end = name.find_last_of(' ');
    string stringVal = name.substr(end + 1);
    name = name.substr(start + 1, end - start - 1);

    std::stringstream ss(stringVal);
    int val = -100;
    ss >> val;

    if(inMap(name))
    {
      MotorPtrVector motors;
      motors = motorMap[name];
      for(MotorPtrVector::iterator it = motors.begin(); it != motors.end(); it++)
        (*it)->setSpeedDec(val);
    }
    else
    {
      cout << "receive unknown command [" << cmd << "]\n";
      return;
    }
  }
  else if(PEEK_CMD_N(cmd, "set profile", 11))
  {
    char name[100]={0};
    sscanf(cmd + 12,"%s %d %d %d", name, &d1, &d2, &d3);
    string stringName((const char*)name);
    if(inMap(stringName))
    {
      MotorPtrVector motors;
      motors = motorMap[name];
      for(MotorPtrVector::iterator it = motors.begin(); it != motors.end(); it++)
      {
        (*it)->setSpeed(d1);
        (*it)->setSpeedAcc(d2);
        (*it)->setSpeedDec(d3);
      }
    }
    else
    {
      cout << "receive unknown command [" << cmd << "]\n";
      return;
    }
  }
  else if(PEEK_CMD_N(cmd, "reset", 5))
  {
    string name(cmd);
    int start = name.find_first_of(' ');
    name = name.substr(start + 1);

    if(inMap(name))
    {
      MotorPtrVector motors;
      motors = motorMap[name];
      for(MotorPtrVector::iterator it = motors.begin(); it != motors.end(); it++)
        (*it)->resetProfile();
    }
    else
    {
      cout << "receive unknown command [" << cmd << "]\n";
      return;
    }
  }
  else if (PEEK_CMD_N(cmd, "speed", 5))
  {
    string name(cmd);
    int start = name.find_first_of(' ');
    int end = name.find_last_of(' ');
    string stringVal = name.substr(end + 1);
    name = name.substr(start + 1, end - start - 1);

    std::stringstream ss(stringVal);
    int val = -100;
    ss >> val;

    if(inMap(name))
    {
      MotorPtrVector motors;
      motors = motorMap[name];
      for(MotorPtrVector::iterator it = motors.begin(); it != motors.end(); it++)
        (*it)->setSpeed(val);
    }
    else
    {
      cout << "receive unknown command [" << cmd << "]\n";
      return;
    }
  }
  else if (PEEK_CMD_N(cmd, "armbk", 5))
  {
    if(runArmThread)
      arm(sz->getPose() > 0 ? 90 : -90, 90, 80, 0, 0);
  }
  else
  {
    cout << "receive unknown command [" << cmd << "]\n";
    return;
  }
}

void MotorManagerNode::publishOdomAndTF(const BaseFeedback & data)
{
  double theta, deltaX, deltaY, turnSpeed, xSpeed, ySpeed, lWheelSpeed, rWheelSpeed;

  calOdom(data.rightWheelDist, data.leftWheelDist, theta, deltaX, deltaY);
  double t = 1.0 / (data.stamp.toSec() - lastOdomTime.toSec());
  lastOdomTime = data.stamp;

  turnSpeed = theta * t;
  xSpeed = deltaX * t;
  ySpeed = deltaY * t;
  lWheelSpeed = data.leftWheelDist * t;
  rWheelSpeed = data.rightWheelDist * t;

  double poseDeltaX = cos(angle) * deltaX - sin(angle) * deltaY;
  double poseDeltaY = sin(angle) * deltaX + cos(angle) * deltaY;
  poseX += poseDeltaX;
  poseY += poseDeltaY;
  angle = anormalize(angle + theta);

  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(angle);
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = data.stamp;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";
  odom_trans.transform.translation.x = poseX;
  odom_trans.transform.translation.y = poseY;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;
  tf_broadcaster->sendTransform(odom_trans);


  nav_msgs::Odometry odom;
  odom.header.stamp = data.stamp;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";

  odom.pose.pose.position.x = poseX;
  odom.pose.pose.position.y = poseY;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  odom.twist.twist.linear.x = xSpeed;
  odom.twist.twist.linear.y = ySpeed;
  odom.twist.twist.angular.x = lWheelSpeed;
  odom.twist.twist.angular.y = rWheelSpeed;
  odom.twist.twist.angular.z = turnSpeed;

  odomPub.publish(odom);
}

void MotorManagerNode::calOdom(const double &rdist, const double &ldist, double &theta, double &deltaX, double &deltaY)
{
  theta = (rdist - ldist) / wheelDist;
  double radius;
  if (fabs(theta) > 0.00000000001)
  {
    radius = (rdist + ldist) / (2 * theta);
    deltaX = sin(theta) * radius;
    deltaY = (1 - cos(theta)) * radius;
  }
  else
  {
    deltaX = (rdist + ldist) / 2;
    deltaY = 0;
  }
}

void MotorManagerNode::calSpeed(const float &xSpeed, const float &turnSpeed, float &rightSpeed, float &leftSpeed)
{
  rightSpeed = xSpeed + (turnSpeed * wheelDist) / 2;
  leftSpeed = xSpeed - (turnSpeed * wheelDist) / 2;
}

int MotorManagerNode::generateWheelSpeedVal(double val)
{
  int sign = 1;
  if (val < 0)
  {
    sign = -1;
    val = -val;
  }
  return (int)(sign * (val / wheelPerimeter) * 10 * 360);
}



void MotorManagerNode::registMotor(Motor *motor)
{
  motorMap[motor->name].push_back(motor);
}

void MotorManagerNode::registMotor(string name, Motor *motor)
{
   motorMap[name].push_back(motor);
}

bool MotorManagerNode::inMap(string &name)
{
  return (motorMap.find(name) != motorMap.end());
}

void MotorManagerNode::wheel(double lSpeed, double rSpeed)
{
  if(!runBaseThread)
    return;
  int lvel, rvel;
  lvel = generateWheelSpeedVal(lSpeed);
  rvel = generateWheelSpeedVal(rSpeed);
  wl->velocityMove(lvel);
  wr->velocityMove(rvel);
}

void MotorManagerNode::arm(double szPose, double syPose, double elPose, double wyPose, double wzPose)
{
  if(runArmThread)
  {
    sz->setupPositionMove(szPose);
    sy->setupPositionMove(syPose);
    el->setupPositionMove(elPose);
    wy->setupPositionMove(wyPose);
    wz->setupPositionMove(wzPose);

    sz->startPositionMove();
    sy->startPositionMove();
    el->startPositionMove();
    wy->startPositionMove();
    wz->startPositionMove();
  }

}

int MotorManagerNode::generateElevatorPose(double pose)
{
  double pos = between(pose, minElevatorHeight, maxElevatorHeight);
  pos = pos - elevatorInitHeight;
  return (int)((pos / elevatorUnitHeight) * 360);
}

void MotorManagerNode::reconfigureCB(we_can_motor_manager::MotorManagerParameterConfig &cfg, uint32_t level)
{
  if(level == 0)
  {
    if(pan)
        pan->setOffset(cfg.panOffset);
    if(tilt)
        tilt->setOffset(cfg.tiltOffset);
  }
  else if(level == 1)
  {
    wheelDist = cfg.wheelDist;
    wheelPerimeter = cfg.wheelPerimeter;
  }
}
