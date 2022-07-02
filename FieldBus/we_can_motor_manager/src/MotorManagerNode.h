/*
 * motorManagerNode.h
 *
 *  Created on: May 4, 2013
 *      Author: kiter
 */

#ifndef MOTORMANAGERNODE_H_
#define MOTORMANAGERNODE_H_

#include <string.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include <boost/thread/pthread/recursive_mutex.hpp>
#include <dynamic_reconfigure/server.h>
#include <boost/thread/thread.hpp>
#include <we_console/WEConsole.h>

#include "CanMotor.h"
#include "SerialComMotor.h"
#include "WEStopButton.h"
#include "pantiltBroadcaster.h"
#include "ConfigManager.h"

namespace we_can_motor_manager
{
  class MotorManagerParameterConfig;
}
struct BaseFeedback
{
  ros::Time stamp;
  double rightWheelDist;
  double leftWheelDist;

};

struct ArmFeedback
{
  ros::Time stamp;
  int shoulderzPose;
  int shoulderyPose;
  int elbowPose;
  int wristyPose;
  int wristzPose;
  int pawPose;
};

struct PanTiltFeedback
{
  ros::Time stamp;
  double tiltPose;
  double panPose;
  double elevatorPose;
};


class MotorManagerNode : public WEConsole
{
public:
  MotorManagerNode();
  ~MotorManagerNode();

  void onCmd(const std_msgs::StringConstPtr & cmd);
  void onCmdVel(const geometry_msgs::TwistConstPtr & twist);

private:
  void initMotors();

  void onStopButton(bool stop);
  void processCmd(const char* tempCmd);
  void publishOdomAndTF(const BaseFeedback & data);
  void publishMotorAngles(const ArmFeedback & data);
  void calOdom(const double &rdist, const double &ldist, double &theta, double &deltaX, double &deltaY);
  void calSpeed(const float &xSpeed, const float &turnSpeed, float &rightlSpeed, float &leftlSpeed);
  int generateWheelSpeedVal(double val);
  void wheel(double lSpeed, double rSpeed);
  void arm(double sz, double sy, double el, double wy, double wz);
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  ros::Subscriber cmdSub;
  ros::Subscriber cmdVelSub;

  ros::Publisher odomPub;
  ros::Publisher motorAnglePub;
  ros::Publisher stopButtonPub;
  tf::TransformBroadcaster* tf_broadcaster;
  dynamic_reconfigure::Server<we_can_motor_manager::MotorManagerParameterConfig> *drsv;
  void reconfigureCB(we_can_motor_manager::MotorManagerParameterConfig & cfg, uint32_t level);

  BaseFeedback baseFeed;
  ArmFeedback armFeed;
  PanTiltFeedback panTiltFeed;
  boost::recursive_mutex panTiltFeedLock;

  PTBroaderPtr pantiltBroader;

  void baseLoop();
  void armLoop();
  void panTiltLoop();
  void stopButtonLoop();

  bool runBase;
  bool runArm;
  bool runPanTilt;
  bool runStopButton;

  int baseLoopRate;
  int armLoopRate;
  int panTiltLoopRate;
  int stopButtonLoopRate;

  boost::thread* baseThread;
  boost::thread* armThread;
  boost::thread* panTiltThread;
  boost::thread* stopButtonThread;

  volatile bool runBaseThread;
  volatile bool runArmThread;
  volatile bool runPanTiltThread;
  volatile bool runStopButtonThread;

  double elevator;
  double poseX;
  double poseY;
  double angle;

  double wlPose;
  double wrPose;
  double wlLastPose;
  double wrLastPose;
  ros::Time lastOdomTime;

  double maxLinearSpeed;
  double maxTurnSpeed;
  double wheelDist;
  double wheelPerimeter;

  double elevatorUnitHeight;
  double elevatorInitHeight;
  double maxElevatorHeight;
  double minElevatorHeight;
  bool saveElevatorHeight;
  ConfigManager elevatorConfig;
  int generateElevatorPose(double pose);

  typedef std::vector<Motor*>  MotorPtrVector;
  std::map<std::string, MotorPtrVector > motorMap;
  void registMotor(Motor* motor);
  void registMotor(std::string name , Motor*);
  bool inMap(std::string& name);

  MotorProfile p;

  CanOpen* can;
  CanMotor* wl;
  CanMotor* wr;
  CanMotor* ev;
  CanMotor* sz;
  CanMotor* sy;
  CanMotor* el;
  CanMotor* wy;
  CanMotor* wz;
  CanMotor* paw;

  CSerialCom* panTiltCom;
  std::string PanTiltComName;
  ComMotorZW* pan;
  ComMotorZW* tilt;


  CSerialCom* stopButtonCom;
  std::string StopButtonComName;
  WEStopButton *stopButton;

};

#endif /* MOTORMANAGERNODE_H_ */
