#include"MotorManagerNode.h"


int main(int argc, char** argv)
{

  ros::init(argc, argv, "we_motor_manager_node");
  ros::NodeHandle nh("~");
  std::string logFileName;
  nh.param("logFileName", logFileName, std::string("b.log"));
  _log.setWELogLevel(WE_LOG::WELog_Everythig);
  _log.setLogFile(logFileName.c_str());
  MotorManagerNode node;
  ros::spin();
}
