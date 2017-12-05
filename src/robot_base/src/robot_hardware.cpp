#include robot_base/robot_hardware.h

extern "C"
{
#include "rc_usefulincludes.h"
}
extern "C"
{
#include "roboticscape.h"
}


RobotHardware::RobotHardware(){
  if (rc_initialize() < 0){
      ROS_INFO("Error: failed to initialize robotics cape drivers");
      return -1;
  }
  rc_enable_motors();


}

RobotHardware::writeToHardware(){

}
