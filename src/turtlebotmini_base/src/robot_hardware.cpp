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

  //connect joint state interface and register handle
  hardware_interface::JointStateHandle lWheel_stateHandle("wheel_left", &pos[LEFTWHEEL], &vel[LEFTWHEEL], &eff[LEFTWHEEL]);
  joint_state_interface.registerHandle(lWheel_stateHandle);

  hardware_interface::JointStateHandle rWheel_stateHandle("wheel_right", &pos[RIGHTWHEEL], &vel[RIGHTWHEEL], &eff[RIGHTWHEEL]);
  joint_state_interface.registerHandle(rWheel_stateHandle);

  //connect joint velocity interface and register handle
  hardware_interface::JointHandle lWheel_velHandle(joint_state_interface.getHandle("wheel_left"), &vel_cmd[LEFTWHEEL]);
  velocity_joint_interface.registerHandle(lWheel_velHandle);

  hardware_interface::JointHandle rWheel_velHandle(joint_state_interface.getHandle("wheel_right"), &vel_cmd[RIGHTWHEEL]);
  velocity_joint_interface.registerHandle(rWheel_velHandle);

  //register interfaces
  registerInterface(&joint_state_interface);
  registerInterface(&velocity_joint_interface);



}

RobotHardware::writeToHardware(){

}
