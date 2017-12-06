#ifndef ROBOT_HARDWARE_H
#define ROBOT_HARDWARE_H

//Joint Interface Includes
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/robot_hw.h"

//ROS Includes
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

#define LEFTWHEEL 0
#define RIGHTWHEEL 1

class RobotHardware:
    public hardware_interface::RobotHW
{

public:

  RobotHardware();
  void updateJoints();
  void writeToHardware();


private:

  //Robot Constants
  float wheelDiameter; //not needed i think
  float maxSpeed = 15;      //in rad/s
  float maxAccelleration; //in rad/s^2

  //ROS Control Interfaces
  hardware_interface::JointStateInterface joint_state_interface;
  hardware_interface::VelocityJointInterface velocity_joint_interface;

  //Robot Control Variables
  // 0 is left, 1 is right
  double vel_cmd[2];
  double pos[2];
  double vel[2];
  double eff[2];



};


#endif // ROBOT_HARDWARE_H
