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

#define LEFTENCODER 4
#define RIGHTENCODER 1

#define LEFTMOTOR 1
#define RIGHTMOTOR 4

//PID Constants
float kP = 0;
float kI = 0;
float kD = 0;

float P = 0;
float I = 0;
float D = 0;
float lastInput = 0;

class TurtleBotMini_Hardware:
    public hardware_interface::RobotHW
{

public:
  TurtleBotMini_Hardware();
  void updateJoints();
  void writeToHardware();

  void changekP(int kp);
  void changekI(int ki);
  void changekD(int kd);

private:

  //Robot Constants
  const float wheelDiameter = 0.0075; //in meters
  const float maxSpeed = 15;      //in rad/s
  const float maxAccelleration; //in rad/s^2

  //Robot Variables
  float leftPID;
  float rightPID;

  //Private Robot Methods
  float calculatePID(ros::Duration timeChange, float input , float setpoint);

  //ROS Control Interfaces
  hardware_interface::JointStateInterface joint_state_interface;
  hardware_interface::VelocityJointInterface velocity_joint_interface;

  //Robot Control Variables
  float vel_cmd[2];
  float pos[2];
  float vel[2];
  float eff[2];

  int lastLeftEncoderPos = 0;
  int lastRightEncoderPos = 0;

};

#endif // ROBOT_HARDWARE_H
