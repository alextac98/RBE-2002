#ifndef ROBOT_HARDWARE_H
#define ROBOT_HARDWARE_H

//Joint Interface Includes
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/robot_hw.h"

//ROS Includes
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

class RobotHardware:
    public hardware_interface::RobotHW
{
  RobotHardware();
public:

private:


  /*Joint structure defined b ros_control InterfaceManager
  to allow control via diff_drive_controller*/

  struct Joint{
    double position;
    double position_offset;
    double velocity;
    double effort;
    double velocity_command;

    Joint() :
      position(0), velocity(0), effort(0), velocity_command(0)
    {}
  } joints[4];



};


#endif // ROBOT_HARDWARE_H
