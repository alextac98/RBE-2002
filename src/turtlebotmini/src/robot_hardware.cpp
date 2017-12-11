#include "turtlebotmini_base/robot_hardware.h"

extern "C"
{
#include "rc_usefulincludes.h"
}
extern "C"
{
#include "roboticscape.h"
}


TurtleBotMini_Hardware::TurtleBotMini_Hardware(){

  //Set up Robotics Cape
  if (rc_initialize() < 0){
      ROS_INFO("Error: failed to initialize robotics cape drivers");
      return -1;
  }
  rc_enable_motors();
  rc_set_encoder_pos(LEFTENCODER, 0);
  rc_set_encoder_pos(RIGHTENCODER, 0);

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

void TurtleBotMini_Hardware::writeToHardware(ros::Duration timeDiff){

    //Make sure you run updateJoints() before running writeToHardware
    //Calculate the PID output so that the motor output can be set as close to eachother as possible
    leftPID = calculatePID(timeDiff, vel[LEFT], vel_cmd[LEFT]);
    rightPID = calculatePID(timeDiff, vel[RIGHT], vel_cmd[RIGHT]);

    //Set motor output
    rc_set_motor(LEFTMOTOR, leftPID);
    rc_set_motor(RIGHTMOTOR, rightPID);
}

void TurtleBotMini_Hardware::updateJoints(ros::Duration timeDiff){

    //Update Velocity
     vel[LEFT] = (rc_get_encoder_pos(LEFTENCODER) - lastLeftEncoderPos)/timeDiff.toSec();
     vel[RIGHT] = (rc_get_encoder_pos(RIGHTENCODER) - lastRightEncoderPos)/timeDiff.toSec();

     //Update Position
     //pos[LEFT] = rc_get_encoder_pos(LEFTENCODER);
     //pos[RIGHT] = rc_get_encoder_pos(RIGHTENCODER);
}

float TurtleBotMini_Hardware::calculatePID(ros::Duration timeChange, float input , float setpoint) {

    volatile float timeDiff = timeChange.toSec();

    volatile float error = setpoint - input;

    //Calculate P
    P = error * kP;

    //Calculate I
    I = I + (error * kI * timeDiff);

    //Calculate D
    D = (lastInput - input) * kD / timeDiff;
    lastInput = input;

    return P + I + D;
}

void changekP(int kp){
    kP = kp;
}

void changekI(int ki){
    kI = ki;
}

void changekD(int kd){
    kD = kd;
}
