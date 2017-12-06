#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "beaglebot_msg/setMotor_msg.h"

#include "beaglebot_blue/motor.h"


extern "C"
{
#include "rc_usefulincludes.h"
}
extern "C"
{
#include "roboticscape.h"
}

void enableMotors_callback(const std_msgs::Bool msg){
  if (msg.data){
    rc_enable_motors(); //TODO: write if statement to test if motors were enabled/disabled successfully
    ROS_INFO("Motors are enabled");
  } else {
    rc_disable_motors();
    ROS_INFO("Motors are enabled");
  }
}

void setMotor1_callback(const std_msgs::Float32 msg){
  float duty = msg.data / 100; //change from -100 to 100 to -1 to 1

  //check for out-of-bounds duty
  if (duty > 1) {
    duty = 1;
  } else if (duty < -1) {
    duty = -1;
  }

  rc_set_motor(1, duty);
}

void setMotor2_callback(const std_msgs::Float32 msg){
  float duty = msg.data / 100; //change from -100 to 100 to -1 to 1

  //check for out-of-bounds duty
  if (duty > 1) {
    duty = 1;
  } else if (duty < -1) {
    duty = -1;
  }

  rc_set_motor(2, duty);
}

void setMotor3_callback(const std_msgs::Float32 msg){
  float duty = msg.data / 100; //change from -100 to 100 to -1 to 1

  //check for out-of-bounds duty
  if (duty > 1) {
    duty = 1;
  } else if (duty < -1) {
    duty = -1;
  }

  rc_set_motor(3, duty);
}

void setMotor4_callback(const std_msgs::Float32 msg){
  float duty = msg.data / 100; //change from -100 to 100 to -1 to 1

  //check for out-of-bounds duty
  if (duty > 1) {
    duty = 1;
  } else if (duty < -1) {
    duty = -1;
  }

  rc_set_motor(4, duty);
}

int main(int argc, char **argv)
{
//ROS Initialization
  ros::init(argc, argv, "motor_node");
  ros::NodeHandle nh;

//BeagleBone Robotics Cape Initialization
  if (rc_get_state() == UNINITIALIZED){ //Check to see if other nodes have initialized robotics cape
      if (rc_initialize() < 0){
        ROS_INFO("Error: failed to initialize robotics cape drivers");
        return -1;
      }
        ROS_INFO("Cape Initialized");
  }

  rc_enable_motors();

  enableMotors_sub = nh.subscribe("enableMotors", 1, enableMotors_callback);
  setMotor1_sub = nh.subscribe("setMotor1", 1, setMotor1_callback);
  setMotor2_sub = nh.subscribe("setMotor2", 1, setMotor2_callback);
  setMotor3_sub = nh.subscribe("setMotor3", 1, setMotor3_callback);
  setMotor4_sub = nh.subscribe("setMotor4", 1, setMotor4_callback);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    ros::spinOnce();

    loop_rate.sleep();
  }

  rc_disable_motors();

  if (rc_cleanup()< 0) {
    ROS_INFO("Error: failed to close robotics cape drivers");
    ROS_INFO("Please run rc_kill in a terminal");
    return -1;
  }

  return 0;
}



