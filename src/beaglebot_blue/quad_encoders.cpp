#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "beaglebot_msg/setMotor_msg.h"

#include "beaglebot_blue/quad_encoders.h"


extern "C"
{
#include "rc_usefulincludes.h"
}
extern "C"
{
#include "roboticscape.h"
}

void setEnc1Pos_callback(const std_msgs::Float32 msg){
    rc_set_encoder_pos(1, (int) msg.data);
}
void setEnc2Pos_callback(const std_msgs::Float32 msg){
    rc_set_encoder_pos(2, (int) msg.data);
}
void setEnc3Pos_callback(const std_msgs::Float32 msg){
    rc_set_encoder_pos(3, (int) msg.data);
}
void setEnc4Pos_callback(const std_msgs::Float32 msg){
    rc_set_encoder_pos(4, (int) msg.data);
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

  getEncoder1 = nh.advertise<std_msgs::Float32>("getEncoder1");
  getEncoder2 = nh.advertise<std_msgs::Float32>("getEncoder2");
  getEncoder3 = nh.advertise<std_msgs::Float32>("getEncoder3");
  getEncoder4 = nh.advertise<std_msgs::Float32>("getEncoder4");

  getEncoder1Pulses = nh.advertise<std_msgs::Float32>("getEncoder1Pulses");
  getEncoder2Pulses = nh.advertise<std_msgs::Float32>("getEncoder2Pulses");
  getEncoder3Pulses = nh.advertise<std_msgs::Float32>("getEncoder3Pulses");
  getEncoder4Pulses = nh.advertise<std_msgs::Float32>("getEncoder4Pulses");

  setEncoder1Pos = nh.subscribe("setEncoder1Position", 1, setEnc1Pos_callback);
  setEncoder2Pos = nh.subscribe("setEncoder1Position", 1, setEnc2Pos_callback);
  setEncoder3Pos = nh.subscribe("setEncoder1Position", 1, setEnc3Pos_callback);
  setEncoder4Pos = nh.subscribe("setEncoder1Position", 1, setEnc4Pos_callback);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    rc_get_encoder_pos(1) = encoder1Pulses.data;
    rc_get_encoder_pos(2) = encoder2Pulses.data;
    rc_get_encoder_pos(3) = encoder3Pulses.data;
    rc_get_encoder_pos(4) = encoder4Pulses.data;

    ros::Duration diff = ros::Time::now() - lastTime;
    volatile float timeDiff = diff.toSec();

    encoder1.data = (encoder1Pulses.data - lastEncoder1)/timeDiff;
    encoder2.data = (encoder2Pulses.data - lastEncoder2)/timeDiff;
    encoder3.data = (encoder3Pulses.data - lastEncoder3)/timeDiff;
    encoder4.data = (encoder4Pulses.data - lastEncoder4)/timeDiff;

    getEncoder1.publish(encoder1);
    getEncoder2.publish(encoder2);
    getEncoder3.publish(encoder3);
    getEncoder4.publish(encoder4);

    getEncoder1Pulses.publish(encoder1Pulses);
    getEncoder2Pulses.publish(encoder2Pulses);
    getEncoder3Pulses.publish(encoder3Pulses);
    getEncoder4Pulses.publish(encoder4Pulses);

    lastEncoder1 = encoder1Pulses.data;
    lastEncoder2 = encoder2Pulses.data;
    lastEncoder3 = encoder3Pulses.data;
    lastEncoder4 = encoder4Pulses.data;

    ros::spinOnce();

    loop_rate.sleep();
  }

  if (rc_cleanup()< 0) {
    ROS_INFO("Error: failed to close robotics cape drivers");
    ROS_INFO("Please run rc_kill in a terminal");
    return -1;
  }

  return 0;
}
