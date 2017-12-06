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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "driveMotors");
    ros::NodeHandle nh;

    if (rc_get_state() == UNINITIALIZED){ //Check to see if other nodes have initialized robotics cape
        if (rc_initialize() < 0){
          ROS_INFO("Error: failed to initialize robotics cape drivers");
          return -1;
        }
          ROS_INFO("Cape Initialized");
    }

    rc_enable_motors();

    enableMotors_sub = nh.subscribe("enableMotors", 1, enableMotors_callback);

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
}

void setMotor(int motor, float duty){   //duty from -1 to 1
  rc_set_motor(motor, duty);
}

int getEncoderTicks(int encoder){
  rc_get_encoder_pos(encoder);
}
