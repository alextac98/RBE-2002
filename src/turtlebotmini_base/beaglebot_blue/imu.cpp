#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include //imu message
#include "beaglebot_blue/imu.h"


extern "C"
{
#include "rc_usefulincludes.h"
}
extern "C"
{
#include "roboticscape.h"
}

void imuInterrupt(){

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

  if (rc_initialize_imu_dmp(&imuData, imuConfig)){
      ROS_INFO("Error: cannot talk to IMU. All hope is lost... \n");
      rc_blink_led(RED, 5, 5);
      return -1;
  }

  rc_set_imu_interrupt_func(&imuInterrupt);



  ros::Rate loop_rate(10);

  while (ros::ok())
  {



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
