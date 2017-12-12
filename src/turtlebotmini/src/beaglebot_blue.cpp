#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"

#include "turtlebotmini/beaglebot_blue.h"

extern "C"
{
#include "rc_usefulincludes.h"
}
extern "C"
{
#include "roboticscape.h"
}


int main(int argc, char **argv)
{
//-----ROS Initialization------------------------------------------
  ros::init(argc, argv, "beaglebot_node");
  ros::NodeHandle nh;

//-----BeagleBone Robotics Cape Initialization---------------------
  if (rc_initialize() < 0){
    ROS_INFO("Error: failed to initialize robotics cape drivers");
    return -1;
  }

  rc_enable_motors();
  rc_enable_servo_power_rail();

  //ROS Publishers and Subscribers
  lMotorVel_sub = nh.subscribe("lwheel_vtarget", 1, setLeftWheel_callback);
  rMotorVel_sub = nh.subscribe("rwheel_vtarget", 1, setRightWheel_callback);

  lWheelPos_pub = nh.advertise<std_msgs::Int16>("lwheel", 1);
  rWheelPos_pub = nh.advertise<std_msgs::Int16>("rwheel", 1);

  fan_sub = nh.subscribe("fanSpeed", 1, setFan_callback);
  servo_sub = nh.subscribe("fanAngle", 1, setServo_callback);

  lastTime = ros::Time::now();

  ros::Rate loop_rate(50);

  while (ros::ok())
  {
    leftEncoder.data = rc_get_encoder_pos(LEFTENCODER);
    rightEncoder.data = rc_get_encoder_pos(RIGHTENCODER);

    lWheelPos_pub.publish(leftEncoder);
    rWheelPos_pub.publish(rightEncoder);

    currentTime = ros::Time::now();

    leftEncoderVelocity = (leftEncoder.data - lastLeftEncoder)/
            ((currentTime.toSec() - lastTime.toSec())*outputticksPerRad);
    rightEncoderVelocity = (rightEncoder.data - lastRightEncoder)/
            ((currentTime.toSec() - lastTime.toSec())*outputticksPerRad);

    lastTime = currentTime;
    lastLeftEncoder = leftEncoder.data;
    lastRightEncoder = rightEncoder.data;

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

void setFan_callback(const std_msgs::Float32 msg){
    volatile float data = msg.data;
    if (data > 1.5) {
        data = 1.5;
    } else if (data < -1.5){
        data = -1.5;
    }
    rc_send_esc_pulse_normalized(FAN, data);
}
void setServo_callback(const std_msgs::Float32 msg){
    volatile float data = msg.data;
    if (data > 1) {
        data = -0.1;
    } else if (data < -0.1){
        data = -0.1;
    }
    rc_send_servo_pulse_normalized(SERVO, data);
}

void setLeftWheel_callback(const std_msgs::Float32 msg){ //msg in m/s

    rc_set_motor(LEFTMOTOR, calculateP(leftEncoderVelocity, msg.data/wheelRadius));

}
void setRightWheel_callback(const std_msgs::Float32 msg){ //msg in m/s

    rc_set_motor(RIGHTMOTOR, calculateP(rightEncoderVelocity, msg.data/wheelRadius));

}

float calculateP(float input, float setpoint){
    return kP * (setpoint - input);
}

float calculatePID(ros::Duration timeChange, float input , float setpoint, float lastInput) {

    volatile float timeDiff = timeChange.toSec();

    volatile float error = setpoint - input;

    //Calculate P
    P = error * kP;

    //Calculate I
    I = I + (error * kI * timeDiff);

    //Calculate D
    D = (lastInput - input) * kD / timeDiff;

    return P + I + D;
}
