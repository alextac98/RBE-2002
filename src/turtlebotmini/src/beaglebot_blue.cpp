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

  rc_set_encoder_pos(1, 0);
  rc_set_encoder_pos(4, 0);

//-----Get PID Parameters------------------------------------------
//nh.param<datatype>("name", [variabletobeset], defaultvalue);
  double _kp;  double _ki;
  double _kd;  double _kf;

  nh.param<double>("kP", _kp);
  nh.param<double>("kI", _ki);
  nh.param<double>("kD", _kd);
  nh.param<double>("kF", _kf, 1.267);

  kP = _kp;  kI = _ki;
  kD = _kd;  kF = _kf;

//-----ROS Publishers and Subscribers------------------------------
  lMotor_sub = nh.subscribe("lmotor_cmd", 1, setLeftWheel_callback);
  rMotor_sub = nh.subscribe("rmotor_cmd", 1, setRightWheel_callback);

  lWheelPos_pub = nh.advertise<std_msgs::Int16>("lwheel", 1);
  rWheelPos_pub = nh.advertise<std_msgs::Int16>("rwheel", 1);

  lWheelVel_pub = nh.advertise<std_msgs::Float32>("lwheel_vel", 1);
  rWheelVel_pub = nh.advertise<std_msgs::Float32>("rwheel_vel", 1);

  fan_sub = nh.subscribe("fanSpeed", 1, setFan_callback);
  servo_sub = nh.subscribe("fanAngle", 1, setServo_callback);

  ros::Rate loop_rate(50);

  lastTime = ros::Time::now().toSec();

  while (ros::ok())
  {
    leftEncoder.data = rc_get_encoder_pos(LEFTENCODER);
    rightEncoder.data = -rc_get_encoder_pos(RIGHTENCODER);

    nowTime = ros::Time::now().toSec();

    lWheelPos_pub.publish(leftEncoder);
    rWheelPos_pub.publish(rightEncoder);

    leftEncoderVel.data = -(lastLeftEncoder - leftEncoder.data)/
             ((nowTime - lastTime) * ticksPerMeter);

    rightEncoderVel.data = -(lastRightEncoder - rightEncoder.data)/
            ((nowTime - lastTime) * ticksPerMeter);

    //Super janky way of taking care of integer value wrap around
    //"If you know the rules, you are allowed to break them" :-D
    if (leftEncoderVel.data < 1 && rightEncoderVel.data < 1){
        lWheelVel_pub.publish(leftEncoderVel);
        rWheelVel_pub.publish(rightEncoderVel);
    }

    lastTime = nowTime;
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

    rc_send_esc_pulse_normalized(FAN, msg.data);
}
void setServo_callback(const std_msgs::Float32 msg){

    rc_send_servo_pulse_normalized(SERVO, msg.data);
}

void setLeftWheel_callback(const std_msgs::Float32 msg){ //msg in m/s

    double timeNow = ros::Time::now().toSec();

    //Calculate P
    float error = msg.data - leftEncoderVel.data;

    //Calculate I
    float integral = leftIntegral + leftError*(timeNow - leftTime);

    //Calculate D
    float derivative = (error - leftError)/(timeNow - leftTime);

    //Calculate PID Output
    float output = kP * error + kI * integral + kD * derivative + kF * msg.data;

    rc_set_motor(LEFTMOTOR, -output);

    leftTime = timeNow;
    leftError = error;
    leftIntegral = integral;

}
void setRightWheel_callback(const std_msgs::Float32 msg){ //msg in m/s

    double timeNow = ros::Time::now().toSec();

    //Calculate P
    float error = msg.data - rightEncoderVel.data;

    //Calculate I
    float integral = rightIntegral + rightError*(timeNow - rightTime);

    //Calculate D
    float derivative = (error - rightError)/(timeNow - rightTime);

    //Calculate PID output
    float output = kP * error + kI * integral + kD * derivative + kF * msg.data;

    rc_set_motor(RIGHTMOTOR, -output);

    rightTime = timeNow;
    rightError = error;
    rightIntegral = integral;

}

