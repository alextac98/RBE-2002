#ifndef BEAGLEBOT_BLUE_H
#define BEAGLEBOT_BLUE_H

#define LEFTWHEEL 0
#define RIGHTWHEEL 1

#define LEFTENCODER 4
#define RIGHTENCODER 1

#define LEFTMOTOR 4
#define RIGHTMOTOR 1

#define FAN 1
#define SERVO 2

//Robot Constants
const float wheelDiameter = 0.07; //in meters
const float wheelRadius = wheelDiameter/2;

//Encoder Variales
const int gearRatio = 120;
const float ticksPerRad = 1.90986093;
const float outputticksPerRad = gearRatio * ticksPerRad;

std_msgs::Int16 leftEncoder;      //in rad/s
std_msgs::Int16 rightEncoder;     //in rad/s

//ROS Publishers and Subscribers
ros::Subscriber lMotor_sub;
ros::Subscriber rMotor_sub;

ros::Publisher lWheelPos_pub;
ros::Publisher rWheelPos_pub;

ros::Subscriber fan_sub;
ros::Subscriber servo_sub;

//Motor callbacks
void setLeftWheel_callback(const std_msgs::Float32 msg);
void setRightWheel_callback(const std_msgs::Float32 msg);

//Fan Callback
void setFan_callback(const std_msgs::Float32 msg);

//Servo Callback
void setServo_callback(const std_msgs::Float32 msg);

#endif // BEAGLEBOT_BLUE_H
