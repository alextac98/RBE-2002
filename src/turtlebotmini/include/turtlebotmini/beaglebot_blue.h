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

double nowTime = 0;
double lastTime = 0;

//PID constants
float kP;
float kI;
float kD;
float kF;

float leftError = 0;
float leftIntegral = 0;
double leftTime;

float rightError = 0;
float rightIntegral = 0;
double rightTime;

//Encoder Variales
const float ticksPerMeter = 6548;

std_msgs::Int16 leftEncoder;
std_msgs::Int16 rightEncoder;

std_msgs::Float32 leftEncoderVel;
std_msgs::Float32 rightEncoderVel;

int lastLeftEncoder = 0;
int lastRightEncoder = 0;

//ROS Publishers and Subscribers
ros::Subscriber lMotor_sub;
ros::Subscriber rMotor_sub;

ros::Publisher lWheelPos_pub;
ros::Publisher rWheelPos_pub;

ros::Publisher lWheelVel_pub;
ros::Publisher rWheelVel_pub;

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
