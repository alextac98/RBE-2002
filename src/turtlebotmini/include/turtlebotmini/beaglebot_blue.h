#ifndef BEAGLEBOT_BLUE_H
#define BEAGLEBOT_BLUE_H

#define LEFTWHEEL 0
#define RIGHTWHEEL 1

#define LEFTENCODER 4
#define RIGHTENCODER 1

#define LEFTMOTOR 1
#define RIGHTMOTOR 4

#define FAN 1
#define SERVO 2

//Robot Constants
const float wheelDiameter = 0.075; //in meters
const float wheelRadius = wheelDiameter/2;

//PID Variables
float kP = 0.025;
float kI = 0;
float kD = 0;

float P = 1;
float I = 0;
float D = 0;

//Encoder Variales
const int gearRatio = 120;
const float ticksPerRad = 1.90986093;
const float outputticksPerRad = gearRatio * ticksPerRad;

std_msgs::Int16 leftEncoder;      //in rad/s
std_msgs::Int16 rightEncoder;     //in rad/s

float leftEncoderVelocity = 0;
float rightEncoderVelocity = 0;

float lastLeftEncoder = 0;
float lastRightEncoder = 0;

float lastLeftInputSpeed;
float lastRightInputSpeed;

ros::Time lastTime;
ros::Time currentTime;

//ROS Publishers and Subscribers
ros::Subscriber lMotorVel_sub;
ros::Subscriber rMotorVel_sub;

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

float calculateP(float input, float setpoint);
float calculatePID(ros::Duration timeChange, float input , float setpoint, float *lastInput, float *lastInputSpeed);
#endif // BEAGLEBOT_BLUE_H
