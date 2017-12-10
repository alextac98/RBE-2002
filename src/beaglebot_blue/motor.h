#ifndef MOTOR_H
#define MOTOR_H

//-----Motor Control----------------------------------
ros::Subscriber enableMotors_sub;

ros::Subscriber setMotor1_sub;
ros::Subscriber setMotor2_sub;
ros::Subscriber setMotor3_sub;
ros::Subscriber setMotor4_sub;
//ros::Subscriber setAllMotors_sub;
//ros::Subscriber freeSpinMotor_sub;
//ros::Subscriber freeSpinAllMotors_sub;

//ros::Subscriber breakMotor_sub;
//ros::Subscriber breakAllMotors_sub;

//Callback Functions
void enableMotors_callback(const std_msgs::Bool msg);

void setMotor1_callback(const std_msgs::Float32 msg);
void setMotor2_callback(const std_msgs::Float32 msg);
void setMotor3_callback(const std_msgs::Float32 msg);
void setMotor4_callback(const std_msgs::Float32 msg);

#endif
