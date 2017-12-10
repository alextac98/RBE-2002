#ifndef QUAD_ENCODERS_H
#define QUAD_ENCODERS_H

ros::Publisher getEncoder1;
ros::Publisher getEncoder2;
ros::Publisher getEncoder3;
ros::Publisher getEncoder4;

ros::Subscriber setEncoder1Pos;
ros::Subscriber setEncoder1Pos;
ros::Subscriber setEncoder1Pos;
ros::Subscriber setEncoder1Pos;

void setEnc1Pos_callback(const std_msgs::Float32 msg);
void setEnc2Pos_callback(const std_msgs::Float32 msg);
void setEnc3Pos_callback(const std_msgs::Float32 msg);
void setEnc4Pos_callback(const std_msgs::Float32 msg);

std_msgs::Float32 encoder1;
std_msgs::Float32 encoder2;
std_msgs::Float32 encoder3;
std_msgs::Float32 encoder4;


#endif // QUAD_ENCODERS_H
