#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "actionlib/client/simple_action_client.h"
#include "std_msgs/Float32.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "littleguy");
    ros::NodeHandle nh;

    ros::Publisher fan = nh.advertise<std_msgs::Float32>("fanSpeed", 1);

//    MoveBaseClient ac("move_base", true);

//    while(!ac.waitForServer(ros::Duration(5.0))){
//       ROS_INFO("Waiting for the move_base action server to come up");
//     }

//    move_base_msgs::MoveBaseGoal goal;

//    goal.target_pose.header.frame_id = "base_link";
//    goal.target_pose.header.stamp = ros::Time::now();

//    goal.target_pose.pose.position.x = .667;
//    goal.target_pose.pose.position.y = .333;
//    goal.target_pose.pose.orientation.w = 1;

//    ROS_INFO("Sending Goal");
//    ac.sendGoal(goal);

//    //only for demo
//    ac.waitForResult();

//    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        std_msgs::Float32 fanPower;
        fanPower.data = 0.5;
        fan.publish(fanPower);

        ROS_INFO("Fire Position X:56.8376inches, Y:-28.997inches, Z:8.254inches");
//    }

//    goal.target_pose.pose.position.x = 0;
//    goal.target_pose.pose.position.y = 0;

//    ac.sendGoal(goal);

//    ac.waitForResult();

//    ROS_INFO("Im Home!");
}
