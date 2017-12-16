#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "littleguy");
    ros::NodeHandle nh;

    MoveBaseClient ac("move_base", true);

    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server");
    }

    move_base_msgs::MoveBaseGoal goal;

    goal.target_pos.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = .667;
    goal.target_pose.pose.position.y = .333;
    goal.target_pose.pose.orientation.w = 1;

    ROS_INFO("Sending Goal");
    ac.sendGoal(goal);

    //only for demo
    ac.waitForResult();

    ROS_INFO("Hello world!");
}
