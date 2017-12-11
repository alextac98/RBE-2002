#include "turtlebotmini_base/robot_hardware.h"
#include "controller_manager/controler_manager.h"

#include <dynamic_reconfigure/server.h>
#include <dynamic_tutorials/turtlebotminiConfig.h>

TurtleBotMini_Hardware robot;

void callback(dynamic_tutorials::TutorialsConfig &config, uint32_t level) {
  robot.changekP(config.kP);
  robot.changekI(config.kI);
  robot.changekD(config.kD);
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "turtlebotmini_base");
    ros::NodeHandle nh;

    dynamic_reconfigure::Server<turtlebotmini_base::turtlebotminiConfig> robot_base;
    dynamic_reconfigure::Server<turtlebotmini_base::turtlebotminiConfig>::CallbackType f;

    f = boos::bind(&callback, _1, _2);
    robot_base.setCallback(f);

    controller_manager::ControllerManager cm(&robot);
    ros::Time lastTime = ros::Time::now();
    ros::Duration timeDiff;
    ros::Time now;

    ros::Rate loop_rate(10);

    while (ros::ok())
      {
        now = ros::Time::now();
        timeDiff = now - lastTime;
        robot.updateJoints(timeDiff);
        cm.update(now, timeDiff);
        robot.writeToHardware(timeDiff);

        ros::spinOnce();
       }
}

//This is where a start/stop function could go
