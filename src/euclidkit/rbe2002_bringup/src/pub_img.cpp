#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include<dirent.h>
#include<string.h>

using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pub_img");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<sensor_msgs::Image>("/camera/image_raw", 1);
  ros::Rate loop_rate(5);

  while (nh.ok())
  {
//    string dirName = "/home/toma/Pictures/";
//    DIR *dir;
//    dir = opendir(dirName.c_str());
//    string imgName;
//    struct dirent *ent;

//    if (dir != NULL)
//    {
//      while ((ent = readdir (dir)) != NULL) {
//          imgName= ent->d_name;
//          //I found some . and .. files here so I reject them.
//          if(imgName.compare(".")!= 0 && imgName.compare("..")!= 0)
//          {
//            string aux;
//            aux.append(dirName);
//            aux.append(imgName);
//            //ROS_INFO("Read File: %s", aux);
//            //cout << aux << endl;

//            cv_bridge::CvImage cv_image;
//            cv_image.image = cv::imread(aux,CV_LOAD_IMAGE_COLOR);
//            cv_image.encoding = "bgr8";
//            sensor_msgs::Image ros_image;
//            cv_image.toImageMsg(ros_image);
//            pub.publish(ros_image);
//            loop_rate.sleep();

//            //Mat image= imread(aux);
//            //imshow(aux,image);
//          }
//     }
//     closedir (dir);
//   }
//    else {
//     cout<<"not present"<<endl;
//    }

    cv_bridge::CvImage cv_image;
    cv_image.image = cv::imread("/home/toma/Pictures/20171212_230625.jpg",CV_LOAD_IMAGE_COLOR);
    cv_image.encoding = "bgr8";
    sensor_msgs::Image ros_image;
    cv_image.toImageMsg(ros_image);
    pub.publish(ros_image);

    loop_rate.sleep();
  }
}
