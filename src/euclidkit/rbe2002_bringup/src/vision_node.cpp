#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "GripPipeline.h"

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  grip::GripPipeline grip;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image_raw", 1, &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    grip.Process(cv_ptr->image);  // *************************

    //cv::Mat * img2_ptr = grip.GetMaskOutput();
    //cv::Mat img2;
    //img2 = *img2_ptr;

    //if (img2.rows > 60 && img2.cols > 60)
    //   cv::circle(img2, cv::Point(50, 50), 10, CV_RGB(255,0,0));
    //cv::imshow(OPENCV_WINDOW, img2);
    //image_pub_.publish(cv_ptr->toImageMsg());

    cv_ptr->image = *grip.GetMaskOutput();

    // Calculate contours ********************
    cv::Point result;
    std::vector<std::vector<cv::Point> > contours;
    int max_area = 0;
    std::vector<cv::Point> large_contour;
    std::vector<cv::Point> convex_hull;
    contours = *grip.GetFindContoursOutput();
    for (unsigned int i = 0; i < contours.size(); ++i) {
      int area = (int)cv::contourArea(contours[i]);
      if (area > max_area) {
        large_contour = contours[i];
        max_area = area;
      }
    }
    if (max_area == 0) ROS_INFO("CANNOT FIND CONTOURS");  // NEED ERROR HANDLING

    // simplify large contours
    cv::approxPolyDP(cv::Mat(large_contour), large_contour, 5, true);

    // convex hull
    cv::convexHull(large_contour, convex_hull, false);
    if (convex_hull.size() < 3 ) ROS_INFO("CANNOT FIND HULLS");  // NEED ERROR HANDLING

    // center of gravity
    cv::Moments mo = cv::moments(convex_hull);
    result = cv::Point(mo.m10/mo.m00 , mo.m01/mo.m00);

    cv::circle(cv_ptr->image, result, 10, CV_RGB(255,0,0));

    // Draw an example circle on the video stream
    //if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    //   cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Update GUI Window
     cv::imshow(OPENCV_WINDOW, cv_ptr->image);
     cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());

  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
