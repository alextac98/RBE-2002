bool find_large_contour_center_of_gravity(cv::Mat &src_img, cv::Point &result)
{
  cv::Mat gray_img, bin_img, erode_img, dilate_img;
  std::vector<std::vector<cv::Point> > contours;
  int max_area = 0;
  std::vector<cv::Point> large_contour;
  std::vector<cv::Point> convex_hull;

  // convert to grayscale
  cv::cvtColor(src_img, gray_img, CV_BGR2GRAY);

  // convert to binary image
  cv::threshold(gray_img, bin_img, 0, 255, cv::THRESH_BINARY|cv::THRESH_OTSU);

  // remove noise
  cv::erode(bin_img, erode_img, cv::Mat());
  cv::erode(erode_img, erode_img, cv::Mat());
  cv::dilate(erode_img, dilate_img, cv::Mat());
  cv::dilate(dilate_img, dilate_img, cv::Mat());

  // find contours
  cv::findContours(dilate_img, contours, CV_RETR_LIST,CV_CHAIN_APPROX_SIMPLE);
  if (contours.size() == 0) return false;

  // find max area contours
  for (unsigned int i = 0; i < contours.size(); ++i) {
    int area = (int)cv::contourArea(contours[i]);
    if (area > max_area) {
      large_contour = contours[i];
      max_area = area;
    }
  }
  if (max_area == 0) ROS_INFO("CANNOT FIND LARGE CONTOURS");

  // simplify large contours
  cv::approxPolyDP(cv::Mat(large_contour), large_contour, 5, true);

  // convex hull
  cv::convexHull(large_contour, convex_hull, false);
  if (convex_hull.size() < 3 ) return false;

  // center of gravity
  cv::Moments mo = cv::moments(convex_hull);
  result = cv::Point(mo.m10/mo.m00 , mo.m01/mo.m00);

  return true;
}
