#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include <stdexcept>
#include <sstream>

#include <vector>
#include <atomic>
#include <unistd.h>

#include <experimental/filesystem>
#include <sstream>


namespace student {
bool findRobot(const cv::Mat& img_in, const double scale, Polygon& triangle, double& x, double& y, double& theta, const std::string& config_folder){
  #define FIND_ROBOT_DEBUG_PLOT 
//(x, y, theta) = 0.1937, 0.2275, 0.758836

  // Convert color space from BGR to HSV
  cv::Mat hsv_img;
  cv::cvtColor(img_in, hsv_img, 
              cv::COLOR_BGR2HSV);

  // Extract blue color region Simulation
  cv::Mat blue_mask;    
  cv::inRange(hsv_img, cv::Scalar(100, 120, 150), 
    cv::Scalar(135, 255, 255), blue_mask);

// Extract blue color region Arena
  //cv::Mat blue_mask;    
  //cv::inRange(hsv_img, cv::Scalar(90, 50, 50), 
    //cv::Scalar(140, 255, 255), blue_mask);

  // BLUE MASK on real world imgs contains noise
  // to get rid of the noise a possible solution is to use:
  // cv::erode and cv::dilate (https://docs.opencv.org/3.3.1/db/df6/tutorial_erosion_dilatation.html)
  
  // Find blue mask contours
  std::vector<std::vector<cv::Point>> contours;    
  cv::findContours(blue_mask, contours, 
                      cv::RETR_EXTERNAL, 
                      cv::CHAIN_APPROX_SIMPLE);

#ifdef FIND_ROBOT_DEBUG_PLOT // do this only if FIND_DEBUG_PLOT is defined
  cv::imshow("findRobotHsv", hsv_img);
  cv::imshow("findRobotMask", blue_mask);
  cv::Mat contours_img;
  contours_img = img_in.clone();
  cv::drawContours(contours_img, contours, -1, cv::Scalar(0,0,0), 4, cv::LINE_AA);
  std::cout << "N. contours: " << contours.size() << std::endl;
#endif

  std::vector<cv::Point> approx_curve;
  std::vector<std::vector<cv::Point>> contours_approx;    
  bool found = false;
  for (int i=0; i<contours.size(); ++i)
  { 
    // Approximate the i-th contours      
    cv::approxPolyDP(contours[i], 
             approx_curve, 30, true);

    // Check the number of edge of the aproximate contour
    if (approx_curve.size() != 3) continue;

    // If we want to chech the area of the poliygon
    double area = cv::contourArea(approx_curve);


#ifdef FIND_ROBOT_DEBUG_PLOT   // do this only if FIND_DEBUG_PLOT is defined
    std::cout << (i+1) << ") Contour size: " << contours[i].size() << std::endl;
    std::cout << (i+1) << ") Aprox Contour size: " << approx_curve.size() << std::endl;
    std::cout << "Area: " << area << std::endl;
    contours_approx = {approx_curve};

    cv::drawContours(contours_img, contours_approx, -1, cv::Scalar(0,0,255), 3, cv::LINE_AA);
#endif

    found = true;  // we found the blue triangle exit!
    break;
  }

  // If we found the robot triangle set robot position and create the triangle poligon
  if (found) 
  {      
    // emplace back every vertex on triangle (output of this function)
    for (const auto& pt: approx_curve) {        
      triangle.emplace_back(pt.x/scale, pt.y/scale);
      // remember to use the scale to convert the position on the image
      // (pixels) to the position in the arena (meters)
    }

    // Find the position of the robot
    // NB: the position of the robot coincide with the baricenter of the triangle
    double cx = 0, cy = 0;

    // Compute the triangle baricenter
    for (auto vertex: triangle) 
    {
      // NB: triangle point are expressed in meters
      cx += vertex.x; 
      cy += vertex.y;
    }
    cx /= static_cast<double>(triangle.size());
    cy /=  static_cast<double>(triangle.size());

    // Find the robot orientation (i.e the angle of height relative to the base with the x axis)
    double dst = 0;
    Point top_vertex; // 
    for (auto& vertex: triangle)
    {
      const double dx = vertex.x-cx;      
      const double dy = vertex.y-cy;
      const double curr_d = dx*dx + dy*dy;
      if (curr_d > dst)
      { 
        dst = curr_d;
        top_vertex = vertex;
      }
    }

    // Store the position of the robot in the output
    x = cx;
    y = cy;

    // Compute the robot orientation
    const double dx = cx - top_vertex.x;
    const double dy = cy - top_vertex.y;
    theta = std::atan2(dy, dx);

#ifdef FIND_ROBOT_DEBUG_PLOT   // do this only if FIND_DEBUG_PLOT is defined   
    // Draw over the imag ethe ba
    cv::Point cv_baricenter(x*scale, y*scale); // convert back m to px
    cv::Point cv_vertex(top_vertex.x*scale, top_vertex.y*scale); // convert back m to px
    cv::line(contours_img, cv_baricenter, cv_vertex, cv::Scalar(0,255,0), 3);
    cv::circle(contours_img, cv_baricenter, 5, cv::Scalar(0,0,255), -1);
    cv::circle(contours_img, cv_vertex, 5, cv::Scalar(0,255,0), -1);      
    std::cout << "(x, y, theta) = " << x << ", " << y << ", " << theta*180/M_PI << std::endl;
#endif  
  }

#ifdef FIND_ROBOT_DEBUG_PLOT   // do this only if FIND_DEBUG_PLOT is defined 
  cv::imshow("findRobot", contours_img);
  cv::waitKey(1);
#endif

  return found;    
}
}
