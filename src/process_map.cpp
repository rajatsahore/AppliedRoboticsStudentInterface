#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include <stdexcept>
#include <sstream>

#include <vector>
#include <atomic>
#include <unistd.h>

#include <experimental/filesystem>
#include <sstream>

#include "clipper/clipper.hpp"

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

#include <opencv2/opencv.hpp>


#include <iostream>
#include <fstream>
#include <algorithm>
#include <math.h>

#include <vector>
#include <atomic>
#include <unistd.h>







namespace student {
static cv::Mat bg_img;
bool sort_pair(const std::pair<int,Polygon>& a, const std::pair<int,Polygon>& b);
//static cv::Mat bg_img;
  static std::vector<cv::Point2f> result;
  static std::string name;
  static std::atomic<bool> done;
  static int n;
  static double show_scale = 1.0;



bool processObstacles(const cv::Mat& hsv_img, const double scale, std::vector<Polygon>& obstacle_list){
    
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((1*2) + 1, (1*2)+1));
    // Find red regions: h values around 0 (positive and negative angle: [0,15] U [160,179])
    cv::Mat red_mask_low, red_mask_high, red_mask; 
//exam image
    //cv::inRange(hsv_img, cv::Scalar(0, 50, 40), cv::Scalar(40, 255, 255), red_mask_low);
    //cv::inRange(hsv_img, cv::Scalar(160, 50, 40), cv::Scalar(180, 255, 255), red_mask_high);  
//simulation  
    cv::inRange(hsv_img, cv::Scalar(0, 50, 50), cv::Scalar(40, 255, 255), red_mask_low);
    cv::inRange(hsv_img, cv::Scalar(160, 50, 50), cv::Scalar(180, 255, 255), red_mask_high);
    cv::addWeighted(red_mask_low, 1.0, red_mask_high, 1.0, 0.0, red_mask); 
    
    // Filter (applying an erosion and dilation) the image
    //cv::erode(red_mask, red_mask, kernel);
    //cv::dilate(red_mask, red_mask, kernel);

    // cv::Mat img_small;
    // cv::resize(red_mask, img_small, cv::Size(640, 512));

    
    std::vector<std::vector<cv::Point>> contours, contours_approx;
    std::vector<cv::Point> approx_curve;
    cv::Mat contours_img;

    // Process red mask
    //contours_img = img_in.clone();
    cv::findContours(red_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    //drawContours(contours_img, contours, -1, cv::Scalar(40,190,40), 1, cv::LINE_AA);
    //std::cout << "N. contours: " << contours.size() << std::endl;

    for (int i=0; i<contours.size(); ++i)
    {
      //std::cout << (i+1) << ") Contour size: " << contours[i].size() << std::endl;
      approxPolyDP(contours[i], approx_curve, 3, true);

      Polygon scaled_contour;
      for (const auto& pt: approx_curve) {
        scaled_contour.emplace_back(pt.x/scale, pt.y/scale);
      }
      obstacle_list.push_back(scaled_contour);
      //contours_approx = {approx_curve};
      //drawContours(contours_img, contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);
      //std::cout << "   Approximated contour size: " << approx_curve.size() << std::endl;
    }
    //std::cout << std::endl;
    //cv::imshow("Original", contours_img);
    //cv::waitKey(1);
/*    std::vector<Polygon> inflated_obstacles;
 
    const double INT_ROUND = 1000.;
 
    for (int obs = 0; obs < obstacle_list.size(); ++obs) {
 
        ClipperLib::Path srcPoly;
        ClipperLib::Paths newPoly;
        ClipperLib::ClipperOffset co;
 
    // Iterate through obstacles
        for (int ver = 0; ver < obstacle_list[obs].size(); ++ver){
            double x = obstacle_list[obs][ver].x * INT_ROUND;
            double y = obstacle_list[obs][ver].y * INT_ROUND;
            // Add list of points to path
            srcPoly << ClipperLib::IntPoint(x,y);
        }
 
    // Provides methods to offset given path
        co.AddPath(srcPoly, ClipperLib::jtSquare, ClipperLib::etClosedPolygon);

        co.Execute(newPoly, 50);    
        for (const ClipperLib::Path &path: newPoly){
            // Obstacle obst =  data structure for current obstacle
            Polygon obst;
            for (const ClipperLib::IntPoint &pt: path){
                double x = pt.X / INT_ROUND;
                double y = pt.Y / INT_ROUND;
                // Add vertex (x,y) to current obstacle
                obst.emplace_back(x,y);
            }
            // Close and export current obstacle
            inflated_obstacles.push_back(obst);
            obstacle_list[obs] = obst;
        }
   
    }*/
 
    return true;
  }

  bool processGate(const cv::Mat& hsv_img, const double scale, Polygon& gate){
    
    // Find purple regions
    cv::Mat green_mask;
    //Exam params
    cv::inRange(hsv_img, cv::Scalar(20, 50, 62), cv::Scalar(51, 255, 162), green_mask);   


    //Simulation
    cv::inRange(hsv_img, cv::Scalar(45, 50, 45), cv::Scalar(80, 255, 255), green_mask);    
    //cv::inRange(hsv_img, cv::Scalar(130, 10, 10), cv::Scalar(165, 255, 255), purple_mask);
    
    
    std::vector<std::vector<cv::Point>> contours, contours_approx;
    std::vector<cv::Point> approx_curve;
    //cv::Mat contours_img;

    // Process purple mask
    //contours_img = hsv_img.clone();
    cv::findContours(green_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    //drawContours(contours_img, contours, -1, cv::Scalar(40,190,40), 4, cv::LINE_AA);
    // std::cout << "N. contours: " << contours.size() << std::endl;

    
    bool res = false;

    for( auto& contour : contours){
      const double area = cv::contourArea(contour);
      //std::cout << "AREA " << area << std::endl;
      //std::cout << "SIZE: " << contours.size() << std::endl;
      if (area > 500){
        approxPolyDP(contour, approx_curve, 30, true);

        if(approx_curve.size()!=4) continue;

        // contours_approx = {approx_curve};
        // drawContours(contours_img, contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);


        for (const auto& pt: approx_curve) {
          gate.emplace_back(pt.x/scale, pt.y/scale);
        }
        res = true;
        break;
      }      
    }


    // cv::imshow("Original", contours_img);
    // cv::waitKey(1);
    
    return res;
  }

cv::Mat rotate_mat(cv::Mat input_ROI, double angle_deg){
    cv::Mat out_ROI;
    cv::Point2f center(input_ROI.cols/2., input_ROI.rows/2.);  
 
    cv::Mat rot_mat = cv::getRotationMatrix2D(center, angle_deg, 1.0);
 
    cv::Rect2f bounding_box = cv::RotatedRect(cv::Point2f(), input_ROI.size(), angle_deg).boundingRect2f();
   
    rot_mat.at<double>(0,2) += bounding_box.width/2.0 - input_ROI.cols/2.0;
    rot_mat.at<double>(1,2) += bounding_box.height/2.0 - input_ROI.rows/2.0;
   
    warpAffine(input_ROI, out_ROI, rot_mat, bounding_box.size());
    return out_ROI;
  }

  bool processVictims(const cv::Mat& hsv_img, const double scale, std::vector<std::pair<int,Polygon>>& victim_list){
cv::Mat green_mask;
   
    // store a binary image in green_mask where the white pixel are those contained in HSV rage (x,x,x) --> (y,y,y)
    //cv::inRange(hsv_img, cv::Scalar(50, 80, 34), cv::Scalar(75, 255, 255), green_mask_victims); //Simulator
    //cv::inRange(hsv_img, cv::Scalar(13, 68, 41), cv::Scalar(86, 255, 80), green_mask_victims);
    //cv::inRange(hsv_img, cv::Scalar(15, 65, 40), cv::Scalar(85, 255, 95), green_mask_victims);
    // Dark w/ light
  cv::inRange(hsv_img, cv::Scalar(45, 50, 26), cv::Scalar(100, 255, 255), green_mask);

    // Apply some filtering
    // Create the kernel of the filter i.e. a rectangle with dimension 3x3
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((1*2) + 1, (1*2)+1));
    // Dilate using the generated kernel
    cv::dilate(green_mask, green_mask, kernel);
    // Erode using the generated kernel
    cv::erode(green_mask,  green_mask, kernel);

    //cv::imshow("filterrrr", green_mask);
    //cv::waitKey(0);
 
    // Find green contours
    std::vector<std::vector<cv::Point>> contours, contours_approx;    
    // Create an image which we can modify not changing the original image
    cv::Mat contours_img;
    contours_img = hsv_img.clone();
 
    // Finds green contours in a binary (new) image
    cv::findContours(green_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    // create an array of rectangle (i.e. bounding box containing the green area contour)  
    std::vector<cv::Rect> bound_rect(contours.size());
    int victim_id = 0;

    for (int i=0; i<contours.size(); ++i){
      double area = cv::contourArea(contours[i]);
      if (area < 500) continue; // filter too small contours to remove false positives

      std::vector<cv::Point> approx_curve;
      approxPolyDP(contours[i], approx_curve, 10, true);
      if(approx_curve.size() < 6) continue; //fitler out the gate
     
      Polygon scaled_contour;
      for (const auto& pt: approx_curve) {
        scaled_contour.emplace_back(pt.x/scale, pt.y/scale);
      }
      // Add victims to the victim_list
      victim_list.push_back({victim_id++, scaled_contour});
 
      contours_approx = {approx_curve};
      // Draw the contours on image with a line color of BGR=(0,170,220) and a width of 3
      drawContours(contours_img, contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);
 
      // find the bounding box of the green blob approx curve
      bound_rect[i] = boundingRect(cv::Mat(approx_curve));

    }
    std::cout << " End process, begin digit recognition"  << std::endl;
    


    //-------------------------Digit Recognition----------------------------------------
    cv::Mat green_mask_inv;
    cv::Mat filtered(hsv_img.rows, hsv_img.cols, CV_8U, cv::Scalar(255,255,255));
 
    // generate binary mask with inverted pixels w.r.t. green mask -> black numbers are part of this mask
    cv::bitwise_not(green_mask, green_mask_inv);
    //std::cout << "   3 "  << std::endl;
    // Load digits template images
    std::vector<cv::Mat> templROIs;
    for (int i=1; i<=5; ++i) {
      auto num_template = cv::imread("template/" + std::to_string(i) + ".png");
      // flip the template to get the shape of number in the unwarped ground image
      cv::flip(num_template, num_template, 1);
      // Store the template in templROIs (vector of mat)
      templROIs.emplace_back(num_template);
    }  
 
    // create copy of inverted pixel image
    hsv_img.copyTo(filtered, green_mask_inv);
 
    // create a kernel (3x3) for img filtering
    kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((2*2) + 1, (2*2)+1));
    //std::cout << "   4 "  << std::endl;
    // For each green blob in the original image containing a digit
    int victim_count = -1;
    for (int i=0; i<bound_rect.size(); ++i){
      cv::Mat processROI(filtered, bound_rect[i]); // extract the ROI containing the digit
 
      if (processROI.empty()) continue;
      victim_count = victim_count+1;
      std::cout << "victim_count: " << victim_count << std::endl;  
      
      //Resize the processROI as the size of the number in the template image should be similar to the dimension
      cv::resize(processROI, processROI, cv::Size(200, 200)); // resize the ROI
      cv::threshold( processROI, processROI, 100, 255, 0 );   // threshold and binarize the image, to suppress some noise
   
      // Apply some additional smoothing and filtering
      cv::erode(processROI, processROI, kernel);
      cv::GaussianBlur(processROI, processROI, cv::Size(5, 5), 2, 2);
      cv::erode(processROI, processROI, kernel);
      //std::cout << "   4.1 "  << std::endl;
      // Find the template digit with the best matching
      double maxScore = 0;
      int maxIdx = -1;
      cv::Mat rot_processROI(filtered, bound_rect[i]);
      for(int k=0;k<36;++k){
        //Rotate processROI
        rot_processROI = rotate_mat(processROI, 10*k);
       
        for (int j=0; j<templROIs.size(); ++j) {
          cv::Mat result;
 
          // Matching the ROI with the templROIs j-th
          cv::matchTemplate(rot_processROI, templROIs[j], result, cv::TM_CCOEFF);
          double score;
          cv::minMaxLoc(result, nullptr, &score);
 
          // Comparing the score with the others, if it is higher save this as the best match!
          if (score > maxScore) {
            maxScore = score;
            maxIdx = j;
 
            //cv::imshow("ROI", rot_processROI);
          }
        }
      }
      //std::cout << "   5 "  << std::endl;
      victim_list.at(victim_count).first = maxIdx + 1;
      // Display the best fitting number
      std::cout << "Recognized Digit: " << maxIdx + 1 << std::endl;
      //cv::waitKey(0);
    }
 
    sort(victim_list.begin(), victim_list.end(), sort_pair);
    //cv::imshow("Original", contours_img);
    //cv::waitKey(0);
 
    //std::cout << "\n\n - - - @@@ end Digit recognition - - - \n\n\n";   
    return true;
}
bool sort_pair(const std::pair<int,Polygon>& a, const std::pair<int,Polygon>& b){
    return (a.first < b.first);
}


  bool processMap(const cv::Mat& img_in, const double scale, std::vector<Polygon>& obstacle_list, std::vector<std::pair<int,Polygon>>& victim_list, Polygon& gate, const std::string& config_folder){

    // Convert color space from BGR to HSV
    cv::Mat hsv_img;
    cv::cvtColor(img_in, hsv_img, cv::COLOR_BGR2HSV);

    
    const bool res1 = processObstacles(hsv_img, scale, obstacle_list);
    if(!res1) std::cout << "processObstacles return false" << std::endl;
    const bool res2 = processGate(hsv_img, scale, gate);
    if(!res2) std::cout << "processGate return false" << std::endl;
    const bool res3 = processVictims(hsv_img, scale, victim_list);
    if(!res3) std::cout << "processVictims return false" << std::endl;

    return res1 && res2 && res3;
  }

}
