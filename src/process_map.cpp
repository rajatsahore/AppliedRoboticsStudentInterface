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

bool processObstacles(const cv::Mat& hsv_img, const double scale, std::vector<Polygon>& obstacle_list){
    
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((1*2) + 1, (1*2)+1));
    // Find red regions: h values around 0 (positive and negative angle: [0,15] U [160,179])
    cv::Mat red_mask_low, red_mask_high, red_mask;     
    cv::inRange(hsv_img, cv::Scalar(0, 50, 50), cv::Scalar(10, 255, 255), red_mask_low);
    cv::inRange(hsv_img, cv::Scalar(160, 50, 50), cv::Scalar(179, 255, 255), red_mask_high);
    cv::addWeighted(red_mask_low, 1.0, red_mask_high, 1.0, 0.0, red_mask); 
    
    // Filter (applying an erosion and dilation) the image
    cv::erode(red_mask, red_mask, kernel);
    cv::dilate(red_mask, red_mask, kernel);

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

    return true;
  }

  bool processGate(const cv::Mat& hsv_img, const double scale, Polygon& gate){
    
    // Find purple regions
    cv::Mat green_mask;
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

  bool processVictims(const cv::Mat& hsv_img, const double scale, std::vector<std::pair<int,Polygon>>& victim_list){
const double MIN_AREA_SIZE = 100;
    
    // Find green regions
    // Display original image
  cv::imshow("Original", hsv_img);
  
  // Convert color space from BGR to HSV
  //cv::Mat hsv_img;
  //cv::cvtColor(img, hsv_img, cv::COLOR_BGR2HSV);
  
  
  // Find green regions
  cv::Mat green_mask;
  cv::inRange(hsv_img, cv::Scalar(45, 40, 40), cv::Scalar(75, 255, 255), green_mask);
  
  // Apply some filtering
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((2*2) + 1, (2*2)+1));
  cv::dilate(green_mask, green_mask, kernel);
  cv::erode(green_mask, green_mask, kernel);
  
  // Display image
  cv::imshow("GREEN_filter", green_mask);
  
  
  
  // Find contours
  std::vector<std::vector<cv::Point>> contours, contours_approx;
  std::vector<cv::Point> approx_curve;
  cv::Mat contours_img;

  contours_img = hsv_img.clone();
  cv::findContours(green_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);  
    
  std::vector<cv::Rect> boundRect(contours.size());
  for (int i=0; i<contours.size(); ++i)
  {
    double area = cv::contourArea(contours[i]);
    //if (area < MIN_AREA_SIZE) continue; // filter too small contours to remove false positives
    approxPolyDP(contours[i], approx_curve, 2, true);
    contours_approx = {approx_curve};
    drawContours(contours_img, contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);
    boundRect[i] = boundingRect(cv::Mat(approx_curve)); // find bounding box for each green blob
  }
  cv::imshow("Original", contours_img);
  cv::waitKey(0);
     
  
  cv::Mat green_mask_inv, filtered(hsv_img.rows, hsv_img.cols, CV_8UC3, cv::Scalar(255,255,255));
  cv::bitwise_not(green_mask, green_mask_inv); // generate binary mask with inverted pixels w.r.t. green mask -> black numbers are part of this mask
  
  cv::imshow("Numbers", green_mask_inv);
  cv::waitKey(0);
  
  // Load digits template images
  std::vector<cv::Mat> templROIs;
  for (int i=0; i<=10; ++i) {
    templROIs.emplace_back(cv::imread("../imgs/template/" + std::to_string(i) + ".png"));
  }  
  
  hsv_img.copyTo(filtered, green_mask_inv);   // create copy of image without green shapes
  
  kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((2*2) + 1, (2*2)+1));
  std::cout << " 1 2 3  " << std::endl;
  
  // For each green blob in the original image containing a digit
  for (int i=0; i<boundRect.size(); ++i)
  {
    cv::Mat processROI(filtered, boundRect[i]); // extract the ROI containing the digit
    
    if (processROI.empty()) continue;
    
    //cv::resize(processROI, processROI, cv::Size(200, 200)); // resize the ROI
    //cv::threshold( processROI, processROI, 100, 255, 0 ); // threshold and binarize the image, to suppress some noise
    
    // Apply some additional smoothing and filtering
    //cv::erode(processROI, processROI, kernel);
    //cv::GaussianBlur(processROI, processROI, cv::Size(3, 3), 2, 2);
    //cv::erode(processROI, processROI, kernel);
    
    // Show the actual image used for the template matching
    cv::imshow("ROI", processROI);
    
    // Find the template digit with the best matching
    double maxScore = 0;
    int maxIdx = -1;
    for (int j=0; j<templROIs.size(); ++j) {
      cv::Mat result;
      cv::matchTemplate(processROI, templROIs[j], result, cv::TM_CCOEFF);
      double score;
      cv::minMaxLoc(result, nullptr, &score); 
      if (score > maxScore) {
        maxScore = score;
        maxIdx = j;
      }
    }
    
    std::cout << "Best fitting template: " << maxIdx << std::endl;
    
    cv::waitKey(0);
    
    return true;
  }

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
