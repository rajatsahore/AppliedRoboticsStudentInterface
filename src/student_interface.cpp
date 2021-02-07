#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include <stdexcept>
#include <sstream>

#include <vector>
#include <atomic>
#include <unistd.h>

#include <experimental/filesystem>
#include <sstream>

#include "dubins.hpp"


namespace student {

 void loadImage(cv::Mat& img_out, const std::string& config_folder){  
  static bool initialized = false;
  static std::vector<cv::String> img_list; // list of images to load
  static size_t idx = 0;  // idx of the current img
  static size_t function_call_counter = 0;  // idx of the current img
  const static size_t freeze_img_n_step = 30; // hold the current image for n iteration
  static cv::Mat current_img; // store the image for a period, avoid to load it from file every time
  
  if(!initialized){
    const bool recursive = false;
    // Load the list of jpg image contained in the config_folder/img_to_load/
    cv::glob(config_folder + "/img_to_load/*.jpg", img_list, recursive);
    
    if(img_list.size() > 0){
      initialized = true;
      idx = 0;
      current_img = cv::imread(img_list[idx]);
      function_call_counter = 0;
    }else{
      initialized = false;
    }
  }
  
  if(!initialized){
    throw std::logic_error( "Load Image can not find any jpg image in: " +  config_folder + "/img_to_load/");
    return;
  }
  
  img_out = current_img;
  function_call_counter++;  
  
  // If the function is called more than N times load increment image idx
  if(function_call_counter > freeze_img_n_step){
    function_call_counter = 0;
    idx = (idx + 1)%img_list.size();    
    current_img = cv::imread(img_list[idx]);
  }
}

 void genericImageListener(const cv::Mat& img_in, std::string topic, const std::string& config_folder){
	std::cout<<"Student Interface: genericImageListener"<<std::endl;
	static size_t id = 0;
  	static bool init = false;
  	static std::string folder_path;

  	if(!init){      
      bool exist = true;
      int i = 0;
      while(exist && i < 1000){
        std::stringstream ss;
        ss << config_folder << "/camera_image" << std::setw(3) << std::setfill('0') << i << "/";
  		  folder_path = ss.str();

        exist = std::experimental::filesystem::exists(folder_path);

        i++;        
      }
      
      if(i > 999 || !std::experimental::filesystem::create_directories(folder_path)){
        throw std::logic_error( "NO EMTY FOLDER" );
      }

  		init = true;
  	}
    	    
    cv::imshow( topic, img_in);
    char c;
    c = cv::waitKey(30);
    
    std::stringstream img_file;
    switch (c) {    	
		case 's':		
			img_file << folder_path << std::setfill('0') 
					<< std::setw(3)  << (id++) << ".jpg";
		 	cv::imwrite( img_file.str(), img_in );

		 	std::cout << "Saved image " << img_file.str() << std::endl;
		 	break;
		default:
				break;
    }
  }


 bool planPath(const Polygon& borders, const std::vector<Polygon>& obstacle_list, const std::vector<std::pair<int,Polygon>>& victim_list, const Polygon& gate, const float x, const float y, const float theta, Path& path){

 
  //Test dubins
  double x1=0.2;
  double x2=0.9;
  float x3= 1.4;

  double y1=0.2;
  double y2=0.8;
  float y3=0.2;

  double theta1= 0.0;
  double theta2= 0.0;
  float theta3= 0.0;

  double s;
  int npts=50;

  int kmax=10;
  dubinsCurve dubins={};

               //-----------INITIAL POINT TO GOAL PATH-----------------

              dubins_shortest_path(dubins, x1, y1, theta1, x2, y2, theta2, kmax);
              discretize_arc(dubins.arc_1, s, npts, path);
              discretize_arc(dubins.arc_2, s, npts, path);
              discretize_arc(dubins.arc_3, s, npts, path);


  std::cout << "inside planPath" << std::endl;
  float xc = 0, yc = 1.5, r = 1.4;
  float ds = 0.05;
  for (float theta = -M_PI/2, s = 0; theta<(-M_PI/2 + 1.2); theta+=ds/r, s+=ds) {
      path.points.emplace_back(s, xc+r*std::cos(theta), yc+r*std::sin(theta), theta+M_PI/2, 1./r);    
  }

  return true;
    //throw std::logic_error( "STUDENT FUNCTION - PLAN PATH - NOT IMPLEMENTED" );     
  }


}

