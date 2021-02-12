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
#include "ompl_planning.hpp"
#include "planPath.hpp"
#include "utils.hpp"
#include "clipper/clipper.hpp"


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




double gate_angle(Polygon borders,double gateX,double gateY){

        double gateAngle;    
        if (fabs(gateX - borders[0].x) < fabs(gateX - borders[1].x)){// Left
            if (fabs(gateY - borders[0].y) < fabs(gateY - borders[3].y)){ // Bottom-left        
                if (fabs(gateY - borders[0].y) < fabs(gateX - borders[0].x)){// Horizontal orientaton
                    gateAngle = -M_PI/2;
                } else {// Vertical Oreintation
                    gateAngle = M_PI;
                }
            } else {// Top-left
                if (fabs(gateY - borders[3].y) < fabs(gateX - borders[0].x)){// Horizontal Oreintation
                    gateAngle = M_PI/2;
                } else {// Vertical orientation
                    gateAngle = M_PI;
                }
            }
        } else {// Right
            if (fabs(gateY - borders[0].y) < fabs(gateY - borders[3].y)){// Bottom-right
                if (fabs(gateY - borders[0].y) < fabs(gateX - borders[1].x)){// Horizontal
                    gateAngle = -M_PI/2;
                } else {// Vertical
                    gateAngle = 0;
                }
            } else {// Top-right
                if (fabs(gateY - borders[3].y) < fabs(gateX - borders[1].x)){// Horizontal
                    gateAngle = M_PI/2;
                } else {// Vertical
                    gateAngle = 0;
                }
            }    
        }
        return gateAngle;                
    }

double internal_angle(double angle1, double angle2){
        if(angle1-angle2 >=0 && angle1-angle2 < M_PI)
            return angle1-angle2;
        else if(angle1-angle2 <0 && angle2-angle1 < M_PI)
            return angle2-angle1;
        else if(angle1-angle2 >=0 && angle1-angle2 > M_PI)
            return 2*M_PI-(angle1-angle2);
        else if(angle1-angle2 <0 && angle2-angle1 > M_PI)
            return 2*M_PI-(angle2-angle1);
        return angle2;
    }

std::pair<double, double> get_center(const Polygon &poly) {
        double cx = 0;
        double cy = 0;
        for (int pt = 0; pt < poly.size(); pt++){
            cx += poly[pt].x;
            cy += poly[pt].y;
        }
        cx /= poly.size();
        cy /= poly.size();
        return std::make_pair(cx, cy);
    }


double angle_dubins(Pose first, Pose second, Pose third) {
        double temp = 0;
        double distance1 = sqrt(pow(second.x - first.x, 2) + pow(second.y - first.y, 2)); // distance difference
        double distance2 = sqrt(pow(third.x - second.x, 2) + pow(third.y - second.y, 2));
        double angle1 = atan2((first.y - second.y), (first.x - second.x));// angle difference
        double angle2 = atan2((third.y - second.y), (third.x - second.x));
        angle1<0? angle1 = 2*M_PI+angle1: angle1+=0; 
        angle2<0? angle2 = 2*M_PI+angle2: angle2+=0;
        angle1 < angle2 ? temp = std::fmod((((angle1+angle2)/2) + M_PI/2), 2*M_PI): temp = std::fmod((((angle1+angle2)/2) - M_PI/2), 2*M_PI);;
        temp<0?temp = 2*M_PI+temp: temp+=0; 

        if(internal_angle(angle1, angle2) < 0.20){ // If the line segments doesnt have big difference, then they are on almost same path and clothoids/dubins can handle easily
	        return temp;
        }
        double final_angle = temp;
        double distanceFactor = (distance1 > distance2) ? 1 - distance2/distance1 : 1 - distance1/distance2;// calculate the distance factor for angle difference in first and second segment 
        double a_minus = std::fmod(temp - (internal_angle(angle2,temp) * distanceFactor), 2*M_PI);// angle which is differnce of existing angle difference and angle difference between second segment and new slope
        double a_plus = std::fmod(temp + (internal_angle(angle2,temp) * distanceFactor), 2*M_PI);
// angle which is sum of existing angle difference and angle difference between second segment and new slope
        double angle_a1_minus = internal_angle(angle2,a_minus);
        double angle_a1_plus = internal_angle(angle2,a_plus);
        if(distance1 > distance2){ 
            if(angle_a1_plus > angle_a1_minus){
                final_angle = a_minus;
            }else{
                final_angle = a_plus;
            }
        }else if (distance2 > distance1){
            if(angle_a1_plus > angle_a1_minus){
                final_angle = a_plus;
            }else{
                final_angle = a_minus;
            }
        }

        final_angle = std::fmod(final_angle, 2*M_PI);
        return final_angle;
    }

std::vector<Polygon> obstacleinflationg(const std::vector<Polygon> ob,int offset_radius){
        std::vector<Polygon> inflatedObstacles;
        for(int i = 0; i < ob.size(); i++){
            ClipperLib::Path srcPoly;
            ClipperLib::Paths newPoly; 
            ClipperLib::ClipperOffset co;
            Polygon temp;
            const double INT_ROUND = 1000; // Scaling constant 
            // Put all points of obstacle obstacleusing clipper
            for(size_t a = 0; a < ob[i].size(); ++a){
                double x = ob[i][a].x * INT_ROUND;
                double y = ob[i][a].y * INT_ROUND;
                srcPoly << ClipperLib::IntPoint(x, y);
            }
            // If not a closed polygon
            if(ob[i].size() == 3)
            {
                co.AddPath(srcPoly, ClipperLib::jtSquare, ClipperLib::etClosedLine);
            }
            else // If it is a closed polygon
            {
                co.AddPath(srcPoly, ClipperLib::jtSquare, ClipperLib::etClosedPolygon);
            }

            //Execute Clipper offset and return the new set of enlarged obstacles
            co.Execute(newPoly, offset_radius);
            for(const ClipperLib::Path &path: newPoly){
                for(const ClipperLib::IntPoint &pt: path){
                    double x = (double)pt.X / INT_ROUND;
                    double y = (double)pt.Y / INT_ROUND;
                    temp.emplace_back(x, y);
                }
            }

            inflatedObstacles.emplace_back(temp);
        }
        return inflatedObstacles;
}



 bool mission_2(const Polygon& borders, const std::vector<Polygon>& obstacle_list, 
                 const std::vector<std::pair<int,Polygon>>& victim_list, const Polygon& gate, 
                 const float x, const float y, const float theta, Path& path,
                 const std::string& config_folder){
//mission_2
std::vector<Polygon> obsctacle_offset = obstacleinflation(obstacle_list,60);
	std::vector<Point> goalCenter;
	goalCenter.emplace_back(x, y);
        for (int i = 0; i < victim_list.size(); i++){
            Polygon currentVictim = std::get<1>(victim_list[i]);
            std::pair<double, double> victimCenter = get_center(currentVictim);
            goalCenter.emplace_back(victimCenter.first, victimCenter.second);
        }
	std::pair<double, double> gateCenter = get_center(gate);
	double gateX = gateCenter.first;
	double gateY = gateCenter.second;
	goalCenter.emplace_back(gateCenter.first, gateCenter.second);
	Path temp_path;
	double MAX_X = (borders[1].x);
	double MIN_X = (borders[0].x);
	double MAX_Y = (borders[3].y);
	double MIN_Y = (borders[0].y);
	
        //std::pair<double, double> gateCenter = get_center(gate);
	//auto start_rrt = high_resolution_clock::now();
	auto space(std::make_shared<ob::RealVectorStateSpace>(2));

	//Set world info
	space->setBounds(0.0, std::max(MAX_X,MAX_Y));

	// Construct a space information instance for this state space
	auto si(std::make_shared<ob::SpaceInformation>(space));

	// Set the object used to check which states in the space are valid
	si->setStateValidityChecker(std::make_shared<ValidityChecker>(si,obsctacle_offset));

	si->setup();

	ob::ScopedState<> start(space);
	ob::ScopedState<> goal(space);
	//Goals setup
        start->as<ob::RealVectorStateSpace::StateType>()->values[0] = x;
        start->as<ob::RealVectorStateSpace::StateType>()->values[1] = y;

        goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = gateCenter.first;
        goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = gateCenter.second;



	// Create the problem
	auto pdef(std::make_shared<ob::ProblemDefinition>(si));

	// Create the optimization objective
	pdef->setOptimizationObjective(allocateObjective(si, OBJECTIVE_PATHLENGTH));

	// Set the start and goal states
	pdef->setStartAndGoalStates(start, goal);
	ob::PlannerPtr optimizingPlanner;
	// Select the planner
	optimizingPlanner = allocatePlanner(si, PLANNER_RRTSTAR);
	// Set the problem instance for our planner to solve
	optimizingPlanner->setProblemDefinition(pdef);
	optimizingPlanner->setup();

	// attempt to solve the planning problem in the given runtime
	ob::PlannerStatus solved = optimizingPlanner->solve(1.0);
	if (solved)
	{
        // store the points
	std::static_pointer_cast<og::PathGeometric>(pdef->getSolutionPath());
	og::PathGeometric path( dynamic_cast< const og::PathGeometric& >(*pdef->getSolutionPath()));
	const std::vector<ob::State*> &states = path.getStates();
	ob::State *state;
	Pose robot_pos;
	robot_pos.x = x;
	robot_pos.y = y;
	if(i == 0 )
                {
                    temp_path.points.emplace_back(robot_pos);
                }
	for( size_t i = 1 ; i < states.size( ) ; ++i )
	{
	state = states[i]->as< ob::State >();
	Pose temp;
	temp.x = state->as<ob::RealVectorStateSpace::StateType>()->values[0];
	temp.y = state->as<ob::RealVectorStateSpace::StateType>()->values[1];
	temp_path.points.emplace_back(temp);
	}
	}
	else{
	cout<<"could not find a solution for goal"<<goal <<endl;
	}
	//print_path(temp_path);
 	for (int iter =0; iter<victim_list.size();iter++) {
            boost_polygon currentVictim = convertPolygonToBoostPolygon(victim_list[iter].second);
            if (bg::distance(currentVictim, temp_path) <= config_params.mission2_threshold_distance) {
                int index = -1;
                double minDist = 10000;
                for (int i = 0; i < temp_path.size(); i++) {
                    float distance = bg::distance(currentVictim, temp_path[i]); // distance b/w victim and line
                    if (distance < minDist) {
                        minDist = distance;
                        index = i; // if victim is near, takeid in path direction
                    }            
                }
                goalCenter.push_back(victim_list[iter].second});
            }
        }
	auto space(std::make_shared<ob::RealVectorStateSpace>(2));

	//Set world info
	space->setBounds(0.0, std::max(MAX_X,MAX_Y));

	// Construct a space information instance for this state space
	auto si(std::make_shared<ob::SpaceInformation>(space));

	// Set the object used to check which states in the space are valid
	si->setStateValidityChecker(std::make_shared<ValidityChecker>(si,obsctacle_offset));

	si->setup();

	ob::ScopedState<> start(space);
	ob::ScopedState<> goal(space);
	//Goals setup
        start->as<ob::RealVectorStateSpace::StateType>()->values[0] = x;
        start->as<ob::RealVectorStateSpace::StateType>()->values[1] = y;

        goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = gateCenter.first;
        goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = gateCenter.second;

	goalCenter.emplace_back(gateCenter.first, gateCenter.second);
	Path temp_path_2;
	for (int i = 0; i < goalCenter.size()-1; i++){
	// Create the problem
	auto pdef(std::make_shared<ob::ProblemDefinition>(si));

	// Create the optimization objective
	pdef->setOptimizationObjective(allocateObjective(si, OBJECTIVE_PATHLENGTH));

	// Set the start and goal states
	pdef->setStartAndGoalStates(start, goal);
	ob::PlannerPtr optimizingPlanner;
	// Select the planner
	optimizingPlanner = allocatePlanner(si, PLANNER_RRTSTAR);
	// Set the problem instance for our planner to solve
	optimizingPlanner->setProblemDefinition(pdef);
	optimizingPlanner->setup();

	// attempt to solve the planning problem in the given runtime
	ob::PlannerStatus solved = optimizingPlanner->solve(1.0);
	if (solved)
	{
        // store the points
	std::static_pointer_cast<og::PathGeometric>(pdef->getSolutionPath());
	og::PathGeometric path( dynamic_cast< const og::PathGeometric& >(*pdef->getSolutionPath()));
	const std::vector<ob::State*> &states = path.getStates();
	ob::State *state;
	Pose robot_pos;
	robot_pos.x = x;
	robot_pos.y = y;
	if(i == 0 )
                {
                    temp_path.points.emplace_back(robot_pos);
                }
	for( size_t i = 1 ; i < states.size( ) ; ++i )
	{
	state = states[i]->as< ob::State >();
	Pose temp_2;
	temp_2.x = state->as<ob::RealVectorStateSpace::StateType>()->values[0];
	temp_2.y = state->as<ob::RealVectorStateSpace::StateType>()->values[1];
	temp_path_2.points.emplace_back(temp);
	}
	}
	else{
	cout<<"could not find a solution for goal"<<goal <<endl;
	}
	double sx,sy,st,ex,ey,et;
	for(int iter=1;iter<temp_path.points.size();iter++)
	{
	if(iter == 1){
	sx = x;
	sy = y;   
	st = theta;
	}
	ex = temp_path_2.points[iter].x;
	ey = temp_path_2.points[iter].y;
	et = angle_dubins(temp_path.points[iter-1],temp_path.points[iter],temp_path.points[iter+1]); // get approach angle between points

	if(iter == temp_path_2.points.size()-1){
	et = gate_angle(borders,gateX,gateY); 
	}   
	//cout<<"num of path points are" <<endl;
	dubinsCurve dubins = {}; // dubins curve        
	dubins_shortest_path(dubins, sx, sy, st, ex, ey, et, 15); // get dubins curve and discretize them
  	int npts=50;
	Path dubins_path;
	dubins_path = getPath(dubins, npts);
	path.points.insert(path.points.end(),dubins_path.points.begin(),dubins_path.points.end());
	
	sx = ex;
	sy = ey;
	st = et;
	}
	return true;
	}

