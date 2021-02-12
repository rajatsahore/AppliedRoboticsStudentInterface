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
#include "dubins.hpp"
#include "ompl_planning.hpp"


namespace student {

void RRT_Star_ompl(x,y,theta, path, obstacle_list){
        double MAX_X = (borders[1].x);
        double MIN_X = (borders[0].x);
        double MAX_Y = (borders[3].y);
        double MIN_Y = (borders[0].y);
        auto start_rrt = high_resolution_clock::now();
        auto space(std::make_shared<ob::RealVectorStateSpace>(2));

        //Set world info
        space->setBounds(0.0, std::max(MAX_X,MAX_Y));

        // Construct a space information instance for this state space
        auto si(std::make_shared<ob::SpaceInformation>(space));
        vector<Polygon> obstacle_list;
        for(int iter=0 ; iter<obstacle_list.size();iter++)
        {
            if(obstacle_list[iter].size()>0)
                new_obstacle_list.emplace_back(obstacle_list[iter]);
            cout<<endl;
        }

        // Set the object used to check which states in the space are valid
        si->setStateValidityChecker(std::make_shared<ValidityChecker>(si,new_obstacle_list));

        si->setup();

        int goal_iter = 1;

        Path temp_path;
        double gateX = 0.2;
        double gateY = 0.8;

        ob::ScopedState<> start(space);
        ob::ScopedState<> goal(space);

        start->as<ob::RealVectorStateSpace::StateType>()->values[0] = x;
        start->as<ob::RealVectorStateSpace::StateType>()->values[1] = y;
  
            

            // Create a problem instance
            auto pdef(std::make_shared<ob::ProblemDefinition>(si));

            // Create the optimization objective specified by us.
            pdef->setOptimizationObjective(allocateObjective(si, OBJECTIVE_PATHLENGTH));

            // Set the start and goal states
            pdef->setStartAndGoalStates(start, goal);
            
            ob::PlannerPtr optimizingPlanner;
            // Construct the optimal planner specified by us in config file.
            
            optimizingPlanner = allocatePlanner(si, PLANNER_RRTSTAR);
			
            // Set the problem instance for our planner to solve
            optimizingPlanner->setProblemDefinition(pdef);
            optimizingPlanner->setup();

            // attempt to solve the planning problem in the given runtime
            ob::PlannerStatus solved = optimizingPlanner->solve(1.0);

            if (solved)
            {
                std::cout
                << optimizingPlanner->getName()
                << " found a solution of length "
                << pdef->getSolutionPath()->length()
                << " with an optimization objective value of "
                << pdef->getSolutionPath()->cost(pdef->getOptimizationObjective()) << std::endl;

                std::static_pointer_cast<og::PathGeometric>(pdef->getSolutionPath());
                og::PathGeometric path_ompl( dynamic_cast< const og::PathGeometric& >(*pdef->getSolutionPath()));
                const std::vector<ob::State*> &states = path_ompl.getStates();
                ob::State *state;
                //cout<<"num of path points are" << states.size()<<endl;

                Pose robot_pos;
                robot_pos.x = x;
                robot_pos.y = y;
                if(goal_iter == 1 )
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
           goal_iter = goal_iter +1;
        }
        auto stop_rrt = high_resolution_clock::now();
        auto duration_rrt = duration_cast<milliseconds>(stop_rrt - start_rrt); 
        cout <<"RRT Star Planning time " << duration_rrt.count() << endl;
}
}

