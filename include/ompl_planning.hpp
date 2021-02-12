
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include "ompl/util/Console.h"
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/PlannerDataGraph.h>


#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>

#include <boost/graph/adjacency_list.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/linestring.hpp>

#include <boost/graph/dijkstra_shortest_paths.hpp>
#include "student_planning_interface.hpp"

#include <memory>

#include <fstream>

#include "utils.hpp"

#include<iostream>
using namespace std;
namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace bg = boost::geometry;
typedef bg::model::d2::point_xy<double> boost_point;
typedef bg::model::polygon <boost_point> boost_polygon;
typedef boost::geometry::model::linestring<boost_point> boost_linestring;

//Planner Name
enum optimalPlanner
{
    /// RRT Star implementation
    PLANNER_RRTSTAR,
};

// Objective function
enum planningObjective
{
    // Path clearance objective
    OBJECTIVE_PATHCLEARANCE,
    // Shortest Path  
    OBJECTIVE_PATHLENGTH,
};

//Set the optimization objective in the space

ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si)
{
    return std::make_shared<ob::PathLengthOptimizationObjective>(si);
}



class ClearanceObjective : public ob::StateCostIntegralObjective
{
public:
    ClearanceObjective(const ob::SpaceInformationPtr& si) :
        ob::StateCostIntegralObjective(si, true)
    {
    }

    // Cost to maximize clearance from the obstacle
    ob::Cost stateCost(const ob::State* s) const override
    {
        return ob::Cost(1 / (si_->getStateValidityChecker()->clearance(s) + std::numeric_limits<double>::min()));
    }
};


//Set the optimization objective in the space
ob::OptimizationObjectivePtr getClearanceObjective(const ob::SpaceInformationPtr& si)
{
    return std::make_shared<ClearanceObjective>(si);
}


// Planner Allocation
ob::PlannerPtr allocatePlanner(ob::SpaceInformationPtr si, optimalPlanner plannerType)
{
    switch (plannerType)
    {
        case PLANNER_RRTSTAR:
        {
            return std::make_shared<og::RRTstar>(si);
            break;
        }
        default:
        {
            OMPL_ERROR("Planner-type enum is not implemented in allocation function.");
            return ob::PlannerPtr(); 
            break;
        }
    }
}

//Assign Objective
ob::OptimizationObjectivePtr allocateObjective(const ob::SpaceInformationPtr& si, planningObjective objectiveType)
{
    switch (objectiveType)
    {
        case OBJECTIVE_PATHCLEARANCE:
            return getClearanceObjective(si);
            break;
        case OBJECTIVE_PATHLENGTH:
            return getPathLengthObjective(si);
            break;
        default:
            OMPL_ERROR("Optimization-objective enum is not implemented in allocation function.");
            return ob::OptimizationObjectivePtr();
            break;
    }
}


//Polygon conversion to boost polygon for obstacles
boost_polygon convertPolygonToBoostPolygon(const Polygon &poly) {
    boost_polygon boost_poly;

    for(auto &iter : poly){
        bg::append(boost_poly.outer(), boost_point(iter.x, iter.y));
    }
    
    bg::append(boost_poly.outer(), boost_point(poly[0].x, poly[0].y));

    return boost_poly;
}


//Check if the state is in colision or not
class ValidityChecker : public ob::StateValidityChecker
{
public:

    std::vector<Polygon> obstacles;
    ValidityChecker(const ob::SpaceInformationPtr& si,std::vector<Polygon> obstacle_list ) :
        ob::StateValidityChecker(si) {obstacles = obstacle_list;}

    bool isValid(const ob::State* state) const override
    {       
	const ob::RealVectorStateSpace::StateType* state2D = state->as<ob::RealVectorStateSpace::StateType>();
        double x = state2D->values[0]; //robot position x
        double y = state2D->values[1]; //robot position y
        boost_point centerPoint(x, y);
        for (Polygon polygon : obstacles) {
                boost_polygon Poly = convertPolygonToBoostPolygon(polygon);
                if (boost::geometry::within(centerPoint, Poly))
// reference : https://www.boost.org/doc/libs/1_64_0/libs/geometry/doc/html/geometry/reference/algorithms/within/within_3_with_strategy.html
                    return false;
        }
        return true;
    }

};

