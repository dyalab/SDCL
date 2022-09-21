#include <thread>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/samplers/GaussianValidStateSampler.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/rrt/BiTRRT.h>
#include "ompl/base/samplers/UniformValidStateSampler.h"
#include "ompl/base/ValidStateSampler.h"
#include "ompl/base/StateSampler.h"
#include "ompl/base/SpaceInformation.h"
#include <ompl/util/PPM.h>
#include <ompl/config.h>
#include "environment.h"
#include "scenes.h"
#include "parser.h"
#include "SDCL.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;



bool solver(Environment* env, double duration, bool use_training, bool use_Gaussian) {

    int N = env->get_dim();

    // construct the state space we are planning in
    auto space(std::make_shared<ob::RealVectorStateSpace>(N));

    // set the bounds
    ob::RealVectorBounds bounds(N);
    pt lows = env->get_lows();
    pt highs = env->get_highs();
    double delta = env->get_delta();
    for (int i = 0; i < N; i++) {
        bounds.setLow(i, lows[i] - delta * 2);
        bounds.setHigh(i, highs[i] + delta * 2);
    }
    space->setBounds(bounds);

    // construct an instance of  space information from this state space
    auto si(std::make_shared<ob::SpaceInformation>(space));

    // set state validity checking for this space
    si->setStateValidityChecker([=](const ob::State *state) {
                                                                pt point(N);
                                                                for (int i = 0; i < N; i++) {
                                                                    point[i] = (float)state->as<ob::RealVectorStateSpace::StateType>()->values[i];
                                                                }
                                                                if (!env->isStateValid(point)) {
                                                                    return false;
                                                                } else {
                                                                    return true;
                                                                }});
    si->setup();

    // create a random start state
    ob::ScopedState<> start(space);
    ob::ScopedState<> goal(space);
    pt p1 = env->get_start();
    pt p2 = env->get_goal();
    for (int i = 0; i < N; i++) {
        start->as<ob::RealVectorStateSpace::StateType>()->values[i] = p1[i];
        goal->as<ob::RealVectorStateSpace::StateType>()->values[i] = p2[i];
    }

    // create a problem instance
    auto pdef_ = std::make_shared<ob::ProblemDefinition>(si);

    // set the start and goal states
    pdef_->setStartAndGoalStates(start, goal);

    // create a planner for the defined space
    auto planner_ = std::make_shared<SDCL>(si, false, use_training, use_Gaussian);

    // set the problem we are trying to solve for the planner
    planner_->setProblemDefinition(pdef_);

    // perform setup steps for the planner
    planner_->setup();

    // print the settings for this space
    si->printSettings(std::cout);

    // print the problem settings
    pdef_->print(std::cout);

    // attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = planner_->ob::Planner::solve(duration);

    if (solved && pdef_->hasExactSolution())
    {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        ob::PathPtr path = pdef_->getSolutionPath();
        std::cout << "Found solution:" << std::endl;

        // print the path to screen
        path->print(std::cout);
        return true;
    } else {
        std::cout << "No solution found" << std::endl;
        return false;
    }
}


int main(int argc, char ** argv)
{
    auto start_time = std::chrono::steady_clock::now();
    std::cout << "SDCL planner start!" << std::endl;
    int scene = 0;
    double duration = 0;
    bool use_Gaussian;
    bool use_training;
    parse(argc, argv, &scene, &duration, &use_training, &use_Gaussian);

    Environment* scene_env = get_scene_env(scene);
    
    bool found_plan = false;

    found_plan = solver(scene_env, duration, use_training, use_Gaussian);

    delete scene_env;

    std::chrono::duration<double> tot = std::chrono::steady_clock::now() - start_time;
    std::cout << "Total time is " << tot.count() << std::endl;
    return 0;
}
