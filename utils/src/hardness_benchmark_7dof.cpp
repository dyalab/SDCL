#include <ompl/geometric/SimpleSetup.h>

#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/prm/SPARS.h>
#include <ompl/geometric/planners/prm/SPARStwo.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/planners/stride/STRIDE.h>
#include <ompl/geometric/planners/pdst/PDST.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/planners/AnytimePathShortening.h>
#include <ompl/geometric/planners/informedtrees/AITstar.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/informedtrees/ABITstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/TSRRT.h>
#include <ompl/geometric/planners/rrt/pRRT.h>
#include <ompl/geometric/planners/sbl/pSBL.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/planners/rrt/RRTXstatic.h>
#include <ompl/geometric/planners/rrt/VFRRT.h>
#include <ompl/geometric/planners/rlrt/BiRLRT.h>
#include <ompl/geometric/planners/est/BiEST.h>
#include <ompl/geometric/planners/rrt/BiTRRT.h>
#include <ompl/geometric/planners/rrt/LazyLBTRRT.h>
#include <ompl/geometric/planners/est/ProjEST.h>
#include <ompl/geometric/planners/rlrt/RLRT.h>
#include <ompl/geometric/planners/sst/SST.h>
#include <ompl/geometric/planners/rrt/TSRRT.h>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/samplers/GaussianValidStateSampler.h>
#include "ompl/base/samplers/UniformValidStateSampler.h"
#include "ompl/base/ValidStateSampler.h"
#include "ompl/base/StateSampler.h"
#include "ompl/base/SpaceInformation.h"
#include <ompl/util/PPM.h>
#include <ompl/config.h>
#include "ompl/tools/benchmark/Benchmark.h"

#include "environment.h"
#include "InfPlanner.h"
#include <thread>


namespace ob = ompl::base;
namespace og = ompl::geometric;

auto last_end_time = std::chrono::steady_clock::now();

void PostRunEvent(const base::PlannerPtr &planner, tools::Benchmark::RunProperties &run)
{
    std::chrono::duration<double> duration = std::chrono::steady_clock::now() - last_end_time;
    last_end_time = std::chrono::steady_clock::now();
 
    std::cout << planner->getName() << " planning time is "<< duration.count() << std::endl;

    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%d-%m-%Y-%H-%M-%S");
    auto str = oss.str();

    std::ofstream csv_file(planner->getName() + str + ".csv");

    csv_file << planner->getName() << ", " << duration.count() << std::endl;
    csv_file.close();
}
 


void benchmark(Environment* env, pt p1, pt p2, std::string scene_name) {
    // plan with RRT-connect
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

    // define a simple setup class
    og::SimpleSetup ss(space);

    // set state validity checking for this space
    ss.setStateValidityChecker([=](const ob::State *state) {pt point(N);
                                                            for (int i = 0; i < N; i++) {
                                                                point[i] = (float)state->as<ob::RealVectorStateSpace::StateType>()->values[i];
                                                            }
                                                            if (!env->isStateValid(point)) {
                                                                return false;
                                                            } else {
                                                                return true;
                                                            }});

    // create start and goal state
    ob::ScopedState<> start(space);
    ob::ScopedState<> goal(space);
    for (int i = 0; i < N; i++) {
        start->as<ob::RealVectorStateSpace::StateType>()->values[i] = p1[i];
        goal->as<ob::RealVectorStateSpace::StateType>()->values[i] = p2[i];
    }

    // set the start and goal states
    ss.setStartAndGoalStates(start, goal);

    // First we create a benchmark class:
    ompl::tools::Benchmark b(ss, "ICRA2023");

    // Optionally, specify some benchmark parameters (doesn't change how the benchmark is run)
    b.addExperimentParameter("num_dofs", "INTEGER", std::to_string(N));
    b.addExperimentParameter("scene_name", "STRING", scene_name);

    // We add the planners to evaluate.
    b.addPlanner(base::PlannerPtr(new InfPlannerPRM(ss.getSpaceInformation(), false, true))); // not using training points

    // b.addPlanner(base::PlannerPtr(new InfPlannerPRM(ss.getSpaceInformation(), false, true))); // using training points

    // b.addPlanner(base::PlannerPtr(new og::KPIECE1(ss.getSpaceInformation())));
    // b.addPlanner(base::PlannerPtr(new og::LBKPIECE1(ss.getSpaceInformation())));
    // b.addPlanner(base::PlannerPtr(new og::BKPIECE1(ss.getSpaceInformation())));

    // b.addPlanner(base::PlannerPtr(new og::SPARS(ss.getSpaceInformation())));
    // b.addPlanner(base::PlannerPtr(new og::SPARStwo(ss.getSpaceInformation())));
    // b.addPlanner(base::PlannerPtr(new og::STRIDE(ss.getSpaceInformation())));
    // b.addPlanner(base::PlannerPtr(new og::PDST(ss.getSpaceInformation())));
    // b.addPlanner(base::PlannerPtr(new og::FMT(ss.getSpaceInformation())));
    // b.addPlanner(base::PlannerPtr(new og::BFMT(ss.getSpaceInformation())));
    // b.addPlanner(base::PlannerPtr(new og::AnytimePathShortening(ss.getSpaceInformation())));

    // b.addPlanner(base::PlannerPtr(new og::RRT(ss.getSpaceInformation())));
    // b.addPlanner(base::PlannerPtr(new og::LazyRRT(ss.getSpaceInformation())));
    // b.addPlanner(base::PlannerPtr(new og::TRRT(ss.getSpaceInformation())));
    // b.addPlanner(base::PlannerPtr(new og::LBTRRT(ss.getSpaceInformation())));
    // b.addPlanner(base::PlannerPtr(new og::RRTConnect(ss.getSpaceInformation())));
    // b.addPlanner(base::PlannerPtr(new og::pRRT(ss.getSpaceInformation())));
    // b.addPlanner(base::PlannerPtr(new og::BiTRRT(ss.getSpaceInformation())));
    // b.addPlanner(base::PlannerPtr(new og::RRTstar(ss.getSpaceInformation())));
    // b.addPlanner(base::PlannerPtr(new og::RRTsharp(ss.getSpaceInformation())));
    // b.addPlanner(base::PlannerPtr(new og::InformedRRTstar(ss.getSpaceInformation())));
    // b.addPlanner(base::PlannerPtr(new og::LazyLBTRRT(ss.getSpaceInformation())));
    // b.addPlanner(base::PlannerPtr(new og::RRTXstatic(ss.getSpaceInformation())));

    // b.addPlanner(base::PlannerPtr(new og::EST(ss.getSpaceInformation())));
    // b.addPlanner(base::PlannerPtr(new og::SBL(ss.getSpaceInformation())));
    // b.addPlanner(base::PlannerPtr(new og::pSBL(ss.getSpaceInformation())));
    // b.addPlanner(base::PlannerPtr(new og::BiEST(ss.getSpaceInformation())));

    // b.addPlanner(base::PlannerPtr(new og::AITstar(ss.getSpaceInformation())));
    // b.addPlanner(base::PlannerPtr(new og::BITstar(ss.getSpaceInformation())));
    // b.addPlanner(base::PlannerPtr(new og::ABITstar(ss.getSpaceInformation())));

    // b.addPlanner(base::PlannerPtr(new og::BiRLRT(ss.getSpaceInformation())));
    // b.addPlanner(base::PlannerPtr(new og::ProjEST(ss.getSpaceInformation())));
    // b.addPlanner(base::PlannerPtr(new og::RLRT(ss.getSpaceInformation())));
    // b.addPlanner(base::PlannerPtr(new og::SST(ss.getSpaceInformation())));

    // b.addPlanner(base::PlannerPtr(new og::PRM(ss.getSpaceInformation())));
    // b.addPlanner(base::PlannerPtr(new og::PRMstar(ss.getSpaceInformation())));
    // b.addPlanner(base::PlannerPtr(new og::LazyPRM(ss.getSpaceInformation())));


    // b.addPlanner(base::PlannerPtr(new og::QRRT(ss.getSpaceInformation())));

    b.setPostRunEvent(std::bind(&PostRunEvent, std::placeholders::_1, std::placeholders::_2));
    // Now we can benchmark: 600 second time limit for each plan computation,
    // 8000 MB maximum memory usage per plan computation, 10 runs for each planner
    // and true means that a text-mode progress bar should be displayed while
    // computation is running.
    ompl::tools::Benchmark::Request req;
    req.maxTime = 300;
    req.maxMem = 8000;
    req.runCount = 30;
    req.displayProgress = true;

    last_end_time = std::chrono::steady_clock::now();
    b.benchmark(req);
     
    // This will generate a file of the form ompl_host_time.log
    b.saveResultsToFile();
}

int main(int argc, char ** argv)
{
    pt start = {-2.47, 2.5093, 0.5, -2.54409, -0.1, 1.72318, -0.30};
    pt goal = {-1.09912, 2.5093, 0.5, -2.54409, -0.1, 1.72318, -0.30};

    std::vector<std::string> blocks = {"shelf-left", "shelf-right", "shelf-top", "shelf-middle", "item1", "item2"};
    std::vector<std::string> joints = {"s0", "s1", "s2", "e", "w0", "w1", "w2"};
    std::vector<std::string> links = {"e-link", "w0-link", "w1-link", "end-link", "s2-link"};

    double delta = 0.2; // the boundaries virtual obstacle region's size and the virtual cfree region's size

    pt lows =  {-M_PI, -2.8, -M_PI/2, -2.8, -M_PI/2, -2.8, -M_PI/2};
    pt highs = {0.0,   2.8, M_PI/2, 2.8, M_PI/2, 2.8, M_PI/2};

    std::vector<std::string> scene_files = {"../resource/7dof_exist_easy/libscene.so", 
                                            "../resource/7dof_exist_medium/libscene.so", 
                                            "../resource/7dof_exist_hard/libscene.so"};
    std::vector<std::string> scene_names = {"7dofexisteasy", 
                                            "7dofexistmedium", 
                                            "7dofexisthard"};
    
    for (int i = 0; i < scene_files.size(); i++) 
    {
        // if (i == 1 or i == 0) continue;
        auto start_time = std::chrono::steady_clock::now();

        Environment* env = new HighDofEnvironment(start, goal, blocks, joints, links, (scene_names[i]).c_str(), (scene_files[i]).c_str(), lows, highs, delta);

        benchmark(env, start, goal, scene_names[i]);
        delete env;

        std::chrono::duration<double> tot = std::chrono::steady_clock::now() - start_time;
        std::cout << "Total benchmarking time for " << scene_names[i] << " is " << tot.count() << std::endl;
    }

    return 0;
}
