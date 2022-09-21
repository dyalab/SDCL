/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Rice University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Rice University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan, James D. Marble, Ryan Luna, Henning Kayser */

#include "SDCL.h"

#define foreach BOOST_FOREACH


double objfunc(unsigned n, const double *x, double *grad, void *data)
{
    double *p = (double *) data;
    double res = 0;
    if (p && x){
        for(int i = 0; i < n; i++){
            res += pow(x[i] - p[i], 2);
        }
    }
    if (grad) {
        for(int i = 0; i < n; i++){
            grad[i] = 2 * (x[i] - p[i]);
        }
    }
    return res;
}

double objfunc2(unsigned n, const double *x, double *grad, void *data)
{
    ModelData *d = (ModelData *) data;
    double b = d->b;
    int num_vectors = d->num_vectors;
    double* coef = d->coef;
    double* vectors = d->vectors;
    double gamma = d->gamma;
    double f = 0;
    double dists_square[num_vectors];
    for(int k = 0; k < num_vectors; k++){
        dists_square[k] = 0;
        for(int i = 0; i < n; i ++){
            dists_square[k] += pow(x[i] - vectors[n*k+i], 2);
        }
        f += coef[k] * exp(-gamma * dists_square[k]);
    }

    if (grad) {
        for(int i = 0; i < n; i++){
            grad[i] = 0;
            for(int k = 0; k < num_vectors; k++){
                grad[i] += coef[k] * exp(-gamma * dists_square[k]) * (-gamma) * 2 * (x[i] - vectors[n*k+i]);
            }
        }
        if (f-b < 0) {
            for(int i = 0; i < n; i++) {
                grad[i] = -grad[i];
            }
        }
    }

    if (f-b < 0) {
        return b-f;
    } else {
        return f-b;
    }
}


double myconstraint(unsigned n, const double *x, double *grad, void *data)
{
    ModelData *d = (ModelData *) data;
    double b = d->b;
    int num_vectors = d->num_vectors;
    double* coef = d->coef;
    double* vectors = d->vectors;
    double gamma = d->gamma;
    double f = 0;
    double dists_square[num_vectors];
    for(int k = 0; k < num_vectors; k++){
        dists_square[k] = 0;
        for(int i = 0; i < n; i ++){
            dists_square[k] += pow(x[i] - vectors[n*k+i], 2);
        }
        f += coef[k] * exp(-gamma * dists_square[k]);
    }
    if (grad) {
        for(int i = 0; i < n; i++){
            grad[i] = 0;
            for(int k = 0; k < num_vectors; k++){
                grad[i] += coef[k] * exp(-gamma * dists_square[k]) * (-gamma) * 2 * (x[i] - vectors[n*k+i]);
            }
        }
    }
    return f-b;
 }

int findClosestPoint(double *p, double *startingP, double *res, int n, ModelData svm_data, std::vector<double> lower_bound, std::vector<double> upper_bound){
    // ModelData svm_data = {b, num_vectors, gamma, coef, vectors};
    
    if (startingP && abs(myconstraint(n, p, NULL, &svm_data)) > abs(myconstraint(n, startingP, NULL, &svm_data))){
        for(int i = 0; i < n; i++){
            res[i] = startingP[i];
        }
    } else {
        for(int i = 0; i < n; i++){
            res[i] = p[i];
        }
    }

    nlopt_opt opt;
    opt = nlopt_create(NLOPT_LD_SLSQP, n);
    nlopt_set_min_objective(opt, objfunc2, &svm_data);
    double maxtime = 0.1;
    nlopt_set_maxtime(opt, maxtime);

    double lb[n] = {};
    double ub[n] = {};

    for (int i = 0; i < n; i++) 
    {
        lb[i] = lower_bound[i];
        ub[i] = upper_bound[i];
    }
    nlopt_set_lower_bounds(opt, lb);
    nlopt_set_upper_bounds(opt, ub);

    // nlopt_add_equality_constraint(opt, myconstraint, &svm_data, 1e-6);
    double minf;
    nlopt_set_xtol_rel(opt, 1e-3);
    int result = nlopt_optimize(opt, res, &minf);
    nlopt_destroy(opt);
    double nul[1];
    return result;
}


namespace ompl
{
    namespace magic
    {
        /** \brief The number of steps to take for a random bounce
            motion generated as part of the expansion step of PRM. */
        static const unsigned int MAX_RANDOM_BOUNCE_STEPS = 5;

        /** \brief The time in seconds for a single roadmap building operation (dt)*/
        static const double ROADMAP_BUILD_TIME = 0.2;

        /** \brief The number of nearest neighbors to consider by
            default in the construction of the PRM roadmap */
        static const unsigned int DEFAULT_NEAREST_NEIGHBORS = 10;
    }  // namespace magic
}  // namespace ompl


SDCL::SDCL(const base::SpaceInformationPtr &si, bool starStrategy, bool use_training, bool use_Gaussian)
  : base::Planner(si, "SDCL")
  , starStrategy_(starStrategy)
  , stateProperty_(boost::get(vertex_state_t(), g_))
  , totalConnectionAttemptsProperty_(boost::get(vertex_total_connection_attempts_t(), g_))
  , successfulConnectionAttemptsProperty_(boost::get(vertex_successful_connection_attempts_t(), g_))
  , weightProperty_(boost::get(boost::edge_weight, g_))
  , disjointSets_(boost::get(boost::vertex_rank, g_), boost::get(boost::vertex_predecessor, g_))
  , stddev_(si->getMaximumExtent() * magic::STD_DEV_AS_SPACE_EXTENT_FRACTION)
  , use_training_(use_training)
  , use_Gaussian_(use_Gaussian)
  , lower_bound_((si_->getStateSpace()->as<base::RealVectorStateSpace>()->getBounds()).low)
  , upper_bound_((si_->getStateSpace()->as<base::RealVectorStateSpace>()->getBounds()).high)
{
    specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
    specs_.approximateSolutions = true;
    specs_.optimizingPaths = true;
    specs_.multithreaded = true;

    if (!starStrategy_)
        Planner::declareParam<unsigned int>("max_nearest_neighbors", this, &SDCL::setMaxNearestNeighbors,
                                            &SDCL::getMaxNearestNeighbors, std::string("8:1000"));

    addPlannerProgressProperty("iterations INTEGER", [this] { return getIterationCount(); });
    addPlannerProgressProperty("best cost REAL", [this] { return getBestCost(); });
    addPlannerProgressProperty("milestone count INTEGER", [this] { return getMilestoneCountString(); });
    addPlannerProgressProperty("edge count INTEGER", [this] { return getEdgeCountString(); });

}

SDCL::~SDCL()
{
    freeMemory();
}

void SDCL::setup()
{
    Planner::setup();
    if (!nn_)
    {
        specs_.multithreaded = false;  // temporarily set to false since nn_ is used only in single thread
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(this));
        specs_.multithreaded = true;
        nn_->setDistanceFunction([this](const Vertex a, const Vertex b) { return distanceFunction(a, b); });
    }
    if (!connectionStrategy_)
        setDefaultConnectionStrategy();
    if (!connectionFilter_)
        connectionFilter_ = [](const Vertex &, const Vertex &) { return true; };

    // Setup optimization objective
    //
    // If no optimization objective was specified, then default to
    // optimizing path length as computed by the distance() function
    // in the state space.
    if (pdef_)
    {
        if (pdef_->hasOptimizationObjective())
            opt_ = pdef_->getOptimizationObjective();
        else
        {
            opt_ = std::make_shared<base::PathLengthOptimizationObjective>(si_);
            if (!starStrategy_)
                opt_->setCostThreshold(opt_->infiniteCost());
        }
    }
    else
    {
        OMPL_INFORM("%s: problem definition is not set, deferring setup completion...", getName().c_str());
        setup_ = false;
    }
}

void SDCL::setMaxNearestNeighbors(unsigned int k)
{
    if (starStrategy_)
        throw Exception("Cannot set the maximum nearest neighbors for " + getName());
    if (!nn_)
    {
        specs_.multithreaded = false;  // temporarily set to false since nn_ is used only in single thread
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(this));
        specs_.multithreaded = true;
        nn_->setDistanceFunction([this](const Vertex a, const Vertex b) { return distanceFunction(a, b); });
    }
    if (!userSetConnectionStrategy_)
        connectionStrategy_ = ompl::geometric::KStrategy<Vertex>(k, nn_);
    if (isSetup())
        setup();
}

unsigned int SDCL::getMaxNearestNeighbors() const
{
    const auto strategy = connectionStrategy_.target<ompl::geometric::KStrategy<Vertex>>();
    return strategy ? strategy->getNumNeighbors() : 0u;
}

void SDCL::setDefaultConnectionStrategy()
{
    if (starStrategy_)
        connectionStrategy_ = ompl::geometric::KStarStrategy<Vertex>([this] { return milestoneCount(); }, nn_, si_->getStateDimension());
    else
        connectionStrategy_ = ompl::geometric::KStrategy<Vertex>(magic::DEFAULT_NEAREST_NEIGHBORS, nn_);
}

void SDCL::setProblemDefinition(const base::ProblemDefinitionPtr &pdef)
{
    Planner::setProblemDefinition(pdef);
    clearQuery();
}

void SDCL::clearQuery()
{
    startM_.clear();
    goalM_.clear();
    pis_.restart();
}

void SDCL::clear()
{
    OMPL_INFORM("Calling clear()...");
    Planner::clear();
    sampler_.reset();
    simpleSampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    clearQuery();

    iterations_ = 0;
    bestCost_ = base::Cost(std::numeric_limits<double>::quiet_NaN());

    if (!savedModelData_.vectors) 
    {
        delete [] savedModelData_.vectors;
        savedModelData_.vectors = NULL;
    }

    if (!savedModelData_.coef) 
    {
        delete [] savedModelData_.coef;
        savedModelData_.coef = nullptr;
    }

    collisionPoints_.reset(new pvec());
    stats_ = proof_stats();
    addedToTraining_ = 0;
}

void SDCL::freeMemory()
{
    foreach (Vertex v, boost::vertices(g_))
        si_->freeState(stateProperty_[v]);
    g_.clear();

    if (!savedModelData_.vectors) delete [] savedModelData_.vectors;

    if (!savedModelData_.coef) delete [] savedModelData_.coef;

    // saved2dPoints_.close();
}

void SDCL::expandRoadmap(double expandTime)
{
    expandRoadmap(base::timedPlannerTerminationCondition(expandTime));
}

void SDCL::expandRoadmap(const base::PlannerTerminationCondition &ptc)
{
    if (!simpleSampler_)
        simpleSampler_ = si_->allocStateSampler();

    std::vector<base::State *> states(magic::MAX_RANDOM_BOUNCE_STEPS);
    si_->allocStates(states);
    expandRoadmap(ptc, states);
    si_->freeStates(states);
}

void SDCL::expandRoadmap(const base::PlannerTerminationCondition &ptc,
                                         std::vector<base::State *> &workStates)
{
    // construct a probability distribution over the vertices in the roadmap
    // as indicated in
    //  "Probabilistic Roadmaps for Path Planning in High-Dimensional Configuration Spaces"
    //        Lydia E. Kavraki, Petr Svestka, Jean-Claude Latombe, and Mark H. Overmars

    PDF<Vertex> pdf;

    graphMutex_.lock();
    foreach (Vertex v, boost::vertices(g_))
    {
        const unsigned long int t = totalConnectionAttemptsProperty_[v];
        pdf.add(v, (double)(t - successfulConnectionAttemptsProperty_[v]) / (double)t);
    }
    graphMutex_.unlock();

    if (pdf.empty())
        return;

    while (!ptc)
    {
        iterations_++;
        Vertex v = pdf.sample(rng_.uniform01());
        unsigned int s =
            si_->randomBounceMotion(simpleSampler_, stateProperty_[v], workStates.size(), workStates, false);
        if (s > 0)
        {
            s--;
            Vertex last = addMilestone(si_->cloneState(workStates[s]));

            graphMutex_.lock();
            for (unsigned int i = 0; i < s; ++i)
            {
                // add the vertex along the bouncing motion
                Vertex m = boost::add_vertex(g_);
                stateProperty_[m] = si_->cloneState(workStates[i]);
                totalConnectionAttemptsProperty_[m] = 1;
                successfulConnectionAttemptsProperty_[m] = 0;
                disjointSets_.make_set(m);

                // add the edge to the parent vertex
                const base::Cost weight = opt_->motionCost(stateProperty_[v], stateProperty_[m]);
                const Graph::edge_property_type properties(weight);
                boost::add_edge(v, m, properties, g_);
                uniteComponents(v, m);

                // add the vertex to the nearest neighbors data structure
                nn_->add(m);
                v = m;
            }

            // if there are intermediary states or the milestone has not been connected to the initially sampled vertex,
            // we add an edge
            if (s > 0 || !sameComponent(v, last))
            {
                // add the edge to the parent vertex
                const base::Cost weight = opt_->motionCost(stateProperty_[v], stateProperty_[last]);
                const Graph::edge_property_type properties(weight);
                boost::add_edge(v, last, properties, g_);
                uniteComponents(v, last);
            }
            graphMutex_.unlock();
        }
    }
}

void SDCL::growRoadmap(double growTime)
{
    growRoadmap(base::timedPlannerTerminationCondition(growTime));
}

void SDCL::growRoadmap(const base::PlannerTerminationCondition &ptc)
{
    if (!isSetup())
        setup();
    if (!sampler_)
        sampler_ = si_->allocValidStateSampler();

    base::State *workState = si_->allocState();
    growRoadmap(ptc, workState);
    si_->freeState(workState);
}

void SDCL::growRoadmap(const base::PlannerTerminationCondition &ptc, base::State *workState)
{
    /* grow roadmap in the regular fashion -- sample valid states, add them to the roadmap, add valid connections */
    while (!ptc)
    {
        iterations_++;

        // search for a valid state
        bool found = false;
        
        while (!found && !ptc)
        {
            unsigned int attempts = 0;
            do
            {
                found = sampleAndSaveCollisionPoints(workState, use_Gaussian_);
                attempts++;
            } while (attempts < magic::FIND_VALID_STATE_ATTEMPTS_WITHOUT_TERMINATION_CHECK && !found);
        }

        // add it as a milestone
        if (found)
            addMilestone(si_->cloneState(workState));
    }
}

void SDCL::checkForSolution(const base::PlannerTerminationCondition &ptc, base::PathPtr &solution)
{
    auto *goal = static_cast<base::GoalSampleableRegion *>(pdef_->getGoal().get());
    while (!ptc && !addedNewSolution_)
    {
        // Check for any new goal states
        if (goal->maxSampleCount() > goalM_.size())
        {
            const base::State *st = pis_.nextGoal();
            if (st != nullptr)
                goalM_.push_back(addMilestone(si_->cloneState(st)));
        }

        // Check for a solution
        addedNewSolution_ = maybeConstructSolution(startM_, goalM_, solution);
        // Sleep for 1ms
        if (!addedNewSolution_)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

bool SDCL::maybeConstructSolution(const std::vector<Vertex> &starts, const std::vector<Vertex> &goals,
                                                  base::PathPtr &solution)
{
    base::Goal *g = pdef_->getGoal().get();
    base::Cost sol_cost(opt_->infiniteCost());
    foreach (Vertex start, starts)
    {
        foreach (Vertex goal, goals)
        {
            // we lock because the connected components algorithm is incremental and may change disjointSets_
            graphMutex_.lock();
            bool same_component = sameComponent(start, goal);
            graphMutex_.unlock();

            if (same_component && g->isStartGoalPairValid(stateProperty_[goal], stateProperty_[start]))
            {
                base::PathPtr p = constructSolution(start, goal);
                if (p)
                {
                    base::Cost pathCost = p->cost(opt_);
                    if (opt_->isCostBetterThan(pathCost, bestCost_))
                        bestCost_ = pathCost;
                    // Check if optimization objective is satisfied
                    if (opt_->isSatisfied(pathCost))
                    {
                        solution = p;
                        return true;
                    }
                    if (opt_->isCostBetterThan(pathCost, sol_cost))
                    {
                        solution = p;
                        sol_cost = pathCost;
                    }
                }
            }
        }
    }

    return false;
}

bool SDCL::addedNewSolution() const
{
    return addedNewSolution_;
}

ompl::base::PlannerStatus SDCL::solve(const base::PlannerTerminationCondition &ptc)
{
    auto start_time = std::chrono::steady_clock::now();
    checkValidity();
    auto *goal = dynamic_cast<base::GoalSampleableRegion *>(pdef_->getGoal().get());

    if (goal == nullptr)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    // Add the valid start states as milestones
    while (const base::State *st = pis_.nextStart())
        startM_.push_back(addMilestone(si_->cloneState(st)));

    if (startM_.empty())
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!goal->couldSample())
    {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }

    // Ensure there is at least one valid goal state
    if (goal->maxSampleCount() > goalM_.size() || goalM_.empty())
    {
        const base::State *st = goalM_.empty() ? pis_.nextGoal(ptc) : pis_.nextGoal();
        if (st != nullptr)
            goalM_.push_back(addMilestone(si_->cloneState(st)));

        if (goalM_.empty())
        {
            OMPL_ERROR("%s: Unable to find any valid goal states", getName().c_str());
            return base::PlannerStatus::INVALID_GOAL;
        }
    }

    unsigned long int nrStartStates = boost::num_vertices(g_);
    OMPL_INFORM("%s: Starting planning with %lu states already in datastructure", getName().c_str(), nrStartStates);

    // Reset addedNewSolution_ member and create solution checking thread
    addedNewSolution_ = false;
    base::PathPtr sol;
    std::thread slnThread([this, &ptc, &sol] { checkForSolution(ptc, sol); });

    // construct new planner termination condition that fires when the given ptc is true, or a solution is found
    base::PlannerTerminationCondition ptcOrSolutionFound([this, &ptc] { return ptc || addedNewSolution(); });

    // create SDCL thread to generate useful samples
    std::thread infThread([this, &ptcOrSolutionFound] {generateSamples(ptcOrSolutionFound); });

    constructRoadmap(ptcOrSolutionFound);

    // Ensure slnThread and SDCLThread is ceased before exiting solve
    slnThread.join();
    infThread.join();

    OMPL_INFORM("%s: Created %u states", getName().c_str(), boost::num_vertices(g_) - nrStartStates);

    OMPL_INFORM("Total training time is %f, total sampling time is %f.", stats_.training_time, stats_.sampling_time);

    if (sol)
    {
        base::PlannerSolution psol(sol);
        psol.setPlannerName(getName());
        // if the solution was optimized, we mark it as such
        psol.setOptimized(opt_, bestCost_, addedNewSolution());
        pdef_->addSolutionPath(psol);

        stats_.solved = 1;
    }
    else
    {
        // Return an approximate solution.
        ompl::base::Cost diff = constructApproximateSolution(startM_, goalM_, sol);
        if (!opt_->isFinite(diff))
        {
            OMPL_INFORM("Closest path is still start and goal");
            return base::PlannerStatus::TIMEOUT;
        }
        OMPL_INFORM("Using approximate solution, heuristic cost-to-go is %f", diff.value());
        pdef_->addSolutionPath(sol, true, diff.value(), getName());

        std::chrono::duration<double> tot = std::chrono::steady_clock::now() - start_time;
        stats_.total_time += tot.count();

        printStat();

        return base::PlannerStatus::APPROXIMATE_SOLUTION;
    }

    std::chrono::duration<double> tot = std::chrono::steady_clock::now() - start_time;
    stats_.total_time += tot.count();

    printStat();

    return sol ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}

void SDCL::constructRoadmap(const base::PlannerTerminationCondition &ptc)
{
    if (!isSetup())
        setup();
    if (!sampler_)
        sampler_ = si_->allocValidStateSampler();
    if (!simpleSampler_)
        simpleSampler_ = si_->allocStateSampler();

    std::vector<base::State *> xstates(magic::MAX_RANDOM_BOUNCE_STEPS);
    si_->allocStates(xstates);
    bool grow = true;

    bestCost_ = opt_->infiniteCost();
    while (!ptc())
    {
        // maintain a 2:1 ratio for growing/expansion of roadmap
        // call growRoadmap() twice as long for every call of expandRoadmap()
        if (grow)
            growRoadmap(base::plannerOrTerminationCondition(
                            ptc, base::timedPlannerTerminationCondition(2.0 * magic::ROADMAP_BUILD_TIME)),
                        xstates[0]);
        else
            expandRoadmap(base::plannerOrTerminationCondition(
                              ptc, base::timedPlannerTerminationCondition(magic::ROADMAP_BUILD_TIME)),
                          xstates);
        grow = !grow;
    }

    si_->freeStates(xstates);
}


SDCL::Vertex SDCL::addMilestone(base::State *state)
{
    std::lock_guard<std::mutex> _(graphMutex_);

    Vertex m = boost::add_vertex(g_);
    stateProperty_[m] = state;
    totalConnectionAttemptsProperty_[m] = 1;
    successfulConnectionAttemptsProperty_[m] = 0;

    // Initialize to its own (dis)connected component.
    disjointSets_.make_set(m);

    // Which milestones will we attempt to connect to?
    const std::vector<Vertex> &neighbors = connectionStrategy_(m);

    foreach (Vertex n, neighbors)
        if (connectionFilter_(n, m))
        {
            totalConnectionAttemptsProperty_[m]++;
            totalConnectionAttemptsProperty_[n]++;
            if (si_->checkMotion(stateProperty_[n], stateProperty_[m]))
            {
                successfulConnectionAttemptsProperty_[m]++;
                successfulConnectionAttemptsProperty_[n]++;
                const base::Cost weight = opt_->motionCost(stateProperty_[n], stateProperty_[m]);
                const Graph::edge_property_type properties(weight);
                boost::add_edge(n, m, properties, g_);
                uniteComponents(n, m);
            }
        }

    nn_->add(m);

    return m;
}

void SDCL::uniteComponents(Vertex m1, Vertex m2)
{
    disjointSets_.union_set(m1, m2);
}

bool SDCL::sameComponent(Vertex m1, Vertex m2)
{
    return boost::same_component(m1, m2, disjointSets_);
}

ompl::base::Cost SDCL::constructApproximateSolution(const std::vector<Vertex> &starts,
                                                                    const std::vector<Vertex> &goals,
                                                                    base::PathPtr &solution)
{
    std::lock_guard<std::mutex> _(graphMutex_);
    base::Goal *g = pdef_->getGoal().get();
    base::Cost closestVal(opt_->infiniteCost());
    bool approxPathJustStart = true;

    foreach (Vertex start, starts)
    {
        foreach (Vertex goal, goals)
        {
            base::Cost heuristicCost(costHeuristic(start, goal));
            if (opt_->isCostBetterThan(heuristicCost, closestVal))
            {
                closestVal = heuristicCost;
                approxPathJustStart = true;
            }
            if (!g->isStartGoalPairValid(stateProperty_[goal], stateProperty_[start]))
            {
                continue;
            }
            base::PathPtr p;
            boost::vector_property_map<Vertex> prev(boost::num_vertices(g_));
            boost::vector_property_map<base::Cost> dist(boost::num_vertices(g_));
            boost::vector_property_map<base::Cost> rank(boost::num_vertices(g_));

            try
            {
                // Consider using a persistent distance_map if it's slow
                boost::astar_search(
                    g_, start, [this, goal](Vertex v) { return costHeuristic(v, goal); },
                    boost::predecessor_map(prev)
                        .distance_map(dist)
                        .rank_map(rank)
                        .distance_compare(
                            [this](base::Cost c1, base::Cost c2) { return opt_->isCostBetterThan(c1, c2); })
                        .distance_combine([this](base::Cost c1, base::Cost c2) { return opt_->combineCosts(c1, c2); })
                        .distance_inf(opt_->infiniteCost())
                        .distance_zero(opt_->identityCost())
                        .visitor(AStarGoalVisitor<Vertex>(goal)));
            }
            catch (AStarFoundGoal &)
            {
            }

            Vertex closeToGoal = start;
            for (auto vp = vertices(g_); vp.first != vp.second; vp.first++)
            {
                // We want to get the distance of each vertex to the goal.
                // Boost lets us get cost-to-come, cost-to-come+dist-to-goal,
                // but not just dist-to-goal.
                ompl::base::Cost dist_to_goal(costHeuristic(*vp.first, goal));
                if (opt_->isFinite(rank[*vp.first]) && opt_->isCostBetterThan(dist_to_goal, closestVal))
                {
                    closeToGoal = *vp.first;
                    closestVal = dist_to_goal;
                    approxPathJustStart = false;
                }
            }
            if (closeToGoal != start)
            {
                auto p(std::make_shared<ompl::geometric::PathGeometric>(si_));
                for (Vertex pos = closeToGoal; prev[pos] != pos; pos = prev[pos])
                    p->append(stateProperty_[pos]);
                p->append(stateProperty_[start]);
                p->reverse();

                solution = p;
            }
        }
    }
    if (approxPathJustStart)
    {
        return opt_->infiniteCost();
    }
    return closestVal;
}

ompl::base::PathPtr SDCL::constructSolution(const Vertex &start, const Vertex &goal)
{
    std::lock_guard<std::mutex> _(graphMutex_);
    boost::vector_property_map<Vertex> prev(boost::num_vertices(g_));

    try
    {
        // Consider using a persistent distance_map if it's slow
        boost::astar_search(
            g_, start, [this, goal](Vertex v) { return costHeuristic(v, goal); },
            boost::predecessor_map(prev)
                .distance_compare([this](base::Cost c1, base::Cost c2) { return opt_->isCostBetterThan(c1, c2); })
                .distance_combine([this](base::Cost c1, base::Cost c2) { return opt_->combineCosts(c1, c2); })
                .distance_inf(opt_->infiniteCost())
                .distance_zero(opt_->identityCost())
                .visitor(AStarGoalVisitor<Vertex>(goal)));
    }
    catch (AStarFoundGoal &)
    {
    }

    if (prev[goal] == goal)
        throw Exception(name_, "Could not find solution path");

    auto p(std::make_shared<ompl::geometric::PathGeometric>(si_));
    for (Vertex pos = goal; prev[pos] != pos; pos = prev[pos])
        p->append(stateProperty_[pos]);
    p->append(stateProperty_[start]);
    p->reverse();

    return p;
}

void SDCL::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    // Explicitly add start and goal states:
    for (unsigned long i : startM_)
        data.addStartVertex(
            base::PlannerDataVertex(stateProperty_[i], const_cast<SDCL *>(this)->disjointSets_.find_set(i)));

    for (unsigned long i : goalM_)
        data.addGoalVertex(
            base::PlannerDataVertex(stateProperty_[i], const_cast<SDCL *>(this)->disjointSets_.find_set(i)));

    // Adding edges and all other vertices simultaneously
    foreach (const Edge e, boost::edges(g_))
    {
        const Vertex v1 = boost::source(e, g_);
        const Vertex v2 = boost::target(e, g_);
        data.addEdge(base::PlannerDataVertex(stateProperty_[v1]), base::PlannerDataVertex(stateProperty_[v2]));

        // Add the reverse edge, since we're constructing an undirected roadmap
        data.addEdge(base::PlannerDataVertex(stateProperty_[v2]), base::PlannerDataVertex(stateProperty_[v1]));

        // Add tags for the newly added vertices
        data.tagState(stateProperty_[v1], const_cast<SDCL *>(this)->disjointSets_.find_set(v1));
        data.tagState(stateProperty_[v2], const_cast<SDCL *>(this)->disjointSets_.find_set(v2));
    }
}

ompl::base::Cost SDCL::costHeuristic(Vertex u, Vertex v) const
{
    return opt_->motionCostHeuristic(stateProperty_[u], stateProperty_[v]);
}




// -------------------------------------------------- SDCL part -------------------------------------------------------

void SDCL::generateSamples(const base::PlannerTerminationCondition &ptc)
{
    // SDCL main iteration
    int itr = 1;

    trainingSetup();

    while (!ptc())
    {
        OMPL_INFORM("------------------------SDCL Main Iteration %d -------------------------", itr);

        // make training data from graph
        graphMutex_.lock();
        int cur_training_size = boost::num_vertices(g_);
        graphMutex_.unlock();

        makeTrainingDataFromGraph(cur_training_size);

        if (n_goal_points_ == 0 || n_start_points_ == 0) continue;

        trainManifold();

        if (ptc()) break;

        sampleManifoldPoints(ptc);

        itr++;
    }
    stats_.n_itr = itr;
}

void SDCL::makeDatesetAfterAddingFreeManifoldPoints()
{
    // make input data set from new training data. 
    int training_size = X_->size();
    
    float* classes = new float[training_size];
    int features = si_->getStateDimension();
    float* data = new float[training_size * features];

    int n_goal_points = 0;
    int n_start_points = 0;
    
    for(int i; i < training_size; i++)
    {
        for (int j = 0; j < features; j++)
        {
            data[features * i + j] = (*X_)[i][j];
        }

        classes[i] = (*Y_)[i];
        if ((*Y_)[i] == 1) n_start_points++;
        else n_goal_points++;
    }

    dataset_.load_from_dense(training_size, features, data, classes);

    OMPL_INFORM("Total number of training points is %d, number of goal points is %d, number of start points is %d", training_size, n_goal_points, n_start_points);

    delete [] data;
    delete [] classes;

}

void SDCL::sampleManifoldPoints(const base::PlannerTerminationCondition &ptc)
{
    auto start = std::chrono::steady_clock::now();

    addedToTraining_ = 0;

    // start thread pool
    int num_threads = std::thread::hardware_concurrency() - 4;

    if (!collisionPoints_) collisionPoints_.reset(new pvec()); // avoid segfault when no collision points are added. 

    // need a copy of the collision points for read and write conflicts.
    pvec collision_copy;
    collisionPointsMutex_.lock();
    int last_num_collision_points = collisionPoints_->size();
    copy(collisionPoints_->begin(), collisionPoints_->end(), back_inserter(collision_copy));
    collisionPointsMutex_.unlock();

    int num_training_points = X_->size();
    
    OMPL_INFORM("Thread pool for calculating manifold points has %d threads.", num_threads);
    boost::asio::thread_pool threadpool(num_threads);
    
    // loop to add thread pool
    for (int i = 0; i < last_num_collision_points; i++) {
        if (ptc()) break;
        boost::asio::post(threadpool, boost::bind(&SDCL::calManifoldPoints, this, collision_copy[i]));
    }
    // also use training points to sample manifold points. 
    if (use_training_) {
        for (int i = 0; i < num_training_points; i++) {
            if (ptc()) break;
            boost::asio::post(threadpool, boost::bind(&SDCL::calManifoldPoints, this, (*X_)[i]));
        }
    }

    threadpool.join();

    OMPL_INFORM("There are %d collision points, %d training points, generated %d free manifold points. ", 
               last_num_collision_points, num_training_points, addedToTraining_);

    stats_.n_valid_mpoints += addedToTraining_;
    stats_.n_collision_points = last_num_collision_points;

    OMPL_INFORM("A total of %d free manifold points used. ", stats_.n_valid_mpoints);

    std::chrono::duration<double> diff_sampling = std::chrono::steady_clock::now() - start;
    stats_.sampling_time += diff_sampling.count();
}

void SDCL::calManifoldPoints(pt intput_point){
    const unsigned int dim = si_->getStateDimension();
    double res_data[dim] = {};
    double cpoint[dim] = {};
    double mpoint[dim] = {};
    for (int ii = 0; ii < dim; ii++) {
        cpoint[ii] = intput_point[ii];
        mpoint[ii] = intput_point[ii];
    }

    // opt to find closest point on manifold
    int success = findClosestPoint(cpoint, mpoint, res_data, dim, savedModelData_, lower_bound_, upper_bound_);

    double svm_value = evaluate(res_data);
    base::State *res_state = si_->allocState();
    auto *rstate = static_cast<base::RealVectorStateSpace::StateType *>(res_state);

    if (success > 0 && abs(svm_value) < 0.001) {
        pt res_point(dim, 0); 
        for (int ii = 0; ii < dim; ii++) {
            res_point[ii] = res_data[ii];
            rstate->values[ii] = res_data[ii];
        }

        if (si_->isValid(rstate)) {
            addToTraining(si_->cloneState(res_state), res_point); // pay attention to whether the state needs to be copied. 
        }
    } 

    si_->freeState(res_state);
}

void SDCL::addToTraining(base::State* state, pt point) 
{
    // this is the addmilestone part.
    std::lock_guard<std::mutex> _(graphMutex_);

    Vertex m = boost::add_vertex(g_);
    stateProperty_[m] = state;
    totalConnectionAttemptsProperty_[m] = 1;
    successfulConnectionAttemptsProperty_[m] = 0;

    // Initialize to its own (dis)connected component.
    disjointSets_.make_set(m);

    // Which milestones will we attempt to connect to? 
    const std::vector<Vertex> &neighbors = connectionStrategy_(m);

    foreach (Vertex n, neighbors)
        if (connectionFilter_(n, m))
        {
            totalConnectionAttemptsProperty_[m]++;
            totalConnectionAttemptsProperty_[n]++;
            if (si_->checkMotion(stateProperty_[n], stateProperty_[m]))
            {
                successfulConnectionAttemptsProperty_[m]++;
                successfulConnectionAttemptsProperty_[n]++;
                const base::Cost weight = opt_->motionCost(stateProperty_[n], stateProperty_[m]);
                const Graph::edge_property_type properties(weight);
                boost::add_edge(n, m, properties, g_);
                uniteComponents(n, m);
            }
        }

    nn_->add(m);

    addedToTraining_++;
}

void SDCL::save2dPoints(pt point, std::ostream& output) {
    for (int ii = 0; ii < point.size(); ii++) {
        output << point[ii] << " ";
    }
    output << std::endl;
}

void SDCL::trainManifold()
{
    // train the manifold using graph points
    auto start = std::chrono::steady_clock::now();

    // train svm
    model_->train(dataset_, param_);

    saveModelData();

    std::chrono::duration<double> diff_training = std::chrono::steady_clock::now() - start;
    stats_.training_time += diff_training.count();
}

void SDCL::trainingSetup()
{
    model_.reset(new SVC());
    param_.kernel_type = SvmParam::RBF;
    param_.degree = 3;
    param_.gamma = 1.0;
    param_.coef0 = 0;
    param_.nu = 0.5;
    param_.C = 100;
    param_.epsilon = 1e-3;
    param_.p = 0.1;
    param_.probability = 0;
    param_.nr_weight = 0;
    param_.weight_label = NULL;
    param_.weight = NULL;

    // thunderSVM logging config. 
    el::Loggers::addFlag(el::LoggingFlag::HierarchicalLogging);
    el::Loggers::setLoggingLevel(el::Level::Unknown);

    stats_.useGaussian = use_Gaussian_;
    stats_.useTraining = use_training_;
}

void SDCL::makeTrainingDataFromGraph(unsigned long int training_size) 
{
    // make input data set from graph disjoint set. 
    std::lock_guard<std::mutex> _(graphMutex_);

    // training_size += 1;
    int data_size = training_size;

    float* classes = new float[data_size];
    int features = si_->getStateDimension();
    float* data = new float[data_size * features];
    int cur_index = 0;
    
    X_.reset(new pvec(data_size, pt(features, 0.0)));
    Y_.reset(new label(data_size, 0));
    n_goal_points_ = 0;
    n_start_points_ = 0;

    bool inGoal = false;
    
    foreach (Vertex v, boost::vertices(g_))
    {
        Vertex cur_set = disjointSets_.find_set(v);
        base::State* s = stateProperty_[v];

        for (int j = 0; j < features; j++)
        {
            data[features * cur_index + j] = (float)s->as<base::RealVectorStateSpace::StateType>()->values[j];
            (*X_)[cur_index][j] = (double)s->as<base::RealVectorStateSpace::StateType>()->values[j];
        }

        inGoal = false;

        foreach (Vertex goal, goalM_)
        {   
            Vertex goal_representation = disjointSets_.find_set(goal);
            if (cur_set == goal_representation)
            {
                inGoal = true;
                break;
            }
        }

        if (inGoal) 
        {
            classes[cur_index] = -1;
            (*Y_)[cur_index] = -1;
            n_goal_points_++;
        } else
        {
            classes[cur_index] = 1;
            (*Y_)[cur_index] = 1;
            n_start_points_++;
        }

        cur_index++;

        if (cur_index == training_size) break;
    }
    
    dataset_.load_from_dense(training_size, features, data, classes);

    OMPL_INFORM("Number of goal points is %d, number of start points is %d", n_goal_points_, n_start_points_);

    delete [] data;
    delete [] classes;
}


void SDCL::saveCollisionPoints(base::State *workState) 
{
    if (!collisionPoints_) collisionPoints_.reset(new pvec());

    const unsigned int dim = si_->getStateDimension();
    pt point(dim);
    for (int i = 0; i < dim; i++) {
        point[i] = (double)workState->as<base::RealVectorStateSpace::StateType>()->values[i];
    }
    collisionPointsMutex_.lock();
    collisionPoints_->push_back(point);
    collisionPointsMutex_.unlock();
}


double SDCL::evaluate(double* point) {
    if (savedModelData_.num_vectors == 0) saveModelData();

    double f = 0;
    double dists_square[savedModelData_.num_vectors];
    int features = si_->getStateDimension();

    for(int k = 0; k < savedModelData_.num_vectors; k++){
        dists_square[k] = 0;
        for(int i = 0; i < features; i++){
            dists_square[k] += pow(point[i] - savedModelData_.vectors[features*k+i], 2);
        }
        f += savedModelData_.coef[k] * exp(-savedModelData_.gamma * dists_square[k]);
    }

    return f-savedModelData_.b; 
}

bool SDCL::testTraining() {
    std::vector<double> predict_y = model_->predict(dataset_.instances(), -1);
    std::shared_ptr<Metric> metric;
    metric.reset(new Accuracy());
    double acc = metric->score(predict_y, dataset_.y());

    LOG(INFO) << metric->name() << " = " << acc;
    if (acc != 1.0) return false;
    else return true;
}

void SDCL::saveModelData() {
    if (!savedModelData_.vectors) delete [] savedModelData_.vectors;

    if (!savedModelData_.coef) delete [] savedModelData_.coef;

    const float_type* rho_data = (model_->get_rho()).host_data();
    DataSet::node2d vectors = model_->svs();
    const float_type *coef_data = (model_->get_coef()).host_data();
    int features = si_->getStateDimension();

    savedModelData_ = ModelData();
    savedModelData_.b = rho_data[0];
    savedModelData_.num_vectors = model_->total_sv();
    savedModelData_.gamma = param_.gamma;
    savedModelData_.coef = new double[savedModelData_.num_vectors];
    savedModelData_.vectors = new double[savedModelData_.num_vectors * features];
    for (int i = 0; i < savedModelData_.num_vectors; i++) {
        for (int j = 0; j < features; j++) {
            savedModelData_.vectors[i * features + j] = vectors[i][j].value;
        }
        savedModelData_.coef[i] = coef_data[i];
    }
}

void SDCL::printStat() {
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%d-%m-%Y-%H-%M-%S");
    auto str = oss.str();
    std::ofstream csv_file("stat_output" + str + "_training.csv");
    csv_file << "Solved, Total planning time, itr, training time, sampling time, free mani-points, collision points, training points, use Gaussian, use Training" << std::endl;
    csv_file << stats_.solved 
             << ", " << stats_.total_time
             << ", " << stats_.n_itr
             << ", " <<  stats_.training_time 
             << ", " << stats_.sampling_time 
             << ", " << stats_.n_valid_mpoints 
             << ", " <<  stats_.n_collision_points
             << ", " << (int)X_->size()
             << ", " << stats_.useGaussian
             << ", " << stats_.useTraining << std::endl;
    csv_file.close();
}

bool SDCL::sampleAndSaveCollisionPoints(base::State* workState, bool use_Gaussian)
{
    bool found = false;
    // Gaussian sampling
    if (use_Gaussian)
    {
        unsigned int tries = 0;
        base::State *temp = si_->allocState();
        do
        {
            simpleSampler_->sampleUniform(workState);
            bool v1 = si_->isValid(workState);
            simpleSampler_->sampleGaussian(temp, workState, stddev_);
            bool v2 = si_->isValid(temp);
            if (v1 != v2)
            {
                if (v2)
                {
                    saveCollisionPoints(workState);
                    si_->copyState(workState, temp);
                } else 
                {
                    saveCollisionPoints(temp);
                }

                found = true;
            }
            ++tries;
        } while (!found && tries < magic::MAX_VALID_SAMPLE_ATTEMPTS);
        si_->freeState(temp);
    } 
    else  // Uniform Sampling
    {
        unsigned int tries = 0;
        do
        {
            simpleSampler_->sampleUniform(workState);
            found = si_->isValid(workState);
            // save collision points
            if (!found) saveCollisionPoints(workState);
            ++tries;
        } while (!found && tries < magic::MAX_VALID_SAMPLE_ATTEMPTS);
    }

    return found;
}
