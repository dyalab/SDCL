#ifndef SDCL_H
#define SDCL_H

#include "ompl/geometric/planners/PlannerIncludes.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include "ompl/base/ValidStateSampler.h"
#include "ompl/base/StateSampler.h"
#include "ompl/base/SpaceInformation.h"
#include <ompl/config.h>
#include "ompl/geometric/planners/prm/ConnectionStrategy.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/datastructures/PDF.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/tools/config/MagicConstants.h"

#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <boost/asio.hpp>
#include <boost/asio/thread_pool.hpp>
#include <boost/foreach.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/pending/disjoint_sets.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/property_map/vector_property_map.hpp>
#include <boost/graph/copy.hpp>

#include <thundersvm/svmparam.h>
#include <thundersvm/syncarray.h>
#include <thundersvm/model/svc.h>
#include <thundersvm/util/metric.h>
#include <thundersvm/util/log.h>

#include <flann/flann.hpp>

#include <nlopt.h>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <iostream>
#include <set>
#include <math.h>
#include <mutex>
#include <chrono>
#include <filesystem>
#include <atomic>
#include <fstream>
#include <thread>
#include <typeinfo>

using namespace ompl;

struct AStarFoundGoal
{
};  // exception for termination

// visitor that terminates when we find the goal
// V is the vertex type
template <typename V>
class AStarGoalVisitor : public boost::default_astar_visitor
{
public:
    AStarGoalVisitor(const V &goal) : goal_(goal)
    {
    }

    // G is the graph type
    template <typename G>
    void examine_vertex(const V &u, const G & /*unused*/)
    {
        if (u == goal_)
            throw AStarFoundGoal();
    }

private:
    V goal_;
};

struct proof_stats {
    int solved = 0;
    int n_facets = 0;
    int n_collision_points = 0;
    int n_valid_mpoints = 0;
    int n_itr = 0;
    double tc_time = 0;
    double mpoints_time = 0;
    double check_time = 0;
    double total_time = 0;
    double training_time = 0;
    double sampling_time = 0;
    bool useGaussian = false;
    bool useTraining = false;
};

struct ModelData{
    ModelData(){
        b = 0;
        num_vectors = 0;
        gamma = 0;
        coef = NULL;
        vectors = NULL;
    };
    double b;
    int num_vectors;
    double gamma;
    double* coef;
    double* vectors;
};

double myconstraint(unsigned n, const double *x, double *grad, void *data);
double objfunc(unsigned n, const double *x, double *grad, void *data);
int findClosestPoint(double *p, double *startingP, double *res, int n, ModelData svm_data);

/** \brief The SDCL Planner with Probabilistic RoadMap planner as the based planner */
class SDCL : public base::Planner
{
public:

    struct vertex_state_t
    {
        using kind = boost::vertex_property_tag;
    };

    struct vertex_total_connection_attempts_t
    {
        using kind = boost::vertex_property_tag;
    };

    struct vertex_successful_connection_attempts_t
    {
        using kind = boost::vertex_property_tag;
    };

    /**
     @brief The underlying roadmap graph.

     @par Any BGL graph representation could be used here. Because we
     expect the roadmap to be sparse (m<n^2), an adjacency_list is more
     appropriate than an adjacency_matrix.

     @par Obviously, a ompl::base::State* vertex property is required.
     The incremental connected components algorithm requires
     vertex_predecessor_t and vertex_rank_t properties.
     If boost::vecS is not used for vertex storage, then there must also
     be a boost:vertex_index_t property manually added.

     @par Edges should be undirected and have a weight property.
     */
    using Graph = boost::adjacency_list<
        boost::vecS, boost::vecS, boost::undirectedS,
        boost::property<
            vertex_state_t, base::State *,
            boost::property<
                vertex_total_connection_attempts_t, unsigned long int,
                boost::property<vertex_successful_connection_attempts_t, unsigned long int,
                                boost::property<boost::vertex_predecessor_t, unsigned long int,
                                                boost::property<boost::vertex_rank_t, unsigned long int>>>>>,
        boost::property<boost::edge_weight_t, base::Cost>>;

    /** @brief The type for a vertex in the roadmap. */
    using Vertex = boost::graph_traits<Graph>::vertex_descriptor;
    /** @brief The type for an edge in the roadmap. */
    using Edge = boost::graph_traits<Graph>::edge_descriptor;

    /** @brief A nearest neighbors data structure for roadmap vertices. */
    using RoadmapNeighbors = std::shared_ptr<ompl::NearestNeighbors<Vertex>>;

    /** @brief A function returning the milestones that should be
     * attempted to connect to. */
    using ConnectionStrategy = std::function<const std::vector<Vertex> &(const Vertex)>;

    /** @brief A function that can reject connections.

     This is called after previous connections from the neighbor list
     have been added to the roadmap.
     */
    using ConnectionFilter = std::function<bool(const Vertex &, const Vertex &)>;

    /** @brief A data structure for storing points in SDCL part */
    using pt =  std::vector<double>;
    
    /** @brief A data structure for storing vector of points in SDCL part */
    using pvec = std::vector<std::vector<double>>;

    using label = std::vector<int>;

    /** \brief Constructor */
    SDCL(const base::SpaceInformationPtr &si, bool starStrategy = false, bool use_training = true, bool use_Gaussian = false);

    ~SDCL() override;

    void setProblemDefinition(const base::ProblemDefinitionPtr &pdef) override;

    /** \brief Set the connection strategy function that specifies the
     milestones that connection attempts will be make to for a
     given milestone.

     \par The behavior and performance of PRM can be changed drastically
     by varying the number and properties if the milestones that are
     connected to each other.

     \param pdef A function that takes a milestone as an argument and
     returns a collection of other milestones to which a connection
     attempt must be made. The default connection strategy is to connect
     a milestone's 10 closest neighbors.
     */
    void setConnectionStrategy(const ConnectionStrategy &connectionStrategy)
    {
        connectionStrategy_ = connectionStrategy;
        userSetConnectionStrategy_ = true;
    }
    /** Set default strategy for connecting to nearest neighbors */
    void setDefaultConnectionStrategy();

    /** \brief Convenience function that sets the connection strategy to the
     default one with k nearest neighbors.
     */
    void setMaxNearestNeighbors(unsigned int k);

    /** \brief return the maximum number of nearest neighbors to connect a sample to
     *
     * This only returns a meaningful answer if the connection strategy is of type KStrategy.
     */
    unsigned int getMaxNearestNeighbors() const;


    /** \brief Set the function that can reject a milestone connection.

     \par The given function is called immediately before a connection
     is checked for collision and added to the roadmap. Other neighbors
     may have already been connected before this function is called.
     This allows certain heuristics that use the structure of the
     roadmap (like connected components or useful cycles) to be
     implemented by changing this function.

     \param connectionFilter A function that takes the new milestone,
     a neighboring milestone and returns whether a connection should be
     attempted.
     */
    void setConnectionFilter(const ConnectionFilter &connectionFilter)
    {
        connectionFilter_ = connectionFilter;
    }

    void getPlannerData(base::PlannerData &data) const override;

    /** \brief While the termination condition allows, this function will construct the roadmap (using
       growRoadmap() and expandRoadmap(),
        maintaining a 2:1 ratio for growing/expansion of roadmap) */
    void constructRoadmap(const base::PlannerTerminationCondition &ptc);

    /** \brief If the user desires, the roadmap can be
        improved for the given time (seconds). The solve()
        method will also improve the roadmap, as needed.*/
    void growRoadmap(double growTime);

    /** \brief If the user desires, the roadmap can be
        improved until a given condition is true. The solve()
        method will also improve the roadmap, as needed.*/
    void growRoadmap(const base::PlannerTerminationCondition &ptc);

    /** \brief Attempt to connect disjoint components in the roadmap
        using random bouncing motions (the PRM expansion step) for the
        given time (seconds). */
    void expandRoadmap(double expandTime);

    /** \brief Attempt to connect disjoint components in the roadmap
        using random bouncing motions (the PRM expansion step) until the
        given condition evaluates true. */
    void expandRoadmap(const base::PlannerTerminationCondition &ptc);

    /** \brief Function that can solve the motion planning
        problem. Grows a roadmap using
        constructRoadmap(). This function can be called
        multiple times on the same problem, without calling
        clear() in between. This allows the planner to
        continue work for more time on an unsolved problem,
        for example. Start and goal states from the currently
        specified ProblemDefinition are cached. This means
        that between calls to solve(), input states are only
        added, not removed. When using PRM as a multi-query
        planner, the input states should be however cleared,
        without clearing the roadmap itself. This can be done
        using the clearQuery() function. */
    base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

    /** \brief Clear the query previously loaded from the ProblemDefinition.
        Subsequent calls to solve() will reuse the previously computed roadmap,
        but will clear the set of input states constructed by the previous call to solve().
        This enables multi-query functionality for PRM. */
    void clearQuery() override;

    void clear() override;

    /** \brief Set a different nearest neighbors datastructure */
    template <template <typename T> class NN>
    void setNearestNeighbors()
    {
        if (nn_ && nn_->size() == 0)
            OMPL_WARN("Calling setNearestNeighbors will clear all states.");
        clear();
        nn_ = std::make_shared<NN<Vertex>>();
        if (!userSetConnectionStrategy_)
            setDefaultConnectionStrategy();
        if (isSetup())
            setup();
    }

    void setup() override;

    const Graph &getRoadmap() const
    {
        return g_;
    }

    /** \brief Return the number of milestones currently in the graph */
    unsigned long int milestoneCount() const
    {
        return boost::num_vertices(g_);
    }

    /** \brief Return the number of edges currently in the graph */
    unsigned long int edgeCount() const
    {
        return boost::num_edges(g_);
    }

    const RoadmapNeighbors &getNearestNeighbors()
    {
        return nn_;
    }

protected:
    /** \brief Free all the memory allocated by the planner */
    void freeMemory();

    /** \brief Construct a milestone for a given state (\e state), store it in the nearest neighbors data
       structure
        and then connect it to the roadmap in accordance to the connection strategy. */
    Vertex addMilestone(base::State *state);

    /** \brief Make two milestones (\e m1 and \e m2) be part of the same connected component. The component with
     * fewer elements will get the id of the component with more elements. */
    void uniteComponents(Vertex m1, Vertex m2);

    /** \brief Check if two milestones (\e m1 and \e m2) are part of the same connected component. This is not a
     * const function since we use incremental connected components from boost */
    bool sameComponent(Vertex m1, Vertex m2);

    /** \brief Randomly sample the state space, add and connect milestones
         in the roadmap. Stop this process when the termination condition
         \e ptc returns true.  Use \e workState as temporary memory. */
    void growRoadmap(const base::PlannerTerminationCondition &ptc, base::State *workState);

    /** \brief Attempt to connect disjoint components in the
        roadmap using random bounding motions (the PRM
        expansion step) */
    void expandRoadmap(const base::PlannerTerminationCondition &ptc, std::vector<base::State *> &workStates);

    /** Thread that checks for solution */
    void checkForSolution(const base::PlannerTerminationCondition &ptc, base::PathPtr &solution);

    /** \brief Check if there exists a solution, i.e., there exists a pair of milestones such that the first is
     * in \e start and the second is in \e goal, and the two milestones are in the same connected component. If
     * a solution is found, it is constructed in the \e solution argument. */
    bool maybeConstructSolution(const std::vector<Vertex> &starts, const std::vector<Vertex> &goals,
                                base::PathPtr &solution);

    /** \brief (Assuming that there is always an approximate solution), finds an
     * approximate solution. */
    ompl::base::Cost constructApproximateSolution(const std::vector<Vertex> &starts, const std::vector<Vertex> &goals, base::PathPtr &solution);

    /** \brief Returns the value of the addedNewSolution_ member. */
    bool addedNewSolution() const;

    /** \brief Given two milestones from the same connected component, construct a path connecting them and set
     * it as the solution */
    base::PathPtr constructSolution(const Vertex &start, const Vertex &goal);

    /** \brief Given two vertices, returns a heuristic on the cost of the path connecting them.
        This method wraps OptimizationObjective::motionCostHeuristic */
    base::Cost costHeuristic(Vertex u, Vertex v) const;

    /** \brief Compute distance between two milestones (this is simply distance between the states of the
     * milestones) */
    double distanceFunction(const Vertex a, const Vertex b) const
    {
        return si_->distance(stateProperty_[a], stateProperty_[b]);
    }

    ///////////////////////////////////////
    // Planner progress property functions
    std::string getIterationCount() const
    {
        return std::to_string(iterations_);
    }
    std::string getBestCost() const
    {
        return std::to_string(bestCost_.value());
    }
    std::string getMilestoneCountString() const
    {
        return std::to_string(milestoneCount());
    }
    std::string getEdgeCountString() const
    {
        return std::to_string(edgeCount());
    }

    /** \brief Flag indicating whether the default connection strategy is the Star strategy */
    bool starStrategy_;

    /** \brief Sampler user for generating valid samples in the state space */
    base::ValidStateSamplerPtr sampler_;

    /** \brief Sampler user for generating random in the state space */
    base::StateSamplerPtr simpleSampler_;

    /** \brief Nearest neighbors data structure */
    RoadmapNeighbors nn_;

    /** \brief Connectivity graph */
    Graph g_;

    /** \brief Array of start milestones */
    std::vector<Vertex> startM_;

    /** \brief Array of goal milestones */
    std::vector<Vertex> goalM_;

    /** \brief Access to the internal base::state at each Vertex */
    boost::property_map<Graph, vertex_state_t>::type stateProperty_;

    /** \brief Access to the number of total connection attempts for a vertex */
    boost::property_map<Graph, vertex_total_connection_attempts_t>::type totalConnectionAttemptsProperty_;

    /** \brief Access to the number of successful connection attempts for a vertex */
    boost::property_map<Graph, vertex_successful_connection_attempts_t>::type
        successfulConnectionAttemptsProperty_;

    /** \brief Access to the weights of each Edge */
    boost::property_map<Graph, boost::edge_weight_t>::type weightProperty_;

    /** \brief Data structure that maintains the connected components */
    boost::disjoint_sets<boost::property_map<Graph, boost::vertex_rank_t>::type,
                         boost::property_map<Graph, boost::vertex_predecessor_t>::type> disjointSets_;

    /** \brief Function that returns the milestones to attempt connections with */
    ConnectionStrategy connectionStrategy_;

    /** \brief Function that can reject a milestone connection */
    ConnectionFilter connectionFilter_;

    /** \brief Flag indicating whether the employed connection strategy was set by the user (or defaults are
     * assumed) */
    bool userSetConnectionStrategy_{false};

    /** \brief Random number generator */
    RNG rng_;

    /** \brief A flag indicating that a solution has been added during solve() */
    bool addedNewSolution_{false};

    /** \brief Mutex to guard access to the Graph member (g_) */
    mutable std::mutex graphMutex_;

    /** \brief Objective cost function for PRM graph edges */
    base::OptimizationObjectivePtr opt_;

    //////////////////////////////
    // Planner progress properties
    /** \brief Number of iterations the algorithm performed */
    unsigned long int iterations_{0};
    /** \brief Best cost found so far by algorithm */
    base::Cost bestCost_{std::numeric_limits<double>::quiet_NaN()};

    
    ///////////////////////////////////////
    // SDCL proof part

    /** \brief svm model related data*/
    std::shared_ptr<pvec> X_;
    std::shared_ptr<label> Y_;
    DataSet dataset_;
    SvmParam param_;
    std::shared_ptr<SvmModel> model_;
    ModelData savedModelData_;

    /** \brief count the number of goal and start points. */
    int n_goal_points_{0};
    int n_start_points_{0};

    /** \brief algorihtm runing stats recoding data structure */
    proof_stats stats_ = {};

    /** \brief collision points, saved in growRoadmap, used in sampleManifoldPoints*/
    std::shared_ptr<pvec> collisionPoints_;

    /** \brief collision points mutex*/
    mutable std::mutex collisionPointsMutex_;

    /** \brief Gaussian sampling std. */
    double stddev_;

    /** \brief whether to use training data as seeds for sampling the manifold */
    bool use_training_;

    /** \brief whether to use Gaussian sampling. */
    bool use_Gaussian_;

    /** \brief upper and lower bound used in opt formulation */
    std::vector<double> upper_bound_;
    std::vector<double> lower_bound_;

    /** \brief The number of points added to training dataset when sampling, in a main run */
    unsigned int addedToTraining_{0};

    /** \brief use SDCL proof manifold to generate samples for planning */
    void generateSamples(const base::PlannerTerminationCondition &ptc);

    /** \brief make training data set from graph disjoint set*/
    void makeTrainingDataFromGraph(unsigned long int training_size);

    /** \brief In a main run, make dateset after new training points are added */
    void makeDatesetAfterAddingFreeManifoldPoints();

    /** \brief train manifold */
    void trainManifold();

    /** \brief Setup training parameters */
    void trainingSetup();

    /** \brief Returns true if training accuracy == 1 */
    bool testTraining();

    /** \brief sampling points on manifold */
    void sampleManifoldPoints(const base::PlannerTerminationCondition &ptc);

    /** \brief save state to collision points */
    void saveCollisionPoints(base::State *workState);

    /** \brief calculate manifold points */
    void calManifoldPoints(pt intput_point);

    /** \brief calcualte the value of the manifold function with given point */
    double evaluate(double* point);

    /** \brief save model data to data structure */
    void saveModelData();

    /** \brief when a free manifold point is found, add the point to the roadmap and the current training set */
    void addToTraining(base::State* state, pt point);

    /** \brief Save a point to an output stream */
    void save2dPoints(pt point, std::ostream& output);

    /** \brief Save running stats to a csv file */
    void printStat();

    /** \brief replace sampler in growRoadmap, since we need to save all the collision points */
    bool sampleAndSaveCollisionPoints(base::State* workState, bool use_Gaussian = false);
};

#endif
