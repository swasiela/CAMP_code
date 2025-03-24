/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2024, LAAS-CNRS
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

/* Author: Simon WASIELA */

// Planner
#include "planning/planners/RandUPRRT.h"

// State Space
#include "planning/state_spaces/RobustStateSpace.h"

// OMPL
#include "ompl/base/Goal.h"
#include "ompl/base/goals/GoalState.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/base/samplers/InformedStateSampler.h"
#include "ompl/base/samplers/informed/RejectionInfSampler.h"
#include "ompl/base/samplers/informed/OrderedInfSampler.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/util/GeometricEquations.h"
#include "ompl/base/goals/GoalSampleableRegion.h"

#define BOUND_TO_01(v) ((v<0)? (0): ( (v>1)? (1): (v)))

ompl::geometric::RandUPRRT::RandUPRRT(const base::SpaceInformationPtr &si, const BulletVisualToolsPtr &visual_tools, const std::string &ros_namespace, bool addIntermediateStates)
  : base::Planner(si, "RandUPRRT"), visual_tools_(visual_tools), ros_namespace_(ros_namespace)
{
    specs_.approximateSolutions = true;
    specs_.directed = true;

    Planner::declareParam<double>("range", this, &RandUPRRT::setRange, &RandUPRRT::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &RandUPRRT::setGoalBias, &RandUPRRT::getGoalBias, "0.:.05:1.");
    Planner::declareParam<double>("padding", this, &RandUPRRT::setPaddingValue, &RandUPRRT::getPaddingValue, "0.:.05:1.");
    Planner::declareParam<int>("nb_particles", this, &RandUPRRT::setNbParticles, &RandUPRRT::getNbParticles, "1:1:100");
    Planner::declareParam<bool>("intermediate_states", this, &RandUPRRT::setIntermediateStates, &RandUPRRT::getIntermediateStates,
                                "0,1");     
    addIntermediateStates_ = addIntermediateStates;
}

ompl::geometric::RandUPRRT::~RandUPRRT()
{
    freeMemory();
}

void ompl::geometric::RandUPRRT::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalMotion_ = nullptr;

    bestCost_ = base::Cost(std::numeric_limits<double>::quiet_NaN());
    prunedCost_ = base::Cost(std::numeric_limits<double>::quiet_NaN());
}

void ompl::geometric::RandUPRRT::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    // Initialize the particles and uncertainties
    assert(nbParticles_ > 0); // The nominal case is the first particle

    // Get the current CAMP path 
    std::filesystem::path campDir_;
    std::filesystem::path sourceFilePath = __FILE__; 
    std::filesystem::path currentDir = sourceFilePath.parent_path();
    // Traverse upwards until the root of the filesystem
    while (currentDir.has_parent_path()) {
        if (currentDir.filename() == "CAMP") {  // Adjust this condition if "CAMP" is not the directory name
            campDir_ = currentDir;
            break;
        }
        currentDir = currentDir.parent_path();
    }

    if (campDir_.empty()) {
        ROS_ERROR("CAMP directory not found.");
    }

    // Node handler
    ros::NodeHandle nh_(ros_namespace_);
    if(!nh_.getParam("planning_parameters/visualize_tree", visualize_tree_))
    {
        ROS_WARN("Value of visualize_tree not found. Using default visualize_tree_ = false.");
        visualize_tree_ = false;
    }
    if(!nh_.getParam("planning_parameters/iterations_file", iteration_file_))
    {
        ROS_WARN("Value of iterations_file not found. Using default iterations_file.");
        iteration_file_ = "CAMP/src/results/iterations_file.txt";
    }
    else
    {
        // Combine the CAMP directory with the relative file path
        std::filesystem::path fullPath = std::filesystem::absolute(campDir_ / iteration_file_);

        // Normalize the path to remove redundant ".." and "."
        fullPath = fullPath.lexically_normal();

        // Convert to string and use the full path
        iteration_file_ = fullPath.string();
    }

    const std::string key_params = "robot_parameters/";

    // Load the nominal values of the uncertain parameters
    std::vector<double> nominal_case; // Nominal parameters (mass, gx, gy, Jx, Jy, Jz)
    if(!nh_.getParam(key_params + "uncertain_params", nominal_case)){
        ROS_WARN("Cannot load 'uncertain_params' param from ROS server: default nominal_case = [0.0]");
        nominal_case = {0.0};
    }
    rand_up_params_.push_back(nominal_case);

    // Load the uncertainty range
    if(!nh_.getParam(key_params + "delta_p", delta_p_)){
        ROS_WARN("Cannot load 'delta_p' param from ROS server: default delta_p_ = [0.0]");
        delta_p_ = {0.0};
    }
    assert(delta_p_.size() == nominal_case.size());

    // Initialize each particle with uncertain parameters
    for(int i = 1; i<nbParticles_; i++)
    {
        std::vector<double> rand_up_particle;
        for(int j = 0; j<delta_p_.size(); j++)
        {
            if (nominal_case.at(j) == 0.0)
                rand_up_particle.push_back(rng_.uniformReal(-delta_p_.at(j), delta_p_.at(j)));
            else
                rand_up_particle.push_back(nominal_case.at(j)*(1+rng_.uniformReal(-delta_p_.at(j), delta_p_.at(j))));
        }
        rand_up_params_.push_back(rand_up_particle);
    } 
    
    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });

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
            OMPL_INFORM("%s: No optimization objective specified. Defaulting to optimizing path length for the allowed "
                        "planning time.",
                        getName().c_str());
            opt_ = std::make_shared<base::PathLengthOptimizationObjective>(si_);

            // Store the new objective in the problem def'n
            pdef_->setOptimizationObjective(opt_);
        }

        // Set the bestCost_ and prunedCost_ as infinite
        bestCost_ = opt_->infiniteCost();
        prunedCost_ = opt_->infiniteCost();
    }
    else
    {
        OMPL_INFORM("%s: problem definition is not set, deferring setup completion...", getName().c_str());
        setup_ = false;
    }
    // To clear the file 
    std::ofstream ofs;
    ofs.open(iteration_file_, std::ofstream::out | std::ofstream::trunc);
    ofs.close();
}

void ompl::geometric::RandUPRRT::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }
}

ompl::base::PlannerStatus ompl::geometric::RandUPRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    // For visualization
    long goalID;
    if(visualize_tree_)
    {
        base::State *gstate = si_->allocState();
        goal_s->sampleGoal(gstate);
        goalID = visual_tools_->addGoal(gstate);
    }

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_);

        for(int ii = 0; ii<nbParticles_; ii++)
            motion->rand_up_states_.push_back(st->as<ompl::base::RobustStateSpace::StateType>()->nominal_state_);

        si_->copyState(motion->state, st);
        nn_->add(motion);   

        // For visualization
        if(visualize_tree_)
        {
            motion->stateVisualID = visual_tools_->addStart(motion->state);
        }
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
    auto *rmotion = new Motion(si_);
    base::State *rstate = rmotion->state;
    base::State *xstate = si_->allocState();

    // Start Time
	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    int iterations_ = 0;
    while (ptc == false)
    {
        iterations_++;
        /* sample random state (with goal biasing) */
        if ((goal_s != nullptr) && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
            sampler_->sampleUniform(rstate);

        /* find closest state in the tree */
        Motion *nmotion = nn_->nearest(rmotion);
        base::State *dstate = rstate;

        /* find state to add */
        double d = si_->distance(nmotion->state, rstate);
        if (d > maxDistance_)
        {
            si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
            dstate = xstate;
        }

        std::vector<std::vector<double>> propagated_states_;
        if (si_->checkMotionRandUp(nmotion->state, dstate, nmotion->rand_up_states_, rand_up_params_, propagated_states_, padding_))
        {
            if (addIntermediateStates_)
            {
                std::vector<base::State *> states;
                const unsigned int count = si_->getStateSpace()->validSegmentCount(nmotion->state, dstate);

                if (si_->getMotionStates(nmotion->state, dstate, states, count, true, true))
                    si_->freeState(states[0]);

                for (std::size_t i = 1; i < states.size(); ++i)
                {
                    auto *motion = new Motion;
                    motion->state = states[i];
                    motion->parent = nmotion;
                    motion->incCost = opt_->motionCost(nmotion->state, motion->state);
                    motion->cost = opt_->combineCosts(nmotion->cost, motion->incCost);
                    motion->rand_up_states_ = propagated_states_;
                    nn_->add(motion);

                    // For visualization
                    if(visualize_tree_)
                    {
                        motion->stateVisualID = visual_tools_->addState(motion->state);
                        motion->edgeVisualID = visual_tools_->addEdge(nmotion->state, motion->state);
                    }

                    nmotion = motion;
                }
            }
            else
            {
                auto *motion = new Motion(si_);
                si_->copyState(motion->state, dstate);
                motion->parent = nmotion;
                motion->incCost = opt_->motionCost(nmotion->state, motion->state);
                motion->cost = opt_->combineCosts(nmotion->cost, motion->incCost);
                motion->rand_up_states_ = propagated_states_;
                nn_->add(motion);

                // For visualization
                if(visualize_tree_)
                {
                    motion->stateVisualID = visual_tools_->addState(motion->state);
                    motion->edgeVisualID = visual_tools_->addEdge(nmotion->state, motion->state);
                }

                nmotion = motion;
            }

            double dist = 0.0;
            bool sat = goal->isSatisfied(nmotion->state, &dist);
            if (sat)
            {
                approxdif = dist;
                solution = nmotion;
                break;
            }
            if (dist < approxdif)
            {
                approxdif = dist;
                approxsol = nmotion;
            }
        }
        std::chrono::steady_clock::time_point end_iter = std::chrono::steady_clock::now();
        exportIter(iterations_, (std::chrono::duration_cast<std::chrono::microseconds>(end_iter - begin).count()));
    }

    bool solved = false;
    bool approximate = false;
    
    if (solution == nullptr)
    {
        solution = approxsol;
        approximate = true;
    }

    bestCost_= solution->cost;
    if (solution != nullptr)
    {
        lastGoalMotion_ = solution;

        /* construct the solution path */
        std::vector<Motion *> mpath;
        while (solution != nullptr)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        /* set the solution path */
        auto path(std::make_shared<PathGeometric>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            path->append(mpath[i]->state);
        pdef_->addSolutionPath(path, approximate, approxdif, getName());
        solved = true;
    }
    si_->freeState(xstate);
    if (rmotion->state != nullptr)
        si_->freeState(rmotion->state);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());
    OMPL_INFORM("%s: Created %u new states. Final solution cost " "%.2f",
                getName().c_str(), bestCost_.value());
                
    return {solved, approximate};
}

void ompl::geometric::RandUPRRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);

    if (lastGoalMotion_ != nullptr)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state));
        else
            data.addEdge(base::PlannerDataVertex(motion->parent->state), base::PlannerDataVertex(motion->state));
    }
}

void ompl::geometric::RandUPRRT::exportIter(int iteration, double running_time) const
{
    std::ofstream iter_overtime(iteration_file_, std::ios::app);
    std::cout.precision(6);

    iter_overtime<< "\t" << "Iteration:" << iteration << ";" << "Running time:" << running_time << "\n";

    iter_overtime.close();
}