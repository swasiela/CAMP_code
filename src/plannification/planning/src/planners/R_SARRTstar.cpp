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

#include "planning/planners/R_SARRTstar.h"

#include "ompl/base/Goal.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/goals/GoalState.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/base/samplers/InformedStateSampler.h"
#include "ompl/base/samplers/informed/RejectionInfSampler.h"
#include "ompl/base/samplers/informed/OrderedInfSampler.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/util/GeometricEquations.h"

ompl::geometric::R_SARRTstar::R_SARRTstar(const base::SpaceInformationPtr &si, const BulletVisualToolsPtr &visual_tools, const std::string &ros_namespace)
  : base::Planner(si, "R_SARRTstar"), visual_tools_(visual_tools), ros_namespace_(ros_namespace)
{
    specs_.approximateSolutions = true;
    specs_.optimizingPaths = true;
    specs_.canReportIntermediateSolutions = true;

    Planner::declareParam<double>("range", this, &R_SARRTstar::setRange, &R_SARRTstar::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &R_SARRTstar::setGoalBias, &R_SARRTstar::getGoalBias, "0.:.05:1.");
    Planner::declareParam<double>("rewire_factor", this, &R_SARRTstar::setRewireFactor, &R_SARRTstar::getRewireFactor,
                                  "1.0:0.01:2.0");
    Planner::declareParam<bool>("use_k_nearest", this, &R_SARRTstar::setKNearest, &R_SARRTstar::getKNearest, "0,1");
    Planner::declareParam<bool>("delay_collision_checking", this, &R_SARRTstar::setDelayCC, &R_SARRTstar::getDelayCC, "0,1");
    Planner::declareParam<bool>("tree_pruning", this, &R_SARRTstar::setTreePruning, &R_SARRTstar::getTreePruning, "0,1");
    Planner::declareParam<double>("prune_threshold", this, &R_SARRTstar::setPruneThreshold, &R_SARRTstar::getPruneThreshold,
                                  "0.:.01:1.");
    Planner::declareParam<bool>("pruned_measure", this, &R_SARRTstar::setPrunedMeasure, &R_SARRTstar::getPrunedMeasure, "0,1");
    Planner::declareParam<bool>("informed_sampling", this, &R_SARRTstar::setInformedSampling, &R_SARRTstar::getInformedSampling,
                                "0,1");
    Planner::declareParam<bool>("sample_rejection", this, &R_SARRTstar::setSampleRejection, &R_SARRTstar::getSampleRejection,
                                "0,1");
    Planner::declareParam<bool>("new_state_rejection", this, &R_SARRTstar::setNewStateRejection,
                                &R_SARRTstar::getNewStateRejection, "0,1");
    Planner::declareParam<bool>("use_admissible_heuristic", this, &R_SARRTstar::setAdmissibleCostToCome,
                                &R_SARRTstar::getAdmissibleCostToCome, "0,1");
    Planner::declareParam<bool>("ordered_sampling", this, &R_SARRTstar::setOrderedSampling, &R_SARRTstar::getOrderedSampling,
                                "0,1");
    Planner::declareParam<unsigned int>("ordering_batch_size", this, &R_SARRTstar::setBatchSize, &R_SARRTstar::getBatchSize,
                                        "1:100:1000000");
    Planner::declareParam<bool>("focus_search", this, &R_SARRTstar::setFocusSearch, &R_SARRTstar::getFocusSearch, "0,1");
    Planner::declareParam<unsigned int>("number_sampling_attempts", this, &R_SARRTstar::setNumSamplingAttempts,
                                        &R_SARRTstar::getNumSamplingAttempts, "10:10:100000");

    addPlannerProgressProperty("iterations INTEGER", [this] { return numIterationsProperty(); });
    addPlannerProgressProperty("best cost REAL", [this] { return bestCostProperty(); });
}

ompl::geometric::R_SARRTstar::~R_SARRTstar()
{
    freeMemory();
}

void ompl::geometric::R_SARRTstar::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

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

    // Get the measure of the entire space:
    prunedMeasure_ = si_->getSpaceMeasure();

    // Calculate some constants:
    calculateRewiringLowerBounds();

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
    if(!nh_.getParam("planning_parameters/iterations_file", iteration_file_))
    {
        ROS_WARN("Value of iterations_file not found. Using default iterations_file.");
        iteration_file_ = "CAMP/src/Results/iterations_file.txt";
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
    if(!nh_.getParam("planning_parameters/cost_file", cost_file_))
    {
        ROS_WARN("Value of cost_file not found. Using default cost_file.");
        cost_file_ = "CAMP/src/Results/cost_filegenerated.txt";
    }
    else
    {
        // Combine the CAMP directory with the relative file path
        std::filesystem::path fullPath = std::filesystem::absolute(campDir_ / cost_file_);

        // Normalize the path to remove redundant ".." and "."
        fullPath = fullPath.lexically_normal();

        // Convert to string and use the full path
        cost_file_ = fullPath.string();
    }

    std::string cost_id;
    if(!nh_.getParam("planning_parameters/optimization_objective", cost_id))
    {
        ROS_WARN("Value of optimization_objective not found. Using default cost_id.");
        cost_id = "Length";
    }

    if (cost_id == "Sensi")
        sensi_cost_ = true;
    else
        sensi_cost_ = false;
    
    if(!nh_.getParam("planning_parameters/visualize_tree", visualize_tree_))
    {
        ROS_WARN("Value of visualize_tree not found. Using default visualize_tree_ = false.");
        visualize_tree_ = false;
    }

    // To clear the file 
    std::ofstream ofs;
    ofs.open(cost_file_, std::ofstream::out | std::ofstream::trunc);
    ofs.close();
    ofs.open(iteration_file_, std::ofstream::out | std::ofstream::trunc);
    ofs.close();
}

void ompl::geometric::R_SARRTstar::clear()
{
    setup_ = false;
    Planner::clear();
    sampler_.reset();
    infSampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();

    bestGoalMotion_ = nullptr;
    goalMotions_.clear();
    startMotions_.clear();

    iterations_ = 0;
    bestCost_ = base::Cost(std::numeric_limits<double>::quiet_NaN());
    prunedCost_ = base::Cost(std::numeric_limits<double>::quiet_NaN());
    prunedMeasure_ = 0.0;
}

ompl::base::PlannerStatus ompl::geometric::R_SARRTstar::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);
    bool symCost = opt_->isSymmetric();

    // For visualization
    long goalID;
    if(visualize_tree_)
    {
        base::State *gstate = si_->allocState();
        goal_s->sampleGoal(gstate);
        goalID = visual_tools_->addGoal(gstate);
    }

    // Check if there are more starts
    if (pis_.haveMoreStartStates() == true)
    {
        // There are, add them
        while (const base::State *st = pis_.nextStart())
        {
            auto *motion = new Motion(si_);
            si_->copyState(motion->state, st);
            motion->cost = opt_->identityCost();
            nn_->add(motion);
            startMotions_.push_back(motion);

            // For visualization
            if(visualize_tree_)
            {
                motion->stateVisualID = visual_tools_->addStart(motion->state);
            }
        }

        // And assure that, if we're using an informed sampler, it's reset
        infSampler_.reset();
    }
    // No else

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    // Allocate a sampler if necessary
    if (!sampler_ && !infSampler_)
    {
        allocSampler();
    }

    OMPL_INFORM("%s: Started planning with %u states. Seeking a solution better than %.5f.", getName().c_str(), nn_->size(), opt_->getCostThreshold().value());

    if ((useTreePruning_ || useRejectionSampling_ || useInformedSampling_) &&
        !si_->getStateSpace()->isMetricSpace())
        OMPL_WARN("%s: The state space (%s) is not metric and as a result the optimization objective may not satisfy "
                  "the triangle inequality. "
                  "You may need to disable pruning or rejection.",
                  getName().c_str(), si_->getStateSpace()->getName().c_str());

    const base::ReportIntermediateSolutionFn intermediateSolutionCallback = pdef_->getIntermediateSolutionCallback();

    Motion *approxGoalMotion = nullptr;
    double approxDist = std::numeric_limits<double>::infinity();

    auto *rmotion = new Motion(si_);
    base::State *rstate = rmotion->state;
    base::State *xstate = si_->allocState();

    std::vector<Motion *> nbh;

    std::vector<base::Cost> costs;
    std::vector<base::Cost> incCosts;
    std::vector<std::size_t> sortedCostIndices;

    std::vector<int> valid;
    unsigned int rewireTest = 0;
    unsigned int statesGenerated = 0;

    if (bestGoalMotion_)
        OMPL_INFORM("%s: Starting planning with existing solution of cost %.5f", getName().c_str(),
                    bestCost_.value());

    if (useKNearest_)
        OMPL_INFORM("%s: Initial k-nearest value of %u", getName().c_str(),
                    (unsigned int)std::ceil(k_rrt_ * log((double)(nn_->size() + 1u))));
    else
        OMPL_INFORM(
            "%s: Initial rewiring radius of %.2f", getName().c_str(),
            std::min(maxDistance_, r_rrt_ * std::pow(log((double)(nn_->size() + 1u)) / ((double)(nn_->size() + 1u)),
                                                     1 / (double)(si_->getStateDimension()))));

    // our functor for sorting nearest neighbors
    CostIndexCompare compareFn(costs, *opt_);
    
    // Start Time
	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    while (ptc == false)
    {
        iterations_++;
        // sample random state (with goal biasing)
        // Goal samples are only sampled until maxSampleCount() goals are in the tree, to prohibit duplicate goal
        // states.
        if (goal_s && goalMotions_.size() < goal_s->maxSampleCount() && rng_.uniform01() < goalBias_ && goal_s->canSample())
        {
            goal_s->sampleGoal(rstate);
        }
        else
        {
            sampler_->sampleUniform(rstate);
        }

        /* find closest state in the tree */
        Motion *nmotion = nn_->nearest(rmotion);
        if (intermediateSolutionCallback && si_->equalStates(nmotion->state, rstate))
            continue;
        base::State *dstate = rstate;

        /* find state to add */
        double d = si_->distance(nmotion->state, rstate);
        if (d > maxDistance_)
        {
            si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
            dstate = xstate;
        }
        
        // The steering between two states (i.e. the local trajectory) 
        std::vector<ompl::base::State*> steering;

        // Check if the motion between the nearest state and the state to add is valid
        if (si_->checkMotionODE(nmotion->state, dstate, steering))
        {
            /** Update the initial state (nmotion) and the last state (dstate) with initial conditions encoded in steering.at(0) and steering.back(). 
             * nmotion and steering.at(0) have the same state values but are not the same object. Same for dstate and steering.back()
            */
            si_->copyState(nmotion->state, steering.at(0));
            si_->copyState(dstate, steering.back());

            // create a motion
            auto *motion = new Motion(si_);
            si_->copyState(motion->state, dstate);
            
            // We copy the steering into the motion, it's now up to the motion object to manage the memory of traj vector
            motion->traj_ = std::move(steering);

            motion->parent = nmotion;
            motion->incCost = opt_->motionCost(nmotion->state, motion->state);
            motion->cost = opt_->combineCosts(nmotion->cost, motion->incCost);

            // Find nearby neighbors of the new motion
            getNeighbors(motion, nbh);

            rewireTest += nbh.size();
            ++statesGenerated;

            // cache for distance computations
            //
            // Our cost caches only increase in size, so they're only
            // resized if they can't fit the current neighborhood
            if (costs.size() < nbh.size())
            {
                costs.resize(nbh.size());
                incCosts.resize(nbh.size());
                sortedCostIndices.resize(nbh.size());
            }

            // cache for motion validity (only useful in a symmetric space)
            //
            // Our validity caches only increase in size, so they're
            // only resized if they can't fit the current neighborhood
            if (valid.size() < nbh.size())
                valid.resize(nbh.size());
            std::fill(valid.begin(), valid.begin() + nbh.size(), 0);

            // Finding the nearest neighbor to connect to
            // By default, neighborhood states are sorted by cost, and collision checking
            // is performed in increasing order of cost
            if (delayCC_)
            {
                // calculate all costs and distances
                for (std::size_t i = 0; i < nbh.size(); ++i)
                {
                    incCosts[i] = opt_->motionCost(nbh[i]->state, motion->state);
                    costs[i] = opt_->combineCosts(nbh[i]->cost, incCosts[i]);
                }

                // sort the nodes
                //
                // we're using index-value pairs so that we can get at
                // original, unsorted indices
                for (std::size_t i = 0; i < nbh.size(); ++i)
                    sortedCostIndices[i] = i;
                std::sort(sortedCostIndices.begin(), sortedCostIndices.begin() + nbh.size(), compareFn);

                // collision check until a valid motion is found
                //
                // ASYMMETRIC CASE: it's possible that none of these
                // neighbors are valid. This is fine, because motion
                // already has a connection to the tree through
                // nmotion (with populated cost fields!).
                for (std::vector<std::size_t>::const_iterator i = sortedCostIndices.begin();
                     i != sortedCostIndices.begin() + nbh.size(); ++i)
                {
                    std::vector<ompl::base::State*> traj_delay;
                    bool robust_valid = si_->checkMotionODE(nbh[*i]->state, motion->state, traj_delay);
                    if (nbh[*i] == nmotion ||
                        ((!useKNearest_ || si_->distance(nbh[*i]->state, motion->state) < maxDistance_) && robust_valid))
                    {
                        /** Update the last state (motion) ODEs conditions */
                        si_->copyState(motion->state, traj_delay.back());

                        // We clear the already existing local trajectory
                        freeMotionTraj(motion->traj_);
                        // We copy the steering into the motion, it's now up to the motion object to manage the memory of traj vector
                        motion->traj_ = std::move(traj_delay);

                        motion->incCost = incCosts[*i];
                        motion->cost = costs[*i];
                        motion->parent = nbh[*i];
                        
                        valid[*i] = 1;
                        break;
                    }
                    else
                    {
                        valid[*i] = -1;
                        freeMotionTraj(traj_delay);
                    } 
                }
            }
            else  // if not delayCC
            {
                motion->incCost = opt_->motionCost(nmotion->state, motion->state);
                motion->cost = opt_->combineCosts(nmotion->cost, motion->incCost);
                // find which one we connect the new state to
                for (std::size_t i = 0; i < nbh.size(); ++i)
                {
                    if (nbh[i] != nmotion)
                    {
                        incCosts[i] = opt_->motionCost(nbh[i]->state, motion->state);
                        costs[i] = opt_->combineCosts(nbh[i]->cost, incCosts[i]);
                        if (opt_->isCostBetterThan(costs[i], motion->cost))
                        {
                            std::vector<ompl::base::State*> traj_no_delay;
                            bool robust_valid = si_->checkMotionODE(nbh[i]->state, motion->state, traj_no_delay);
                            if ((!useKNearest_ || si_->distance(nbh[i]->state, motion->state) < maxDistance_) && robust_valid)
                            {
                                /** Update the last state (motion) ODEs conditions */
                                si_->copyState(motion->state, traj_no_delay.back());

                                // We clear the already existing local trajectory
                                freeMotionTraj(motion->traj_);
                                // We copy the steering into the motion, it's now up to the motion object to manage the memory of traj vector
                                motion->traj_ = std::move(traj_no_delay);

                                motion->incCost = incCosts[i];
                                motion->cost = costs[i];
                                motion->parent = nbh[i];
                                
                                valid[i] = 1;
                            }
                            else
                            {
                                valid[i] = -1;
                                freeMotionTraj(traj_no_delay);
                            }
                        }
                    }
                    else
                    {
                        incCosts[i] = motion->incCost;
                        costs[i] = motion->cost;
                        valid[i] = 1;
                    }
                }
            }

            if (useNewStateRejection_)
            {
                if (opt_->isCostBetterThan(solutionHeuristic(motion), bestCost_))
                {
                    nn_->add(motion);
                    motion->parent->children.push_back(motion);

                    // For visualization
                    if(visualize_tree_)
                    {
                        motion->stateVisualID = visual_tools_->addState(motion->state);
                        motion->edgeVisualID = visual_tools_->addEdge(motion->parent->state, motion->state);
                    }
                }
                else  // If the new motion does not improve the best cost it is ignored.
                {
                    si_->freeState(motion->state);
                    freeMotionTraj(motion->traj_);
                    delete motion;
                    continue;
                }
            }
            else
            {
                // add motion to the tree
                nn_->add(motion);
                motion->parent->children.push_back(motion);

                // For visualization
                if(visualize_tree_)
                {
                    motion->stateVisualID = visual_tools_->addState(motion->state);
                    motion->edgeVisualID = visual_tools_->addEdge(motion->parent->state, motion->state);
                }
            }

            bool checkForSolution = false;
            for (std::size_t i = 0; i < nbh.size(); ++i)
            {
                if (nbh[i] != motion->parent)
                {
                    base::Cost nbhIncCost;
                    if (symCost)
                        nbhIncCost = incCosts[i];
                    else
                        nbhIncCost = opt_->motionCost(motion->state, nbh[i]->state);
                    base::Cost nbhNewCost = opt_->combineCosts(motion->cost, nbhIncCost);
                    if (opt_->isCostBetterThan(nbhNewCost, nbh[i]->cost))
                    {
                        bool motionValid;
                        std::vector<ompl::base::State*> traj_rewire;
                        if (valid[i] == 0)
                        {
                            bool robust_valid = si_->checkMotionODE(motion->state, nbh[i]->state, traj_rewire);
                            motionValid =
                                (!useKNearest_ || si_->distance(nbh[i]->state, motion->state) < maxDistance_) && robust_valid;
                        }
                        else
                        {
                            motionValid = (valid[i] == 1);
                        }

                        if (motionValid)
                        {
                            // Remove this node from its parent list
                            removeFromParent(nbh[i]);

                            /** Update the last state (nbh) ODEs conditions */
                            si_->copyState(nbh[i]->state, traj_rewire.back());

                            // We clear the already existing local trajectory
                            freeMotionTraj(nbh[i]->traj_);
                            // We copy the steering into the motion, it's now up to the motion object to manage the memory of traj vector
                            nbh[i]->traj_ = std::move(traj_rewire);

                            // Add this node to the new parent
                            nbh[i]->parent = motion;
                            nbh[i]->incCost = nbhIncCost;
                            nbh[i]->cost = nbhNewCost;
                            nbh[i]->parent->children.push_back(nbh[i]);

                            // Update the costs of the node's children
                            updateChildCosts(nbh[i]);

                            // For visualization
                            if(visualize_tree_)
                            {
                                visual_tools_->removeItem(nbh[i]->edgeVisualID);
                                nbh[i]->edgeVisualID = visual_tools_->addEdge(motion->state, nbh[i]->state);
                            }

                            checkForSolution = true;
                        }
                        else
                        {
                            freeMotionTraj(traj_rewire);
                        }
                    }
                }
            }

            // Add the new motion to the goalMotion_ list, if it satisfies the goal
            double distanceFromGoal;
            if (goal->isSatisfied(motion->state, &distanceFromGoal))
            {
                motion->inGoal = true;
                goalMotions_.push_back(motion);
                checkForSolution = true;
            }

            // Checking for solution or iterative improvement
            if (checkForSolution)
            {
                bool updatedSolution = false;
                if (!bestGoalMotion_ && !goalMotions_.empty())
                {
                    // We have found our first solution, store it as the best. We only add one
                    // vertex at a time, so there can only be one goal vertex at this moment.
                    bestGoalMotion_ = goalMotions_.front();
                    bestCost_ = bestGoalMotion_->cost;
                    updatedSolution = true;

                    std::chrono::steady_clock::time_point cost_measure = std::chrono::steady_clock::now();
                    exportCost(bestCost_.value(), (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - begin).count()), 0.);
                    OMPL_INFORM("%s: Found an initial solution with a cost of %.2f in %u iterations (%u "
                                "vertices in the graph)",
                                getName().c_str(), bestCost_.value(), iterations_, nn_->size());
                }
                else
                {
                    // We already have a solution, iterate through the list of goal vertices
                    // and see if there's any improvement.
                    for (auto &goalMotion : goalMotions_)
                    {
                        // Is this goal motion better than the (current) best?
                        if (opt_->isCostBetterThan(goalMotion->cost, bestCost_))
                        {
                            bestGoalMotion_ = goalMotion;
                            bestCost_ = bestGoalMotion_->cost;
                            updatedSolution = true;

                            std::chrono::steady_clock::time_point cost_measure = std::chrono::steady_clock::now();
                            exportCost(bestCost_.value(), (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - begin).count()), 0.);

                            // Check if it satisfies the optimization objective, if it does, break the for loop
                            if (opt_->isSatisfied(bestCost_))
                            {
                                break;
                            }
                        }
                    }
                }

                if (updatedSolution)
                {
                    if (useTreePruning_)
                    {
                        pruneTree(bestCost_);
                    }

                    if (intermediateSolutionCallback)
                    {
                        std::vector<const base::State *> spath;
                        Motion *intermediate_solution =
                            bestGoalMotion_->parent;  // Do not include goal state to simplify code.

                        // Push back until we find the start, but not the start itself
                        while (intermediate_solution->parent != nullptr)
                        {
                            spath.push_back(intermediate_solution->state);
                            intermediate_solution = intermediate_solution->parent;
                        }

                        intermediateSolutionCallback(this, spath, bestCost_);
                    }
                }
            }

            // Checking for approximate solution (closest state found to the goal)
            if (goalMotions_.size() == 0 && distanceFromGoal < approxDist)
            {
                approxGoalMotion = motion;
                approxDist = distanceFromGoal;
            }
        }
        else
        {
            // We free the states stored in the steering object if a collision is found. Otherwise it's now the motion object responsability
            freeMotionTraj(steering);
        }
        // terminate if a sufficient solution is found
        if (bestGoalMotion_ && opt_->isSatisfied(bestCost_))
            break;

        std::chrono::steady_clock::time_point end_iter = std::chrono::steady_clock::now();
        exportIter(iterations_, (std::chrono::duration_cast<std::chrono::microseconds>(end_iter - begin).count()));
    }

    // Add our solution (if it exists)
    Motion *newSolution = nullptr;
    if (bestGoalMotion_)
    {
        // We have an exact solution
        newSolution = bestGoalMotion_;
    }
    else if (approxGoalMotion)
    {
        // We don't have a solution, but we do have an approximate solution
        newSolution = approxGoalMotion;
    }
    // No else, we have nothing

    // Add what we found
    if (newSolution)
    {
        ptc.terminate();
        // construct the solution path
        std::vector<Motion *> mpath;
        Motion *iterMotion = newSolution;
        while (iterMotion != nullptr)
        {
            mpath.push_back(iterMotion);
            iterMotion = iterMotion->parent;
        }

        // set the solution path
        auto path(std::make_shared<PathGeometric>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            path->append(mpath[i]->state);

        // Add the solution path.
        base::PlannerSolution psol(path);
        psol.setPlannerName(getName());

        // If we don't have a goal motion, the solution is approximate
        if (!bestGoalMotion_)
            psol.setApproximate(approxDist);

        // Does the solution satisfy the optimization objective?
        psol.setOptimized(opt_, newSolution->cost, opt_->isSatisfied(bestCost_));
        pdef_->addSolutionPath(psol);
    }
    // No else, we have nothing

    si_->freeState(xstate);
    if (rmotion->state)
        si_->freeState(rmotion->state);
    freeMotionTraj(rmotion->traj_);
    delete rmotion;

    OMPL_INFORM("%s: Created %u new states. Checked %u rewire options. %u goal states in tree. Final solution cost "
                "%.3f",
                getName().c_str(), statesGenerated, rewireTest, goalMotions_.size(), bestCost_.value());

    // We've added a solution if newSolution == true, and it is an approximate solution if bestGoalMotion_ == false
    return {newSolution != nullptr, bestGoalMotion_ == nullptr};
}

void ompl::geometric::R_SARRTstar::getNeighbors(Motion *motion, std::vector<Motion *> &nbh) const
{
    auto cardDbl = static_cast<double>(nn_->size() + 1u);
    if (useKNearest_)
    {
        //- k-nearest RRT*
        unsigned int k = std::ceil(k_rrt_ * log(cardDbl));
        nn_->nearestK(motion, k, nbh);
    }
    else
    {
        double r = std::min(
            maxDistance_, r_rrt_ * std::pow(log(cardDbl) / cardDbl, 1 / static_cast<double>(si_->getStateDimension())));
        nn_->nearestR(motion, r, nbh);
    }
}

void ompl::geometric::R_SARRTstar::removeFromParent(Motion *m)
{
    for (auto it = m->parent->children.begin(); it != m->parent->children.end(); ++it)
    {
        if (*it == m)
        {
            m->parent->children.erase(it);
            break;
        }
    }
}

void ompl::geometric::R_SARRTstar::updateChildCosts(Motion *m)
{
    for (std::size_t i = 0; i < m->children.size(); ++i)
    {
        /// Re check the already computed motion between m->state and the i-th children but with potentially new tubes due to the rewiring phase
        if (si_->reCheckMotionODE(m->children[i]->traj_))
        {
            /** Update the final state (i-th children) with initial conditions encoded in last state of traj_ (i.e. PI_f, PI_xi_f, qn_f) */
            si_->copyState(m->children[i]->state, m->children[i]->traj_.back());

            // We need to recompute the sensitivity-based cost
            if(sensi_cost_)
                m->children[i]->incCost = opt_->motionCost(m->children[i]->parent->state, m->children[i]->state);
                
            m->children[i]->cost = opt_->combineCosts(m->cost, m->children[i]->incCost);
            updateChildCosts(m->children[i]);
        }
        // Remove the children and the whole branch of the tree associated to it
        else
        {
            removeUpdateChild(m->children[i]);
        }
    }
}

void ompl::geometric::R_SARRTstar::addChildrenToDelete(std::queue<Motion *, std::deque<Motion *>>& motionList, Motion *m)
{
    for (std::size_t i = 0; i < m->children.size(); ++i)
    {
        motionList.push(m->children[i]);
        addChildrenToDelete(motionList, m->children[i]);
    }
}

void ompl::geometric::R_SARRTstar::removeUpdateChild(Motion *m)
{
    // The queue of Motions to process:
    std::queue<Motion *, std::deque<Motion *>> motionQueue;

    // Put actual motion to remove and its children into the queue:
    addChildrenToDelete(motionQueue, m);

    // We now have a list of Motions to definitely remove, and a list of Motions to recheck
    // Iteratively check the two lists until there is nothing to to remove
 
    // First empty the current leaves-to-prune
    while (motionQueue.empty() == false)
    {
        if(motionQueue.front()->inGoal)
        {
            motionQueue.pop();
            continue;
        }
            
        // Remove the leaf from its parent
        removeFromParent(motionQueue.front());

        // Erase the actual motion
        // First free the state
        nn_->remove(motionQueue.front());
        si_->freeState(motionQueue.front()->state);

        // Free the local trajectory
        freeMotionTraj(motionQueue.front()->traj_);

        // then delete the pointer
        delete motionQueue.front();

        // And finally remove it from the list, erase returns the next iterator
        motionQueue.pop();
    }
}

void ompl::geometric::R_SARRTstar::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state)
                si_->freeState(motion->state);
            // Free the local trajectory
            freeMotionTraj(motion->traj_);
            delete motion;
        }
    }
}

void ompl::geometric::R_SARRTstar::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);

    if (bestGoalMotion_)
        data.addGoalVertex(base::PlannerDataVertex(bestGoalMotion_->state));

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state));
        else
            data.addEdge(base::PlannerDataVertex(motion->parent->state), base::PlannerDataVertex(motion->state));
    }
}

int ompl::geometric::R_SARRTstar::pruneTree(const base::Cost &pruneTreeCost)
{
    // Variable
    // The percent improvement (expressed as a [0,1] fraction) in cost
    double fracBetter;
    // The number pruned
    int numPruned = 0;

    if (opt_->isFinite(prunedCost_))
    {
        fracBetter = std::abs((pruneTreeCost.value() - prunedCost_.value()) / prunedCost_.value());
    }
    else
    {
        fracBetter = 1.0;
    }

    if (fracBetter > pruneThreshold_)
    {
        // We are only pruning motions if they, AND all descendents, have a estimated cost greater than pruneTreeCost
        // The easiest way to do this is to find leaves that should be pruned and ascend up their ancestry until a
        // motion is found that is kept.
        // To avoid making an intermediate copy of the NN structure, we process the tree by descending down from the
        // start(s).
        // In the first pass, all Motions with a cost below pruneTreeCost, or Motion's with children with costs below
        // pruneTreeCost are added to the replacement NN structure,
        // while all other Motions are stored as either a 'leaf' or 'chain' Motion. After all the leaves are
        // disconnected and deleted, we check
        // if any of the the chain Motions are now leaves, and repeat that process until done.
        // This avoids (1) copying the NN structure into an intermediate variable and (2) the use of the expensive
        // NN::remove() method.

        // Variable
        // The queue of Motions to process:
        std::queue<Motion *, std::deque<Motion *>> motionQueue;
        // The list of leaves to prune
        std::queue<Motion *, std::deque<Motion *>> leavesToPrune;
        // The list of chain vertices to recheck after pruning
        std::list<Motion *> chainsToRecheck;

        // Clear the NN structure:
        nn_->clear();

        // Put all the starts into the NN structure and their children into the queue:
        // We do this so that start states are never pruned.
        for (auto &startMotion : startMotions_)
        {
            // Add to the NN
            nn_->add(startMotion);

            // Add their children to the queue:
            addChildrenToList(&motionQueue, startMotion);
        }

        while (motionQueue.empty() == false)
        {
            // Test, can the current motion ever provide a better solution?
            if (keepCondition(motionQueue.front(), pruneTreeCost))
            {
                // Yes it can, so it definitely won't be pruned
                // Add it back into the NN structure
                nn_->add(motionQueue.front());

                // Add it's children to the queue
                addChildrenToList(&motionQueue, motionQueue.front());
            }
            else
            {
                // No it can't, but does it have children?
                if (motionQueue.front()->children.empty() == false)
                {
                    // Yes it does.
                    // We can minimize the number of intermediate chain motions if we check their children
                    // If any of them won't be pruned, then this motion won't either. This intuitively seems
                    // like a nice balance between following the descendents forever.

                    // Variable
                    // Whether the children are definitely to be kept.
                    bool keepAChild = false;

                    // Find if any child is definitely not being pruned.
                    for (unsigned int i = 0u; keepAChild == false && i < motionQueue.front()->children.size(); ++i)
                    {
                        // Test if the child can ever provide a better solution
                        keepAChild = keepCondition(motionQueue.front()->children.at(i), pruneTreeCost);
                    }

                    // Are we *definitely* keeping any of the children?
                    if (keepAChild)
                    {
                        // Yes, we are, so we are not pruning this motion
                        // Add it back into the NN structure.
                        nn_->add(motionQueue.front());
                    }
                    else
                    {
                        // No, we aren't. This doesn't mean we won't though
                        // Move this Motion to the temporary list
                        chainsToRecheck.push_back(motionQueue.front());
                    }

                    // Either way. add it's children to the queue
                    addChildrenToList(&motionQueue, motionQueue.front());
                }
                else
                {
                    // No, so we will be pruning this motion:
                    leavesToPrune.push(motionQueue.front());
                }
            }

            // Pop the iterator, std::list::erase returns the next iterator
            motionQueue.pop();
        }

        // We now have a list of Motions to definitely remove, and a list of Motions to recheck
        // Iteratively check the two lists until there is nothing to to remove
        while (leavesToPrune.empty() == false)
        {
            // First empty the current leaves-to-prune
            while (leavesToPrune.empty() == false)
            {
                // If this leaf is a goal, remove it from the goal set
                if (leavesToPrune.front()->inGoal == true)
                {
                    // Warn if pruning the _best_ goal
                    if (leavesToPrune.front() == bestGoalMotion_)
                    {
                        OMPL_ERROR("%s: Pruning the best goal.", getName().c_str());
                    }
                    // Remove it
                    goalMotions_.erase(std::remove(goalMotions_.begin(), goalMotions_.end(), leavesToPrune.front()),
                                       goalMotions_.end());
                }

                // Remove the leaf from its parent
                removeFromParent(leavesToPrune.front());

                // Erase the actual motion
                // First free the state
                si_->freeState(leavesToPrune.front()->state);

                // Free the local trajectory
                freeMotionTraj(leavesToPrune.front()->traj_);

                // then delete the pointer
                delete leavesToPrune.front();

                // And finally remove it from the list, erase returns the next iterator
                leavesToPrune.pop();

                // Update our counter
                ++numPruned;
            }

            // Now, we need to go through the list of chain vertices and see if any are now leaves
            auto mIter = chainsToRecheck.begin();
            while (mIter != chainsToRecheck.end())
            {
                // Is the Motion a leaf?
                if ((*mIter)->children.empty() == true)
                {
                    // It is, add to the removal queue
                    leavesToPrune.push(*mIter);

                    // Remove from this queue, getting the next
                    mIter = chainsToRecheck.erase(mIter);
                }
                else
                {
                    // Is isn't, skip to the next
                    ++mIter;
                }
            }
        }

        // Now finally add back any vertices left in chainsToReheck.
        // These are chain vertices that have descendents that we want to keep
        for (const auto &r : chainsToRecheck)
            // Add the motion back to the NN struct:
            nn_->add(r);

        // All done pruning.
        // Update the cost at which we've pruned:
        prunedCost_ = pruneTreeCost;

        // And if we're using the pruned measure, the measure to which we've pruned
        if (usePrunedMeasure_)
        {
            prunedMeasure_ = infSampler_->getInformedMeasure(prunedCost_);

            if (useKNearest_ == false)
            {
                calculateRewiringLowerBounds();
            }
        }
        // No else, prunedMeasure_ is the si_ measure by default.
    }

    return numPruned;
}

void ompl::geometric::R_SARRTstar::addChildrenToList(std::queue<Motion *, std::deque<Motion *>> *motionList, Motion *motion)
{
    for (auto &child : motion->children)
    {
        motionList->push(child);
    }
}

bool ompl::geometric::R_SARRTstar::keepCondition(const Motion *motion, const base::Cost &threshold) const
{
    // We keep if the cost-to-come-heuristic of motion is <= threshold, by checking
    // if !(threshold < heuristic), as if b is not better than a, then a is better than, or equal to, b
    if (bestGoalMotion_ && motion == bestGoalMotion_)
    {
        // If the threshold is the theoretical minimum, the bestGoalMotion_ will sometimes fail the test due to floating point precision. Avoid that.
        return true;
    }

    return !opt_->isCostBetterThan(threshold, solutionHeuristic(motion));
}

ompl::base::Cost ompl::geometric::R_SARRTstar::solutionHeuristic(const Motion *motion) const
{
    base::Cost costToCome;
    if (useAdmissibleCostToCome_)
    {
        // Start with infinite cost
        costToCome = opt_->infiniteCost();

        // Find the min from each start
        for (auto &startMotion : startMotions_)
        {
            costToCome = opt_->betterCost(
                costToCome, opt_->motionCost(startMotion->state,
                                             motion->state));  // lower-bounding cost from the start to the state
        }
    }
    else
    {
        costToCome = motion->cost;  // current cost from the state to the goal
    }

    const base::Cost costToGo =
        opt_->costToGo(motion->state, pdef_->getGoal().get());  // lower-bounding cost from the state to the goal
    return opt_->combineCosts(costToCome, costToGo);            // add the two costs
}

void ompl::geometric::R_SARRTstar::setTreePruning(const bool prune)
{
    if (static_cast<bool>(opt_) == true)
    {
        if (opt_->hasCostToGoHeuristic() == false)
        {
            OMPL_INFORM("%s: No cost-to-go heuristic set. Informed techniques will not work well.", getName().c_str());
        }
    }

    // If we just disabled tree pruning, but we wee using prunedMeasure, we need to disable that as it required myself
    if (prune == false && getPrunedMeasure() == true)
    {
        setPrunedMeasure(false);
    }

    // Store
    useTreePruning_ = prune;
}

void ompl::geometric::R_SARRTstar::setPrunedMeasure(bool informedMeasure)
{
    if (static_cast<bool>(opt_) == true)
    {
        if (opt_->hasCostToGoHeuristic() == false)
        {
            OMPL_INFORM("%s: No cost-to-go heuristic set. Informed techniques will not work well.", getName().c_str());
        }
    }

    // This option only works with informed sampling
    if (informedMeasure == true && (useInformedSampling_ == false || useTreePruning_ == false))
    {
        OMPL_ERROR("%s: InformedMeasure requires InformedSampling and TreePruning.", getName().c_str());
    }

    // Check if we're changed and update parameters if we have:
    if (informedMeasure != usePrunedMeasure_)
    {
        // Store the setting
        usePrunedMeasure_ = informedMeasure;

        // Update the prunedMeasure_ appropriately, if it has been configured.
        if (setup_ == true)
        {
            if (usePrunedMeasure_)
            {
                prunedMeasure_ = infSampler_->getInformedMeasure(prunedCost_);
            }
            else
            {
                prunedMeasure_ = si_->getSpaceMeasure();
            }
        }

        // And either way, update the rewiring radius if necessary
        if (useKNearest_ == false)
        {
            calculateRewiringLowerBounds();
        }
    }
}

void ompl::geometric::R_SARRTstar::setInformedSampling(bool informedSampling)
{
    if (static_cast<bool>(opt_) == true)
    {
        if (opt_->hasCostToGoHeuristic() == false)
        {
            OMPL_INFORM("%s: No cost-to-go heuristic set. Informed techniques will not work well.", getName().c_str());
        }
    }

    // This option is mutually exclusive with setSampleRejection, assert that:
    if (informedSampling == true && useRejectionSampling_ == true)
    {
        OMPL_ERROR("%s: InformedSampling and SampleRejection are mutually exclusive options.", getName().c_str());
    }

    // If we just disabled tree pruning, but we are using prunedMeasure, we need to disable that as it required myself
    if (informedSampling == false && getPrunedMeasure() == true)
    {
        setPrunedMeasure(false);
    }

    // Check if we're changing the setting of informed sampling. If we are, we will need to create a new sampler, which
    // we only want to do if one is already allocated.
    if (informedSampling != useInformedSampling_)
    {
        // If we're disabled informedSampling, and prunedMeasure is enabled, we need to disable that
        if (informedSampling == false && usePrunedMeasure_ == true)
        {
            setPrunedMeasure(false);
        }

        // Store the value
        useInformedSampling_ = informedSampling;

        // If we currently have a sampler, we need to make a new one
        if (sampler_ || infSampler_)
        {
            // Reset the samplers
            sampler_.reset();
            infSampler_.reset();

            // Create the sampler
            allocSampler();
        }
    }
}

void ompl::geometric::R_SARRTstar::setSampleRejection(const bool reject)
{
    if (static_cast<bool>(opt_) == true)
    {
        if (opt_->hasCostToGoHeuristic() == false)
        {
            OMPL_INFORM("%s: No cost-to-go heuristic set. Informed techniques will not work well.", getName().c_str());
        }
    }

    // This option is mutually exclusive with setInformedSampling, assert that:
    if (reject == true && useInformedSampling_ == true)
    {
        OMPL_ERROR("%s: InformedSampling and SampleRejection are mutually exclusive options.", getName().c_str());
    }

    // Check if we're changing the setting of rejection sampling. If we are, we will need to create a new sampler, which
    // we only want to do if one is already allocated.
    if (reject != useRejectionSampling_)
    {
        // Store the setting
        useRejectionSampling_ = reject;

        // If we currently have a sampler, we need to make a new one
        if (sampler_ || infSampler_)
        {
            // Reset the samplers
            sampler_.reset();
            infSampler_.reset();

            // Create the sampler
            allocSampler();
        }
    }
}

void ompl::geometric::R_SARRTstar::setOrderedSampling(bool orderSamples)
{
    // Make sure we're using some type of informed sampling
    if (useInformedSampling_ == false && useRejectionSampling_ == false)
    {
        OMPL_ERROR("%s: OrderedSampling requires either informed sampling or rejection sampling.", getName().c_str());
    }

    // Check if we're changing the setting. If we are, we will need to create a new sampler, which we only want to do if
    // one is already allocated.
    if (orderSamples != useOrderedSampling_)
    {
        // Store the setting
        useOrderedSampling_ = orderSamples;

        // If we currently have a sampler, we need to make a new one
        if (sampler_ || infSampler_)
        {
            // Reset the samplers
            sampler_.reset();
            infSampler_.reset();

            // Create the sampler
            allocSampler();
        }
    }
}

void ompl::geometric::R_SARRTstar::allocSampler()
{
    // Allocate the appropriate type of sampler.
    if (useInformedSampling_)
    {
        // We are using informed sampling, this can end-up reverting to rejection sampling in some cases
        OMPL_INFORM("%s: Using informed sampling.", getName().c_str());
        infSampler_ = opt_->allocInformedStateSampler(pdef_, numSampleAttempts_);
    }
    else if (useRejectionSampling_)
    {
        // We are explicitly using rejection sampling.
        OMPL_INFORM("%s: Using rejection sampling.", getName().c_str());
        infSampler_ = std::make_shared<base::RejectionInfSampler>(pdef_, numSampleAttempts_);
    }
    else
    {
        // We are using a regular sampler
        sampler_ = si_->allocStateSampler();
    }

    // Wrap into a sorted sampler
    if (useOrderedSampling_ == true)
    {
        infSampler_ = std::make_shared<base::OrderedInfSampler>(infSampler_, batchSize_);
    }
    // No else
}

bool ompl::geometric::R_SARRTstar::sampleUniform(base::State *statePtr)
{
    // Use the appropriate sampler
    if (useInformedSampling_ || useRejectionSampling_)
    {
        // Attempt the focused sampler and return the result.
        // If bestCost is changing a lot by small amounts, this could
        // be prunedCost_ to reduce the number of times the informed sampling
        // transforms are recalculated.
        return infSampler_->sampleUniform(statePtr, bestCost_);
    }
    else
    {
        // Simply return a state from the regular sampler
        sampler_->sampleUniform(statePtr);

        // Always true
        return true;
    }
}

void ompl::geometric::R_SARRTstar::calculateRewiringLowerBounds()
{
    const auto dimDbl = static_cast<double>(si_->getStateDimension());

    // k_rrt > 2^(d + 1) * e * (1 + 1 / d).  K-nearest RRT*
    k_rrt_ = rewireFactor_ * (std::pow(2, dimDbl + 1) * boost::math::constants::e<double>() * (1.0 + 1.0 / dimDbl));

    // r_rrt > (2*(1+1/d))^(1/d)*(measure/ballvolume)^(1/d)
    // If we're not using the informed measure, prunedMeasure_ will be set to si_->getSpaceMeasure();
    r_rrt_ =
        rewireFactor_ *
        std::pow(2 * (1.0 + 1.0 / dimDbl) * (prunedMeasure_ / unitNBallMeasure(si_->getStateDimension())), 1.0 / dimDbl);
}

void ompl::geometric::R_SARRTstar::exportCost(double cost, double running_time, double sensi) const
{
    std::ofstream cost_overtime(cost_file_, std::ios::app);
    std::cout.precision(6);

    cost_overtime<< "\t" << "Cost:" << cost << ";\t" << "Running time:" << running_time << ";\tSensitivity:" << sensi << "\n";

    cost_overtime.close();
}

void ompl::geometric::R_SARRTstar::exportIter(int iteration, double running_time) const
{
    std::ofstream iter_overtime(iteration_file_, std::ios::app);
    std::cout.precision(6);

    iter_overtime<< "\t" << "Iteration:" << iteration << ";" << "Running time:" << running_time << "\n";

    iter_overtime.close();
}
