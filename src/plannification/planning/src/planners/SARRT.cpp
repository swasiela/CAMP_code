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

#include "planning/planners/SARRT.h"

#define BOUND_TO_01(v) ((v<0)? (0): ( (v>1)? (1): (v)))

ompl::geometric::SARRT::SARRT(const base::SpaceInformationPtr &si, const BulletVisualToolsPtr &visual_tools, const std::string &ros_namespace, bool addIntermediateStates)
  : base::Planner(si, "SARRT"), visual_tools_(visual_tools), ros_namespace_(ros_namespace)
{
    specs_.approximateSolutions = true;
    specs_.directed = true;

    Planner::declareParam<double>("range", this, &SARRT::setRange, &SARRT::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &SARRT::setGoalBias, &SARRT::getGoalBias, "0.:.05:1.");
    Planner::declareParam<bool>("intermediate_states", this, &SARRT::setIntermediateStates, &SARRT::getIntermediateStates,
                                "0,1");

    addIntermediateStates_ = addIntermediateStates;
}

ompl::geometric::SARRT::~SARRT()
{
    freeMemory();
}

void ompl::geometric::SARRT::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalMotion_ = nullptr;
}

void ompl::geometric::SARRT::setup()
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
    }
    else
    {
        OMPL_INFORM("%s: problem definition is not set, deferring setup completion...", getName().c_str());
        setup_ = false;
    }

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
    if(!nh_.getParam("planning_parameters/visualize_tree", visualize_tree_))
    {
        ROS_WARN("Value of visualize_tree not found. Using default visualize_tree_ = false.");
        visualize_tree_ = false;
    }

    // To clear the file 
    std::ofstream ofs;
    ofs.open(iteration_file_, std::ofstream::out | std::ofstream::trunc);
    ofs.close();
}

void ompl::geometric::SARRT::freeMemory()
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

ompl::base::PlannerStatus ompl::geometric::SARRT::solve(const base::PlannerTerminationCondition &ptc)
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

    // We just reconnect the tree
    bool recheck = false;
    // Start Time
	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    int iterations_ = 0;
    while (ptc == false)
    {
        iterations_++;
        /* sample random state (with goal biasing) */
        // We just reconnect the tree, we need to check if it is still connected to the goal
        if ((goal_s != nullptr) && rng_.uniform01() < goalBias_ && goal_s->canSample() || recheck)
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

        if (si_->checkMotion(nmotion->state, dstate))
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
                    nn_->add(motion);
                    motion->parent->children.push_back(motion);

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
                nn_->add(motion);
                motion->parent->children.push_back(motion);

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
                /* construct the possible solution path */
                Motion *possible_solution = nmotion;
                std::vector<Motion *> possible_sol;
                while (possible_solution != nullptr)
                {
                    possible_sol.push_back(possible_solution);
                    possible_solution = possible_solution->parent;
                }
                // Reverse the possible solution
                std::reverse(std::begin(possible_sol), std::end(possible_sol));

                // Check the possible solution with tubes
                int invalid_idx = -1;
                for(int i = 0; i<possible_sol.size()-1; i++)
                {
                    std::vector<ompl::base::State*> sol;
                    if(!si_->checkMotionODE(possible_sol.at(i)->state, possible_sol.at(i+1)->state, sol))
                    {
                        invalid_idx = i+1;
                        break;
                    }
                    /** Update the last state ODEs conditions */
                    si_->copyState(possible_sol.at(i+1)->state, sol.back());     
                }
                
                if(invalid_idx < 0)
                {
                    approxdif = dist;
                    solution = nmotion;
                    break;
                }
                else
                {
                    // Disconnet and reconnect the tree from the node find in collision using the tube
                    robustExtend(possible_sol.at(invalid_idx));
                    recheck = goal->isSatisfied(nmotion->state, &dist); // Next iteration we will recheck the current solution, no need to add new sample we continue refining the current solution until it becomes robust or the goal is disconnected
                }
            }
            else
            {
                recheck = false;
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

    return {solved, approximate};
}

void ompl::geometric::SARRT::getPlannerData(base::PlannerData &data) const
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

void ompl::geometric::SARRT::robustExtend(Motion *invalid_motion)
{
    // The list of motions to delete if they are not reconnectable
    std::vector<Motion *> to_delete;

    // The list of all disconnected nodes to reconnect
    std::vector<Motion *> disconnected_nodes;

    //We add the actual parent of the non feasible motion to the list to non feasible connection
    invalid_motion->non_robust_parent.push_back(invalid_motion->parent);

    // We add it to the list of disconnected nodes
    disconnected_nodes.push_back(invalid_motion);

    // Remove from the tree structure because we don't want to use it during the nearest search
    nn_->remove(invalid_motion);

    // We remove the node from its parent children list
    removeFromParent(invalid_motion);  

    //We get all the nodes that are connected after the invalid branch
    if(invalid_motion->children.size() > 0)
    {
        // Get its children
        removeChildrenFromTree(invalid_motion); 
    }

    //We try to reconnect all the disconnected nodes considering those which are to be deleted
    while(disconnected_nodes.size() > 0)
    {
        reconnect_nodes(to_delete, disconnected_nodes);
    }

    // We delete the nodes that have to be deleted
    for (auto &motion : to_delete)
    {
        // For visualization
        if(visualize_tree_)
        {
            visual_tools_->removeItem(motion->edgeVisualID);
            visual_tools_->removeItem(motion->stateVisualID);
        }

        // Free the state
        if (motion->state)
            si_->freeState(motion->state);
        // Delete it
        delete motion;
    }
}

void ompl::geometric::SARRT::getDisconnectedChildren(Motion *intermediate_motion, std::vector<Motion *> &disconnected_nodes)
{
    while(intermediate_motion->children.size()>0)
    {
        // We add it to the list of disconnected nodes
        disconnected_nodes.push_back(intermediate_motion->children[0]);
        // We remove the node from its parent children list
        removeFromParent(intermediate_motion->children[0]);
    }
}

void ompl::geometric::SARRT::removeChildrenFromTree(Motion *intermediate_motion)
{
    for(auto &child : intermediate_motion->children)
    {
        removeChildrenFromTree(child);
        // Remove from the tree structure because we don't want to use it during the nearest search
        nn_->remove(child);
        // For visualization
        // if(visualize_tree_)
        // {
        //     visual_tools_->removeItem(child->edgeVisualID);
        // }
    }
}

void ompl::geometric::SARRT::addChildrenToTree(Motion *intermediate_motion)
{
    for(auto &child : intermediate_motion->children)
    {
        // Re-add the motion to the tree structure because it was possible to reconnect the tree from the node "intermediate_motion"
        nn_->add(child);
        // For visualization
        // if(visualize_tree_)
        // {
        //     child->edgeVisualID = visual_tools_->addEdge(child->parent->state, child->state, reconnect_color_);
        // }

        addChildrenToTree(child);
    }
}

void ompl::geometric::SARRT::removeFromParent(Motion *m)
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

void ompl::geometric::SARRT::reconnect_nodes(std::vector<Motion *>& to_delete, std::vector<Motion *> &disconnected_nodes)
{
    Motion *to_reconnect = disconnected_nodes.at(0);

    // The list of the k-nearest neighboors to the current node 
    std::vector<Motion *> nbh_reconnect;

    // Find nearby neighbors of the new motion
    unsigned int k = 1;
    if(to_reconnect->non_robust_parent.size()>0)
        k = to_reconnect->non_robust_parent.size()+1; // non_robust_parent.size()+1 because we want at least to have one connection attempt
    nn_->nearestK(to_reconnect, k, nbh_reconnect);

    // collision check until a valid motion is found, if not we add the current disconnected node to the delete list
    bool new_valid_connection = false;
    for (auto &nbh: nbh_reconnect)
    {
        if(nbh == to_reconnect)
            continue;
        if(std::find(to_reconnect->non_robust_parent.begin(), to_reconnect->non_robust_parent.end(), nbh)!=to_reconnect->non_robust_parent.end()) // if the actual vertex selected is in the non robust list of the node we want to reconnect
            continue;
        if(si_->checkMotion(nbh->state, to_reconnect->state))
        {
            // Add this node to the new parent
            to_reconnect->parent = nbh;
            to_reconnect->parent->children.push_back(to_reconnect);
            nn_->add(to_reconnect); // We re-add the motion to the data structure
            // Re-add its unmodified children
            addChildrenToTree(to_reconnect);
            new_valid_connection = true;

            // For visualization
            if(visualize_tree_)
            {
                visual_tools_->removeItem(to_reconnect->edgeVisualID);
                to_reconnect->edgeVisualID = visual_tools_->addEdge(nbh->state, to_reconnect->state, reconnect_color_);
            }
            break;
        }
    }

    if(!new_valid_connection)
    {
        to_delete.push_back(to_reconnect);

        //We get all the nodes that are connected after the invalid branch
        if(to_reconnect->children.size() > 0)
        {
            // Get its children and add them to the disconnected_nodes list
            getDisconnectedChildren(to_reconnect, disconnected_nodes);
        }
    }
    
    disconnected_nodes.erase(disconnected_nodes.begin());
}

void ompl::geometric::SARRT::exportIter(int iteration, double running_time) const
{
    std::ofstream iter_overtime(iteration_file_, std::ios::app);
    std::cout.precision(6);

    iter_overtime<< "\t" << "Iteration:" << iteration << ";" << "Running time:" << running_time << "\n";

    iter_overtime.close();
}