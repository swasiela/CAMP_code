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

#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_SARRT_
#define OMPL_GEOMETRIC_PLANNERS_RRT_SARRT_

// OMPL
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/base/OptimizationObjective.h"
#include "ompl/base/Goal.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/goals/GoalState.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/base/samplers/InformedStateSampler.h"
#include "ompl/base/samplers/informed/RejectionInfSampler.h"
#include "ompl/base/samplers/informed/OrderedInfSampler.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/util/GeometricEquations.h"

// SYSTEM
#include "ros/ros.h"
#include <limits>
#include <vector>
#include <queue>
#include <deque>
#include <utility>
#include <list>
#include <iostream>
#include <fstream>
#include <string>
#include <stdlib.h>
#include <chrono>

// Visual tools
#include "planning/visual_tools/BulletVisualTools.h"

namespace ompl
{
    namespace geometric
    {
        /**
           @anchor gSARRT
           @par Short description
           Sensitivity-Aware RRT
           @par External documentation
           Wasiela, S., Giordano, P. R., Cortés, J., & Simeon, T. (2023, May). 
           A sensitivity-aware motion planner (samp) to generate intrinsically-robust trajectories. 
           In 2023 IEEE International Conference on Robotics and Automation (ICRA) (pp. 12707-12713). IEEE.
        */

        /** \brief Sensitivity-Aware RRT */
        class SARRT : public base::Planner
        {
        public:
            /** \brief Constructor */
            SARRT(const base::SpaceInformationPtr &si, const BulletVisualToolsPtr &visual_tools, const std::string &ros_namespace, bool addIntermediateStates = false);

            ~SARRT() override;

            void getPlannerData(base::PlannerData &data) const override;

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            void clear() override;

            /** \brief Set the goal bias

                In the process of randomly selecting states in
                the state space to attempt to go towards, the
                algorithm may in fact choose the actual goal state, if
                it knows it, with some probability. This probability
                is a real number between 0.0 and 1.0; its value should
                usually be around 0.05 and should not be too large. It
                is probably a good idea to use the default value. */
            void setGoalBias(double goalBias)
            {
                goalBias_ = goalBias;
            }

            /** \brief Get the goal bias the planner is using */
            double getGoalBias() const
            {
                return goalBias_;
            }

            /** \brief Return true if the intermediate states generated along motions are to be added to the tree itself
             */
            bool getIntermediateStates() const
            {
                return addIntermediateStates_;
            }

            /** \brief Specify whether the intermediate states generated along motions are to be added to the tree
             * itself */
            void setIntermediateStates(bool addIntermediateStates)
            {
                addIntermediateStates_ = addIntermediateStates;
            }

            /** \brief Set the range the planner is supposed to use.

                This parameter greatly influences the runtime of the
                algorithm. It represents the maximum length of a
                motion to be added in the tree of motions. */
            void setRange(double distance)
            {
                maxDistance_ = distance;
            }

            /** \brief Get the range the planner is using */
            double getRange() const
            {
                return maxDistance_;
            }

            /** \brief Set a different nearest neighbors datastructure */
            template <template <typename T> class NN>
            void setNearestNeighbors()
            {
                if (nn_ && nn_->size() != 0)
                    OMPL_WARN("Calling setNearestNeighbors will clear all states.");
                clear();
                nn_ = std::make_shared<NN<Motion *>>();
                setup();
            }

            void setup() override;

        protected:
            /** \brief Representation of a motion

                This only contains pointers to parent motions as we
                only need to go backwards in the tree. */
            class Motion
            {
            public:
                Motion() = default;

                /** \brief Constructor that allocates memory for the state */
                Motion(const base::SpaceInformationPtr &si) : state(si->allocState())
                {
                }

                ~Motion() = default;

                /** \brief The state contained by the motion */
                base::State *state{nullptr};

                /** \brief The parent motion in the exploration tree */
                Motion *parent{nullptr};

                /** \brief The unique ID of the state for bullet debug visualization */
                long stateVisualID;

                /** \brief The unique ID of the edge leading to the motion state for bullet debug visualization */
                long edgeVisualID;

                /** \brief List of already disconnected nodes because of tube collisions for this node */
                std::vector<Motion *> non_robust_parent;

                /** \brief The set of motions descending from the current motion */
                std::vector<Motion *> children;
            };

            /** \brief Free the memory allocated by this planner */
            void freeMemory();

            /** \brief Compute distance between motions (actually distance between contained states) */
            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distance(a->state, b->state);
            }

            /** \brief Reconnect all the disconnected children of the first non robust node find in the solution */
            void robustExtend(Motion *motion_to_rewire);

            /** \brief Get the disconnected children of intermediate_motion */
            void getDisconnectedChildren(Motion *intermediate_motion, std::vector<Motion *> &disconnected_nodes);

            /** \brief Temporary remove the children of the intermediate_motion from the datastructure so they wont be considered in the nearest search */
            void removeChildrenFromTree(Motion *intermediate_motion);

            /** \brief Re-add the chikdren of intermediate_motion in the datastructure */
            void addChildrenToTree(Motion *intermediate_motion);

            /** \brief Removes the given motion from the parent's child list */
            void removeFromParent(Motion *m);

            /** \brief Try to reconnect all the disconneted nodes, otherwise add them to the delete list */
            void reconnect_nodes(std::vector<Motion *>& to_delete, std::vector<Motion *> &disconnected_nodes);

            /** \brief The function counting the number of iterations versus planning time */
            void exportIter(int iteration, double running_time) const;

            /// \brief Visual tools
            BulletVisualToolsPtr visual_tools_;

            /// \brief If we want to have a visual feedback of the tree 
            bool visualize_tree_;

            /** \brief State sampler */
            base::StateSamplerPtr sampler_;

            /** \brief A nearest-neighbors datastructure containing the tree of motions */
            std::shared_ptr<NearestNeighbors<Motion *>> nn_;

            /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is
             * available) */
            double goalBias_{.05};

            /** \brief The maximum length of a motion to be added to a tree */
            double maxDistance_{0.};

            /** \brief Flag indicating whether intermediate states are added to the built tree of motions */
            bool addIntermediateStates_;

            /** \brief The random number generator */
            RNG rng_;

            /** \brief The most recent goal motion.  Used for PlannerData computation */
            Motion *lastGoalMotion_{nullptr};

            /** \brief Objective we're optimizing */
            base::OptimizationObjectivePtr opt_;

            /// \brief The ROS namespace  
            std::string ros_namespace_;

            /** \brief The file counting the number of iterations versus planning time */
            std::string iteration_file_;

            /** \brief Color for the reconnected edges */
            std::array<double, 3> reconnect_color_ = {0.51, 0.0, 0.51};
        };
    }
}

#endif