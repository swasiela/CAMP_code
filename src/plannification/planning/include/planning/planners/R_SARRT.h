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

#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_R_SARRT_
#define OMPL_GEOMETRIC_PLANNERS_RRT_R_SARRT_

#include "ros/ros.h"

#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/base/OptimizationObjective.h"

#include <limits>
#include <vector>
#include <queue>
#include <deque>
#include <utility>
#include <list>
#include <fstream> 

// Visual tools
#include "planning/visual_tools/BulletVisualTools.h"

namespace ompl
{
    namespace geometric
    {
        /**
           @anchor gR_SARRT
           @par Short description
           R_SARRT is a robust implementation of the RRT using a sensitivity-based uncertainty tube computation at each collision check.
           @par External documentation
           J. Kuffner and S.M. LaValle, RRT-connect: An efficient approach to single-query path planning, in <em>Proc.
           2000 IEEE Intl. Conf. on Robotics and Automation</em>, pp. 995–1001, Apr. 2000. DOI:
           [10.1109/ROBOT.2000.844730](http://dx.doi.org/10.1109/ROBOT.2000.844730)<br>
           [[PDF]](http://ieeexplore.ieee.org/ielx5/6794/18246/00844730.pdf?tp=&arnumber=844730&isnumber=18246)
           [[more]](http://msl.cs.uiuc.edu/~lavalle/rrtpubs.html)

           Simon Wasiela, Marco Cognetti, Paolo Robuffo Giordano, Juan Cortés and Thierry Siméon, 
           "Robust Motion Planning with Accuracy Optimization based on Learned Sensitivity Metrics", in RA-L 2024.
        */

        /** \brief Robust version of the Rapidly-exploring Random Trees */
        class R_SARRT : public base::Planner
        {
        public:
            /** \brief Constructor */
            R_SARRT(const base::SpaceInformationPtr &si, const BulletVisualToolsPtr &visual_tools, const std::string &ros_namespace, bool addIntermediateStates = false);

            ~R_SARRT() override;

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

                /** \brief The trajectory connecting the initial and final states of the motion */
                std::vector <ompl::base::State*> traj_;

                /** \brief The unique ID of the state for bullet debug visualization */
                long stateVisualID;

                /** \brief The unique ID of the edge leading to the motion state for bullet debug visualization */
                long edgeVisualID;
            };

            /** \brief Free the memory allocated by this planner */
            void freeMemory();

            /** \brief Free the states contained in the traj_ object of a motion */
            void freeMotionTraj(std::vector<ompl::base::State*> traj)
            {
                for(auto &state : traj)
                {
                    if (state != nullptr)
                        si_->freeState(state);
                }
            }

            /** \brief Compute distance between motions (actually distance between contained states) */
            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distance(a->state, b->state);
            }

            bool isValidWithUncertainty(Motion *sol);

            /** \brief Measure the number of planner iterations agaisnt planning time */
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

            /** \brief The file to store the number of planner iterations agaisnt planning time */
            std::string iteration_file_;

        };
    }
}

#endif