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

#ifndef OMPL_GEOMETRIC_PLANNERS_RandUPRRT_RandUPRRT_
#define OMPL_GEOMETRIC_PLANNERS_RandUPRRT_RandUPRRT_

#include "ros/ros.h"

#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/base/OptimizationObjective.h"

// Visual tools
#include "planning/visual_tools/BulletVisualTools.h"

// System
#include <limits>
#include <vector>
#include <queue>
#include <deque>
#include <utility>
#include <list>

namespace ompl
{
    namespace geometric
    {
        /**
           @anchor RandUPRRT
           @par Short description
           RandUPRRT C++ implementation based on the Python one found in : https://github.com/StanfordASL/randUP_RRT.git. Mainly based on the quadrotor example.
           @par External documentation
           Wu, A., Lew, T., Solovey, K., Schmerling, E., & Pavone, M. (2022, September). 
           Robust-RRT: Probabilistically-complete motion planning for uncertain nonlinear systems. 
           In The International Symposium of Robotics Research (pp. 538-554). Cham: Springer Nature Switzerland.
        */

        /** \brief Randomized Uncertainty Propagation RRT */
        class RandUPRRT : public base::Planner
        {
        public:
            /** \brief Constructor */
            RandUPRRT(const base::SpaceInformationPtr &si, const BulletVisualToolsPtr &visual_tools, const std::string &ros_namespace, bool addIntermediateStates = false);

            ~RandUPRRT() override;

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

            /** \brief Set the number of particles the planner is using */
            void setNbParticles(int nbParticle)
            {
                nbParticles_ = nbParticle;
            }

            /** \brief Get the number of particles the planner is using */
            double getNbParticles() const
            {
                return nbParticles_;
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

            /** \brief Set the padding value the planner is supposed to use. */
            void setPaddingValue(double padding)
            {
                padding_ = padding;
            }

            /** \brief Get the padding value the planner is using */
            double getPaddingValue() const
            {
                return padding_;
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

                /** \brief The cost up to this motion */
                base::Cost cost;

                /** \brief The incremental cost of this motion's parent to this motion (this is stored to save distance
                 * computations in the updateChildCosts() method) */
                base::Cost incCost;

                /** \brief Propagated uncertain states */ 
                std::vector<std::vector<double>> rand_up_states_;

                /** \brief The unique ID of the state for bullet debug visualization */
                long stateVisualID;

                /** \brief The unique ID of the edge leading to the motion state for bullet debug visualization */
                long edgeVisualID;
            };

            /** \brief Free the memory allocated by this planner */
            void freeMemory();

            /** \brief Compute distance between motions (actually distance between contained states) */
            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distance(a->state, b->state);
            }

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

            /** \brief The number of particles used for sampling and propagating uncertainties. */
            int nbParticles_{1};

            /** \brief The maximum length of a motion to be added to a tree */
            double maxDistance_{0.};

            /** \brief Time step use for planning, specified in the yaml file*/
            double dt_planning_{0.05};

            /** \brief Padding value used for the epsilon-RandUp approximation*/
            double padding_{0.0};

            /** \brief Flag indicating whether intermediate states are added to the built tree of motions */
            bool addIntermediateStates_;

            /** \brief The random number generator */
            RNG rng_;

            /** \brief The most recent goal motion.  Used for PlannerData computation */
            Motion *lastGoalMotion_{nullptr};

            /** \brief Objective we're optimizing */
            base::OptimizationObjectivePtr opt_;

            /** \brief Best cost found so far by algorithm */
            base::Cost bestCost_{std::numeric_limits<double>::quiet_NaN()};

            /** \brief The cost at which the graph was last pruned */
            base::Cost prunedCost_{std::numeric_limits<double>::quiet_NaN()};

            /** \brief To count the number of planner iteration */
            int iterations_ = 0;

            /** \brief Randomize system parameters (delta p) */
            std::vector<std::vector<double>> rand_up_params_;

            /** \brief The range of the uncertain parameters */
            std::vector<double> delta_p_;

            /// \brief The ROS namespace  
            std::string ros_namespace_;

            /** \brief The file to store the number of planner iterations agaisnt planning time */
            std::string iteration_file_;
        };
    }
}

#endif