/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Rutgers the State University of New Jersey, New Brunswick
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
*   * Neither the name of Rutgers University nor the names of its
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

/* Authors: Simon WASIELA */

#ifndef OMPL_GEOMETRIC_PLANNERS_SST_R_SASSTstar_
#define OMPL_GEOMETRIC_PLANNERS_SST_R_SASSTstar_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/objectives/MinimaxObjective.h"
#include "ompl/base/objectives/MaximizeMinClearanceObjective.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/tools/config/SelfConfig.h"

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
           @anchor gSST
           @par Short description
           \ref gSSTstar "SSTstar" (Assymptotically Optimal Stable Sparse RRT) is an asymptotically near-optimal incremental
           sampling-based motion planning algorithm. It is recommended for geometric problems
           to use an alternative method that makes use of a steering function. Using SSTstar for
           geometric problems does not take advantage of this function.
           @par External documentation
           Yanbo Li, Zakary Littlefield, Kostas E. Bekris, Sampling-based
           Asymptotically Optimal Sampling-based Kinodynamic Planning.
           [[PDF]](https://arxiv.org/abs/1407.2896)
        */
        class R_SASSTstar : public base::Planner
        {
        public:
            /** \brief Constructor */
            R_SASSTstar(const base::SpaceInformationPtr &si, const BulletVisualToolsPtr &visual_tools, const std::string &ros_namespace);

            ~R_SASSTstar() override;

            void setup() override;

            /** \brief Continue solving for some amount of time. Return true if solution was found. */
            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            void getPlannerData(base::PlannerData &data) const override;

            /** \brief Clear datastructures. Call this function if the
                input data to the planner has changed and you do not
                want to continue planning */
            void clear() override;

            /** In the process of randomly selecting states in the state
                space to attempt to go towards, the algorithm may in fact
                choose the actual goal state, if it knows it, with some
                probability. This probability is a real number between 0.0
                and 1.0; its value should usually be around 0.05 and
                should not be too large. It is probably a good idea to use
                the default value. */
            void setGoalBias(double goalBias)
            {
                goalBias_ = goalBias;
            }

            /** \brief Get the goal bias the planner is using */
            double getGoalBias() const
            {
                return goalBias_;
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

            /**
                \brief Set the radius for selecting nodes relative to random sample.

                This radius is used to mimic behavior of RRT* in that it promotes
                extending from nodes with good path cost from the root of the tree.
                Making this radius larger will provide higher quality paths, but has two
                major drawbacks; exploration will occur much more slowly and exploration
                around the boundary of the state space may become impossible. */
            void setSelectionRadius(double selectionRadius)
            {
                selectionRadius_ = selectionRadius;
            }

            /** \brief Get the selection radius the planner is using */
            double getSelectionRadius() const
            {
                return selectionRadius_;
            }

            /**
                \brief Set the radius for pruning nodes.

                This is the radius used to surround nodes in the witness set.
                Within this radius around a state in the witness set, only one
                active tree node can exist. This limits the size of the tree and
                forces computation to focus on low path costs nodes. If this value
                is too large, narrow passages will be impossible to traverse. In addition,
                children nodes may be removed if they are not at least this distance away
                from their parent nodes.*/
            void setPruningRadius(double pruningRadius)
            {
                pruningRadius_ = pruningRadius;
            }

            /** \brief Get the pruning radius the planner is using */
            double getPruningRadius() const
            {
                return pruningRadius_;
            }

            /** \brief Set the epsilon parameter */
            void setEpsilon(double eps)
            {
                epsilon_ = eps;
            }

            /** \brief Get the epsilon parameter */
            double getEpsilon() const
            {
                return epsilon_;
            }

            /** \brief Set the number of iterations of the current SST instance */
            void setNbIteration(int nbIterations)
            {
                maxIterationsInit_ = nbIterations;
            }

            /** \brief Get the number of iterations of the current SST instance */
            int getNbIteration() const
            {
                return maxIterationsInit_;
            }

            /** \brief Set a different nearest neighbors datastructure */
            template <template <typename T> class NN>
            void setNearestNeighbors()
            {
                if (nn_ && nn_->size() != 0)
                    OMPL_WARN("Calling setNearestNeighbors will clear all states.");
                clear();
                nn_ = std::make_shared<NN<Motion *>>();
                witnesses_ = std::make_shared<NN<Motion *>>();
                setup();
            }

        protected:
            /** \brief Representation of a motion

                This only contains pointers to parent motions as we
                only need to go backwards in the tree. */
            class Motion
            {
            public:
                Motion() = default;

                /** \brief Constructor that allocates memory for the state and the control */
                Motion(const base::SpaceInformationPtr &si)
                  : state_(si->allocState())
                {
                }

                virtual ~Motion() = default;

                virtual base::State *getState() const
                {
                    return state_;
                }
                virtual Motion *getParent() const
                {
                    return parent_;
                }
                base::Cost accCost_{0.};

                /** \brief The state contained by the motion */
                base::State *state_{nullptr};

                /** \brief The parent motion in the exploration tree */
                Motion *parent_{nullptr};

                /** \brief Number of children */
                unsigned numChildren_{0};

                /** \brief If inactive, this node is not considered for selection.*/
                bool inactive_{false};

                /** \brief The trajectory connecting the initial and final states of the motion */
                std::vector <ompl::base::State*> traj_;

                /** \brief The unique ID of the state for bullet debug visualization */
                long stateVisualID;

                /** \brief The unique ID of the edge leading to the motion state for bullet debug visualization */
                long edgeVisualID;
            };

            class Witness : public Motion
            {
            public:
                Witness() = default;

                Witness(const base::SpaceInformationPtr &si) : Motion(si)
                {
                }
                base::State *getState() const override
                {
                    return rep_->state_;
                }
                Motion *getParent() const override
                {
                    return rep_->parent_;
                }

                void linkRep(Motion *lRep)
                {
                    rep_ = lRep;
                }

                /** \brief The node in the tree that is within the pruning radius.*/
                Motion *rep_{nullptr};
            };

            /** \brief Finds the best node in the tree withing the selection radius around a random sample.*/
            Motion *selectNode(Motion *sample);

            /** \brief Find the closest witness node to a newly generated potential node.*/
            Witness *findClosestWitness(Motion *node);

            /** \brief Randomly propagate a new edge.*/
            base::State *monteCarloProp(Motion *m);

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
                return si_->distance(a->state_, b->state_);
            }

            /** \brief The function exporting the cost optimization */
            void exportCost(double cost, double running_time) const;

            /** \brief The function counting the number of iterations versus planning time */
            void exportIter(int iteration, double running_time) const;

            /** \brief The function counting the time spent in the subprocess */
            void exportProfiling(std::string process_id, double running_time) const;

            /// \brief Visual tools
            BulletVisualToolsPtr visual_tools_;

            /// \brief If we want to have a visual feedback of the tree 
            bool visualize_tree_;

            /** \brief State sampler */
            base::StateSamplerPtr sampler_;

            /** \brief A nearest-neighbors datastructure containing the tree of motions */
            std::shared_ptr<NearestNeighbors<Motion *>> nn_;

            /** \brief A nearest-neighbors datastructure containing the tree of witness motions */
            std::shared_ptr<NearestNeighbors<Motion *>> witnesses_;

            /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is
             * available) */
            double goalBias_{.05};

            /** \brief The maximum length of a motion to be added to a tree */
            double maxDistance_{5.};

            /** \brief The radius for determining the node selected for extension. */
            double selectionRadius_{5.};

            /** \brief The radius for determining the size of the pruning region. */
            double pruningRadius_{3.};

            /** \brief The epsilon parameter used to update the number of iterations of the next SST instance. */
            double epsilon_{0.5};

            /** \brief The number of iterations that the current SST instance has to perform. */
            int maxIterationsInit_{1000};

            /** \brief The random number generator */
            RNG rng_;

            /** \brief The best solution we found so far. */
            std::vector<base::State *> prevSolution_;

            /** \brief The best solution cost we found so far. */
            base::Cost prevSolutionCost_{std::numeric_limits<double>::quiet_NaN()};

            /** \brief The optimization objective. */
            base::OptimizationObjectivePtr opt_;

            /// \brief The ROS namespace  
            std::string ros_namespace_;

            /** \brief The file counting the number of iterations versus planning time */
            std::string iteration_file_;

            /** \brief The file counting the profiling of the subprocess */
            std::string profiling_file_;

            /** \brief The cost file */
            std::string cost_file_;
        };
    }
}

#endif