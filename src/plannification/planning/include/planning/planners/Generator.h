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

#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_Generator_
#define OMPL_GEOMETRIC_PLANNERS_RRT_Generator_

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

// State space
#include "planning/state_spaces/dubins/DubinsUnicycleSpace.h"
#include "planning/state_spaces/kinospline/KinosplineStateSpace.h"
#include "planning/state_spaces/kinospline/KinoQuadrotorSpace.h"

// Visual tools
#include "planning/visual_tools/BulletVisualTools.h"

namespace ompl
{
    namespace geometric
    {
        /**
           @anchor gRRT
           @par Short description
           Database generator, produces splines, call the sensitivity services and then export the trajectory, prune branches that have been exported.
        */

        /** \brief Rapidly-exploring Random Trees */
        class Generator : public base::Planner
        {
        public:
            /** \brief Constructor */
            Generator(const base::SpaceInformationPtr &si, const BulletVisualToolsPtr &visual_tools, const std::string &ros_namespace, bool addIntermediateStates = false);

            ~Generator() override;

            void getPlannerData(base::PlannerData &data) const override;

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            void clear() override;

            /** \brief Set the range the planner is supposed to use.

                This parameter greatly influences the runtime of the
                algorithm. It represents the maximum length of a
                motion to be added in the tree of motions. */
            void setLocalLength(double distance)
            {
                if(distance == 0.0)
                    Tl_ = std::numeric_limits<double>::max();
                else
                    Tl_ = distance;
            }

            /** \brief Get the range the planner is using */
            double getLocalLength() const
            {
                return Tl_;
            }

            /** \brief Set the length a trajectory should reach before beeing exported */
            void setGlobalLength(double distance)
            {
                Tf_ = distance;
            }

            /** \brief Get the global length the generator is using */
            double getGlobalLength() const
            {
                return Tf_;
            }

            /** \brief Set time step used to perform simulation */
            void setDtPlanning(double dt)
            {
                dt_planning_ = dt;
            }

            /** \brief Get time step used to perform simulation */
            double getDtPlanning() const
            {
                return dt_planning_;
            }

            /** \brief Set the number of global trajectories to be generated */
            void setNbData(int nb)
            {
                nb_data_ = nb;
            }

            /** \brief Get the number of global trajectories to be generated */
            int getNbData() const
            {
                return nb_data_;
            }

            /** \brief Set the number maximum filtering attempt */
            void setNbFilter(int nb)
            {
                max_filtering_ = nb;
            }

            /** \brief Get the number maximum filtering attempt */
            int getNbFilter() const
            {
                return max_filtering_;
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

                /** \brief The cost up to this motion */
                base::Cost cost;

                /** \brief The incremental cost of this motion's parent to this motion (this is stored to save distance
                 * computations in the updateChildCosts() method) */
                base::Cost incCost;

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

            template<typename T>
            void pop_front(std::vector<T>& vec)
            {
                assert(!vec.empty());
                vec.erase(vec.begin());
            }

            /** \brief Reset the tree to the starting state */
            void resetTree();

            void freeStateVector(std::vector<ompl::base::State*> &states) const
            {
                for (auto &state :  states)
                {
                    if (state != nullptr)
                        si_->freeState(state);
                }
            }

            std::vector<ompl::base::State*> interpolate(Motion* motion, std::vector<std::vector<double>>& traj_vectorized);

            /// \brief Filter the outliers and then export the trajectory and the outputs in a single file. Not suitable for exporting the gains values too.
            bool filterAndExportTrajectory(const std::vector<std::vector<double>> &traj, const std::vector<std::vector<double>> &control_inputs, const std::vector<std::vector<double>> &radii);

            /// \brief Only export the trajectory
            bool exportTrajectoryOnly(const std::vector<std::vector<double>> &traj);

            /// \brief Only export the outputs with the gains
            bool filterAndExportOutputsOnly(const std::vector<std::vector<double>> &robot_gains, const std::vector<std::vector<double>> &control_inputs, const std::vector<std::vector<double>> &radii);

            /// \brief Visual tools
            BulletVisualToolsPtr visual_tools_;

            /// \brief If we want to have a visual feedback of the tree 
            bool visualize_tree_;

            /** \brief State sampler */
            base::StateSamplerPtr sampler_;

            /** \brief A nearest-neighbors datastructure containing the tree of motions */
            std::shared_ptr<NearestNeighbors<Motion *>> nn_;

            /** \brief The maximum length of a motion to be added to a tree */
            double Tl_{0.};

            /** \brief The maximum length of a motion to be exported */
            double Tf_{0.};

            /** \brief The time step used for CC */
            double dt_planning_{0.05};

            /** \brief Number of datas to generate */
            int nb_data_{1000};

            /** \brief Number of maximum outliers filtering attempts */
            int max_filtering_{0};

            /** \brief The random number generator */
            RNG rng_;

            /** \brief Objective we're optimizing */
            base::OptimizationObjectivePtr opt_;

            /** \brief The nominal controller gains */
            std::vector<std::vector<double>> gains_;

            /// \brief The ROS namespace  
            std::string ros_namespace_;

            /** \brief The file where to export the database */
            std::string export_file_no_gains_ = "/home/swasiela/CAMP/src/results/learning_file.csv";

            /** \brief The file where to export the database */
            std::string export_traj_w_gains_ = "/home/swasiela/CAMP/src/results/inputswGains.csv";

            /** \brief The file where to export the database */
            std::string export_outputs_w_gains_ = "/home/swasiela/CAMP/src/results/OutputswGains.csv";
        };
    }
}

#endif