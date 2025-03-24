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

#ifndef OMPL_BASE_DISCRETE_ROBUST_MOTION_VALIDATOR_
#define OMPL_BASE_DISCRETE_ROBUST_MOTION_VALIDATOR_

// SYSTEM
#include <queue>
#include <string>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <chrono>
#include <thread>  

// OMPL
#include "ompl/base/DiscreteMotionValidator.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/ProblemDefinition.h"
#include "ompl/util/Exception.h"

// Robots
#include "robots/Robot.h"

// State space
#include "planning/state_spaces/RobustStateSpace.h"

// Collision with bullet
#include "planning/state_checker/RobustStateValidityChecker.h"

namespace ompl
{
    namespace base
    {           
        OMPL_CLASS_FORWARD(DiscreteRobustMotionValidator);

        /** \brief A motion validator that only uses the state validity checker. Motions are checked for validity at a specified resolution. */
        class DiscreteRobustMotionValidator : public MotionValidator
        {
        public:

            /** \brief Constructor */
            DiscreteRobustMotionValidator(SpaceInformation *si, double dt, ompl::RobotPtr robot, const ros::NodeHandle& node_handler) : MotionValidator(si), robot_(robot), dt_(dt), nh_(node_handler)
            {
                setupValidator();
            }

            /** \brief Constructor */
            DiscreteRobustMotionValidator(const SpaceInformationPtr &si, double dt, const ompl::RobotPtr robot, const ros::NodeHandle& node_handler) : MotionValidator(si), robot_(robot), dt_(dt), nh_(node_handler)
            {
                setupValidator();
            }

            virtual ~DiscreteRobustMotionValidator()
            {
            }

            void freeStateVector(std::vector<ompl::base::State*> &states) const
            {
                for (auto &state :  states)
                {
                    if (state != nullptr)
                        si_->freeState(state);
                }
            }

            double getDt() const override
            {
                return dt_;
            }
            
            void setDt(const double new_dt) override
            {
                dt_ = new_dt;
            }

            //############################################################################################################
            // Interpolate a motion between two states and check its validity

            bool checkMotion(const State *s1, const State *s2) const override;

            bool checkMotion(const State *s1, const State *s2, std::pair<State *, double> &lastValid) const override
            {
                throw ompl::Exception("checkMotion", "not implemented");
            }

            bool checkMotion(const State *s1, const State *s2, std::vector<ompl::base::State*>& states) const override;
            
            bool checkMotionLearning(const State *s1, const State *s2, std::vector<ompl::base::State*>& states, torch::Tensor &traj_tensor) const override;

            bool checkMotionODE(const State *s1, const State *s2, std::vector<ompl::base::State*>& states) const override;

            bool checkMotionRandUp(const State *s1, const State *s2, const std::vector<std::vector<double>> &init_dynamic_states, 
                                        const std::vector<std::vector<double>> &rand_up_params, std::vector<std::vector<double>> &propagated_states, double padding) const override;

            //############################################################################################################
            // Re-check a precomputed motion

            /// \brief Re-check a precomputed motion in the 'traditional' way
            bool reCheckMotion(const std::vector <ompl::base::State*>& states) const override;

            /// \brief Re-check a precomputed motion using uncertainty tubes prediction
            bool reCheckMotionLearning(const std::vector<ompl::base::State*>& states, const torch::Tensor& traj_tensor) const override;

            /// \brief Re-check a precomputed motion using ODE tubes computation
            bool reCheckMotionODE(const std::vector<ompl::base::State*>& states) const override;

            /** \brief The function counting the time spent in the subprocess */
            void exportProfiling(std::string process_id, double running_time) const;

        private:

            void setupValidator();

            /// \brief Time step used for interpolation en collision checking
            double dt_;

            /// \brief Pointer to the Robot object
            ompl::RobotPtr robot_;

            /// \brief Pointer to the state space
            StateSpace *stateSpace_;

            /// \brief Node handler
            ros::NodeHandle nh_;

            /** \brief The file counting the profiling of the subprocess */
            std::string profiling_file_;
        };

    }
}

#endif
