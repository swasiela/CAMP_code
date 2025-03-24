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

#ifndef ROBUST_STATE_VALIDITY_CHECKER_H
#define ROBUST_STATE_VALIDITY_CHECKER_H

// OMPL
#include "ompl/base/StateValidityChecker.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/util/Exception.h"

// Robots
#include "robots/Robot.h"

// State space
#include "planning/state_spaces/RobustStateSpace.h"

// Collision with bullet
#include "planning/bullet/PhysicsServers.h"

// Utils
#include "utils/project_ellipsoid.h"

// System
#include <iostream>
#include <chrono>

namespace ompl
{
    namespace base
    {
        OMPL_CLASS_FORWARD(RobustStateValidityChecker);

        class RobustStateValidityChecker : public StateValidityChecker
        {
            public:
                /** \brief Constructor */
                RobustStateValidityChecker(SpaceInformation *si, const ompl::RobotPtr robot, owds::BulletClient* blt_client, const ros::NodeHandle& node_handler) : StateValidityChecker(si), robot_(robot), blt_client_(blt_client), nh_(node_handler)
                {
                    setup();
                }

                /** \brief Constructor */
                RobustStateValidityChecker(const SpaceInformationPtr &si, const ompl::RobotPtr robot, owds::BulletClient* blt_client, const ros::NodeHandle& node_handler) : StateValidityChecker(si), robot_(robot), blt_client_(blt_client), nh_(node_handler)
                {
                    setup();
                }

                ~RobustStateValidityChecker(){}

                // Classic state validity checker
                bool isValid(const ompl::base::State *state) const override;

                //############################################################################################################
                // Collision checking functions in the input and state spaces

                /// \brief Check the feasability of the control inputs according to some maximum allowed values
                bool isInputsValid(std::vector<double> &control_inputs, std::vector<double> &uncertainties) const override;

                /// \brief Check the validity of the given state w.r.t. the environment
                bool isStateValid(const std::vector<double> &state, const std::vector<double> &uncertainty) const override;

                /// \brief The first tube mode that use a constant spheric robot growth in all direction 
                bool sphericGrowth(const b3ContactInformation &cc_blt_ellipsoid, const std::vector<double> &uncertainty) const;

                /** \brief Approximate the hull of an ellipsoid whose semi axes are equal to the worst case deviation in each direction,
                * the original ellipsoid may not be contain within it */
                bool approximateEllipsoid(const std::vector<double> &state, const std::vector<double> &uncertainty) const;

                /** \brief The third tube mode that approximate the hull of a rectangle whose axes are equal to the worst case deviation in each direction, 
                * fully encapsulate the original ellispoid but may be conservative */
                bool approximateRectangle(const std::vector<double> &state, const std::vector<double> &uncertainty) const;

                /** \brief The second tube mode that computes the current robot AABB, and create a new AABB taking into account the uncertainty ellipsoid AABB */
                bool uncertainAABB(const std::vector<double> &state, const std::vector<double> &uncertainty) const;

                /// \brief Check the validity of the given state w.r.t. the workspace
                bool checkWorkspace(const std::vector<double> &state) const;
     
            private:

                /** \brief Setup the validity checker */
                void setup()
                {
                    // Get initial robot configuration
                    std::vector<double> init_q = robot_->getInitConfiguration();
                    // Bullet
                    robot_id_blt_ = blt_client_->loadURDF(robot_->getUrdf(), {init_q.at(0),init_q.at(1),init_q.at(2)}, {init_q.at(3),init_q.at(4),init_q.at(5),init_q.at(6)});
                    env_id_blt_ = blt_client_->loadURDF(robot_->getScene(), {0,0,0}, {0,0,0,1});
                    blt_client_->changeRgbaColor(robot_id_blt_, 0, {1,0,0,1});

                    sleep(2.0);

                    // Setup the workspace bounds
                    qmax_ = robot_->getQUpperBounds();
                    qmin_ = robot_->getQLowerBounds();

                    // The planner config path
                    const std::string key_params = "planning_parameters/";

                    // Load the tube mode for collision checking
                    if(!nh_.getParam(key_params + "tube_mode", tube_mode_)){
                        ROS_WARN("Cannot load 'tube_mode' param from ROS server: default tube_mode_ = 1");
                        tube_mode_ = 1;
                    } 

                    stateSpace_ = si_->getStateSpace().get();
                    if (!stateSpace_)
                        throw Exception("No state space for state validity checker");
                }

                /// \brief Pointer to the state space
                ompl::base::StateSpace *stateSpace_;

                /** \brief Pointer to the Robot object */
                ompl::RobotPtr robot_;

                /// \brief Pointer to the Bullet client for CC
                owds::BulletClient* blt_client_;

                // Node handler
                ros::NodeHandle nh_;

                /// \brief ID to the robot and scene in the Bullet client
                int robot_id_blt_, env_id_blt_;

                /** \brief The mode used for collision checking with the tubes */
                int tube_mode_;

                /** \brief The workspace bounds */
                std::vector<double> qmax_, qmin_; 
        };
    }
}
#endif
