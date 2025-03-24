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

#ifndef VISUAL_TOOLS_H
#define VISUAL_TOOLS_H

// Robots
#include "robots/Robot.h"

// State space
#include "planning/state_spaces/kinospline/KinosplineStateSpace.h"
#include "planning/state_spaces/dubins/DubinsUnicycleSpace.h"

// Bullet
#include "planning/bullet/PhysicsServers.h"


class BulletVisualTools
{
    public:
        BulletVisualTools(owds::BulletClient* blt_client, const ompl::RobotPtr robot) : blt_client_(blt_client), robot_(robot){}
        ~BulletVisualTools(){}

        /** \brief To visualize the start state. Return the unique ID of the visualized point */
        long addStart(const ompl::base::State* state) const;

        /** \brief To visualize the goal state. Return the unique ID of the visualized point */
        long addGoal(const ompl::base::State* state) const;

        /** \brief To visualize the state. Return the unique ID of the visualized point */
        long addState(const ompl::base::State* state) const;

        /** \brief To visualize the edge between two state. Return the unique ID of the edge */
        long addEdge(const ompl::base::State* from, const ompl::base::State* to, const std::array<double, 3>& color = {0.96, 0.56, 0.26}) const;

        /** \brief To visualize a vector of states */
        std::vector<long> addTraj(const std::vector<ompl::base::State*> &traj) const;

        /** \brief Remove the item associated to the unique id */
        bool removeItem(const int unique_id);

        /** \brief Remove all the items */
        bool removeAll();

        /** \brief Get the robot object */
        ompl::RobotPtr getRobot()
        {
          return robot_;
        }

    private:

        /// \brief Bullet server
        owds::BulletClient* blt_client_;

        /** \brief Pointer to the Robot object */
        ompl::RobotPtr robot_;

        /** \brief Color for a tree state */
        std::array<double, 3> color_st_ = {0.1, 0.4, 0.8};

        /** \brief Color for the start state */
        std::array<double, 3> color_start_ = {0.153, 0.38, 0.13};

        /** \brief Color for the goal state */
        std::array<double, 3> color_goal_ = {0.8, 0.1, 0.2};

        /** \brief Color for the final trajectory */
        std::array<double, 3> traj_color_ = {0.87, 0.15, 0.15};

        /** \brief Point size for a tree state */
        double st_size_ = 5.0;

        /** \brief Point size for the starting or goal state */
        double goal_size_ = 10.0;

        /** \brief Edge width */
        double edge_width_ = 5.0;

};
typedef std::shared_ptr<BulletVisualTools> BulletVisualToolsPtr;
#endif
