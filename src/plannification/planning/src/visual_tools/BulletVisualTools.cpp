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

// Bullet
#include "planning/visual_tools/BulletVisualTools.h"

// State space to recover xyz
#include "planning/state_spaces/kinospline/KinosplineStateSpace.h"

long BulletVisualTools::addStart(const ompl::base::State* state) const
{
  std::array<double, 3> xyz;

  if(robot_->getTrajType() == "Kinospline")
  {
    std::vector<double> q = state->as<ompl::base::KinosplineStateSpace::StateType>()->getQValues();

    if(robot_->getName() == "quadrotor")
    {
      for(int i = 0; i<3; i++)
        xyz[i] = q[i];
    }
    else if(robot_->getName() == "unicycle")
    {
      for(int i = 0; i<2; i++)
        xyz[i] = q[i];
      xyz[2] = 0.0;
    } 
  }
  else if(robot_->getTrajType() == "Dubins")
  {
    xyz[0] = state->as<ompl::base::DubinsUnicycleSpace::StateType>()->getX();
    xyz[1] = state->as<ompl::base::DubinsUnicycleSpace::StateType>()->getY();
    xyz[2] = 0.0;
  }
  
  return blt_client_->addUserDebugPoint(xyz, color_start_, goal_size_);
}

long BulletVisualTools::addGoal(const ompl::base::State* state) const
{
  std::array<double, 3> xyz;

  if(robot_->getTrajType() == "Kinospline")
  {
    std::vector<double> q = state->as<ompl::base::KinosplineStateSpace::StateType>()->getQValues();

    if(robot_->getName() == "quadrotor")
    {
      for(int i = 0; i<3; i++)
        xyz[i] = q[i];
    }
    else if(robot_->getName() == "unicycle")
    {
      for(int i = 0; i<2; i++)
        xyz[i] = q[i];
      xyz[2] = 0.0;
    } 
  }
  else if(robot_->getTrajType() == "Dubins")
  {
    xyz[0] = state->as<ompl::base::DubinsUnicycleSpace::StateType>()->getX();
    xyz[1] = state->as<ompl::base::DubinsUnicycleSpace::StateType>()->getY();
    xyz[2] = 0.0;
  }
  
  return blt_client_->addUserDebugPoint(xyz, color_goal_, goal_size_);
}

long BulletVisualTools::addState(const ompl::base::State* state) const
{
  std::array<double, 3> xyz;

  if(robot_->getTrajType() == "Kinospline")
  {
    std::vector<double> q = state->as<ompl::base::KinosplineStateSpace::StateType>()->getQValues();

    if(robot_->getName() == "quadrotor")
    {
      for(int i = 0; i<3; i++)
        xyz[i] = q[i];
    }
    else if(robot_->getName() == "unicycle")
    {
      for(int i = 0; i<2; i++)
        xyz[i] = q[i];
      xyz[2] = 0.0;
    }  
  }
  else if(robot_->getTrajType() == "Dubins")
  {
    xyz[0] = state->as<ompl::base::DubinsUnicycleSpace::StateType>()->getX();
    xyz[1] = state->as<ompl::base::DubinsUnicycleSpace::StateType>()->getY();
    xyz[2] = 0.0;
  }
  
  return blt_client_->addUserDebugPoint(xyz, color_st_, st_size_);
}

long BulletVisualTools::addEdge(const ompl::base::State* from, const ompl::base::State* to, const std::array<double, 3>& color) const
{
  std::array<double, 3> xyz_from;
  std::array<double, 3> xyz_to;

  if(robot_->getTrajType() == "Kinospline")
  {
    std::vector<double> qfrom = from->as<ompl::base::KinosplineStateSpace::StateType>()->getQValues();
    std::vector<double> qto = to->as<ompl::base::KinosplineStateSpace::StateType>()->getQValues();

    if(robot_->getName() == "quadrotor")
    {
      for(int i = 0; i<3; i++)
      {
        xyz_from[i] = qfrom[i];
        xyz_to[i] = qto[i];
      }
    }
    else if(robot_->getName() == "unicycle")
    {
      for(int i = 0; i<2; i++)
      {
        xyz_from[i] = qfrom[i];
        xyz_to[i] = qto[i];
      }
      xyz_from[2] = 0.0;
      xyz_to[2] = 0.0;
    } 
  }
  else if(robot_->getTrajType() == "Dubins")
  {
    xyz_from[0] = from->as<ompl::base::DubinsUnicycleSpace::StateType>()->getX();
    xyz_from[1] = from->as<ompl::base::DubinsUnicycleSpace::StateType>()->getY();
    xyz_from[2] = 0.0;

    xyz_to[0] = to->as<ompl::base::DubinsUnicycleSpace::StateType>()->getX();
    xyz_to[1] = to->as<ompl::base::DubinsUnicycleSpace::StateType>()->getY();
    xyz_to[2] = 0.0;
  }

  return blt_client_->addUserDebugLine(xyz_from, xyz_to, color, edge_width_);
}

std::vector<long> BulletVisualTools::addTraj(const std::vector<ompl::base::State*> &traj) const
{
  std::vector<long> ids;
  for(int i = 1; i<traj.size(); i++)
    ids.push_back(addEdge(traj.at(i-1), traj.at(i), traj_color_));
  return ids;
}

bool BulletVisualTools::removeItem(const int unique_id)
{
  return blt_client_->removeUserDebugItem(unique_id);
}

bool BulletVisualTools::removeAll()
{
  return blt_client_->removeAllUserDebugItem();
}
