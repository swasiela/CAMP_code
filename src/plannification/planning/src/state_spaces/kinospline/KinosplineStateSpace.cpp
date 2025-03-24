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

#include "planning/state_spaces/kinospline/KinosplineStateSpace.h"

#include <cstring>
#include <ros/ros.h>

// ###########################################################################################################################################
// STATE MANAGEMENT FUNCTIONS

ompl::base::State* ompl::base::KinosplineStateSpace::allocState() const{
    StateType *state = new StateType(robot_->getDesiredDim());
    allocStateComponents(state);
    return state;
}

void ompl::base::KinosplineStateSpace::freeState(ompl::base::State *state) const{
    CompoundStateSpace::freeState(state);
}

void ompl::base::KinosplineStateSpace::copyState(ompl::base::State *destination, const ompl::base::State *source) const{
    // copy the state data
    destination->as<StateType>()->setQValues(source->as<StateType>()->getQValues());
    destination->as<StateType>()->setQdotValues(source->as<StateType>()->getQdotValues());
    destination->as<StateType>()->setQddotValues(source->as<StateType>()->getQddotValues());

    // copy the Sensitivity initial conditions 
    if(source->as<StateType>()->PI_.size() > 0)
      destination->as<StateType>()->PI_ = source->as<StateType>()->PI_;
    if(source->as<StateType>()->PI_xi_.size() > 0)
      destination->as<StateType>()->PI_xi_ = source->as<StateType>()->PI_xi_;
    if(source->as<StateType>()->xi_.size() > 0)
        destination->as<StateType>()->xi_ = source->as<StateType>()->xi_;

    // copy the hidden state  
    if(source->as<StateType>()->h0_.sizes()[0]>0)
      destination->as<StateType>()->h0_ = source->as<StateType>()->h0_;

    // copy the nominal state
    if(source->as<StateType>()->nominal_state_.size()>0)
      destination->as<StateType>()->nominal_state_ = source->as<StateType>()->nominal_state_;

    destination->as<StateType>()->st_time_ = source->as<StateType>()->st_time_;
}

// ###########################################################################################################################################
// INTERPOLATE FUNCTIONS

void ompl::base::KinosplineStateSpace::interpolate(const ompl::base::State *from, const ompl::base::State *to, const double t, ompl::base::State *state) const
{
  ///Get q, qdot and qddot
  std::vector<double> from_q, from_qdot, from_qddot, to_q, to_qdot, to_qddot;

  from_q = from->as<StateType>()->getQValues();
  to_q = to->as<StateType>()->getQValues();
  from_qdot = from->as<StateType>()->getQdotValues();
  to_qdot = to->as<StateType>()->getQdotValues();
  from_qddot = from->as<StateType>()->getQddotValues();
  to_qddot = to->as<StateType>()->getQddotValues();

  kdtp::LocalPathPtr LP = compute_local_path(from_q, from_qdot, from_qddot, to_q, to_qdot, to_qddot, robot_->getKinodynamicLimits());

  // t in [0;1], it represents a percentage of the computed distance
  // double time = distance(from, to)*t;

  //Directly use the fraction when the trajectory is too long, easier to ensure that the resulting trajectory is indeed in the range use for the learning
  double time = 0.0;
  if(t >= 0.05)
    time = t;   
  else 
    time = LP->getDuration()*t;

  to_q.clear();
  to_qdot.clear();
  to_qddot.clear();

  for(int i = 0; i<from_q.size(); i++)
  {
    to_q.push_back(LP->getPositionAt(time)[i]);
    to_qdot.push_back(LP->getVelocityAt(time)[i]);
    to_qddot.push_back(LP->getAccelerationAt(time)[i]);
  }

  state->as<StateType>()->setQValues(to_q);
  state->as<StateType>()->setQdotValues(to_qdot);
  state->as<StateType>()->setQddotValues(to_qddot);

  state->as<StateType>()->st_time_ = from->as<StateType>()->st_time_ + time;

  return;
}

void ompl::base::KinosplineStateSpace::interpolate(const ompl::base::State *from, const ompl::base::State *to, const double dt, std::vector<ompl::base::State*>& states) const
{
  ///Get q, qdot and qddot
  std::vector<double> from_q, from_qdot, from_qddot, to_q, to_qdot, to_qddot;

  from_q = from->as<StateType>()->getQValues();
  to_q = to->as<StateType>()->getQValues();
  from_qdot = from->as<StateType>()->getQdotValues();
  to_qdot = to->as<StateType>()->getQdotValues();
  from_qddot = from->as<StateType>()->getQddotValues();
  to_qddot = to->as<StateType>()->getQddotValues();

  // Copy the initial state
  State* from_cpy = allocState();
  copyState(from_cpy, from);
  states.push_back(from_cpy);

  kdtp::LocalPathPtr LP = compute_local_path(from_q, from_qdot, from_qddot, to_q, to_qdot, to_qddot, robot_->getKinodynamicLimits());
  double time = dt;

  while(time <= LP->getDuration())
  {
    ompl::base::State *state = allocState();
    to_q.clear();
    to_qdot.clear();
    to_qddot.clear();

    //Position of the aerial vehicle
    for(int i = 0; i<from_q.size(); i++)
    {
      to_q.push_back(LP->getPositionAt(time)[i]);
      to_qdot.push_back(LP->getVelocityAt(time)[i]);
      to_qddot.push_back(LP->getAccelerationAt(time)[i]);
    }

    state->as<StateType>()->setQValues(to_q);
    state->as<StateType>()->setQdotValues(to_qdot);
    state->as<StateType>()->setQddotValues(to_qddot);
    state->as<StateType>()->st_time_ = from->as<StateType>()->st_time_ + time;
    states.push_back(state);
    
    time+=dt;
  }

  // Copy the final state and add time instant
  State* to_cpy = allocState();
  copyState(to_cpy, to);
  to_cpy->as<StateType>()->st_time_ = from->as<StateType>()->st_time_ + LP->getDuration();
  states.push_back(to_cpy);

  return;
}

void ompl::base::KinosplineStateSpace::interpolateLearning(const ompl::base::State *from, const ompl::base::State *to, const double dt, 
    std::vector<ompl::base::State*>& states, std::vector<std::vector<float>> &vector_to_tensor) const
{
  //Get q, qdot and qddot
  std::vector<double> from_q, from_qdot, from_qddot, to_q, to_qdot, to_qddot;
  std::vector<std::vector<double>> des_learning;

  from_q = from->as<StateType>()->getQValues();
  to_q = to->as<StateType>()->getQValues();
  from_qdot = from->as<StateType>()->getQdotValues();
  to_qdot = to->as<StateType>()->getQdotValues();
  from_qddot = from->as<StateType>()->getQddotValues();
  to_qddot = to->as<StateType>()->getQddotValues();

  des_learning.push_back(from_qdot);
  des_learning.push_back(from_qddot);
  vector_to_tensor.push_back(robot_->getRobotLearningInput(des_learning));

  // Copy the initial state
  State* from_cpy = allocState();
  copyState(from_cpy, from);
  states.push_back(from_cpy);

  kdtp::LocalPathPtr LP = compute_local_path(from_q, from_qdot, from_qddot, to_q, to_qdot, to_qddot, robot_->getKinodynamicLimits());
  double time = dt;

  while(time <= LP->getDuration())
  {
    ompl::base::State *state = allocState();
    to_q.clear();
    to_qdot.clear();
    to_qddot.clear();
    des_learning.clear();

    //Position of the aerial vehicle
    for(int i = 0; i<from_q.size(); i++)
    {
      to_q.push_back(LP->getPositionAt(time)[i]);
      to_qdot.push_back(LP->getVelocityAt(time)[i]);
      to_qddot.push_back(LP->getAccelerationAt(time)[i]);
    }

    state->as<StateType>()->setQValues(to_q);
    state->as<StateType>()->setQdotValues(to_qdot);
    state->as<StateType>()->setQddotValues(to_qddot);
    state->as<StateType>()->st_time_ = from->as<StateType>()->st_time_ + time;
    states.push_back(state);

    des_learning.push_back(to_qdot);
    des_learning.push_back(to_qddot);
    vector_to_tensor.push_back(robot_->getRobotLearningInput(des_learning));
    
    time+=dt;
  }
  
  // Copy the final state
  State* to_cpy = allocState();
  copyState(to_cpy, to);
  to_cpy->as<StateType>()->st_time_ = from->as<StateType>()->st_time_ + LP->getDuration();
  states.push_back(to_cpy);

  return;
}

// ###########################################################################################################################################
// DISTANCE FUNCTIONS

double ompl::base::KinosplineStateSpace::distancePos(const ompl::base::State *state1, const ompl::base::State *state2) const{
  double dist = 0.;

  dist += as<RealVectorStateSpace>(0)->distance(state1->as<CompoundState>()->components[0], state2->as<CompoundState>()->components[0]);

  return dist;
}

double ompl::base::KinosplineStateSpace::distanceVel(const ompl::base::State *state1, const ompl::base::State *state2) const{
  double dist = 0.;

  dist += as<RealVectorStateSpace>(1)->distance(state1->as<CompoundState>()->components[1], state2->as<CompoundState>()->components[1]);

  return dist;
}

double ompl::base::KinosplineStateSpace::distanceAcc(const ompl::base::State *state1, const ompl::base::State *state2) const{
  double dist = 0.;

  dist += as<RealVectorStateSpace>(2)->distance(state1->as<CompoundState>()->components[2], state2->as<CompoundState>()->components[2]);

  return dist;
}

double ompl::base::KinosplineStateSpace::distanceState(const ompl::base::State *state1, const ompl::base::State *state2) const{
  double dist = 0.;

  dist += distancePos(state1, state2);
  dist += distanceVel(state1, state2);
  dist += distanceAcc(state1, state2);

  return dist;
}

double ompl::base::KinosplineStateSpace::distance(const ompl::base::State *state1, const ompl::base::State *state2) const{

  double dist = std::numeric_limits<double>::max();

  if(opt_tag_ == "KS_time")
  {
    dist = callKinoMetric(state1, state2);
  }
  else
  {
    dist = 0.0;
    dist+=distanceState(state1, state2);
  }
  return dist;
}

double ompl::base::KinosplineStateSpace::get_interpolate_time(const ompl::base::State *from, const ompl::base::State *to) const{

  double time = 0.0;

  //Get q, qdot and qddot
  std::vector<double> from_q, from_qdot, from_qddot, to_q, to_qdot, to_qddot;

  from_q = from->as<StateType>()->getQValues();
  to_q = to->as<StateType>()->getQValues();
  from_qdot = from->as<StateType>()->getQdotValues();
  to_qdot = to->as<StateType>()->getQdotValues();
  from_qddot = from->as<StateType>()->getQddotValues();
  to_qddot = to->as<StateType>()->getQddotValues();

  kdtp::LocalPathPtr LP = compute_local_path(from_q, from_qdot, from_qddot, to_q, to_qdot, to_qddot, robot_->getKinodynamicLimits());
  time = LP->getDuration();

  return time;
}