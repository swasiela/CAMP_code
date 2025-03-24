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

#include "planning/samplers/DubinsStateSampler.h"


void ompl::base::DubinsStateSampler::sampleUniform(ompl::base::State *state)
{
  state->as<ompl::base::DubinsUnicycleSpace::StateType>()->setX(rng_.uniformReal(q_bounds_.low[0], q_bounds_.high[0]));
  state->as<ompl::base::DubinsUnicycleSpace::StateType>()->setY(rng_.uniformReal(q_bounds_.low[1], q_bounds_.high[1]));
  state->as<ompl::base::DubinsUnicycleSpace::StateType>()->setYaw(rng_.uniformReal(-3.1415, 3.1415));
}

void ompl::base::DubinsStateSampler::sampleUniformNear(ompl::base::State *state, const ompl::base::State *near, const double distance)
{
  // Sample to zero vel and acc, it's not the sample near implementation !! 
  ROS_ERROR("DubinsStateSampler::sampleUniformNear NOT IMPLEMENTED !");
}

void ompl::base::DubinsStateSampler::sampleGaussian(ompl::base::State *state, const ompl::base::State *mean, const double stdDev)
{
  ROS_ERROR("DubinsStateSampler::sampleGaussian NOT IMPLEMENTED !");
}

