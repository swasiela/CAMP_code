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

#include "planning/opt_objectives/KS_time_obj.h"

ompl::base::Cost ompl::base::MinKSTime::stateCost(const ompl::base::State* s) const
{
  ROS_ERROR("stateCost function NOT IMPLEMENTED for time optimisation objective while using KS.");
  return ompl::base::Cost(0.0);
}

bool ompl::base::MinKSTime::isCostBetterThan(ompl::base::Cost c1, ompl::base::Cost c2) const
{
  if(c1.value() < c2.value())
    return true;
  else
    return false;
}

ompl::base::Cost ompl::base::MinKSTime::motionCost(const ompl::base::State *s1, const ompl::base::State *s2) const
{
  return ompl::base::Cost(si_->getStateSpace()->as<ompl::base::KinosplineStateSpace>()->get_interpolate_time(s1, s2));
}

ompl::base::Cost ompl::base::MinKSTime::motionCostHeuristic(const State *s1, const State *s2) const
{
  return ompl::base::Cost(si_->getStateSpace()->as<ompl::base::KinosplineStateSpace>()->callKinoMetric(s1, s2));
}

ompl::base::Cost ompl::base::MinKSTime::combineCosts(ompl::base::Cost c1, ompl::base::Cost c2) const
{
  return ompl::base::Cost(c1.value() + c2.value());
}

ompl::base::Cost ompl::base::MinKSTime::identityCost() const
{
  return ompl::base::Cost(0.0);
}

ompl::base::Cost ompl::base::MinKSTime::infiniteCost() const
{
  return ompl::base::Cost(std::numeric_limits<double>::infinity());
}