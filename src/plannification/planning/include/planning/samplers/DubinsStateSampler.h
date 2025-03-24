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

#ifndef DUBINS_STATE_SAMPLER_
#define DUBINS_STATE_SAMPLER_

// State spaces
#include "planning/state_spaces/dubins/DubinsUnicycleSpace.h"

// Utils
#include "utils/transform.h"

// OMPL
#include <ompl/base/StateSampler.h>

// EIGEN
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Dense>

namespace ompl{
  namespace base{
    OMPL_CLASS_FORWARD(DubinsStateSampler);

    class DubinsStateSampler : public StateSampler{
    public:
       DubinsStateSampler(const StateSpace *space, const int dim) : StateSampler(space), dim_(dim), q_bounds_(dim)
      {
        /** Get the dubins state bounds (i.e on q). The bounds on q are given by the workspace */
        for(int i = 0; i<dim_; i++)
        {
          q_bounds_.low = space_->as<ompl::base::DubinsUnicycleSpace>()->getQBounds().low;
          q_bounds_.high = space_->as<ompl::base::DubinsUnicycleSpace>()->getQBounds().high;
        }
      }

       virtual void sampleUniform(State *state);

       virtual void sampleUniformNear(State *state, const State *near, const double distance);

       virtual void sampleGaussian(State *state, const State *mean, const double stdDev);

    private:
      /// \brief The dimension of the state to sample
      int dim_;

      RealVectorBounds q_bounds_;

    }; // class DubinsStateSampler
  } // namespace base
} // namespace ompl

#endif // KINOSPLINE_STATE_SAMPLER_
