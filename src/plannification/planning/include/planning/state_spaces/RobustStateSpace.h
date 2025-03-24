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

#ifndef ROBUST_STATE_SPACE_
#define ROBUST_STATE_SPACE_

// Torch
#include "torch/torch.h"

// OMPL
#include "ompl/base/StateSpace.h"
#include "ompl/base/spaces/SE3StateSpace.h"
#include "ompl/util/ClassForward.h"
#include "ompl/util/Exception.h"

// SYSTEM
#include <ros/ros.h>
#include <console_bridge/console.h>

// Define a new type of state space: need to be larger than STATE_SPACE_TYPE_COUNT
#define ROBUST_STATE_SPACE (ompl::base::STATE_SPACE_TYPE_COUNT + 1)

namespace ompl
{
  namespace base
  {

    OMPL_CLASS_FORWARD(RobustStateSpace);

    /** \brief A robust state space */
    class RobustStateSpace : public CompoundStateSpace
    {
      public:

        /** \brief A robust state to store the either the simulated robot state or the closed loop sensitivity values (PI, PI_xi, h0)*/
        class StateType : public CompoundStateSpace::StateType
        {
          public:
            StateType() : CompoundStateSpace::StateType(){}
        
            // To store the sensitivity values of the motion leading to this state (e.g. PI_F)
            std::vector<double> PI_, PI_xi_, xi_;

            /** \brief The hidden state at this state used for the GRU-based network */
            torch::Tensor h0_;

            /** \brief The nominal state of the robot used for collision checking. 
             * The state is either simulated or the result of a simple projection, depending on the planner used. */
            std::vector<double> nominal_state_;

            /** \brief The associated time of the state */
            double st_time_;
        };

        RobustStateSpace() : CompoundStateSpace()
        {}

        virtual ~RobustStateSpace()
        {}
    };
  }
}
#endif // PHYSIC_POINT_STATE_SPACE_H