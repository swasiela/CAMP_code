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

#ifndef OMPL_BASE_OPTIMIZATION_OBJECTIVE_KINOSPLINE_TIME
#define OMPL_BASE_OPTIMIZATION_OBJECTIVE_KINOSPLINE_TIME

// OMPL
#include "ompl/base/OptimizationObjective.h"

// State space
#include "planning/state_spaces/kinospline/KinosplineStateSpace.h"

namespace ompl
{
    namespace base
    {
        class MinKSTime : public OptimizationObjective
        {
            public:
                MinKSTime(const SpaceInformationPtr &si, double dt) : OptimizationObjective(si) {dt_default_ = dt;}
            
                virtual Cost stateCost(const State* s) const;
                virtual bool isCostBetterThan(Cost c1, Cost c2) const;
                virtual Cost motionCost(const State *s1, const State *s2) const;
                virtual Cost motionCostHeuristic(const State *s1, const State *s2) const;
                virtual Cost combineCosts(Cost c1, Cost c2) const;      
                virtual Cost identityCost() const;
                virtual Cost infiniteCost() const;

                // Kino metric is a quasi metric
                bool isSymmetric() const
                {
                    return false;
                }
            
            private:
                double dt_default_;
        };
    }
}
#endif

