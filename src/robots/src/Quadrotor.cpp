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

#include <robots/Quadrotor.h>

namespace ompl
{
    Quadrotor::~Quadrotor()
    {
    }

    std::vector<float> Quadrotor::getRobotLearningInput(const std::vector<std::vector<double>>& des_state) const
    {
        //Sequence format
        // Vx, Vy, Vz, Wyaw, Ax, Ay, Az

        std::vector<double> qdot = des_state[0];
        std::vector<double> qddot = des_state[1];

        // Min Max scaling of the inputs (val - min / (max - min))
        std::vector<float> seq;
        // seq.push_back(((float)qdot[0] + getKinodynamicLimits()[0])/(2*getKinodynamicLimits()[0]));
        // seq.push_back(((float)qdot[1] + getKinodynamicLimits()[5])/(2*getKinodynamicLimits()[5]));
        // seq.push_back(((float)qdot[2] + getKinodynamicLimits()[10])/(2*getKinodynamicLimits()[10]));
        // seq.push_back(((float)qdot[3] + getKinodynamicLimits()[15])/(2*getKinodynamicLimits()[15]));
        // seq.push_back(((float)qddot[0] + getKinodynamicLimits()[1])/(2*getKinodynamicLimits()[1]));
        // seq.push_back(((float)qddot[1] + getKinodynamicLimits()[6])/(2*getKinodynamicLimits()[6]));
        // seq.push_back(((float)qddot[2] + getKinodynamicLimits()[11])/(2*getKinodynamicLimits()[11]));

        // Std scaling, more robust to the dataset ranges
        seq.push_back(((float)qdot[0] - sensiNN_.getMeanInTubes(0))/(sensiNN_.getStdInTubes(0)));
        seq.push_back(((float)qdot[1] - sensiNN_.getMeanInTubes(1))/(sensiNN_.getStdInTubes(1)));
        seq.push_back(((float)qdot[2] - sensiNN_.getMeanInTubes(2))/(sensiNN_.getStdInTubes(2)));
        seq.push_back(((float)qdot[3] - sensiNN_.getMeanInTubes(3))/(sensiNN_.getStdInTubes(3)));
        seq.push_back(((float)qddot[0] - sensiNN_.getMeanInTubes(4))/(sensiNN_.getStdInTubes(4)));
        seq.push_back(((float)qddot[1] - sensiNN_.getMeanInTubes(5))/(sensiNN_.getStdInTubes(5)));
        seq.push_back(((float)qddot[2] - sensiNN_.getMeanInTubes(6))/(sensiNN_.getStdInTubes(6)));

        return seq;
    }
}