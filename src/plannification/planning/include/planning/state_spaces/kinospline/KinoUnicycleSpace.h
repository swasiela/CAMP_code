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

#ifndef KINO_UNICYCLE_SPACE_
#define KINO_UNICYCLE_SPACE_

#include "planning/state_spaces/kinospline/KinosplineStateSpace.h"

namespace ompl
{
  namespace base
  {
    OMPL_CLASS_FORWARD(KinoUnicycleSpace);

    class KinoUnicycleSpace : public KinosplineStateSpace
    {
      public:
        /** \brief A state type inherited from the Kinospline state type */
        class StateType : public KinosplineStateSpace::StateType
        {
        };

        KinoUnicycleSpace(unsigned int dim, std::string opt_tag, ompl::RobotPtr robot) : KinosplineStateSpace(dim, opt_tag, robot){};
        ~KinoUnicycleSpace(){};

        // Projection and simulation functions
        void projectStates(const std::vector<ompl::base::State*> &des_states, std::vector<std::vector<double>> &collision_states) const override;
        bool simulateStates(const std::vector<ompl::base::State*> &des_states, const double dt, std::vector<std::vector<double>> &collision_states) const override;
        bool simulateStatesAndTubes(const std::vector<ompl::base::State*> &des_states, double dt, std::vector<std::vector<double>> &collision_states,
                                        std::vector<std::vector<double>> &control_inputs, std::vector<std::vector<double>> &radii) const override;
        
        // Local optimisation using nlopt
        bool localOpt(const std::vector<ompl::base::State*> &section_to_optimize, double dt, std::string stopping_condition_, double tolerance_, double max_time, int window_size, int nb_iter, std::string cost_id, std::string opti_params, 
                                                std::vector<ompl::base::State*> &traj_opt, double &cost) const override;
        
        // Local optimization using mpc formulation
        bool mpc(const std::vector<ompl::base::State*> &section_to_optimize, double dt, int nb_iter, int nb_simu, std::string cost_id, std::string opti_params, 
                                                std::vector<ompl::base::State*> &traj_opt, double &cost) const override;

        virtual void getPlatformFromEEF(std::vector<double>& q, std::vector<double>& acc) const override;
      private:
        
    };
  }
}
#endif