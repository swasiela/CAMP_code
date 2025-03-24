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

#ifndef SHORTCUT_
#define SHORTCUT_

// OMPL
#include "ompl/util/RandomNumbers.h"

// Planning
#include "planning/post_processing/PostProcessor.h"

namespace ompl
{
  OMPL_CLASS_FORWARD(Shortcut);

  class Shortcut : public PostProcessor
  {
      public:
          Shortcut(base::SpaceInformation *si, const RobotPtr robot, const ros::NodeHandle& node_handler): PostProcessor(si, robot, node_handler)
          {
            setup();
          }
          Shortcut(const base::SpaceInformationPtr &si, RobotPtr robot, const ros::NodeHandle& node_handler): PostProcessor(si, robot, node_handler)
          {
            setup();
          }
          ~Shortcut(){};

          // #######################################################
          // post processing operator
          std::vector<ompl::base::State*> postProcess(ompl::geometric::PathGeometric &pg, std::vector<int>& index_wpt_in_traj_) override;

          /** \brief Setup function */
          void setup()
          {
            // The planner config path
            const std::string key_params = "post_processing_configs/Shortcut/";

          }

          // #######################################################
          // cost function

          /** \brief Compute the trajectory cost */
          double cost(const std::vector<ompl::base::State*> &traj, const std::vector<int>& wpt_index, const std::vector<double> &lambda) const;

          // #######################################################
          // export

          /** \brief To export the optimized gains */
          void exportGains(std::vector<std::vector<double>>& gains, double cost) const
          {
            std::ofstream gains_file(gains_file_, std::ios::app);
            std::cout.precision(6);
            gains_file << "Gains \n";

            for(std::size_t i=0; i< gains.size(); ++i)
            {
              gains_file << "Vector number : " << i << "\n";
              for(int j = 0; j<gains.at(i).size(); j++)
                gains_file<< gains.at(i).at(j)<<"\n";
            }
            gains_file << "Cost \n";
            gains_file << cost << "\n";
            gains_file.close();
          }

          /** \brief Export the current best trajectory found with its cost */
          void exportShct(std::vector<ompl::base::State *>& traj, double cost, double plan_time) const
          {
            std::ofstream post_process_file(post_process_file_, std::ios::app);
            std::cout.precision(6);
            if(getRobot()->getName() == "quadrotor")
            {
              post_process_file << "Trajectory" << "\n";

              for(std::size_t i=0; i< traj.size(); ++i)
              {
                for(std::size_t j=0; j<getRobot()->getDesiredDim(); ++j)
                {
                  post_process_file<<";"<< traj.at(i)->as<ompl::base::KinosplineStateSpace::StateType>()->getQValues()[j];
                }
                for(std::size_t j=0; j<getRobot()->getDesiredDim(); ++j)
                {
                  post_process_file<<";"<< traj.at(i)->as<ompl::base::KinosplineStateSpace::StateType>()->getQdotValues()[j];
                }
                for(std::size_t j=0; j<getRobot()->getDesiredDim(); ++j)
                {
                  post_process_file<<";"<< traj.at(i)->as<ompl::base::KinosplineStateSpace::StateType>()->getQddotValues()[j];
                }
                post_process_file<<"\n"; 
              }
            }
            post_process_file << "Cost" << "\n";
            post_process_file << cost << "\n";
            post_process_file << "Time" << "\n";
            post_process_file << plan_time << "\n";
            post_process_file.close();
          }

      private:

        /** \brief The random number generator */
        ompl::RNG rng_;
  };
}
#endif