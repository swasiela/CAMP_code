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
#include "planning/post_processing/STOMP.h"


using Trajectory = Eigen::MatrixXd; /**< Assign Type Trajectory to Eigen::MatrixXd Type */

/**
 * @brief Creates a STOMP configuration object with default parameters.
 * @return A STOMP configuration object
 */
stomp::StompConfiguration create4DOFConfiguration(double nb_timesteps, double dt, int max_iter, int nb_rollout, int nb_iter_after_valid)
{
  //! [Create Config]
  using namespace stomp;

  StompConfiguration c;
  c.num_timesteps = nb_timesteps;
  c.num_iterations = max_iter;
  c.num_dimensions = 4;
  c.delta_t = dt;
  c.control_cost_weight = 0.0;
  c.num_iterations_after_valid = nb_iter_after_valid;
  c.num_rollouts = nb_rollout;
  c.max_rollouts = nb_rollout;
  //! [Create Config]

  return c;
}

std::vector<ompl::base::State*> ompl::STOMP::postProcess(ompl::geometric::PathGeometric &pg, std::vector<int>& index_wpt_in_traj_)
{
    using namespace stomp;

    /**< Creating a Task with an intial trajectory according to the robot information**/
    std::string robot_name = getRobot()->getName();
    
    // Convert the ompl based trajectory into an Eigen based trajectory
    Trajectory initial_traj;
    Trajectory times; // times vector

    StompConfiguration config;

    std::vector<double> std_dev;

    // Create the trajectory to the right dimensions
    if(robot_name == "unicycle")
    {
      std::cout << "STOMP not implemented for this type of robot" << std::endl;
      return {};
    }
    else if(robot_name == "quadrotor")
    {
      initial_traj = Trajectory::Zero(4, pg.getStates().size()); // X Y Z Yaw dimensions
      times = Trajectory::Zero(1, pg.getStates().size());
      for(int i = 0; i<pg.getStates().size(); i++)
      {
        std::vector<double> q = pg.getStates().at(i)->as<ompl::base::KinosplineStateSpace::StateType>()->getQValues();
        double time = pg.getStates().at(i)->as<ompl::base::KinosplineStateSpace::StateType>()->st_time_;
        initial_traj(0,i) = q[0]; // X
        initial_traj(1,i) = q[1]; // Y
        initial_traj(2,i) = q[2]; // Z
        initial_traj(3,i) = q[3]; // Yaw
        times(0,i) = time; // time
      }
      config = create4DOFConfiguration(pg.getStates().size(), getSpaceInformation()->getMotionValidator()->getDt(), max_iter_, nb_rollout_, nb_iter_after_valid_);
      for(int i = 0; i<4; i++)
      {
        // Creating std_dev for generating noisy parameters
        std_dev.push_back(std_dev_);
      }
    }
    else
    {
      std::cout << "STOMP not implemented for this type of robot" << std::endl;
      return {};
    }

    //! [Create Task Object]
    
    TaskPtr task(new AccuracyOptimizationTask(initial_traj, times, std_dev, cost_, post_process_file_, stopping_condition_, max_iter_, windowSize_, tolerance_, max_time_, 
                                              getSpaceInformation()->getMotionValidator()->getDt(), getSpaceInformation(), getRobot()));
    //! [Create Task Object]

    //! [Create STOMP]
    /**< Creating STOMP to find a trajectory close enough to the bias **/
    Stomp stomp(config, task);
    //! [Create STOMP]

    //! [Solve]
    /**< Optimizing a trajectory close enough to the bias is produced **/
    Trajectory optimized;
    if (stomp.solve(initial_traj, optimized))
    {
        std::cout << "STOMP succeeded" << std::endl;
    }
    else
    {
        std::cout << "A valid solution was not found" << std::endl;
        return {};
    }
    //! [Solve]

    // Convert the optimized trajectory into an ompl based trajectory
    std::vector<ompl::base::State*> best_traj;
    if(robot_name == "unicycle")
    {
      std::cout << "STOMP not implemented for this type of robot" << std::endl;
      return {};
    }
    else if(robot_name == "quadrotor")
    {
      best_traj = task->toOMPL(optimized);
    }
    else
    {
      std::cout << "STOMP not implemented for this type of robot" << std::endl;
      return {};
    }
    return best_traj;
}