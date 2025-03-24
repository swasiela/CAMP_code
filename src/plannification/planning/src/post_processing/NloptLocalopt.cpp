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

#include "planning/post_processing/NloptLocalopt.h"

std::vector<ompl::base::State*> ompl::NloptLocalopt::postProcess(ompl::geometric::PathGeometric &pg, std::vector<int>& index_wpt_in_traj_)
{
    auto start = std::chrono::high_resolution_clock::now();

    // Check if there is a trajectory to optimize
    if (pg.getStateCount() < 2)
    {
        ROS_ERROR("No trajectory found for post processing !!!");
        return pg.getStates();
    }

    // Init the trajectory to optimize
    std::vector<ompl::base::State *> init_traj = pg.getStates();

    // Init the robot gains to optimize
    std::vector<std::vector<double>> init_gains = getRobot()->getGains();

    // To store the values after the optimization process
    std::vector<ompl::base::State *> traj_opt;
    std::vector<std::vector<double>> gains_opt = init_gains;
    std::vector<int> index_opt = index_wpt_in_traj_;
    std::vector<double> cost_wpts;

    // Compute the number of iterations to perform for each section
    int max_iter_sec = int(max_iter_/index_wpt_in_traj_.size());

    // Optimizes each subpath between waypoints 
    for(int i = 0; i<index_wpt_in_traj_.size(); i++)
    {
        std::vector<ompl::base::State *> section_to_optimize;
        double section_cost;

        if(i == 0)
            section_to_optimize = std::vector<ompl::base::State *>(init_traj.begin(), init_traj.begin()+index_wpt_in_traj_.at(i)+1);
        else
            section_to_optimize = std::vector<ompl::base::State *>(init_traj.begin()+index_wpt_in_traj_.at(i-1), init_traj.begin()+index_wpt_in_traj_.at(i)+1);
        
        std::vector<ompl::base::State *> section_opt;
        getSpaceInformation()->getStateSpace()->localOpt(section_to_optimize, getSpaceInformation()->getMotionValidator()->getDt(), 
                                                        stopping_condition_, tolerance_, max_time_, windowSize_, max_iter_sec, cost_, opti_params_,
                                                        section_opt, section_cost);

        if(i == 0)
            index_opt.at(i) = section_opt.size()-1;
        else
            index_opt.at(i) = index_opt.at(i-1) + section_opt.size()-1;
        
        // Save the section cost
        cost_wpts.push_back(section_cost);

        // Save the section optimization result
        for(auto &st : section_opt)
          traj_opt.push_back(st);
    }   

    // Compute cost
    double cost_opt = cost_wpts.back();

    std::swap(index_wpt_in_traj_, index_opt);

    // Get the ending time point
    auto end = std::chrono::high_resolution_clock::now();

    // Calculate the duration by subtracting start time from end time
    std::chrono::duration<double> elapsed = end - start;

    exportGains(gains_opt, cost_opt);
    exportShct(traj_opt, cost_opt, elapsed.count());

    getRobot()->setGains(gains_opt);
    return traj_opt;
} 

