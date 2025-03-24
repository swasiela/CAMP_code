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

#ifndef POST_PROCESSOR_
#define POST_PROCESSOR_

// SYSTEM
#include <ros/ros.h>
#include <string>
#include <iostream>
#include <fstream>
#include <filesystem>  // C++17 and later
#include <chrono>  // For time measurement

// OMPL
#include "ompl/base/SpaceInformation.h"
#include "ompl/geometric/PathGeometric.h"
#include "ompl/util/ClassForward.h"
#include "ompl/util/Exception.h"

// Robots
#include "robots/Robot.h"

// State spaces
#include "planning/state_spaces/kinospline/KinosplineStateSpace.h"

namespace ompl
{

    OMPL_CLASS_FORWARD(PostProcessor);

    class PostProcessor
    {
        public:
            PostProcessor(base::SpaceInformation *si, const RobotPtr robot, const ros::NodeHandle& node_handler) : si_(si), robot_(robot), nh_(node_handler)
            {
                setup();
            }
            PostProcessor(const base::SpaceInformationPtr &si, RobotPtr robot, const ros::NodeHandle& node_handler) : si_(si.get()), robot_(robot), nh_(node_handler)
            {
                setup();
            }

            ~PostProcessor(){};

            /** \brief Post processing operator */
            virtual std::vector<ompl::base::State*> postProcess(ompl::geometric::PathGeometric &pg, std::vector<int>& index_wpt_in_traj_)
            {
                throw Exception("postProcess", "not implemented");
            }

            /** \brief Setup function */
            void setup()
            {
                // Get the current CAMP path 
                std::filesystem::path sourceFilePath = __FILE__; 
                std::filesystem::path currentDir = sourceFilePath.parent_path();
                // Traverse upwards until the root of the filesystem
                while (currentDir.has_parent_path()) {
                    if (currentDir.filename() == "CAMP") {  // Adjust this condition if "CAMP" is not the directory name
                        campDir_ = currentDir;
                        break;
                    }
                    currentDir = currentDir.parent_path();
                }

                if (campDir_.empty()) {
                    ROS_ERROR("CAMP directory not found.");
                }

                // Load the number of iteration to perform
                if(!nh_.getParam("planning_parameters/max_iter", max_iter_)){
                    ROS_WARN("Cannot load 'max_iter' param from ROS server: default max_iter_ = 10");
                    max_iter_ = 10;
                }

                // Load the stopping criterion
                if(!nh_.getParam("planning_parameters/stopping_condition", stopping_condition_)){
                    ROS_WARN("Cannot load 'stopping_condition' param from ROS server: default stopping_condition_ = Iter");
                    stopping_condition_ = "Iter";
                }

                if(!nh_.getParam("planning_parameters/window_size", windowSize_)){
                    ROS_WARN("Cannot load 'window_size' param from ROS server: default windowSize_ = 3");
                    windowSize_ = 3;
                }

                if(!nh_.getParam("planning_parameters/threshold", tolerance_)){
                    ROS_WARN("Cannot load 'threshold' param from ROS server: default tolerance_ = 1e-6");
                    tolerance_ = 1e-6;
                }

                if(!nh_.getParam("planning_parameters/max_time", max_time_)){
                    ROS_WARN("Cannot load 'max_time' param from ROS server: default max_time_ = 10");
                    max_time_ = 10;
                }
                
                // Load the cost operator to use
                if(!nh_.getParam("planning_parameters/cost", cost_)){
                    ROS_WARN("Cannot load 'cost' param from ROS server: default cost_ = Length");
                    cost_ = "Length";
                }
                
                // Load the parameters to optimize
                std::string opti_;
                if(!nh_.getParam("planning_parameters/opti_params", opti_)){
                    ROS_WARN("Cannot load 'opti_params' param from ROS server: default opti_ = Traj");
                    opti_ = "Traj";
                }

                if(opti_ == "Traj")
                {
                    opti_traj_ = true;
                    opti_gains_ = false;
                    opti_all_ = false;
                }
                else if(opti_ == "Gains")
                {
                    opti_traj_ = false;
                    opti_gains_ = true;
                    opti_all_ = false;
                }
                else if(opti_ == "All")
                {
                    opti_traj_ = false;
                    opti_gains_ = false;
                    opti_all_ = true;
                }

                // The file to store the gain optimization results
                if(!nh_.getParam("planning_parameters/gains_file", gains_file_)){
                    ROS_WARN("Cannot load 'gains_file' param from ROS server: default gains_file_ = /gains.txt");
                    gains_file_ = "/gains.txt";
                }
                else
                {
                    // Combine the CAMP directory with the relative file path
                    std::filesystem::path fullPath = std::filesystem::absolute(campDir_ / gains_file_);

                    // Normalize the path to remove redundant ".." and "."
                    fullPath = fullPath.lexically_normal();

                    // Convert to string and use the full path
                    gains_file_ = fullPath.string();
                }

                // The file to store the gain optimization results
                if(!nh_.getParam("planning_parameters/post_process_file", post_process_file_)){
                    ROS_WARN("Cannot load 'post_process_file' param from ROS server: default post_process_file_ = /post_process.txt");
                    post_process_file_ = "/post_process.txt";
                }
                else
                {
                    // Combine the CAMP directory with the relative file path
                    std::filesystem::path fullPath = std::filesystem::absolute(campDir_ / post_process_file_);

                    // Normalize the path to remove redundant ".." and "."
                    fullPath = fullPath.lexically_normal();

                    // Convert to string and use the full path
                    post_process_file_ = fullPath.string();
                }

                // If the motion validator has the same time step that the one used for trainning we use the model, otherwise no
                // For the moment only available for the trajectory optimiztion only
                // FUTURE WORK: learn gains
                if(getSpaceInformation()->getMotionValidator()->getDt() == getRobot()->sensiNN_.getDtTrain() && opti_traj_)
                {
                    use_NN_ = true;
                }
                else
                {
                    use_NN_ = false;
                }

                // To clear the file 
                std::ofstream ofs;
                ofs.open(post_process_file_, std::ofstream::out | std::ofstream::trunc);
                ofs.close();
                ofs.open(gains_file_, std::ofstream::out | std::ofstream::trunc);
                ofs.close();
            }

            /** \brief Get access to the robot object */
            RobotPtr getRobot() const
            {
                return robot_;
            }

            /** \brief Get access to the space information object */
            base::SpaceInformation* getSpaceInformation() const
            {
                return si_;
            }

            /** \brief Cost convergence termination condition */
            bool costConvergence()
            {
                bool converged = false;

                // If the window exceeds the desired size, remove the oldest cost
                if (costsWindow_.size() > windowSize_)
                {
                    costsWindow_.erase(costsWindow_.begin());
                }

                // Check for convergence only if we have enough data in the window
                if (costsWindow_.size() == windowSize_)
                {
                    // Compute the maximum percentage variation between consecutive costs
                    double max_percentage_variation = 0.0;
                    for (std::size_t i = 1; i < costsWindow_.size(); ++i)
                    {
                        double current_cost = costsWindow_[i];
                        double previous_cost = costsWindow_[i - 1];

                        // Avoid division by zero (check if previous_cost is close to zero)
                        if (std::abs(previous_cost) > 1e-10)
                        {
                            // Calculate the percentage variation
                            double percentage_variation = (std::abs(current_cost - previous_cost) / std::abs(previous_cost)) * 100.0;

                            // Keep track of the maximum percentage variation
                            if (percentage_variation > max_percentage_variation)
                            {
                                max_percentage_variation = percentage_variation;
                            }
                        }
                    }

                    // If the maximum percentage variation is less than the tolerance, consider it converged
                    if (max_percentage_variation < tolerance_)
                    {
                        converged = true;
                    }
                }

                return converged;
            }

            /** \brief Termination condition */
            bool stoppingCriterion()
            {
                if(stopping_condition_ == "Iter")
                    return nb_iter_ > max_iter_;
                else if(stopping_condition_ == "Convergence")
                    return costConvergence();
                else if(stopping_condition_ == "Time")
                {
                    auto current_time = std::chrono::steady_clock::now();
                    double elapsed_time = std::chrono::duration<double>(current_time - start_time_).count();
                    return elapsed_time > max_time_;
                }
                else
                {
                    ROS_ERROR("Stopping criterion not implemented for post processing !");
                    return false;
                } 
            }

            // Node handler
            ros::NodeHandle nh_;

            /// \brief Path of the CAMP repository
            std::filesystem::path campDir_;

            /** \brief Number of iteration */
            int nb_iter_, max_iter_;

            /** \brief Window size used for the cost convergence criterion */
            int windowSize_;

            /** \brief Threshold used for the cost convergence criterion */
            double tolerance_;

            /** \brief The maximum planning time */
            double max_time_;

            /** \brief The parameter to optimize */
            bool opti_traj_, opti_gains_, opti_all_;

            /** \brief If we use the NN model to time steps correspondence */
            bool use_NN_;

            /** \brief The cost to consider */
            std::string cost_;

            /** \brief The file to store the gains */
            std::string gains_file_;

            /** \brief The file to store the optimized trajectories and their cost */
            std::string post_process_file_;

            /// \brief The stopping criterion
            std::string stopping_condition_;

            /** \brief All the costs found so far, used for the cost convergence criterion */
            std::vector<double> costsWindow_;

            /** \brief The starting planning time */
            std::chrono::steady_clock::time_point start_time_;
            
        private:

            /** \brief The instance of space information this state validity checker operates on */
            base::SpaceInformation *si_;

            /** \brief The current robot */
            RobotPtr robot_; 

    };
    typedef std::shared_ptr<PostProcessor> PostProcessorPtr;
}
#endif