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

#ifndef KINO_SHORTCUT_
#define KINO_SHORTCUT_

// OMPL
#include "ompl/util/RandomNumbers.h"

// Planning
#include "planning/post_processing/PostProcessor.h"

namespace ompl
{
    OMPL_CLASS_FORWARD(ExtendedShortcut);

    class ExtendedShortcut : public PostProcessor
    {
        public:
            ExtendedShortcut(base::SpaceInformation *si, const RobotPtr robot, const ros::NodeHandle& node_handler): PostProcessor(si, robot, node_handler)
            {
                setup();
            }
            ExtendedShortcut(const base::SpaceInformationPtr &si, RobotPtr robot, const ros::NodeHandle& node_handler): PostProcessor(si, robot, node_handler)
            {
                setup();
            }
            ~ExtendedShortcut(){};

            // #######################################################
            // post processing operator
            std::vector<ompl::base::State*> postProcess(ompl::geometric::PathGeometric &pg, std::vector<int>& index_wpt_in_traj_) override;

            /** \brief Setup function */
            void setup()
            {
                // The planner config path
                const std::string key_params = "post_processing_configs/ExtendedShortcut/";

                // Load the number of samples around a section state
                if(!nh_.getParam(key_params + "nb_in_ball", nb_in_ball_)){
                    ROS_WARN("Cannot load 'nb_in_ball' param from ROS server: nb_in_ball_ = 1");
                    nb_in_ball_ = 1;
                }

                // Load the number of samples before updating the radius
                if(!nh_.getParam(key_params + "nb_update", nb_update_)){
                    ROS_WARN("Cannot load 'nb_update' param from ROS server: nb_update_ = 1");
                    nb_update_ = 10;
                }

                // Load the ball radius for local sampling
                if(!nh_.getParam(key_params + "ball_radius", ball_radius_)){
                    ROS_WARN("Cannot load 'ball_radius' param from ROS server: default ball_radius_ = 0.0");
                    ball_radius_ = 0.0;
                }
                std::cout << "ExtendedShortcut using ball radius = " << ball_radius_ << std::endl;

                // Load the ball radius for local sampling
                if(!nh_.getParam(key_params + "sampling_method", sampling_method_)){
                    ROS_WARN("Cannot load 'sampling_method' param from ROS server: default sampling_method_ = uniform");
                    sampling_method_ = "uniform";
                }   

                // Load the type of radius for local sampling
                if(!nh_.getParam(key_params + "radius_type", radius_type_)){
                    ROS_WARN("Cannot load 'radius_type' param from ROS server: default radius_type_ = fixed");
                    radius_type_ = "fixed";
                }   
            }

            // #######################################################
            // sampling function
            void sampleAround(ompl::base::State* new_st, const ompl::base::State* cur_st)
            {
                std::vector<double> q, qdot, qddot, q_cur, qdot_cur, qddot_cur;
                
                q_cur = cur_st->as<ompl::base::KinosplineStateSpace::StateType>()->getQValues();
                qdot_cur = cur_st->as<ompl::base::KinosplineStateSpace::StateType>()->getQdotValues();
                qddot_cur = cur_st->as<ompl::base::KinosplineStateSpace::StateType>()->getQddotValues();

                // Sample Fixed Ball
                // #########################################################################
                if(sampling_method_== "uniform")
                {
                    for(int i = 0; i<getRobot()->getDesiredDim(); i++)
                    {
                        q.push_back(rng_.uniformReal(q_cur.at(i)-ball_radius_, q_cur.at(i)+ball_radius_));

                        // Sample velocity around with clamping to the kinodynamic limits if necessary
                        double new_vel = rng_.uniformReal(qdot_cur.at(i)-ball_radius_, qdot_cur.at(i)+ball_radius_);
                        if(abs(new_vel) > getRobot()->getKinodynamicLimits().at(i*5))
                            new_vel = getRobot()->getKinodynamicLimits().at(i*5);
                        qdot.push_back(new_vel);

                        // Sample acceleration around with clamping to the kinodynamic limits if necessary
                        double new_acc = rng_.uniformReal(qddot_cur.at(i)-ball_radius_, qddot_cur.at(i)+ball_radius_);
                        if(abs(new_acc) > getRobot()->getKinodynamicLimits().at(i*5+1))
                            new_acc = getRobot()->getKinodynamicLimits().at(i*5+1);
                        qddot.push_back(new_acc);
                    }
                }
                // #########################################################################

                // Sample Gaussian Ball
                // #########################################################################
                if(sampling_method_== "gaussian")
                {
                    for(int i = 0; i<getRobot()->getDesiredDim(); i++)
                    {
                        q.push_back(rng_.gaussian(q_cur.at(i),ball_radius_));

                        // Sample velocity around with clamping to the kinodynamic limits if necessary
                        double new_vel = rng_.gaussian(qdot_cur.at(i),ball_radius_);
                        if(abs(new_vel) > getRobot()->getKinodynamicLimits().at(i*5))
                            new_vel = getRobot()->getKinodynamicLimits().at(i*5);
                        qdot.push_back(new_vel);

                        // Sample acceleration around with clamping to the kinodynamic limits if necessary
                        double new_acc = rng_.gaussian(qddot_cur.at(i),ball_radius_);
                        if(abs(new_acc) > getRobot()->getKinodynamicLimits().at(i*5+1))
                            new_acc = getRobot()->getKinodynamicLimits().at(i*5+1);
                        qddot.push_back(new_acc);
                    }
                    // After generating a sample, update the counters
                    total_sampled_++;
                }
                // #########################################################################

                new_st->as<ompl::base::KinosplineStateSpace::StateType>()->setQValues(q);
                new_st->as<ompl::base::KinosplineStateSpace::StateType>()->setQdotValues(qdot);
                new_st->as<ompl::base::KinosplineStateSpace::StateType>()->setQddotValues(qddot);

                if(radius_type_ == "adaptive") 
                {
                    // Adjust the adaptive radius every N samples
                    if (total_sampled_ % nb_update_ == 0)
                    {
                        double acceptance_rate = static_cast<double>(acceptance_count_) / nb_update_;

                        if (acceptance_rate > 0.5)
                        {
                            // If more than 50% of samples are accepted, reduce the radius to refine exploration
                            ball_radius_ *= 0.9; // Reduce radius by 10%
                        }
                        else
                        {
                            // If fewer than 50% are accepted, increase the radius to explore more widely
                            ball_radius_ *= 1.1; // Increase radius by 10%
                        }

                        // Ensure radius doesn't drop below a minimum threshold or exceed a maximum limit
                        ball_radius_ = std::max(ball_radius_, 0.1);
                        ball_radius_ = std::min(ball_radius_, 0.001);

                        // Reset counters for the next batch of samples
                        acceptance_count_ = 0;
                        total_sampled_ = 0;
                    }
                }
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

            /** \brief Number of samples in the ball around a random state of a given section */
            int nb_in_ball_;

            /** \brief Track the number of sampled states */
            int total_sampled_ = 0;

            /** \brief The number of samples to perform before updating the radius */
            int nb_update_;

            /** \brief The number of accepted samples */
            int acceptance_count_;

            /** \brief Radius of the ball used to sample around a state in all directions */
            double ball_radius_;

            /** \brief The sampling strategy to use for sampling states in the sphere */
            std::string sampling_method_;

            /** \brief The type of radius we are using, fixed or adaptive */
            std::string radius_type_;

            /** \brief The random number generator */
            ompl::RNG rng_;
  };
}
#endif