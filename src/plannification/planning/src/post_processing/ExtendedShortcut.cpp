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

#include "planning/post_processing/ExtendedShortcut.h"

double ompl::ExtendedShortcut::cost(const std::vector<ompl::base::State*> &traj, const std::vector<int>& wpt_index, const std::vector<double> &lambda) const
{
  if(cost_ == "Length")
  {
    return traj.size();
  }
  else if(cost_ == "Sensi")
  {
    return std::accumulate(lambda.begin(), lambda.end(), 0.0);
  }
  else if(cost_ == "Accuracy")
  {
    // Uncertainty based cost
    double w1 = 0.5;
    double w2 = 0.5;
    std::vector<double> L; // The vector to store all the pnorm at the desired states in the trajectory

    for(int t = 0; t<wpt_index.size(); t++)
    {
      L.push_back(lambda.at(wpt_index.at(t)));
    }

    // First element of the cost function link to the eigenvalues of the projector (mean of the lambda max for the geometric output (x y z r p y))
    double mean = 0.;
    for(int t = 0; t<L.size(); t++)
    {
        mean += L.at(t);
    }
    mean = mean/L.size();

    double var = 0.;
    for(int t = 0; t<L.size(); t++)
    {
        var += std::pow((L.at(t)-mean),2);
    }
    var = var/L.size();

    return w1*mean + w2*var;
  }
  else
  {
    ROS_ERROR("Cost function not implemented for post processing ! Cost is set to inf !");
    return std::numeric_limits<double>::max();
  }
}

std::vector<ompl::base::State*> ompl::ExtendedShortcut::postProcess(ompl::geometric::PathGeometric &pg, std::vector<int>& index_wpt_in_traj_)
{
    auto start = std::chrono::high_resolution_clock::now();
    
    // Check if there is a trajectory to optimize
    if (pg.getStateCount() < 2)
    {
        ROS_ERROR("No trajectory found for post processing !!!");
        return pg.getStates();
    }

    // If the motion validator has the same time step that the one used for trainning we use the model, otherwise no
    // For the moment only available for the trajectory optimiztion only
    // FUTURE WORK: learn gains
    if(opti_traj_)
    {
        use_NN_ = false;
    }
    else
    {
        use_NN_ = false;
    }

    // Recover some shapes
    int rqsize = getRobot()->sensiNN_.getRqSize();
    int usize = getRobot()->sensiNN_.getUSize();
        
    // Init the trajectory to optimize
    std::vector<ompl::base::State *> best_traj = pg.getStates();

    // Init the robot gains to optimize
    std::vector<std::vector<double>> bestGains = getRobot()->getGains();

    // Init the cost
    double best_cost_;
    std::vector<std::vector<double>> collision_states, control_inputs, radii;
    bool simu = getSpaceInformation()->getStateSpace()->simulateStatesAndTubes(best_traj, getSpaceInformation()->getMotionValidator()->getDt(), collision_states, control_inputs, radii); 

    //Check state collision with the scene
    for(int i=0; i<collision_states.size(); i++)
    {
        std::vector<double> r_pos;
        for(int j = 0; j<rqsize; j++)
        {
            r_pos.push_back(radii[j][i]);
        }
        if(!getSpaceInformation()->isStateValid(collision_states[i], r_pos))
        {
            ROS_ERROR("Non robust initial solution, can't perform ExtendedShortcut !!!");
            return pg.getStates();
        }
    }
    
    // If the simulation goes wrong, it's very likely that the robot/controller is not able to track the trajectory.
    if(!simu)
    {
        best_cost_ = std::numeric_limits<double>::max();
    }
    else
    {
        std::vector<double> lambda = radii.back(); //The last element of the radii vector is the pnorm of the radii of interest
        best_cost_ = cost(best_traj, index_wpt_in_traj_, lambda);
    }  

    // Export the initialization
    // Get the ending time point
    auto end = std::chrono::high_resolution_clock::now();
    // Calculate the duration by subtracting start time from end time
    std::chrono::duration<double> elapsed = end - start;
    exportShct(best_traj, best_cost_, elapsed.count());

    // Add the new cost to the window
    costsWindow_.push_back(best_cost_);

    // Reset iteration counter 
    nb_iter_ = 0;
    // Start planning time
    start_time_ = std::chrono::steady_clock::now();
    while(!stoppingCriterion())
    {
        std::cout << "Post processing attempt n: " << nb_iter_ << std::endl;

        /////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////////////////
        // SAMPLING A SECTION AND TWO STATES FROM THIS SECTION

        int idx_s1, idx_s2, idx_section;
        // If we optimize the trajectory states
        if(opti_traj_ || opti_all_)
        {
            // First we choose a section on which to sample (because we don't want to shortcut some desired waypoints)
            idx_section = rng_.uniformReal(0, index_wpt_in_traj_.size()-1); 

            //Then we sample 2 states on this same section
            // Minimum distance between the two states (because of Kinospline stability (a minimum number oh phases have to be computed))
            int min_dist = int(0.3/getSpaceInformation()->getMotionValidator()->getDt());

            if(idx_section == 0)
            {
                idx_s1 = rng_.uniformReal(0, index_wpt_in_traj_.at(idx_section)); 
                idx_s2 = rng_.uniformReal(0, index_wpt_in_traj_.at(idx_section)); 
            }
            else
            {
                idx_s1 = rng_.uniformReal(index_wpt_in_traj_.at(idx_section-1), index_wpt_in_traj_.at(idx_section)); 
                idx_s2 = rng_.uniformReal(index_wpt_in_traj_.at(idx_section-1), index_wpt_in_traj_.at(idx_section)); 
            }

            // Put the sampled index in the right order
            if(idx_s1>idx_s2)
                std::swap(idx_s1,idx_s2);
            
            // Check if there is the minimum distance between the two states 
            if(idx_s2 <= idx_s1+min_dist) 
            {
                // Put the minimum distance between the two state if possible or reset the attempt
                if(idx_s1+min_dist <= index_wpt_in_traj_.at(idx_section))
                {
                    idx_s2 = idx_s1+min_dist;
                }
                else
                {
                    // Reset the shortcut attempt
                    nb_iter_++;
                    continue;
                }
            } 
        }
        
        /////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////////////////
        // SAMPLING THE CONTROLLER GAINS IF ASKED

        std::vector<std::vector<double>> gainsVector;
        if(opti_gains_ || opti_all_)
        {
        //Sample gains
        for(int j = 0; j<bestGains.size(); j++)
        {
            std::vector<double> gains;
            // Sample a gain between 50% and 150% of its nominal value
            for(int k = 0; k<bestGains.at(j).size() ; k++)
                gains.push_back(rng_.uniformReal(0.5*bestGains.at(j).at(k), 1.5*bestGains.at(j).at(k)));
            
            gains.at(0) = gains.at(1); //Same gains for x & y axis
            gainsVector.push_back(gains);
        }
        }
        else
        {
            gainsVector = bestGains;
        }

        /////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////////////////
        // SAMPLING STATES IN A BALL AROUND THE TWO STATES FROM THE SECTION

        // Sample k states in a ball around the two states chosen for the shortcut
        std::vector<ompl::base::State*> st_around_s1, st_around_s2;

        for(int k = 0; k<nb_in_ball_; k++)
        {
            ompl::base::State* st_shct_idx1 = getSpaceInformation()->allocState();
            ompl::base::State* st_shct_idx2 = getSpaceInformation()->allocState();

            sampleAround(st_shct_idx1, best_traj.at(idx_s1)); 
            sampleAround(st_shct_idx2, best_traj.at(idx_s2)); 

            st_around_s1.push_back(st_shct_idx1);
            st_around_s2.push_back(st_shct_idx2);
        }

        /////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////////////////
        // CREATE ALL THE POSSIBLE SHORTCUT

        // We create the new possible shortcut
        std::vector<std::vector<ompl::base::State*>> all_section_shct;

        for(int k = 0; k<st_around_s1.size(); k++)
        {
            // First consruct the trajectory from s0_section to the starting state of the shortcut
            std::vector<ompl::base::State*> startwpt_to_shct;
            if(idx_section == 0)
                getSpaceInformation()->getStateSpace()->interpolate(best_traj.at(0), st_around_s1.at(k), getSpaceInformation()->getMotionValidator()->getDt(), startwpt_to_shct);
            else
                getSpaceInformation()->getStateSpace()->interpolate(best_traj.at(index_wpt_in_traj_.at(idx_section-1)), st_around_s1.at(k), getSpaceInformation()->getMotionValidator()->getDt(), startwpt_to_shct); 

            // For all the possible end state of the shortcut create a trajectory
            for(int l = 0; l<st_around_s2.size(); l++)
            {
                // Create the local shortcut in the section
                std::vector<ompl::base::State*> section_shct, traj_shct, shct_to_endwpt;

                getSpaceInformation()->getStateSpace()->interpolate(st_around_s1.at(k), st_around_s2.at(l), getSpaceInformation()->getMotionValidator()->getDt(), traj_shct);
                getSpaceInformation()->getStateSpace()->interpolate(st_around_s2.at(l), best_traj.at(index_wpt_in_traj_.at(idx_section)), getSpaceInformation()->getMotionValidator()->getDt(), shct_to_endwpt);
                
                // Create the shortcut for the section (starting from the waypoint of interest of the previous section to the one of the current section)
                for(int j = 0; j<startwpt_to_shct.size()-1; j++) // From s0_section to sidx1-1
                {
                    section_shct.push_back(startwpt_to_shct.at(j));
                }
                for(int j = 0; j<traj_shct.size()-1; j++) // From sidx1 to sidx2-1
                {
                    section_shct.push_back(traj_shct.at(j));
                }
                for(int j = 0; j<shct_to_endwpt.size(); j++) // From sidx2 to sF_section
                {
                    section_shct.push_back(shct_to_endwpt.at(j));
                }
                all_section_shct.push_back(section_shct);
            }
        }
    
        /////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////////////////
        // CREATE ALL THE POSSIBLE NEW TRAJECTORIES

        std::vector<std::vector<ompl::base::State*>> all_possible_new_traj;

        for(int k = 0; k<all_section_shct.size(); k++)
        {
            std::vector<ompl::base::State*> new_traj;
            // We concatenate the new section with all the other section
            if(idx_section != 0)
            {
                for(int j = 0; j<index_wpt_in_traj_.at(idx_section-1); j++) // From s0 to s0_section-1
                    new_traj.push_back(best_traj.at(j));
            }
            for(int j = 0; j<all_section_shct.at(k).size(); j++) // The new section from s0_section to sF_section
                new_traj.push_back(all_section_shct.at(k).at(j));
            if(idx_section+1 < index_wpt_in_traj_.size())
            {
                for(int j = index_wpt_in_traj_.at(idx_section)+1; j<best_traj.size(); j++) // From sF_section+1 to sF
                    new_traj.push_back(best_traj.at(j));
            }

            all_possible_new_traj.push_back(new_traj);
        }

        /////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////////////////
        // WE COMPUTE NEW POINT-OF-INTEREST INDICES FOR ALL POSSIBLE NEW TRAJECTORIES 

        std::vector<std::vector<int>> all_new_wpt_index;

        for(int k = 0; k<all_possible_new_traj.size(); k++)
        {
            all_new_wpt_index.push_back(index_wpt_in_traj_);
            for (size_t j = idx_section; j < index_wpt_in_traj_.size(); j++)
            {
                if(j==idx_section)
                {
                    if(j==0)
                        all_new_wpt_index.at(k).at(j) = all_section_shct.at(k).size()-1; 
                    else
                        all_new_wpt_index.at(k).at(j) = all_new_wpt_index.at(k).at(idx_section-1) + all_section_shct.at(k).size()-1;   
                }
                else
                {
                    all_new_wpt_index.at(k).at(j) = all_new_wpt_index.at(k).at(idx_section) + (index_wpt_in_traj_.at(j) - index_wpt_in_traj_.at(idx_section));
                }
            }
        }

        /////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////////////////
        // CHECK THE FEASABILITY OF ALL SHORTCUTS WITH TUBES AND COMPUTE THEIR COST

        std::vector<double> all_new_cost;
        for(int k = 0; k<all_possible_new_traj.size(); k++)
        {
            std::vector<std::vector<double>> collision_shct, control_inputs_shct, radii_shct;

            bool simu_shct = getSpaceInformation()->getStateSpace()->simulateStatesAndTubes(all_possible_new_traj.at(k), getSpaceInformation()->getMotionValidator()->getDt(), collision_shct, control_inputs_shct, radii_shct); 

            if(!simu_shct)
                all_new_cost.push_back(std::numeric_limits<double>::max());
            else
            {
                // Robustly check the motion
                bool validity = true;

                // Check inputs first, it's less computationally expansive
                for(int i=0; i<collision_shct.size(); i++)
                {
                    std::vector<double> u, ru;
                    for(int j = 0; j<usize; j++)
                    {
                        u.push_back(control_inputs_shct[j][i]);
                        ru.push_back(radii_shct[rqsize+j][i]);
                    }
                    if(!getSpaceInformation()->isInputsValid(u, ru))
                    {
                        validity =  false;
                        break;
                    }
                }

                // If not valid inputs we check the next possible trajectory
                if(!validity)
                {
                    all_new_cost.push_back(std::numeric_limits<double>::max());
                    continue;
                }

                //Check state collision with the scene
                for(int i=0; i<collision_shct.size(); i++)
                {
                    std::vector<double> r_pos;
                    for(int j = 0; j<rqsize; j++)
                    {
                        r_pos.push_back(radii_shct[j][i]);
                    }
                    if(!getSpaceInformation()->isStateValid(collision_shct[i], r_pos))
                    {
                        validity =  false;
                        break;
                    }
                }
                
                // If not valid states we check the next possible trajectory
                if(!validity)
                {
                    all_new_cost.push_back(std::numeric_limits<double>::max());
                    continue;
                }

                // Compute the cost
                std::vector<double> lambda_shct = radii_shct.back(); //The last element of the radii vector is the pnorm of the radii of interest
                all_new_cost.push_back(cost(all_possible_new_traj.at(k), all_new_wpt_index.at(k), lambda_shct));
            }
        }

        
        /////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////////////////
        // GET THE BEST COST AMONG ALL POSSIBLE SHORTCUTS

        int new_best_traj = -1;
        double best_new_cost = best_cost_;
        for(int k = 0; k<all_new_cost.size(); k++)
        {
            if(all_new_cost.at(k)<best_new_cost)
            {
                new_best_traj = k;
                best_new_cost = all_new_cost.at(k);
            }
        }

        if(new_best_traj >= 0)
        {
            best_cost_ = best_new_cost;
            std::swap(best_traj,all_possible_new_traj.at(new_best_traj));  
            std::swap(index_wpt_in_traj_,all_new_wpt_index.at(new_best_traj));
            std::swap(bestGains, gainsVector);
            exportGains(bestGains, best_cost_);
            // Get the ending time point
            end = std::chrono::high_resolution_clock::now();
            // Calculate the duration by subtracting start time from end time
            elapsed = end - start;
            exportShct(best_traj, best_cost_, elapsed.count());

            // Add the new cost to the window
            costsWindow_.push_back(best_cost_);
        }

        // If we use an adaptative ball radius we update the acceptance rate according to the cost evolution in the window (note that non valid states (collision) have infinity cost)
        if(radius_type_ == "adaptive") 
        {
            for(int k = 0; k<all_new_cost.size(); k++)
            {
                // Calculate the average cost from the costs window
                double avg_cost = std::accumulate(costsWindow_.begin(), costsWindow_.end(), 0.0) / costsWindow_.size();

                // Check if the new sample's cost is below the adaptive threshold
                bool is_accepted = all_new_cost.at(k) <= avg_cost;

                if (is_accepted)
                {
                    // Update the accepted sample statistics
                    acceptance_count_++;
                }
            }
        }

        nb_iter_++;
    }
    getRobot()->setGains(bestGains);
    return best_traj;
} 

