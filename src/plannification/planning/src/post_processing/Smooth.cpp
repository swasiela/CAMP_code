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

#include "planning/post_processing/Smooth.h"

double ompl::Smooth::cost(const std::vector<ompl::base::State*> &traj) const
{
    return traj.size();
}

std::vector<ompl::base::State*> ompl::Smooth::postProcess(ompl::geometric::PathGeometric &pg, std::vector<int>& index_wpt_in_traj_)
{
  /** Smoothing performed using simple robust shortcut implmentation but only sonsidering length path as cost function */
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
    
  // Init the trajectory to optimize
  std::vector<ompl::base::State *> best_traj = pg.getStates();

  // Init the robot gains to optimize
  std::vector<std::vector<double>> bestGains = getRobot()->getGains();

  // Init the cost
  double best_cost_ = cost(best_traj);

  // Export the initialization
  // Get the ending time point
  auto end = std::chrono::high_resolution_clock::now();
  // Calculate the duration by subtracting start time from end time
  std::chrono::duration<double> elapsed = end - start;
  exportShct(best_traj, best_cost_, elapsed.count());

  // Reset iteration counter 
  nb_iter_ = 0;
  // Start planning time
  start_time_ = std::chrono::steady_clock::now();
  while(nb_iter_ < 50)
  {
    /////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////
    // SAMPLING

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
    // SHORTCUT VALIDITY

    // Check first if the shortcut is geometrically feasible
    std::vector<ompl::base::State*> traj_shortcut;
    double odl_tshct = best_traj.at(idx_s2)->as<ompl::base::RobustStateSpace::StateType>()->st_time_; // old time idx_s2
    if(getSpaceInformation()->checkMotion(best_traj.at(idx_s1), best_traj.at(idx_s2), traj_shortcut))
    {
      //If the shortcut is feasible we create the new possible trajectory
      std::vector<ompl::base::State*> possible_shortcut;
      for(int j = 0; j<idx_s1; j++) // From s0 to sidx1-1
      {
        possible_shortcut.push_back(best_traj.at(j));
      }
      for(int j = 0; j<traj_shortcut.size()-1; j++) // From sidx1 to sidx2-1
      {
        possible_shortcut.push_back(traj_shortcut.at(j));
      }
      int shct_idx = possible_shortcut.size()-1;
      double new_tshct = traj_shortcut.back()->as<ompl::base::RobustStateSpace::StateType>()->st_time_; // new time idx_s2
      for(int j = idx_s2; j<best_traj.size(); j++) // From sidx2 to sF and update state time according to the new shortcut
      {
        ompl::base::State* possible_end_st = getSpaceInformation()->allocState();
        getSpaceInformation()->copyState(possible_end_st, best_traj.at(j));
        possible_end_st->as<ompl::base::RobustStateSpace::StateType>()->st_time_= possible_shortcut.back()->as<ompl::base::RobustStateSpace::StateType>()->st_time_+getSpaceInformation()->getMotionValidator()->getDt();
        possible_shortcut.push_back(possible_end_st);
      } 
        
      // We compute new point-of-interest indices 
      std::vector<int> new_wpt_index = index_wpt_in_traj_;
      for (size_t j = idx_section; j < index_wpt_in_traj_.size(); j++)
      {
        if(j==idx_section)
        {
          new_wpt_index.at(j) = idx_s1 + traj_shortcut.size()-1 + (index_wpt_in_traj_.at(j)-idx_s2);   
        }
        else
        {
          new_wpt_index.at(j) = new_wpt_index.at(idx_section) + (index_wpt_in_traj_.at(j) - index_wpt_in_traj_.at(idx_section));
        }
      }

      // We start with the cost, because it's less expensive to compute.
      double new_cost = cost(possible_shortcut);
      if(new_cost>best_cost_)
        continue;

      // Check the feasability of the whole shortcuted trajectory with tubes and compute its cost
      std::vector<std::vector<double>> collision_shct, control_inputs_shct, radii_shct;

      bool simu_shct = getSpaceInformation()->getStateSpace()->simulateStatesAndTubes(possible_shortcut, getSpaceInformation()->getMotionValidator()->getDt(), collision_shct, control_inputs_shct, radii_shct); 

      if(!simu_shct)
      {
        nb_iter_++;
        continue;
      }   
      else
      {  
        if(new_cost<best_cost_)
        {
          // Robustly check the motion
          bool validity = true;

          //Check inputs first
          int rqsize = getRobot()->sensiNN_.getRqSize();
          int usize = getRobot()->sensiNN_.getUSize();
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

          if(validity)
          {
            //Check state collision with the scene
            for(int i=0; i<collision_shct.size(); i++)
            {
              std::vector<double> r_pos;
              for(int j = 0; j<rqsize; j++)
              {
                r_pos.push_back(radii_shct[j][i]+0.0); // Bigger margin because some other post process are very sensitive to the initial robustness
              }
              if(!getSpaceInformation()->isStateValid(collision_shct[i], r_pos))
              {
                validity =  false;
                break;
              }
            }
          }
          
          if(validity)
          {
            for(int j = shct_idx+1; j<possible_shortcut.size(); j++) // From sidx2 to sF, here we need to update the time vector
            {
                double new_time = new_tshct + possible_shortcut.at(j)->as<ompl::base::RobustStateSpace::StateType>()->st_time_ - odl_tshct;
                possible_shortcut.at(j)->as<ompl::base::RobustStateSpace::StateType>()->st_time_ = new_time;
            }

            best_cost_ = new_cost;
            std::swap(best_traj,possible_shortcut);  
            std::swap(index_wpt_in_traj_,new_wpt_index);
            // Get the ending time point
            end = std::chrono::high_resolution_clock::now();
            // Calculate the duration by subtracting start time from end time
            elapsed = end - start;
            exportShct(best_traj,best_cost_, elapsed.count());
          }
        }
      }
    }
    nb_iter_++;
  }

  getRobot()->setGains(bestGains);
  return best_traj;
} 

