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

// Planner
#include "planning/planners/Generator.h"

// OMPL
#include "ompl/base/Goal.h"
#include "ompl/base/goals/GoalState.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/base/samplers/InformedStateSampler.h"
#include "ompl/base/samplers/informed/RejectionInfSampler.h"
#include "ompl/base/samplers/informed/OrderedInfSampler.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/util/GeometricEquations.h"
#include "ompl/base/goals/GoalSampleableRegion.h"

#define BOUND_TO_01(v) ((v<0)? (0): ( (v>1)? (1): (v)))

ompl::geometric::Generator::Generator(const base::SpaceInformationPtr &si, const BulletVisualToolsPtr &visual_tools, const std::string &ros_namespace, bool addIntermediateStates)
  : base::Planner(si, "Generator"), visual_tools_(visual_tools), ros_namespace_(ros_namespace)
{
    specs_.approximateSolutions = true;
    specs_.directed = true;

    Planner::declareParam<double>("tl", this, &Generator::setLocalLength, &Generator::getLocalLength, "0.:0.1:10000.");
    Planner::declareParam<double>("tf", this, &Generator::setGlobalLength, &Generator::getGlobalLength, "0.:0.1:10000.");
    Planner::declareParam<double>("dt", this, &Generator::setDtPlanning, &Generator::getDtPlanning, "0.:0.05:0.1");
    Planner::declareParam<int>("nb_data", this, &Generator::setNbData, &Generator::getNbData, "1:1:10000");
    Planner::declareParam<int>("max_filter", this, &Generator::setNbFilter, &Generator::getNbFilter, "0:1:100");
}

ompl::geometric::Generator::~Generator()
{
    freeMemory();
}

void ompl::geometric::Generator::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
}

void ompl::geometric::Generator::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(Tl_);

    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });

    // Setup optimization objective
    //
    // If no optimization objective was specified, then default to
    // optimizing path length as computed by the distance() function
    // in the state space.
    if (pdef_)
    {
        if (pdef_->hasOptimizationObjective())
            opt_ = pdef_->getOptimizationObjective();
        else
        {
            OMPL_INFORM("%s: No optimization objective specified. Defaulting to optimizing path length for the allowed "
                        "planning time.",
                        getName().c_str());
            opt_ = std::make_shared<base::PathLengthOptimizationObjective>(si_);

            // Store the new objective in the problem def'n
            pdef_->setOptimizationObjective(opt_);
        }
    }
    else
    {
        OMPL_INFORM("%s: problem definition is not set, deferring setup completion...", getName().c_str());
        setup_ = false;
    }

    // To clear the file 
    std::ofstream ofs;
    ofs.open(export_file_no_gains_, std::ofstream::out | std::ofstream::trunc);
    ofs.close(); 
    ofs.open(export_traj_w_gains_, std::ofstream::out | std::ofstream::trunc);
    ofs.close();
    ofs.open(export_outputs_w_gains_, std::ofstream::out | std::ofstream::trunc);
    ofs.close();

    if(visual_tools_->getRobot()->getName() == "quadrotor")
    {
        std::ofstream data_(export_file_no_gains_, std::ios::app);
        std::cout.precision(9);

        data_ << "Vx," << "Vy," << "Vz," << "Wyaw,"<< "Ax,"<< "Ay,"<< "Az,"<<
            "rx,"<< "ry,"<< "rz,"<< "u1," << "u2,"<< "u3,"<< "u4,"<<"ru1,"<<"ru2,"<<"ru3,"<<"ru4,"<<"lambda_pnorm"<<"\n";
        data_.close();
    }
    else if(visual_tools_->getRobot()->getName() == "unicycle")
    {
        std::ofstream data_(export_file_no_gains_, std::ios::app);
        std::cout.precision(9);

        data_ << "Vx," << "Vy," << "Ax,"<< "Ay,"<<
            "rx,"<< "ry,"<< "u1," << "u2," <<"ru1,"<<"ru2,"<<"lambda_pnorm"<<"\n";
        data_.close();
    }

    // Node handler
    ros::NodeHandle nh_(ros_namespace_);
    if(!nh_.getParam("planning_parameters/visualize_tree", visualize_tree_))
    {
        ROS_WARN("Value of visualize_tree not found. Using default visualize_tree_ = false.");
        visualize_tree_ = false;
    }
    
    // Set the nominal gains
    gains_ = visual_tools_->getRobot()->getGains();
}

void ompl::geometric::Generator::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }
}

ompl::base::PlannerStatus ompl::geometric::Generator::solve(const base::PlannerTerminationCondition &ptc)
{
    // The number of trajectories currently exported
    int nb_export = 0;

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        nn_->add(motion);
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    auto *rmotion = new Motion(si_);
    base::State *rstate = rmotion->state;
    base::State *xstate = si_->allocState();

    while (nb_export < nb_data_)
    {
        sampler_->sampleUniform(rstate);

        /* find closest state in the tree */
        Motion *nmotion = nn_->nearest(rmotion);
        base::State *dstate = rstate;

        /* find state to add */
        double d = si_->distance(nmotion->state, rstate);
        if (d > Tl_)
        {
            si_->getStateSpace()->interpolate(nmotion->state, rstate, Tl_ / d, xstate);
            dstate = xstate;
        }

        auto *motion = new Motion(si_);
        si_->copyState(motion->state, dstate);
        motion->parent = nmotion;
        motion->incCost = opt_->motionCost(nmotion->state, motion->state);
        motion->cost = opt_->combineCosts(nmotion->cost, motion->incCost);

        motion->parent->children.push_back(motion);
        nn_->add(motion);

        // For visualization
        if(visualize_tree_)
        {
            motion->stateVisualID = visual_tools_->addState(motion->state);
            motion->edgeVisualID = visual_tools_->addEdge(nmotion->state, motion->state);
        }

        nmotion = motion;

        if(motion->cost.value() > Tf_)
        {
            std::vector<std::vector<double>> traj, nom_states, control_inputs, radii;

            // Interpolate the desired trajectory
            std::vector<ompl::base::State*> traj_states = interpolate(motion, traj); 

            // Simulate with the nominal gains
            if(!si_->getStateSpace()->simulateStatesAndTubes(traj_states, dt_planning_, nom_states, control_inputs, radii))
            {
                continue;
            }

            // We filter the outliers and export the trajectory
            if(filterAndExportTrajectory(traj, control_inputs, radii))
            {
                nb_export++;
                ROS_INFO_STREAM("Trajectory nb "<< nb_export);
                resetTree();
            }
            else
            {
                resetTree();
                continue;
            }
        }          
    }

    bool solved = true;
    bool approximate = false;
    si_->freeState(xstate);
    if (rmotion->state != nullptr)
        si_->freeState(rmotion->state);
    delete rmotion;

    OMPL_INFORM("DATABASE GENERATION DONE !");

    return {solved, approximate};
}

void ompl::geometric::Generator::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state));
        else
            data.addEdge(base::PlannerDataVertex(motion->parent->state), base::PlannerDataVertex(motion->state));
    }
}

void ompl::geometric::Generator::resetTree() 
{
    // Reset the tree
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        base::State *st = si_->allocState();

        for (auto &motion : motions)
        {
            if(motion->parent != nullptr)
            {
                if (motion->state != nullptr)
                    si_->freeState(motion->state);
                
                // For visualization
                if(visualize_tree_)
                {
                    visual_tools_->removeItem(motion->edgeVisualID);
                    visual_tools_->removeItem(motion->stateVisualID);
                }

                delete motion;
            }
            else
                si_->copyState(st, motion->state);
        }
        nn_->clear();
        auto *start_motion = new Motion(si_);
        si_->copyState(start_motion->state, st);
        nn_->add(start_motion);
    }
}

std::vector<ompl::base::State*> ompl::geometric::Generator::interpolate(Motion* motion, std::vector<std::vector<double>>& traj_vectorized)
{

    std::vector<base::State*> states, traj_states;
    
    Motion* current_motion = motion;
    while(current_motion->parent != nullptr)
    {
        states.push_back(current_motion->state);
        current_motion = current_motion->parent;
    }
    states.push_back(current_motion->state);
    std::reverse(states.begin(), states.end());

    if(visual_tools_->getRobot()->getTrajType() == "Kinospline")
    {
        for(size_t i=0; i<states.size()-1; ++i)
        {                
            std::vector<ompl::base::State*> states_interpolate;
            
            si_->getStateSpace()->interpolate(states.at(i), states.at(i+1), dt_planning_, states_interpolate);

            /** Update the initial state (nmotion) and the last state (dstate) with initial conditions encoded in steering.at(0) and steering.back(). 
             * nmotion and steering.at(0) have the same state values but are not the same object. Same for dstate and steering.back()
            */
            si_->copyState(states.at(i), states_interpolate.at(0));
            si_->copyState(states.at(i+1), states_interpolate.back());

            for(size_t j=0; j<states_interpolate.size()-1; j++)
            {
                std::vector<double> state_vectorized;

                auto pos = states_interpolate.at(j)->as<ompl::base::KinosplineStateSpace::StateType>()->getQValues();
                auto vel = states_interpolate.at(j)->as<ompl::base::KinosplineStateSpace::StateType>()->getQdotValues();
                auto acc = states_interpolate.at(j)->as<ompl::base::KinosplineStateSpace::StateType>()->getQddotValues();

                state_vectorized.push_back(pos[0]);//x
                state_vectorized.push_back(pos[1]);//y
                state_vectorized.push_back(pos[2]);//z
                state_vectorized.push_back(pos[3]);//yaw
                state_vectorized.push_back(vel[0]);//vx
                state_vectorized.push_back(vel[1]);//vy
                state_vectorized.push_back(vel[2]);//vz
                state_vectorized.push_back(vel[3]);//wyaw
                state_vectorized.push_back(acc[0]);//ax
                state_vectorized.push_back(acc[1]);//ay
                state_vectorized.push_back(acc[2]);//az
                state_vectorized.push_back(acc[3]);//awyaw

                // Fill the traj_vectorized which is used for the export and traj_states used for simulation
                traj_vectorized.push_back(state_vectorized);
                traj_states.push_back(states_interpolate[j]);
            }

            if( i == states.size()-2)
            {
                // Copy the last state
                std::vector<double> state_vectorized;

                auto pos = states_interpolate.back()->as<ompl::base::KinosplineStateSpace::StateType>()->getQValues();
                auto vel = states_interpolate.back()->as<ompl::base::KinosplineStateSpace::StateType>()->getQdotValues();
                auto acc = states_interpolate.back()->as<ompl::base::KinosplineStateSpace::StateType>()->getQddotValues();

                state_vectorized.push_back(pos[0]);//x
                state_vectorized.push_back(pos[1]);//y
                state_vectorized.push_back(pos[2]);//z
                state_vectorized.push_back(pos[3]);//yaw
                state_vectorized.push_back(vel[0]);//vx
                state_vectorized.push_back(vel[1]);//vy
                state_vectorized.push_back(vel[2]);//vz
                state_vectorized.push_back(vel[3]);//wyaw
                state_vectorized.push_back(acc[0]);//ax
                state_vectorized.push_back(acc[1]);//ay
                state_vectorized.push_back(acc[2]);//az
                state_vectorized.push_back(acc[3]);//awyaw

                traj_vectorized.push_back(state_vectorized);
                traj_states.push_back(states_interpolate.back());
            }
        }
    }
    else if(visual_tools_->getRobot()->getTrajType() == "Dubins")
    {
        // If the robot is quadrotor
        if(visual_tools_->getRobot()->getName() == "quadrotor")
        {
            ROS_ERROR("Generator interpolate not implemented for quadrotor with Dubins !");
        }
        else if(visual_tools_->getRobot()->getName() == "unicycle")
        {
            for(size_t i=0; i<states.size()-1; ++i)
            {          
                // Interpolate the trajectory between two desired state at a new resolution
                std::vector<ompl::base::State*> new_res_states;
                si_->getStateSpace()->interpolate(states.at(i), states.at(i+1), dt_planning_, new_res_states);

                si_->copyState(states.at(i), new_res_states.at(0));
                si_->copyState(states.at(i+1), new_res_states.back());

                for(size_t j=0; j<new_res_states.size()-1; j++)
                {
                    traj_states.push_back(new_res_states[j]);
                }
                if( i == states.size()-2)
                {
                    // Copy the last state
                    traj_states.push_back(new_res_states.back());
                }
            }

            /* Compute velocities and acceleration by differentation */
            std::vector<double> des_x, des_y, times;
            for(int i = 0; i<traj_states.size(); i++)
            {
                des_x.push_back(traj_states.at(i)->as<ompl::base::DubinsUnicycleSpace::StateType>()->getX());
                des_y.push_back(traj_states.at(i)->as<ompl::base::DubinsUnicycleSpace::StateType>()->getY());
                times.push_back(traj_states.at(i)->as<ompl::base::DubinsUnicycleSpace::StateType>()->st_time_);
            }

            std::vector<double> des_vx(des_x.size()), des_vy(des_x.size()), des_ax(des_x.size()), des_ay(des_x.size());

            // Finite difference
            des_vx[0] = (des_x[0] - traj_states.at(0)->as<ompl::base::DubinsUnicycleSpace::StateType>()->getPrevX()) / (traj_states.at(0)->as<ompl::base::DubinsUnicycleSpace::StateType>()->st_time_ - traj_states.at(0)->as<ompl::base::DubinsUnicycleSpace::StateType>()->getPrevTime());
            des_vy[0] = (des_y[0] - traj_states.at(0)->as<ompl::base::DubinsUnicycleSpace::StateType>()->getPrevY()) / (traj_states.at(0)->as<ompl::base::DubinsUnicycleSpace::StateType>()->st_time_ - traj_states.at(0)->as<ompl::base::DubinsUnicycleSpace::StateType>()->getPrevTime());

            // Main loop
            for (size_t i = 1; i < des_x.size(); ++i) {
                des_vx[i] = (des_x[i] - des_x[i - 1]) / (times[i] - times[i - 1]);
                des_vy[i] = (des_y[i] - des_y[i - 1]) / (times[i] - times[i - 1]);
            }

            /* Fill the desired trajectory. The first one has already been used so we skip it. */
            for(int i = 1; i<des_x.size(); i++)
            {
                std::vector<double> state_vectorized;

                state_vectorized.push_back(des_x.at(i));
                state_vectorized.push_back(des_y.at(i));
                state_vectorized.push_back(0.0);
                state_vectorized.push_back(des_vx.at(i));
                state_vectorized.push_back(des_vy.at(i));
                state_vectorized.push_back(0.0);
                state_vectorized.push_back(des_ax.at(i));
                state_vectorized.push_back(des_ay.at(i));

                traj_vectorized.push_back(state_vectorized);
            }
        }
    }
    
    traj_states.push_back(states.at(states.size()-1));

    return traj_states;
}

bool ompl::geometric::Generator::filterAndExportTrajectory(const std::vector<std::vector<double>> &traj, const std::vector<std::vector<double>> &control_inputs, const std::vector<std::vector<double>> &radii)
{

    //     data_ << "Vx," << "Vy," << "Vz," << "Wyaw,"<< "Ax,"<< "Ay,"<< "Az,"<<
    //         "rx,"<< "ry,"<< "rz,"<< "u1," << "u2,"<< "u3,"<< "u4,"<<"ru1,"<<"ru2,"<<"ru3,"<<"ru4,"<<"lambda"<<"\n";

    //     data_ << "Vx," << "Vy," << "Ax,"<< "Ay"<<
    //            "rx,"<< "ry,"<< "u1," << "u2," <<"ru1,"<<"ru2,"<<"lambda_pnorm"<<"\n";

    if(visual_tools_->getRobot()->getName() == "quadrotor")
    {
        // Filter outliers
        bool valid_radius = true;
        for(int i = 0; i<radii.at(3).size(); i++)
        {
            double max_input = std::max(control_inputs.at(0).at(i), std::max(control_inputs.at(1).at(i),std::max(control_inputs.at(2).at(i),control_inputs.at(3).at(i))));
            if(max_input > 20000.)
            {
                valid_radius = false;
                break;
            }

            double max_radius_input = std::max(radii.at(3).at(i), std::max(radii.at(4).at(i),std::max(radii.at(5).at(i),radii.at(6).at(i))));
            if(max_radius_input > 20000.)
            {
                valid_radius = false;
                break;
            }
        }
        if(!valid_radius)
        {
            return false;
        }
            
        std::ofstream data_(export_file_no_gains_, std::ios::app);
        std::cout.precision(15);

        // Export trajectory values
        // ######################################################################################################
        // Velocity
        for(int j = 4; j<8; j++)
        {
            for(int i = 0; i<traj.size()-1; i++)
            {
                double data_traj = traj.at(i).at(j);
                data_ << data_traj << " ";
            }
            data_ << ",";
        }
        
        // Acceleration
        for(int i = 0; i<traj.size()-1; i++)
        {
            double data_traj = traj.at(i).at(8);
            data_ << data_traj << " ";   
        }
        data_ << ",";
        for(int i = 0; i<traj.size()-1; i++)
        {
            double data_traj = traj.at(i).at(9);
            data_ << data_traj << " ";
        }
        data_ << ",";
        for(int i = 0; i<traj.size()-1; i++)
        {
            double data_traj = traj.at(i).at(10);
            data_ << data_traj << " ";
        }
        data_ << ",";
        // ######################################################################################################

        // Export radii
        // ######################################################################################################
        for (int j = 0; j<radii.size()-control_inputs.size()-1; j++)
        {
            for(int i = 0; i<radii.at(j).size(); i++)
            {
                data_ << radii.at(j).at(i) << " ";
            }
            data_ << ",";
        }
        // ######################################################################################################

        // Export control_inputs
        // ######################################################################################################
        for (int j = 0; j<control_inputs.size(); j++)
        {
            for(int i = 0; i<control_inputs.at(j).size(); i++)
            {
                data_ << control_inputs.at(j).at(i) << " ";
            }
            data_ << ",";
        }

        // Export control inputs radii
        // ######################################################################################################
        for (int j = radii.size()-control_inputs.size()-1; j<radii.size()-1; j++)
        {
            for(int i = 0; i<radii.at(j).size(); i++)
            {
                data_ << radii.at(j).at(i) << " ";
            }
            data_ << ",";
        }

        // Export lambda (pnorm of the radii of interest)
        for(int i = 0; i<radii.back().size(); i++)
        {
            data_ << radii.back().at(i) << " ";
        }
        
        data_ << "\n";
        data_.close();

        return true;
    }
    else if(visual_tools_->getRobot()->getName() == "unicycle")
    {
        // Filter outliers
        bool valid_radius = true;
        for(int i = 0; i<radii.at(2).size(); i++)
        {
            double max_input = std::max(control_inputs.at(0).at(i), control_inputs.at(1).at(i));
            if(max_input > 100.)
            {
                valid_radius = false;
                break;
            }

            double max_radius_input = std::max(radii.at(2).at(i), radii.at(3).at(i));
            if(max_radius_input > 100.)
            {
                valid_radius = false;
                break;
            }
        }
        if(!valid_radius)
        {
            return false;
        }
            
        std::ofstream data_(export_file_no_gains_, std::ios::app);
        std::cout.precision(15);

        // Export trajectory values
        // ######################################################################################################
        // Velocity
        for(int j = 3; j<5; j++)
        {
            for(int i = 0; i<traj.size()-1; i++)
            {
                double data_traj = traj.at(i).at(j);
                data_ << data_traj << " ";
            }
            data_ << ",";
        }
        
        // Acceleration
        for(int i = 0; i<traj.size()-1; i++)
        {
            double data_traj = traj.at(i).at(6);
            data_ << data_traj << " ";   
        }
        data_ << ",";
        for(int i = 0; i<traj.size()-1; i++)
        {
            double data_traj = traj.at(i).at(7);
            data_ << data_traj << " ";
        }
        data_ << ",";
        // ######################################################################################################

        // Export radii
        // ######################################################################################################
        for (int j = 0; j<radii.size()-control_inputs.size()-1; j++)
        {
            for(int i = 0; i<radii.at(j).size(); i++)
            {
                data_ << radii.at(j).at(i) << " ";
            }
            data_ << ",";
        }
        // ######################################################################################################

        // Export control_inputs
        // ######################################################################################################
        for (int j = 0; j<control_inputs.size(); j++)
        {
            for(int i = 0; i<control_inputs.at(j).size(); i++)
            {
                data_ << control_inputs.at(j).at(i) << " ";
            }
            data_ << ",";
        }

        // Export control inputs radii
        // ######################################################################################################
        for (int j = radii.size()-control_inputs.size()-1; j<radii.size()-1; j++)
        {
            for(int i = 0; i<radii.at(j).size(); i++)
            {
                data_ << radii.at(j).at(i) << " ";
            }
            data_ << ",";
        }

        // Export lambda (pnorm of the radii of interest)
        for(int i = 0; i<radii.back().size(); i++)
        {
            data_ << radii.back().at(i) << " ";
        }
        
        data_ << "\n";
        data_.close();

        return true;
    }
}