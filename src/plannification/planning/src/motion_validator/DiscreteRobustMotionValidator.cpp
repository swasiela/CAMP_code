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

#include "planning/motion_validator/DiscreteRobustMotionValidator.h"

// ###########################################################################################################################################
// SETUP FUNCTIONS

void ompl::base::DiscreteRobustMotionValidator::setupValidator()
{
    stateSpace_ = si_->getStateSpace().get();
    if (!stateSpace_)
        throw Exception("No state space for motion validator");

    // Get the current CAMP path 
    std::filesystem::path campDir_;
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

    if(!nh_.getParam("planning_parameters/profiling_file", profiling_file_))
    {
        ROS_WARN("Value of profiling_file not found. Using default profiling_file.");
        profiling_file_ = "CAMP/src/results/profiling_file.txt";
    }
    else
    {
        // Combine the CAMP directory with the relative file path
        std::filesystem::path fullPath = std::filesystem::absolute(campDir_ / profiling_file_);

        // Normalize the path to remove redundant ".." and "."
        fullPath = fullPath.lexically_normal();

        // Convert to string and use the full path
        profiling_file_ = fullPath.string();
    }
}

// ###########################################################################################################################################
// CHECK MOTION FUNCTIONS

bool ompl::base::DiscreteRobustMotionValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const
{
    std::vector<double> fake_tube{}; // Fake the uncertainty tubes

    // Assume motion starts in a valid configuration so s1 is valid check if s2 is valid or not to avoid full interpolation and collision check
    // Vector to recover the projection
    std::vector<std::vector<double>> sF; 
    // Make a copy of the final state
    ompl::base::State* s2_cpy = stateSpace_->allocState();
    stateSpace_->copyState(s2_cpy, s2);
    // Initialize the vector of states to be projected
    std::vector<ompl::base::State*> end_state{s2_cpy};
    stateSpace_->projectStates(end_state, sF); // Get the robot position and orientation in SO3 for collision checking from the desired states
    if (!si_->isStateValid(sF.at(0), fake_tube))
    {
        invalid_++;
        stateSpace_->freeState(s2_cpy);
        return false;
    }
    stateSpace_->freeState(s2_cpy);
    
    std::vector<ompl::base::State*> states;
    stateSpace_->interpolate(s1, s2, dt_, states); // Local steering between the two desired states

    std::vector<std::vector<double>> collision_states; // States used for collision checking, may differ from the desired one in the case of an underactuated system for example
    stateSpace_->projectStates(states, collision_states); // Get the robot position and orientation in SO3 for collision checking from the desired states

    for(int i=0; i< collision_states.size(); i++)
    {
        if(!si_->isStateValid(collision_states[i], fake_tube))
        {
            invalid_++;
            freeStateVector(states);
            return false;
        }
    }
 
    valid_++;  
    freeStateVector(states);
    return true;
}

bool ompl::base::DiscreteRobustMotionValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2, std::vector<ompl::base::State*>& states) const
{

    std::vector<double> fake_tube{}; // Fake the uncertainty tubes

    // Assume motion starts in a valid configuration so s1 is valid check if s2 is valid or not to avoid full interpolation and collision check
    // Vector to recover the projection
    std::vector<std::vector<double>> sF; 
    // Make a copy of the final state
    ompl::base::State* s2_cpy = stateSpace_->allocState();
    stateSpace_->copyState(s2_cpy, s2);
    // Initialize the vector of states to be projected
    std::vector<ompl::base::State*> end_state{s2_cpy};
    stateSpace_->projectStates(end_state, sF); // Get the robot position and orientation in SO3 for collision checking from the desired states
    if (!si_->isStateValid(sF.at(0), fake_tube))
    {
        invalid_++;
        stateSpace_->freeState(s2_cpy);
        return false;
    }
    stateSpace_->freeState(s2_cpy);
    
    stateSpace_->interpolate(s1, s2, dt_, states); // Local steering between the two desired states

    std::vector<std::vector<double>> collision_states; // States used for collision checking, may differ from the desired one in the case of an underactuated system for example
    stateSpace_->projectStates(states, collision_states); // Get the robot position and orientation in SO3 for collision checking from the desired states

    for(int i=0; i< collision_states.size(); i++)
    {
        if(!si_->isStateValid(collision_states[i], fake_tube))
        {
            invalid_++;
            return false;
        }
    }
 
    valid_++;  
    return true;
}

bool ompl::base::DiscreteRobustMotionValidator::checkMotionLearning(const State *s1, const State *s2, std::vector<ompl::base::State*>& states, torch::Tensor &traj_tensor) const
{
    // Make sure we use the NN at the same time step that the one used for trainning
    assert(dt_ == robot_->sensiNN_.getDtTrain() && "Time step used for NN prediction is not equal to the one used for training !");

    // Make sure the trajectory is empty
    freeStateVector(states);
    states.clear();

    std::vector<double> fake_tube{}; // Fake the uncertainty tubes

    // Assume motion starts in a valid configuration so s1 is valid check if s2 is valid or not to avoid full interpolation and collision check
    // Vector to recover the projection
    std::vector<std::vector<double>> sF; 
    // Make a copy of the final state
    ompl::base::State* s2_cpy = stateSpace_->allocState();
    stateSpace_->copyState(s2_cpy, s2);
    // Initialize the vector of states to be projected
    std::vector<ompl::base::State*> end_state{s2_cpy};
    stateSpace_->projectStates(end_state, sF); // Get the robot position and orientation in SO3 for collision checking from the desired states
    if (!si_->isStateValid(sF.at(0), fake_tube))
    {
        invalid_++;
        stateSpace_->freeState(s2_cpy);
        return false;
    }
    stateSpace_->freeState(s2_cpy);

    // auto start = std::chrono::high_resolution_clock::now();

    /* Interpolate between s1 and s2*/
    std::vector<std::vector<float>> vector_to_tensor_tubes;
    stateSpace_->interpolateLearning(s1, s2, dt_, states, vector_to_tensor_tubes); // Local steering between the two desired states

    /* Simulate the tracking to get the nominal trajectory */
    std::vector<std::vector<double>> collision_states; // States used for collision checking, may differ from the desired one in the case of an underactuated system for example
    bool simu = stateSpace_->simulateStates(states, dt_, collision_states); // Get the robot position and orientation in SO3 for collision checking from the desired states

    // Time
    // auto end = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    // std::cout << "Duration: " << duration.count() << " microseconds" << std::endl;

    // If the simulation goes wrong, it's very likely that the robot/controller is not able to track the trajectory.
    if(!simu)
    {
        invalid_++;
        return false;
    }

    // PREDICTION
    // Put neural network inputs in batch sequence features format
    std::vector<torch::jit::IValue> inputs;

    auto options = torch::TensorOptions().dtype(at::kFloat);
    traj_tensor = torch::zeros({1,vector_to_tensor_tubes.size(),vector_to_tensor_tubes[0].size()}, options);
    for (int i = 0; i < vector_to_tensor_tubes.size(); i++)
        traj_tensor[0].slice(0, i,i+1) = torch::from_blob(vector_to_tensor_tubes[i].data(), {vector_to_tensor_tubes[0].size()}, options);

    // Set the hidden state
    torch::Tensor h0 = s1->as<ompl::base::RobustStateSpace::StateType>()->h0_;
    inputs.push_back(traj_tensor);
    inputs.push_back(h0);

    // Predict
    c10::intrusive_ptr<c10::ivalue::Tuple> tubes_prediction = robot_->sensiNN_.predict(inputs);

    torch::Tensor tubes_pred = tubes_prediction->elements()[0].toTensor();
    states.back()->as<ompl::base::RobustStateSpace::StateType>()->h0_ = tubes_prediction->elements()[1].toTensor(); // Set the final hidden state

    inputs.clear();

    // CC
    //Check inputs first
    int rqsize = robot_->sensiNN_.getRqSize();
    int usize = robot_->sensiNN_.getUSize();
    for(int i=0; i<collision_states.size()-1; i++)
    {
        std::vector<double> u, ru;
        for(int j = 0; j<usize; j++)
        {
            u.push_back((tubes_pred[0][i][j+rqsize].item().toFloat())*robot_->sensiNN_.getStdOutTubes(j+rqsize)+robot_->sensiNN_.getMeanOutTubes(j+rqsize)+robot_->sensiNN_.getQuartileTubes(j+rqsize));
            ru.push_back((tubes_pred[0][i][j+rqsize+usize].item().toFloat())*robot_->sensiNN_.getStdOutTubes(j+rqsize+usize)+robot_->sensiNN_.getMeanOutTubes(j+rqsize+usize)+robot_->sensiNN_.getQuartileTubes(j+rqsize+usize));
        }
        if(!si_->isInputsValid(u, ru))
        {
            invalid_++;
            return false;
        }
    }
    //Check state collision with the scene, we don't take the last state because it was already checked
    for(int i=0; i<collision_states.size()-1; i++)
    {
        std::vector<double> r_pos;
        for(int j = 0; j<rqsize; j++)
        {
            double r = (tubes_pred[0][i][j].item().toFloat())*robot_->sensiNN_.getStdOutTubes(j)+robot_->sensiNN_.getMeanOutTubes(j)+robot_->sensiNN_.getQuartileTubes(j);
            if(r<0) // Should not happen, just for safety
                r = 0;
            r_pos.push_back(r);
        }
        if(!si_->isStateValid(collision_states[i], r_pos))
        {
            invalid_++;
            return false;
        }
    }

    valid_++;
    return true;
}

bool ompl::base::DiscreteRobustMotionValidator::checkMotionODE(const State *s1, const State *s2, std::vector<ompl::base::State*>& states) const
{
    std::vector<double> fake_tube{}; // Fake the uncertainty tubes

    // Make sure the trajectory is empty
    freeStateVector(states);
    states.clear();

    // Assume motion starts in a valid configuration so s1 is valid check if s2 is valid or not to avoid full interpolation and collision check
    // Vector to recover the projection
    std::vector<std::vector<double>> sF; 
    // Make a copy of the final state
    ompl::base::State* s2_cpy = stateSpace_->allocState();
    stateSpace_->copyState(s2_cpy, s2);
    // Initialize the vector of states to be projected
    std::vector<ompl::base::State*> end_state{s2_cpy};
    stateSpace_->projectStates(end_state, sF); // Get the robot position and orientation in SO3 for collision checking from the desired states
    if (!si_->isStateValid(sF.at(0), fake_tube))
    {
        invalid_++;
        stateSpace_->freeState(s2_cpy);
        return false;
    }
    stateSpace_->freeState(s2_cpy);

    /* Interpolate between s1 and s2*/
    stateSpace_->interpolate(s1, s2, dt_, states); // Local steering between the two desired states

    /* Simulate the tracking to get the nominal trajectory 
    * collision_states => the robot state in S03 used for collision checking, may differ from the desired one in the case of an underactuated system for example
    * control_inputs => nominal control inputs exerted by the system and computed by simulating the tracking under nominal system parameters
    * radii => state and control input tubes
    */
    // auto start = std::chrono::high_resolution_clock::now();
    std::vector<std::vector<double>> collision_states, control_inputs, radii;
    bool simu = stateSpace_->simulateStatesAndTubes(states, dt_, collision_states, control_inputs, radii); 

    // Time
    // auto end = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    // std::cout << "Duration: " << duration.count() << " microseconds" << std::endl;

    // Security in case the interpolation went wrong
    if(states.size() < 0)
        return false;
    
    // If the simulation goes wrong, it's very likely that the robot/controller is not able to track the trajectory.
    if(!simu)
    {
        invalid_++;
        return false;
    }

    // CC
    //Check inputs first
    int rqsize = robot_->sensiNN_.getRqSize();
    for(int i=0; i<collision_states.size(); i++)
    {
        std::vector<double> u, ru;
        for(int j = 0; j<control_inputs.size(); j++)
        {
            u.push_back(control_inputs[j][i]);
            ru.push_back(radii[rqsize+j][i]);
        }
        if(!si_->isInputsValid(u, ru))
        {
            invalid_++;
            return false;
        }
    }
    //Check state collision with the scene
    for(int i=0; i<collision_states.size(); i++)
    {
        std::vector<double> r_pos;
        for(int j = 0; j<rqsize; j++)
        {
            r_pos.push_back(radii[j][i]);
        }
        if(!si_->isStateValid(collision_states[i], r_pos))
        {
            invalid_++;
            return false;
        }
    }

    valid_++;
    return true;
}

/** \brief Implementation based on the python implementation of https://github.com/StanfordASL/randUP_RRT
 * the collision checking is done in the same way as the one found in the file "plan_quadrotor.py", "is_safe" function line 86.
 */
bool ompl::base::DiscreteRobustMotionValidator::checkMotionRandUp(const State *s1, const State *s2, const std::vector<std::vector<double>> &init_dynamic_states, 
                                                const std::vector<std::vector<double>> &rand_up_params, std::vector<std::vector<double>> &propagated_states, double padding) const
{
    std::vector<double> pad{padding, padding, padding}; // Padding for the epsilon-randup

    // Assume motion starts in a valid configuration so s1 is valid check if s2 is valid or not to avoid full interpolation and collision check
    // Vector to recover the projection
    std::vector<std::vector<double>> sF; 
    // Make a copy of the final state
    ompl::base::State* s2_cpy = stateSpace_->allocState();
    stateSpace_->copyState(s2_cpy, s2);
    // Initialize the vector of states to be projected
    std::vector<ompl::base::State*> end_state{s2_cpy};
    stateSpace_->projectStates(end_state, sF); // Get the robot position and orientation in SO3 for collision checking from the desired states
    if (!si_->isStateValid(sF.at(0), pad))
    {
        invalid_++;
        stateSpace_->freeState(s2_cpy);
        return false;
    }
    stateSpace_->freeState(s2_cpy);

    // Local steering between the two desired states
    std::vector<ompl::base::State*> states;
    stateSpace_->interpolate(s1, s2, dt_, states); 

    for(int i = 0; i<rand_up_params.size(); i++)
    {
        // Set the current robot parameters
        robot_->setUncertainParams(rand_up_params.at(i));

        // Set the starting nominal state to the current perturbed state
        states.at(0)->as<ompl::base::RobustStateSpace::StateType>()->nominal_state_ = init_dynamic_states.at(i);

        /* Simulate the tracking to get the nominal trajectory */
        std::vector<std::vector<double>> collision_states; // States used for collision checking, may differ from the desired one in the case of an underactuated system for example
        bool simu = stateSpace_->simulateStates(states, dt_, collision_states); // Get the robot position and orientation in SO3 for collision checking from the desired states

        // If the simulation goes wrong, it's very likely that the robot/controller is not able to track the trajectory.
        if(!simu)
        {
            invalid_++;
            freeStateVector(states);
            return false;
        }

        for(int i=0; i< collision_states.size(); i++)
        {
            if(!si_->isStateValid(collision_states[i], pad))
            {
                invalid_++;
                freeStateVector(states);
                return false;
            }
        }
        propagated_states.push_back(states.back()->as<ompl::base::RobustStateSpace::StateType>()->nominal_state_);
    }

    valid_++;  
    freeStateVector(states);
    return true;
}

// ###########################################################################################################################################
// RE-CHECK MOTION FUNCTIONS

bool ompl::base::DiscreteRobustMotionValidator::reCheckMotion(const std::vector <ompl::base::State*>& states) const
{
    // Assume that the start and final states have already been checked and are valid.
    std::vector<double> fake_tube{}; // Fake the uncertainty tubes
    std::vector<std::vector<double>> collision_states; // States used for collision checking, may differ from the desired one in the case of an underactuated system for example
    stateSpace_->projectStates(states, collision_states); // Get the robot position and orientation in SO3 for collision checking from the desired states

    for(int i=0; i< collision_states.size(); i++)
    {
        if(!si_->isStateValid(collision_states[i], fake_tube))
        {
            invalid_++;
            return false;
        }
    }
 
    valid_++;  
    return true;
}

bool ompl::base::DiscreteRobustMotionValidator::reCheckMotionLearning(const std::vector<ompl::base::State*>& states, const torch::Tensor& traj_tensor) const
{
    // Make sure we use the NN at the same time step that the one used for trainning
    assert(dt_ == robot_->sensiNN_.getDtTrain() && "Time step used for NN prediction is not equal to the one used for training !");

    // Assume that the start and final states have already been checked and are valid.

    /* Simulate the tracking to get the nominal trajectory. The robot's final nominal state is updated internally in the function. */
    std::vector<std::vector<double>> collision_states; // States used for collision checking, may differ from the desired one in the case of an underactuated system for example
    bool simu = stateSpace_->simulateStates(states, dt_, collision_states); // Get the robot position and orientation in SO3 for collision checking from the desired states

    // If the simulation goes wrong, it's very likely that the robot/controller is not able to track the trajectory.
    if(!simu)
    {
        invalid_++;
        return false;
    }

    // PREDICTION
    // Put neural network inputs in batch sequence features format
    std::vector<torch::jit::IValue> inputs;

    // Set the initial hidden state and the trajectory
    torch::Tensor h0 = states.at(0)->as<ompl::base::RobustStateSpace::StateType>()->h0_;
    inputs.push_back(traj_tensor);
    inputs.push_back(h0);

    // Predict
    c10::intrusive_ptr<c10::ivalue::Tuple> tubes_prediction = robot_->sensiNN_.predict(inputs);

    torch::Tensor tubes_pred = tubes_prediction->elements()[0].toTensor();
    states.back()->as<ompl::base::RobustStateSpace::StateType>()->h0_ = tubes_prediction->elements()[1].toTensor(); // Set the final hidden state

    inputs.clear();

    // CC
    //Check inputs first
    int rqsize = robot_->sensiNN_.getRqSize();
    int usize = robot_->sensiNN_.getUSize();
    for(int i=0; i<collision_states.size()-1; i++)
    {
        std::vector<double> u, ru;
        for(int j = 0; j<usize; j++)
        {
            u.push_back((tubes_pred[0][i][j+rqsize].item().toFloat())*robot_->sensiNN_.getStdOutTubes(j+rqsize)+robot_->sensiNN_.getMeanOutTubes(j+rqsize)+robot_->sensiNN_.getQuartileTubes(j+rqsize));
            ru.push_back((tubes_pred[0][i][j+rqsize+usize].item().toFloat())*robot_->sensiNN_.getStdOutTubes(j+rqsize+usize)+robot_->sensiNN_.getMeanOutTubes(j+rqsize+usize)+robot_->sensiNN_.getQuartileTubes(j+rqsize+usize));
        }
        if(!si_->isInputsValid(u, ru))
        {
            invalid_++;
            return false;
        }
    }
    //Check state collision with the scene
    for(int i=0; i<collision_states.size()-1; i++)
    {
        std::vector<double> r_pos;
        for(int j = 0; j<rqsize; j++)
        {
            double r = (tubes_pred[0][i][j].item().toFloat())*robot_->sensiNN_.getStdOutTubes(j)+robot_->sensiNN_.getMeanOutTubes(j)+robot_->sensiNN_.getQuartileTubes(j);
            if(r<0) // Should not happen, just for safety
                r = 0;
            r_pos.push_back(r);
        }
        if(!si_->isStateValid(collision_states[i], r_pos))
        {
            invalid_++;
            return false;
        }
    }

    valid_++;
    return true;
}

bool ompl::base::DiscreteRobustMotionValidator::reCheckMotionODE(const std::vector<ompl::base::State*>& states) const
{
    /* Simulate the tracking to get the nominal trajectory 
    * collision_states => the robot state in S03 used for collision checking, may differ from the desired one in the case of an underactuated system for example
    * control_inputs => nominal control inputs exerted by the system and computed by simulating the tracking under nominal system parameters
    * radii => state and control input tubes
    */
    std::vector<std::vector<double>> collision_states, control_inputs, radii;
    bool simu = stateSpace_->simulateStatesAndTubes(states, dt_, collision_states, control_inputs, radii);  

    // If the simulation goes wrong, it's very likely that the robot/controller is not able to track the trajectory.
    if(!simu)
    {
        invalid_++;
        return false;
    }

    // CC
    //Check inputs first
    int rqsize = robot_->sensiNN_.getRqSize();
    for(int i=0; i<collision_states.size(); i++)
    {
        std::vector<double> u, ru;
        for(int j = 0; j<control_inputs.size(); j++)
        {
            u.push_back(control_inputs[j][i]);
            ru.push_back(radii[rqsize+j][i]);
        }
        if(!si_->isInputsValid(u, ru))
        {
            invalid_++;
            return false;
        }
    }
    //Check state collision with the scene
    for(int i=0; i<collision_states.size(); i++)
    {
        std::vector<double> r_pos;
        for(int j = 0; j<rqsize; j++)
        {
            r_pos.push_back(radii[j][i]);
        }
        if(!si_->isStateValid(collision_states[i], r_pos))
        {
            invalid_++;
            return false;
        }
    }

    valid_++;
    return true;
}
// ###########################################################################################################################################

void ompl::base::DiscreteRobustMotionValidator::exportProfiling(std::string process_id, double running_time) const
{
    std::ofstream iter_overtime(profiling_file_, std::ios::app);
    std::cout.precision(6);

    iter_overtime<< "\t" << "ID:" << process_id << ";" << "Running time:" << running_time << "\n";

    iter_overtime.close();
}