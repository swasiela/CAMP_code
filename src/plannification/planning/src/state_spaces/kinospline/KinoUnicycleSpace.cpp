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

#include "planning/state_spaces/kinospline/KinoUnicycleSpace.h"

void ompl::base::KinoUnicycleSpace::projectStates(const std::vector<ompl::base::State*> &des_states, std::vector<std::vector<double>> &collision_states) const
{
    std::vector<double> st, q, qdot;
    for (auto &state : des_states)
    {
        st.clear();
        q.clear();
        qdot.clear();
        
        q = state->as<StateType>()->getQValues();
        qdot = state->as<StateType>()->getQdotValues();

        st.push_back(q[0]); // X
        st.push_back(q[1]); // Y
        st.push_back(0.0); // Z

        // yaw (atan2(vy,vx))
        double yaw = atan2(qdot[1], qdot[0]);

        // Convert the orientations from rpy to quaternion
        Eigen::Quaterniond quat = euler2Quaternion(0.0,0.0,yaw); 
        
        st.push_back(quat.w()); // qw
        st.push_back(quat.x()); // qx
        st.push_back(quat.y()); // qy
        st.push_back(quat.z()); // qz
 
        collision_states.push_back(st);
    }   
}

bool ompl::base::KinoUnicycleSpace::simulateStates(const std::vector<ompl::base::State*> &des_states, const double dt, std::vector<std::vector<double>> &collision_states) const
{
    /* S03 state for CC: x, y, z, qw, qx, qy, qz */
    /* Unicycle final nominal state: x, y, theta */

    services_msgs::DynamicSrv dynamicSrv; // To comunicate with the dynamic service.

    /* Message initialization */
    dynamicSrv.request.t_init = 0.0; // Always start at 0 for the local path (even if the initial conditions are present).
    dynamicSrv.request.t_final = des_states.size()*dt; // Desired path length, we approximate the last state at a time step of dt.
    dynamicSrv.request.dt = dt; // Integration time step.

    /* Initialize the robot state (i.e. q0) */
    dynamicSrv.request.init_robot_state.x = des_states.at(0)->as<StateType>()->nominal_state_.at(0);
    dynamicSrv.request.init_robot_state.y = des_states.at(0)->as<StateType>()->nominal_state_.at(1);
    dynamicSrv.request.init_robot_state.vx = des_states.at(0)->as<StateType>()->nominal_state_.at(2);
    dynamicSrv.request.init_robot_state.vy = des_states.at(0)->as<StateType>()->nominal_state_.at(3);
    dynamicSrv.request.init_robot_state.qw = des_states.at(0)->as<StateType>()->nominal_state_.at(4);
    dynamicSrv.request.init_robot_state.qx = des_states.at(0)->as<StateType>()->nominal_state_.at(5);
    dynamicSrv.request.init_robot_state.qy = des_states.at(0)->as<StateType>()->nominal_state_.at(6);
    dynamicSrv.request.init_robot_state.qz = des_states.at(0)->as<StateType>()->nominal_state_.at(7);
    dynamicSrv.request.init_robot_state.wz = 0.0;

    /* Initialize uncertain model parameters and controler gains */
    dynamicSrv.request.model_params = getRobot()->getUncertainParams();
    
    std::vector<std::vector<double>> robot_gains = getRobot()->getGains();
    for(int i = 0; i<robot_gains.size(); i++)
    {
        services_msgs::GainsValues gains_values;
        gains_values.values = robot_gains.at(i);
        dynamicSrv.request.gains.push_back(gains_values); 
    }

    /* Fill the desired trajectory. The first one has already been used so we ski it. */
    for(int i = 1; i<des_states.size(); i++)
    {
        services_msgs::DesiredState des_state;
        std::vector<double> q = des_states.at(i)->as<StateType>()->getQValues();
        std::vector<double> qdot = des_states.at(i)->as<StateType>()->getQdotValues();
        std::vector<double> qddot = des_states.at(i)->as<StateType>()->getQddotValues();

        des_state.x = q.at(0);
        des_state.y = q.at(1);
        des_state.vx = qdot.at(0);
        des_state.vy = qdot.at(1);
        des_state.ax = qddot.at(0);
        des_state.ay = qddot.at(1);

        dynamicSrv.request.desired_states.push_back(des_state);
    }

    // To compare the service call time with the ODE resolution time
    auto start = std::chrono::high_resolution_clock::now();
    // Solve the dynamic
    if(!getRobot()->dynamic_.call(dynamicSrv))
    {
        ROS_WARN("Unable to solve tracking.");
        return false;
    }
    else
    {
        // To compare the service call time with the ODE resolution time
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> srv_duration = end - start;
        
        // Get the simulated states
        std::vector<double> trajX = dynamicSrv.response.trajX;
        std::vector<double> trajY = dynamicSrv.response.trajY;
        std::vector<double> trajVX = dynamicSrv.response.trajVX;
        std::vector<double> trajVY = dynamicSrv.response.trajVY;
        std::vector<double> trajQw = dynamicSrv.response.trajQw;
        std::vector<double> trajQx = dynamicSrv.response.trajQx;
        std::vector<double> trajQy = dynamicSrv.response.trajQy;
        std::vector<double> trajQz = dynamicSrv.response.trajQz;
        std::vector<double> trajWz = dynamicSrv.response.trajWz;

        for(int i = 0; i<trajX.size(); i++)
        {
            std::vector<double> st;
            st.push_back(trajX.at(i));
            st.push_back(trajY.at(i));
            st.push_back(0.0);
            st.push_back(trajQw.at(i));
            st.push_back(trajQx.at(i));
            st.push_back(trajQy.at(i));
            st.push_back(trajQz.at(i));
            collision_states.push_back(st);
        }

        // Make sure there is is not already a registered state
        des_states.back()->as<StateType>()->nominal_state_.clear();
        // Register the new nominal state
        des_states.back()->as<StateType>()->nominal_state_.push_back(trajX.back());
        des_states.back()->as<StateType>()->nominal_state_.push_back(trajY.back());
        des_states.back()->as<StateType>()->nominal_state_.push_back(trajVX.back());
        des_states.back()->as<StateType>()->nominal_state_.push_back(trajVY.back());
        des_states.back()->as<StateType>()->nominal_state_.push_back(trajQw.back());
        des_states.back()->as<StateType>()->nominal_state_.push_back(trajQx.back());
        des_states.back()->as<StateType>()->nominal_state_.push_back(trajQy.back());
        des_states.back()->as<StateType>()->nominal_state_.push_back(trajQz.back());
        des_states.back()->as<StateType>()->nominal_state_.push_back(0.0);

        // To compare the service call time with the ODE resolution time
        double srv_time_percentage = ((srv_duration.count()-dynamicSrv.response.executionTime)/srv_duration.count())*100;
        // std::cout << "Loss of service call time in percent : " << srv_time_percentage << std::endl;
        return true;
    }
}

bool ompl::base::KinoUnicycleSpace::simulateStatesAndTubes(const std::vector<ompl::base::State*> &des_states, double dt, std::vector<std::vector<double>> &collision_states,
                                        std::vector<std::vector<double>> &control_inputs, std::vector<std::vector<double>> &radii) const
{
    /* S03 state for CC: x, y, z, qw, qx, qy, qz */
    /* Quadrotor final nominal state: x, y, z, vx, vy, vz, qw, qx, qy, qz, wx, wy, wz */

    services_msgs::SensitivitySrv sensitivitySrv; // To comunicate with the sensitivity service.

    /* Message initialization */
    sensitivitySrv.request.t_init = 0.0; // Always start at 0 for the local path (even if the initial conditions are present).
    sensitivitySrv.request.t_final = des_states.size()*dt; // Desired path length, we approximate the last state at a time step of dt.
    sensitivitySrv.request.dt = dt; // Integration time step.

    /* Initialize the robot state (i.e. q0) */
    sensitivitySrv.request.init_robot_state.x = des_states.at(0)->as<StateType>()->nominal_state_.at(0);
    sensitivitySrv.request.init_robot_state.y = des_states.at(0)->as<StateType>()->nominal_state_.at(1);
    sensitivitySrv.request.init_robot_state.vx = des_states.at(0)->as<StateType>()->nominal_state_.at(2);
    sensitivitySrv.request.init_robot_state.vy = des_states.at(0)->as<StateType>()->nominal_state_.at(3);
    sensitivitySrv.request.init_robot_state.qw = des_states.at(0)->as<StateType>()->nominal_state_.at(4);
    sensitivitySrv.request.init_robot_state.qx = des_states.at(0)->as<StateType>()->nominal_state_.at(5);
    sensitivitySrv.request.init_robot_state.qy = des_states.at(0)->as<StateType>()->nominal_state_.at(6);
    sensitivitySrv.request.init_robot_state.qz = des_states.at(0)->as<StateType>()->nominal_state_.at(7);
    sensitivitySrv.request.init_robot_state.wz = 0.0;

    /* Initialize the sensitivity conditions (i.e. PI0, PI_xi0) */
    sensitivitySrv.request.initial_PI = des_states.at(0)->as<StateType>()->PI_;
    sensitivitySrv.request.initial_PI_xi = des_states.at(0)->as<StateType>()->PI_xi_;

    /* Initialize uncertain model parameters and controler gains */
    sensitivitySrv.request.model_params = getRobot()->getUncertainParams();

    std::vector<std::vector<double>> robot_gains = getRobot()->getGains();
    for(int i = 0; i<robot_gains.size(); i++)
    {
        services_msgs::GainsValues gains_values;
        gains_values.values = robot_gains.at(i);
        sensitivitySrv.request.gains.push_back(gains_values); 
    }

    /* Fill the desired trajectory. The first one has already been used so we ski it. */
    for(int j = 1; j<des_states.size();j++)
    {
        services_msgs::DesiredState des_state;
        std::vector<double> pos = des_states.at(j)->as<StateType>()->getQValues();
        std::vector<double> vel = des_states.at(j)->as<StateType>()->getQdotValues();
        std::vector<double> acc = des_states.at(j)->as<StateType>()->getQddotValues();

        des_state.x = pos.at(0);
        des_state.y = pos.at(1);
        des_state.vx = vel.at(0);
        des_state.vy = vel.at(1);
        des_state.ax = acc.at(0);
        des_state.ay = acc.at(1);

        sensitivitySrv.request.desired_states.push_back(des_state);
    }

    // To compare the service call time with the ODE resolution time
    auto start = std::chrono::high_resolution_clock::now();
    // Solve the dynamic
    if(!getRobot()->sensiODE_.call(sensitivitySrv))
    {
        ROS_WARN("Unable to solve tracking.");
        return false;
    }
    else
    {
        // To compare the service call time with the ODE resolution time
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> srv_duration = end - start;
        
        // Get the simulated states
        std::vector<double> trajX = sensitivitySrv.response.trajX;
        std::vector<double> trajY = sensitivitySrv.response.trajY;
        std::vector<double> trajVX = sensitivitySrv.response.trajVX;
        std::vector<double> trajVY = sensitivitySrv.response.trajVY;
        std::vector<double> trajQw = sensitivitySrv.response.trajQw;
        std::vector<double> trajQx = sensitivitySrv.response.trajQx;
        std::vector<double> trajQy = sensitivitySrv.response.trajQy;
        std::vector<double> trajQz = sensitivitySrv.response.trajQz;
        std::vector<double> trajWz = sensitivitySrv.response.trajWz;

        for(int i = 0; i<trajX.size(); i++)
        {
            std::vector<double> st;
            st.push_back(trajX.at(i));
            st.push_back(trajY.at(i));
            st.push_back(0.0);
            st.push_back(trajQw.at(i));
            st.push_back(trajQx.at(i));
            st.push_back(trajQy.at(i));
            st.push_back(trajQz.at(i));
            collision_states.push_back(st);
        }

        // Get the simulated control inputs
        control_inputs.push_back(sensitivitySrv.response.u1);
        control_inputs.push_back(sensitivitySrv.response.u2);

        // Get the uncertainty tubes
        radii.push_back(sensitivitySrv.response.ellipsoid_alongX);
        radii.push_back(sensitivitySrv.response.ellipsoid_alongY);
        radii.push_back(sensitivitySrv.response.ellipsoid_u1);
        radii.push_back(sensitivitySrv.response.ellipsoid_u2);

        // The last element is the pnorm of the radii of interest
        radii.push_back(sensitivitySrv.response.radii_lambda);

        // Make sure there is is not already a registered state
        des_states.back()->as<StateType>()->nominal_state_.clear();
        des_states.back()->as<StateType>()->PI_.clear();
        des_states.back()->as<StateType>()->PI_xi_.clear();
        // Register the new nominal state
        des_states.back()->as<StateType>()->nominal_state_.push_back(trajX.back());
        des_states.back()->as<StateType>()->nominal_state_.push_back(trajY.back());
        des_states.back()->as<StateType>()->nominal_state_.push_back(trajVX.back());
        des_states.back()->as<StateType>()->nominal_state_.push_back(trajVY.back());
        des_states.back()->as<StateType>()->nominal_state_.push_back(trajQw.back());
        des_states.back()->as<StateType>()->nominal_state_.push_back(trajQx.back());
        des_states.back()->as<StateType>()->nominal_state_.push_back(trajQy.back());
        des_states.back()->as<StateType>()->nominal_state_.push_back(trajQz.back());
        des_states.back()->as<StateType>()->nominal_state_.push_back(0.0);
        // Register new sensitivity conditions
        des_states.back()->as<StateType>()->PI_ = sensitivitySrv.response.final_PI;
        des_states.back()->as<StateType>()->PI_xi_ = sensitivitySrv.response.final_PI_xi;

        // To compare the service call time with the ODE resolution time
        double srv_time_percentage = ((srv_duration.count()-sensitivitySrv.response.executionTime)/srv_duration.count())*100;
        // std::cout << "Loss of service call time in percent : " << srv_time_percentage << std::endl;
        return true;
    }
}

bool ompl::base::KinoUnicycleSpace::localOpt(const std::vector<ompl::base::State*> &section_to_optimize, double dt, std::string stopping_condition_, double tolerance_, double max_time, int window_size, int nb_iter, std::string cost_id, std::string opti_params, 
                                                std::vector<ompl::base::State*> &traj_opt, double &cost) const
{
    ROS_ERROR("Unycicle localOpt NOT IMPLEMENTED !");
    return false;
}   

bool ompl::base::KinoUnicycleSpace::mpc(const std::vector<ompl::base::State*> &section_to_optimize, double dt, int nb_iter, int horizon, std::string cost_id, std::string opti_params, 
                                                std::vector<ompl::base::State*> &traj_opt, double &cost) const
{
    ROS_ERROR("Unycicle mpc NOT IMPLEMENTED !");
    return false;
}

void ompl::base::KinoUnicycleSpace::getPlatformFromEEF(std::vector<double>& q, std::vector<double>& acc) const
{
    ROS_ERROR("Unycicle getPlatformFromEEF NOT IMPLEMENTED !");
}