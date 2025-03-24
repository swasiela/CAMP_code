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

#include "planning/state_spaces/kinospline/KinoQuadrotorSpace.h"

void ompl::base::KinoQuadrotorSpace::projectStates(const std::vector<ompl::base::State*> &des_states, std::vector<std::vector<double>> &collision_states) const
{
    /* S03 state for CC: x, y, z, qw, qx, qy, qz */
    std::vector<double> st, q, qddot;
    for (auto &state : des_states)
    {
        st.clear();
        q.clear();
        qddot.clear();
        
        q = state->as<StateType>()->getQValues();
        qddot = state->as<StateType>()->getQddotValues();

        st.push_back(q[0]); // X
        st.push_back(q[1]); // Y
        st.push_back(q[2]); // Z

        // Define the thrust vector according to Newton law
        double tx = getRobot()->getMass()*qddot[0];
        double ty = getRobot()->getMass()*qddot[1];
        double tz = getRobot()->getMass()*qddot[2]+getRobot()->getMass()*9.81;

        // Compute the rotation of the thrust given the desired yaw
        double newTx = tx * cos(q[3]) - ty * sin(q[3]);
        double newTy = tx * sin(q[3]) + ty * cos(q[3]);
        double newTz = tz;

        // Compute the roll and pitch angles
        double roll_angle = atan2(newTy, newTz);
        double pitch_angle = - atan2(-newTx, sqrt(newTy*newTy + newTz*newTz));

        // Convert the orientations from rpy to quaternion
        Eigen::Quaterniond quat = euler2Quaternion(roll_angle,pitch_angle,q[3]); 
        
        st.push_back(quat.w()); // qw
        st.push_back(quat.x()); // qx
        st.push_back(quat.y()); // qy
        st.push_back(quat.z()); // qz

        collision_states.push_back(st);
    }
}

bool ompl::base::KinoQuadrotorSpace::simulateStates(const std::vector<ompl::base::State*> &des_states, const double dt, std::vector<std::vector<double>> &collision_states) const
{
    /* S03 state for CC: x, y, z, qw, qx, qy, qz */
    /* Quadrotor final nominal state: x, y, z, vx, vy, vz, qw, qx, qy, qz, wx, wy, wz */

    services_msgs::DynamicSrv dynamicSrv; // To comunicate with the dynamic service.

    /* Initialize integrator */
    dynamicSrv.request.integrator = getRobot()->getIntegratorType();

    /* Message initialization */
    dynamicSrv.request.t_init = des_states.at(0)->as<StateType>()->st_time_; // Always start at 0 for the local path (even if the initial conditions are present).
    dynamicSrv.request.t_final = des_states.back()->as<StateType>()->st_time_; // Desired path length, we approximate the last state at a time step of dt.
    dynamicSrv.request.dt = dt; // Integration time step.

    /* Initialize the robot state (i.e. q0) */
    dynamicSrv.request.init_robot_state.x = des_states.at(0)->as<StateType>()->nominal_state_.at(0);
    dynamicSrv.request.init_robot_state.y = des_states.at(0)->as<StateType>()->nominal_state_.at(1);
    dynamicSrv.request.init_robot_state.z = des_states.at(0)->as<StateType>()->nominal_state_.at(2);
    dynamicSrv.request.init_robot_state.vx = des_states.at(0)->as<StateType>()->nominal_state_.at(3);
    dynamicSrv.request.init_robot_state.vy = des_states.at(0)->as<StateType>()->nominal_state_.at(4);
    dynamicSrv.request.init_robot_state.vz = des_states.at(0)->as<StateType>()->nominal_state_.at(5);
    dynamicSrv.request.init_robot_state.qw = des_states.at(0)->as<StateType>()->nominal_state_.at(6);
    dynamicSrv.request.init_robot_state.qx = des_states.at(0)->as<StateType>()->nominal_state_.at(7);
    dynamicSrv.request.init_robot_state.qy = des_states.at(0)->as<StateType>()->nominal_state_.at(8);
    dynamicSrv.request.init_robot_state.qz = des_states.at(0)->as<StateType>()->nominal_state_.at(9);
    dynamicSrv.request.init_robot_state.wx = des_states.at(0)->as<StateType>()->nominal_state_.at(10);
    dynamicSrv.request.init_robot_state.wy = des_states.at(0)->as<StateType>()->nominal_state_.at(11);
    dynamicSrv.request.init_robot_state.wz = des_states.at(0)->as<StateType>()->nominal_state_.at(12);

    /* Initialize uncertain model parameters and controler gains */
    dynamicSrv.request.model_params = getRobot()->getUncertainParams();
    
    std::vector<std::vector<double>> robot_gains = getRobot()->getGains();
    for(int i = 0; i<robot_gains.size(); i++)
    {
        services_msgs::GainsValues gains_values;
        gains_values.values = robot_gains.at(i);
        dynamicSrv.request.gains.push_back(gains_values); 
    }

    dynamicSrv.request.time_vec.push_back(des_states.at(0)->as<StateType>()->st_time_);
    /* Fill the desired trajectory. The first one has already been used so we ski it. */
    for(int i = 1; i<des_states.size(); i++)
    {
        services_msgs::DesiredState des_state;
        std::vector<double> q = des_states.at(i)->as<StateType>()->getQValues();
        std::vector<double> qdot = des_states.at(i)->as<StateType>()->getQdotValues();
        std::vector<double> qddot = des_states.at(i)->as<StateType>()->getQddotValues();

        des_state.x = q.at(0);
        des_state.y = q.at(1);
        des_state.z = q.at(2);
        des_state.yaw = q.at(3);
        des_state.vx = qdot.at(0);
        des_state.vy = qdot.at(1);
        des_state.vz = qdot.at(2);
        des_state.wyaw = qdot.at(3);
        des_state.ax = qddot.at(0);
        des_state.ay = qddot.at(1);
        des_state.az = qddot.at(2);

        dynamicSrv.request.desired_states.push_back(des_state);
        dynamicSrv.request.time_vec.push_back(des_states.at(i)->as<StateType>()->st_time_);
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
        std::vector<double> trajZ = dynamicSrv.response.trajZ;
        std::vector<double> trajQw = dynamicSrv.response.trajQw;
        std::vector<double> trajQx = dynamicSrv.response.trajQx;
        std::vector<double> trajQy = dynamicSrv.response.trajQy;
        std::vector<double> trajQz = dynamicSrv.response.trajQz;

        for(int i = 0; i<trajX.size(); i++)
        {
            std::vector<double> st;
            st.push_back(trajX.at(i));
            st.push_back(trajY.at(i));
            st.push_back(trajZ.at(i));
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
        des_states.back()->as<StateType>()->nominal_state_.push_back(trajZ.back());
        des_states.back()->as<StateType>()->nominal_state_.push_back(dynamicSrv.response.trajVX.back());
        des_states.back()->as<StateType>()->nominal_state_.push_back(dynamicSrv.response.trajVY.back());
        des_states.back()->as<StateType>()->nominal_state_.push_back(dynamicSrv.response.trajVZ.back());
        des_states.back()->as<StateType>()->nominal_state_.push_back(trajQw.back());
        des_states.back()->as<StateType>()->nominal_state_.push_back(trajQx.back());
        des_states.back()->as<StateType>()->nominal_state_.push_back(trajQy.back());
        des_states.back()->as<StateType>()->nominal_state_.push_back(trajQz.back());
        des_states.back()->as<StateType>()->nominal_state_.push_back(dynamicSrv.response.trajWx.back());
        des_states.back()->as<StateType>()->nominal_state_.push_back(dynamicSrv.response.trajWy.back());
        des_states.back()->as<StateType>()->nominal_state_.push_back(dynamicSrv.response.trajWz.back());

        // To compare the service call time with the ODE resolution time
        double srv_time_percentage = ((srv_duration.count()-dynamicSrv.response.executionTime)/srv_duration.count())*100;
        // std::cout << "Loss of service call time in percent : " << srv_time_percentage << std::endl;
        return true;
    }
}

bool ompl::base::KinoQuadrotorSpace::simulateStatesAndTubes(const std::vector<ompl::base::State*> &des_states, double dt, std::vector<std::vector<double>> &collision_states,
                                        std::vector<std::vector<double>> &control_inputs, std::vector<std::vector<double>> &radii) const
{
    /* S03 state for CC: x, y, z, qw, qx, qy, qz */
    /* Quadrotor final nominal state: x, y, z, vx, vy, vz, qw, qx, qy, qz, wx, wy, wz */

    services_msgs::SensitivitySrv sensitivitySrv; // To comunicate with the sensitivity service.

    /* Initialize integrator */
    sensitivitySrv.request.integrator = getRobot()->getIntegratorType();

    /* Message initialization */
    sensitivitySrv.request.t_init = des_states.at(0)->as<StateType>()->st_time_; // Always start at 0 for the local path (even if the initial conditions are present).
    sensitivitySrv.request.t_final = des_states.back()->as<StateType>()->st_time_; // Desired path length, we approximate the last state at a time step of dt.
    sensitivitySrv.request.dt = dt; // Integration time step.

    /* Initialize the robot state (i.e. q0) */
    sensitivitySrv.request.init_robot_state.x = des_states.at(0)->as<StateType>()->nominal_state_.at(0);
    sensitivitySrv.request.init_robot_state.y = des_states.at(0)->as<StateType>()->nominal_state_.at(1);
    sensitivitySrv.request.init_robot_state.z = des_states.at(0)->as<StateType>()->nominal_state_.at(2);
    sensitivitySrv.request.init_robot_state.vx = des_states.at(0)->as<StateType>()->nominal_state_.at(3);
    sensitivitySrv.request.init_robot_state.vy = des_states.at(0)->as<StateType>()->nominal_state_.at(4);
    sensitivitySrv.request.init_robot_state.vz = des_states.at(0)->as<StateType>()->nominal_state_.at(5);
    sensitivitySrv.request.init_robot_state.qw = des_states.at(0)->as<StateType>()->nominal_state_.at(6);
    sensitivitySrv.request.init_robot_state.qx = des_states.at(0)->as<StateType>()->nominal_state_.at(7);
    sensitivitySrv.request.init_robot_state.qy = des_states.at(0)->as<StateType>()->nominal_state_.at(8);
    sensitivitySrv.request.init_robot_state.qz = des_states.at(0)->as<StateType>()->nominal_state_.at(9);
    sensitivitySrv.request.init_robot_state.wx = des_states.at(0)->as<StateType>()->nominal_state_.at(10);
    sensitivitySrv.request.init_robot_state.wy = des_states.at(0)->as<StateType>()->nominal_state_.at(11);
    sensitivitySrv.request.init_robot_state.wz = des_states.at(0)->as<StateType>()->nominal_state_.at(12);
    
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

    sensitivitySrv.request.time_vec.push_back(des_states.at(0)->as<StateType>()->st_time_);
    /* Fill the desired trajectory. The first one has already been used so we ski it. */
    for(int j = 1; j<des_states.size();j++)
    {
        services_msgs::DesiredState des_state;
        std::vector<double> pos = des_states.at(j)->as<StateType>()->getQValues();
        std::vector<double> vel = des_states.at(j)->as<StateType>()->getQdotValues();
        std::vector<double> acc = des_states.at(j)->as<StateType>()->getQddotValues();

        des_state.x = pos.at(0);
        des_state.y = pos.at(1);
        des_state.z = pos.at(2);
        des_state.yaw = pos.at(3);
        des_state.vx = vel.at(0);
        des_state.vy = vel.at(1);
        des_state.vz = vel.at(2);
        des_state.wyaw = vel.at(3);
        des_state.ax = acc.at(0);
        des_state.ay = acc.at(1);
        des_state.az = acc.at(2);

        sensitivitySrv.request.desired_states.push_back(des_state);
        sensitivitySrv.request.time_vec.push_back(des_states.at(j)->as<StateType>()->st_time_);
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
        std::vector<double> trajZ = sensitivitySrv.response.trajZ;
        std::vector<double> trajQw = sensitivitySrv.response.trajQw;
        std::vector<double> trajQx = sensitivitySrv.response.trajQx;
        std::vector<double> trajQy = sensitivitySrv.response.trajQy;
        std::vector<double> trajQz = sensitivitySrv.response.trajQz;

        for(int i = 0; i<trajX.size(); i++)
        {
            std::vector<double> st;
            st.push_back(trajX.at(i));
            st.push_back(trajY.at(i));
            st.push_back(trajZ.at(i));
            st.push_back(trajQw.at(i));
            st.push_back(trajQx.at(i));
            st.push_back(trajQy.at(i));
            st.push_back(trajQz.at(i));
            collision_states.push_back(st);
        }

        // Get the simulated control inputs
        control_inputs.push_back(sensitivitySrv.response.u1);
        control_inputs.push_back(sensitivitySrv.response.u2);
        control_inputs.push_back(sensitivitySrv.response.u3);
        control_inputs.push_back(sensitivitySrv.response.u4);

        // Get the uncertainty tubes
        radii.push_back(sensitivitySrv.response.ellipsoid_alongX);
        radii.push_back(sensitivitySrv.response.ellipsoid_alongY);
        radii.push_back(sensitivitySrv.response.ellipsoid_alongZ);
        radii.push_back(sensitivitySrv.response.ellipsoid_u1);
        radii.push_back(sensitivitySrv.response.ellipsoid_u2);
        radii.push_back(sensitivitySrv.response.ellipsoid_u3);
        radii.push_back(sensitivitySrv.response.ellipsoid_u4);

        // The last element is the pnorm of the radii of interest
        radii.push_back(sensitivitySrv.response.radii_lambda);

        // Make sure there is is not already a registered state
        des_states.back()->as<StateType>()->nominal_state_.clear();
        des_states.back()->as<StateType>()->PI_.clear();
        des_states.back()->as<StateType>()->PI_xi_.clear();
        // Register the new nominal state
        des_states.back()->as<StateType>()->nominal_state_.push_back(trajX.back());
        des_states.back()->as<StateType>()->nominal_state_.push_back(trajY.back());
        des_states.back()->as<StateType>()->nominal_state_.push_back(trajZ.back());
        des_states.back()->as<StateType>()->nominal_state_.push_back(sensitivitySrv.response.trajVX.back());
        des_states.back()->as<StateType>()->nominal_state_.push_back(sensitivitySrv.response.trajVY.back());
        des_states.back()->as<StateType>()->nominal_state_.push_back(sensitivitySrv.response.trajVZ.back());
        des_states.back()->as<StateType>()->nominal_state_.push_back(trajQw.back());
        des_states.back()->as<StateType>()->nominal_state_.push_back(trajQx.back());
        des_states.back()->as<StateType>()->nominal_state_.push_back(trajQy.back());
        des_states.back()->as<StateType>()->nominal_state_.push_back(trajQz.back());
        des_states.back()->as<StateType>()->nominal_state_.push_back(sensitivitySrv.response.trajWx.back());
        des_states.back()->as<StateType>()->nominal_state_.push_back(sensitivitySrv.response.trajWy.back());
        des_states.back()->as<StateType>()->nominal_state_.push_back(sensitivitySrv.response.trajWz.back());
        // Register new sensitivity conditions
        des_states.back()->as<StateType>()->PI_ = sensitivitySrv.response.final_PI;
        des_states.back()->as<StateType>()->PI_xi_ = sensitivitySrv.response.final_PI_xi;

        // To compare the service call time with the ODE resolution time
        double srv_time_percentage = ((srv_duration.count()-sensitivitySrv.response.executionTime)/srv_duration.count())*100;
        // std::cout << "Loss of service call time in percent : " << srv_time_percentage << std::endl;
        return true;
    }
}

bool ompl::base::KinoQuadrotorSpace::localOpt(const std::vector<ompl::base::State*> &section_to_optimize, double dt, std::string stopping_condition_, double tolerance_, double max_time, int window_size, int nb_iter, std::string cost_id, std::string opti_params, 
                                                std::vector<ompl::base::State*> &traj_opt, double &cost) const
{
    services_msgs::LocaloptSrv localoptSrv; // To comunicate with the sensitivity service.
    
    /* Initialize integrator */
    localoptSrv.request.integrator = getRobot()->getIntegratorType();

    /* Message initialization */
    localoptSrv.request.t_init = section_to_optimize.at(0)->as<StateType>()->st_time_; // Always start at 0 for the local path (even if the initial conditions are present).
    localoptSrv.request.t_final = section_to_optimize.back()->as<StateType>()->st_time_; // Desired path length, we approximate the last state at a time step of dt.
    localoptSrv.request.dt = dt; // Integration time step.

    /* Initialize the robot state (i.e. q0) */
    localoptSrv.request.init_robot_state.x = section_to_optimize.at(0)->as<StateType>()->nominal_state_.at(0);
    localoptSrv.request.init_robot_state.y = section_to_optimize.at(0)->as<StateType>()->nominal_state_.at(1);
    localoptSrv.request.init_robot_state.z = section_to_optimize.at(0)->as<StateType>()->nominal_state_.at(2);
    localoptSrv.request.init_robot_state.vx = section_to_optimize.at(0)->as<StateType>()->nominal_state_.at(3);
    localoptSrv.request.init_robot_state.vy = section_to_optimize.at(0)->as<StateType>()->nominal_state_.at(4);
    localoptSrv.request.init_robot_state.vz = section_to_optimize.at(0)->as<StateType>()->nominal_state_.at(5);
    localoptSrv.request.init_robot_state.qw = section_to_optimize.at(0)->as<StateType>()->nominal_state_.at(6);
    localoptSrv.request.init_robot_state.qx = section_to_optimize.at(0)->as<StateType>()->nominal_state_.at(7);
    localoptSrv.request.init_robot_state.qy = section_to_optimize.at(0)->as<StateType>()->nominal_state_.at(8);
    localoptSrv.request.init_robot_state.qz = section_to_optimize.at(0)->as<StateType>()->nominal_state_.at(9);
    localoptSrv.request.init_robot_state.wx = section_to_optimize.at(0)->as<StateType>()->nominal_state_.at(10);
    localoptSrv.request.init_robot_state.wy = section_to_optimize.at(0)->as<StateType>()->nominal_state_.at(11);
    localoptSrv.request.init_robot_state.wz = section_to_optimize.at(0)->as<StateType>()->nominal_state_.at(12);

    /* Initialize the sensitivity conditions (i.e. PI0, PI_xi0) */
    localoptSrv.request.initial_PI = section_to_optimize.at(0)->as<StateType>()->PI_;
    localoptSrv.request.initial_PI_xi = section_to_optimize.at(0)->as<StateType>()->PI_xi_;

    /* Initialize uncertain model parameters and controler gains */
    localoptSrv.request.model_params = getRobot()->getUncertainParams();

    std::vector<std::vector<double>> robot_gains = getRobot()->getGains();
    for(int i = 0; i<robot_gains.size(); i++)
    {
        services_msgs::GainsValues gains_values;
        gains_values.values = robot_gains.at(i);
        localoptSrv.request.gains.push_back(gains_values); 
    }

    localoptSrv.request.time_vec.push_back(section_to_optimize.at(0)->as<StateType>()->st_time_);
    /* Fill the desired trajectory.*/
    for(int j = 0; j<section_to_optimize.size();j++)
    {
        services_msgs::DesiredState des_state;
        std::vector<double> pos = section_to_optimize.at(j)->as<StateType>()->getQValues();
        std::vector<double> vel = section_to_optimize.at(j)->as<StateType>()->getQdotValues();
        std::vector<double> acc = section_to_optimize.at(j)->as<StateType>()->getQddotValues();

        des_state.x = pos.at(0);
        des_state.y = pos.at(1);
        des_state.z = pos.at(2);
        des_state.yaw = pos.at(3);
        des_state.vx = vel.at(0);
        des_state.vy = vel.at(1);
        des_state.vz = vel.at(2);
        des_state.wyaw = vel.at(3);
        des_state.ax = acc.at(0);
        des_state.ay = acc.at(1);
        des_state.az = acc.at(2);

        localoptSrv.request.desired_states.push_back(des_state);
        localoptSrv.request.time_vec.push_back(section_to_optimize.at(j)->as<StateType>()->st_time_);
    }

    /* Initialize the optimization hyperparameters */
    localoptSrv.request.stopping_condition = stopping_condition_;
    localoptSrv.request.tolerance = tolerance_;
    localoptSrv.request.max_time = max_time;
    localoptSrv.request.window_size = window_size;
    localoptSrv.request.nb_iter = nb_iter;
    localoptSrv.request.cost_id = cost_id;
    localoptSrv.request.opti_params = opti_params;

    // Optimize the trajectory
    if(!getRobot()->localoptSrv_.call(localoptSrv))
    {
        ROS_WARN("Unable to solve tracking.");
        return false;
    }
    else
    {
        // Get the optimized states
        std::vector<services_msgs::DesiredState> traj_localopt = localoptSrv.response.desired_states_opt;

        std::vector<double> q, qdot, qddot;
        for(auto &st_opt: traj_localopt)
        {
            ompl::base::State *state = allocState();
            q.clear();
            qdot.clear();
            qddot.clear();

            // Position
            q.push_back(st_opt.x);
            q.push_back(st_opt.y);
            q.push_back(st_opt.z);
            q.push_back(st_opt.yaw);
            // Velocity
            qdot.push_back(st_opt.vx);
            qdot.push_back(st_opt.vy);
            qdot.push_back(st_opt.vz);
            qdot.push_back(st_opt.wyaw);
            // Acceleration
            qddot.push_back(st_opt.ax);
            qddot.push_back(st_opt.ay);
            qddot.push_back(st_opt.az);
            qddot.push_back(0.0);

            state->as<StateType>()->setQValues(q);
            state->as<StateType>()->setQdotValues(qdot);
            state->as<StateType>()->setQddotValues(qddot);
            traj_opt.push_back(state);
        }
        // Make sure there is is not already a registered state
        traj_opt.front()->as<StateType>()->nominal_state_.clear();
        traj_opt.front()->as<StateType>()->PI_.clear();
        traj_opt.front()->as<StateType>()->PI_xi_.clear();
        // Register the new nominal state
        traj_opt.front()->as<StateType>()->nominal_state_.push_back(localoptSrv.request.init_robot_state.x);
        traj_opt.front()->as<StateType>()->nominal_state_.push_back(localoptSrv.request.init_robot_state.y);
        traj_opt.front()->as<StateType>()->nominal_state_.push_back(localoptSrv.request.init_robot_state.z);
        traj_opt.front()->as<StateType>()->nominal_state_.push_back(localoptSrv.request.init_robot_state.vx);
        traj_opt.front()->as<StateType>()->nominal_state_.push_back(localoptSrv.request.init_robot_state.vy);
        traj_opt.front()->as<StateType>()->nominal_state_.push_back(localoptSrv.request.init_robot_state.vz);
        traj_opt.front()->as<StateType>()->nominal_state_.push_back(localoptSrv.request.init_robot_state.qw);
        traj_opt.front()->as<StateType>()->nominal_state_.push_back(localoptSrv.request.init_robot_state.qx);
        traj_opt.front()->as<StateType>()->nominal_state_.push_back(localoptSrv.request.init_robot_state.qy);
        traj_opt.front()->as<StateType>()->nominal_state_.push_back(localoptSrv.request.init_robot_state.qz);
        traj_opt.front()->as<StateType>()->nominal_state_.push_back(localoptSrv.request.init_robot_state.wx);
        traj_opt.front()->as<StateType>()->nominal_state_.push_back(localoptSrv.request.init_robot_state.wy);
        traj_opt.front()->as<StateType>()->nominal_state_.push_back(localoptSrv.request.init_robot_state.wz);
        // Register new sensitivity conditions
        traj_opt.front()->as<StateType>()->PI_ = localoptSrv.request.initial_PI;
        traj_opt.front()->as<StateType>()->PI_xi_ = localoptSrv.request.initial_PI_xi;

        // Make sure there is is not already a registered state
        traj_opt.back()->as<StateType>()->nominal_state_.clear();
        traj_opt.back()->as<StateType>()->PI_.clear();
        traj_opt.back()->as<StateType>()->PI_xi_.clear();
        // Register the new nominal state
        traj_opt.back()->as<StateType>()->nominal_state_.push_back(localoptSrv.response.trajX.back());
        traj_opt.back()->as<StateType>()->nominal_state_.push_back(localoptSrv.response.trajY.back());
        traj_opt.back()->as<StateType>()->nominal_state_.push_back(localoptSrv.response.trajZ.back());
        traj_opt.back()->as<StateType>()->nominal_state_.push_back(localoptSrv.response.trajVX.back());
        traj_opt.back()->as<StateType>()->nominal_state_.push_back(localoptSrv.response.trajVY.back());
        traj_opt.back()->as<StateType>()->nominal_state_.push_back(localoptSrv.response.trajVZ.back());
        traj_opt.back()->as<StateType>()->nominal_state_.push_back(localoptSrv.response.trajQw.back());
        traj_opt.back()->as<StateType>()->nominal_state_.push_back(localoptSrv.response.trajQx.back());
        traj_opt.back()->as<StateType>()->nominal_state_.push_back(localoptSrv.response.trajQy.back());
        traj_opt.back()->as<StateType>()->nominal_state_.push_back(localoptSrv.response.trajQz.back());
        traj_opt.back()->as<StateType>()->nominal_state_.push_back(localoptSrv.response.trajWx.back());
        traj_opt.back()->as<StateType>()->nominal_state_.push_back(localoptSrv.response.trajWy.back());
        traj_opt.back()->as<StateType>()->nominal_state_.push_back(localoptSrv.response.trajWz.back());
        // Register new sensitivity conditions
        traj_opt.back()->as<StateType>()->PI_ = localoptSrv.response.final_PI;
        traj_opt.back()->as<StateType>()->PI_xi_ = localoptSrv.response.final_PI_xi;

        cost = localoptSrv.response.cost;

        return true;
    }
}

bool ompl::base::KinoQuadrotorSpace::mpc(const std::vector<ompl::base::State*> &section_to_optimize, double dt, int nb_iter, int horizon, std::string cost_id, std::string opti_params, 
                                                std::vector<ompl::base::State*> &traj_opt, double &cost) const
{
    ROS_ERROR("Quadrotor mpc NOT IMPLEMENTED !");
    return false;
}

void ompl::base::KinoQuadrotorSpace::getPlatformFromEEF(std::vector<double>& q, std::vector<double>& acc) const
{
    double perch_length_ = getRobot()->as<ompl::Quadrotor>()->getPerchLenght();
    double angle_perche_ = getRobot()->as<ompl::Quadrotor>()->getPerchAngle();
    double perche_altitude_ = getRobot()->as<ompl::Quadrotor>()->getPerchAlt();

    // Define perch position (x, y, z)
    Eigen::Vector3d platform_position(q[0], q[1], q[2]); 

    // define the thrust vector 
    double tx = getRobot()->getMass()*acc[0];
    double ty = getRobot()->getMass()*acc[1];
    double tz = getRobot()->getMass()*acc[2]+getRobot()->getMass()*9.81;

    // Compute the rotation of the thrust given the desired yaw
    double newTx = tx * cos(q[3]) - ty * sin(q[3]);
    double newTy = tx * sin(q[3]) + ty * cos(q[3]);
    double newTz = tz;

    // calculate the roll and pitch angles
    double roll = atan2(newTy, newTz);
    double pitch = - atan2(-newTx, sqrt(newTy*newTy + newTz*newTz));
    double yaw = q[3];   

    // Create a transformation matrix for the platform's orientation
    Eigen::AngleAxisd roll_rotation(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch_rotation(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw_rotation(yaw, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond orientation = yaw_rotation * pitch_rotation * roll_rotation;

    // Define a translation vector for the platform's position
    Eigen::Vector3d translation(platform_position);

    // Include the perch's length in the translation
    Eigen::Vector3d perch_length_vector(perch_length_*cos(angle_perche_), perch_length_*sin(angle_perche_), perche_altitude_);
    translation -= orientation * perch_length_vector;

    // Define the perch frame (pose)
    Eigen::Isometry3d perch_frame = Eigen::Isometry3d::Identity();
    perch_frame.linear() = orientation.toRotationMatrix();
    perch_frame.translation() = translation;

    // Now, you have the perch frame. To get the quadrotor's position and orientation, you can transform it.

    // Define a transformation from the perch to the quadrotor (for example, if the quadrotor is mounted on the perch)
    Eigen::Isometry3d perch_to_quadrotor = Eigen::Isometry3d::Identity();

    // Apply the transformation to compute the quadrotor frame (pose)
    Eigen::Isometry3d quadrotor_frame = perch_frame * perch_to_quadrotor;

    // Extract the quadrotor position and orientation
    Eigen::Vector3d quadrotor_position = quadrotor_frame.translation();
    Eigen::Quaterniond quadrotor_orientation(quadrotor_frame.linear());

    q[0] = quadrotor_position[0]; // X
    q[1] = quadrotor_position[1]; // Y
    q[2] = quadrotor_position[2]; // Z

    auto euler = quadrotor_orientation.toRotationMatrix().eulerAngles(0, 1, 2);

    //Quaternion
    q[3] = euler[2];
}