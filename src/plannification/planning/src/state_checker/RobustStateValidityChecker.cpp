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

#include "ros/ros.h"
#include "planning/state_checker/RobustStateValidityChecker.h"

bool ompl::base::RobustStateValidityChecker::isValid(const ompl::base::State *state) const
{
    std::vector<double> nom_state = state->as<ompl::base::RobustStateSpace::StateType>()->nominal_state_;

    if(!nom_state.size() > 0)
    {
        // Fake tubes for CC
        std::vector<double> fake_tube;

        // Make a copy of the state
        ompl::base::State* st_cpy = stateSpace_->allocState();
        stateSpace_->copyState(st_cpy, state);

        // Vector to recover the projection
        std::vector<std::vector<double>> sF; 

        // Initialize the vector of states to be projected
        std::vector<ompl::base::State*> st_check{st_cpy};
        // Get the state projection
        stateSpace_->projectStates(st_check, sF); 

        nom_state = sF.at(0);

        // Free the copy
        stateSpace_->freeState(st_cpy);
    }
    
    std::vector<double> collision_state;
    if(robot_->getName()=="quadrotor")
    {
        collision_state.push_back(nom_state[0]); // x
        collision_state.push_back(nom_state[1]); // y
        collision_state.push_back(nom_state[2]); // z

        collision_state.push_back(nom_state[6]); // qw
        collision_state.push_back(nom_state[7]); // qx
        collision_state.push_back(nom_state[8]); // qy
        collision_state.push_back(nom_state[9]); // qz
    }
    else if(robot_->getName()=="unicycle")
    {
        collision_state.push_back(nom_state[0]); // x
        collision_state.push_back(nom_state[1]); // y
        collision_state.push_back(0); // z

        collision_state.push_back(nom_state[6]); // qw
        collision_state.push_back(nom_state[7]); // qx
        collision_state.push_back(nom_state[8]); // qy
        collision_state.push_back(nom_state[9]); // qz
    }
    else
    {
        return false;
    }

    // First check the workspace bounds
    if(!checkWorkspace(collision_state))
        return false;

    blt_client_->resetBasePositionAndOrientation(robot_id_blt_,{collision_state[0], collision_state[1], collision_state[2]}, {collision_state[4], collision_state[5], collision_state[6], collision_state[3]});
    blt_client_->performCollisionDetection();

    auto cc_blt = blt_client_->getContactPoints(robot_id_blt_, env_id_blt_);
    
    // No tubes are used; this is a “traditional” collision check.
    if(cc_blt.m_numContactPoints > 0)
    {
        if(cc_blt.m_contactPointData->m_contactDistance < 0)
            return false;
    }
    else
    {
        return true;
    }
}

bool ompl::base::RobustStateValidityChecker::isInputsValid(std::vector<double> &control_inputs, std::vector<double> &uncertainties) const
{
    bool isValid = true;

    for(int i = 0; i<control_inputs.size(); i++)
    {
        if(control_inputs.at(i) + uncertainties.at(i) > robot_->getMaxInputs().at(i))
        {
            isValid = false;
            break;
        }
    }
    
    return isValid;
}

bool ompl::base::RobustStateValidityChecker::isStateValid(const std::vector<double> &state, const std::vector<double> &uncertainty) const
{
    // First check the workspace bounds
    if(!checkWorkspace(state))
        return false;

    // Send a request to check collision with the nominal robot state and the envronment
    blt_client_->resetBasePositionAndOrientation(robot_id_blt_,{state[0], state[1], state[2]}, {state[4], state[5], state[6], state[3]});
    blt_client_->performCollisionDetection();

    auto cc_blt = blt_client_->getContactPoints(robot_id_blt_, env_id_blt_);

    // If the nominal state is in collision we don't need to check the tube
    if(cc_blt.m_numContactPoints > 0) 
    {
        // Bullet uses a margin of 0.04m, we want the exact collision
        if(cc_blt.m_contactPointData->m_contactDistance < 0)
            return false;
    }
    else
    {
        // No tubes are used; this is a "standard" collision check.
        if(uncertainty.empty())
        {
            return true;
        }
        // If we use the tube, as a practical implementation we approximate its hull by sampling positions to cover the surface of an ellipsoid or a rectangle composed of the maximum deviation along each component
        else
        {
            switch (tube_mode_)
            {
                case 0:
                    return sphericGrowth(cc_blt, uncertainty);
                case 1:
                    return uncertainAABB(state, uncertainty);
                case 2:
                    return approximateRectangle(state, uncertainty);
                default:
                    ROS_WARN("TUBE_MODE not IMPLEMENTED !");
                    return false;
            }
        }
    } 
}

bool ompl::base::RobustStateValidityChecker::sphericGrowth(const b3ContactInformation &cc_blt_ellipsoid, const std::vector<double> &uncertainty) const
{
    if(cc_blt_ellipsoid.m_numContactPoints > 0)
    {
        if(cc_blt_ellipsoid.m_contactPointData->m_contactDistance < *max_element(uncertainty.begin(), uncertainty.end()))
            return false;
    }
    return true;
}

bool ompl::base::RobustStateValidityChecker::approximateEllipsoid(const std::vector<double> &state, const std::vector<double> &uncertainty) const
{
    // std::cout << "Max Rq : " << *max_element(uncertainty.begin(), uncertainty.end()) << std::endl;
    std::vector<std::vector<double>> angles; //Latitude and longitude angle to project on the ellipsoid 

    // We first check for all the maximum values
    if(uncertainty.size() == 2) // In 2D 
    {
        angles = {{0,M_PI/2},{M_PI/2,M_PI/2},{M_PI,M_PI/2},{-M_PI/2,M_PI/2}};
    }
    if(uncertainty.size() == 3)  // In 3D 
    {
        angles = {{0,M_PI/2},{M_PI/2,M_PI/2},{M_PI,M_PI/2},{-M_PI/2,M_PI/2},{0, 0},{0, M_PI}};
    }
    for(int i = 0; i<angles.size(); i++)
    {
        std::vector<double> state_ellipsoid;

        getSurfacePoint(state_ellipsoid, angles.at(i), {state[0], state[1], state[2]}, uncertainty);

        // The orientation is the same as the nominal state
        blt_client_->resetBasePositionAndOrientation(robot_id_blt_,{state_ellipsoid[0], state_ellipsoid[1], state_ellipsoid[2]}, {state[4], state[5], state[6], state[3]});
        blt_client_->performCollisionDetection();

        auto cc_blt_ellipsoid = blt_client_->getContactPoints(robot_id_blt_, env_id_blt_);
        if(cc_blt_ellipsoid.m_numContactPoints > 0)
        {
            if(cc_blt_ellipsoid.m_contactPointData->m_contactDistance < 0)
                return false;
        } 
    }
    
    // Then we fill some gaps on the hull of the ellipsoid to refine the approximation (here we consider each center of the eight of the ellipsoid)
    angles.clear();
    if(uncertainty.size() == 2) // In 2D 
    {
        angles = {{M_PI/4,M_PI/2},{3*M_PI/4,M_PI/2},{5*M_PI/4,M_PI/2},{7*M_PI/4,M_PI/2}};
    }
    if(uncertainty.size() == 3)  // In 3D we need the longitute and latitude angles
    {
        angles = {{M_PI/4,M_PI/4},{3*M_PI/4,M_PI/4},{5*M_PI/4,M_PI/4},{7*M_PI/4,M_PI/4},
                    {M_PI/4,3*M_PI/4},{3*M_PI/4,3*M_PI/4},{5*M_PI/4,3*M_PI/4},{7*M_PI/4,3*M_PI/4}};
    }
    for(int i = 0; i<angles.size(); i++)
    {
        std::vector<double> state_ellipsoid;

        getSurfacePoint(state_ellipsoid, angles.at(i), {state[0], state[1], state[2]}, uncertainty);

        blt_client_->resetBasePositionAndOrientation(robot_id_blt_,{state_ellipsoid[0], state_ellipsoid[1], state_ellipsoid[2]}, {state[4], state[5], state[6], state[3]});
        blt_client_->performCollisionDetection();

        auto cc_blt_ellipsoid = blt_client_->getContactPoints(robot_id_blt_, env_id_blt_);
        if(cc_blt_ellipsoid.m_numContactPoints > 0)
        {
            if(cc_blt_ellipsoid.m_contactPointData->m_contactDistance < 0)
                return false;
        }
    }

    // No collisions are found with the tube
    return true;  
}

bool ompl::base::RobustStateValidityChecker::approximateRectangle(const std::vector<double> &state, const std::vector<double> &uncertainty) const
{
    std::vector<std::vector<double>> cubes; //Latitude and longitude angle to project on the ellipsoid 

    // We check for all the maximum values (i.e. cube angles and middle of the edges)
    if(uncertainty.size() == 2) // In 2D 
    {
        cubes = {{uncertainty[0],0.0,0.0},{0.0,uncertainty[1],0.0},{-uncertainty[0],0.0,0.0},{0.0,-uncertainty[1],0.0},
                {uncertainty[0],uncertainty[1],0.0},{uncertainty[0],-uncertainty[1],0.0},{-uncertainty[0],uncertainty[1],0.0},{-uncertainty[0],-uncertainty[1],0.0}};
    }
    if(uncertainty.size() == 3)  // In 3D 
    {
        cubes = {{uncertainty[0],0.0,uncertainty[2]},{0.0,uncertainty[1],uncertainty[2]},{-uncertainty[0],0.0,uncertainty[2]},{0.0,-uncertainty[1],uncertainty[2]},
                {uncertainty[0],uncertainty[1],uncertainty[2]},{uncertainty[0],-uncertainty[1],uncertainty[2]},{-uncertainty[0],uncertainty[1],uncertainty[2]},{-uncertainty[0],-uncertainty[1],uncertainty[2]},
                {uncertainty[0],0.0,-uncertainty[2]},{0.0,uncertainty[1],-uncertainty[2]},{-uncertainty[0],0.0,-uncertainty[2]},{0.0,-uncertainty[1],-uncertainty[2]},
                {uncertainty[0],uncertainty[1],-uncertainty[2]},{uncertainty[0],-uncertainty[1],-uncertainty[2]},{-uncertainty[0],uncertainty[1],-uncertainty[2]},{-uncertainty[0],-uncertainty[1],-uncertainty[2]}};
    }
    for(int i = 0; i<cubes.size(); i++)
    {
        std::vector<double> state_cube_hull;

        state_cube_hull.push_back(state[0] + cubes[i][0]);
        state_cube_hull.push_back(state[1] + cubes[i][1]);
        state_cube_hull.push_back(state[2] + cubes[i][2]);

        // The orientation is the same as the nominal state
        blt_client_->resetBasePositionAndOrientation(robot_id_blt_,{state_cube_hull[0], state_cube_hull[1], state_cube_hull[2]}, {state[4], state[5], state[6], state[3]});
        blt_client_->performCollisionDetection();

        auto cc_blt_ellipsoid = blt_client_->getContactPoints(robot_id_blt_, env_id_blt_);
        if(cc_blt_ellipsoid.m_numContactPoints > 0)
        {
            if(cc_blt_ellipsoid.m_contactPointData->m_contactDistance < 0)
                return false;
        } 
    }

    // No collisions are found with the tube
    return true;  
}

bool ompl::base::RobustStateValidityChecker::uncertainAABB(const std::vector<double> &state, const std::vector<double> &uncertainty) const
{
    // Get the current robot AABB
    blt_client_->resetBasePositionAndOrientation(robot_id_blt_, {state[0], state[1], state[2]}, {state[4], state[5], state[6], state[3]});
    auto bounding_box = blt_client_->getAABB(robot_id_blt_, 0);

    // Compute half extents with uncertainty AABB
    std::array<double, 3> halfExtents;
    for (int i = 0; i < 3; ++i) 
    {
        halfExtents[i] = (bounding_box.max[i] - bounding_box.min[i]) * 0.5;
    }
    std::array<double, 3> extendedHalfExtents = halfExtents;
    for (int i = 0; i < uncertainty.size() ; ++i) 
    {
        extendedHalfExtents[i] += uncertainty[i];
    }

    // Create temporary collision shape with extended AABB
    int temp_box_id = blt_client_->createCollisionShapeBox(extendedHalfExtents, 0);

    // Set position and orientation for collision box
    blt_client_->resetBasePositionAndOrientation(temp_box_id, {state[0], state[1], state[2]}, {state[4], state[5], state[6], state[3]});

    // Perform collision detection with environment
    blt_client_->performCollisionDetection();

    auto cc_blt_ellipsoid = blt_client_->getContactPoints(temp_box_id, env_id_blt_);
    bool hasCollision = (cc_blt_ellipsoid.m_numContactPoints > 0 &&
                         cc_blt_ellipsoid.m_contactPointData->m_contactDistance < 0);

    // Delete the temporary box shape
    blt_client_->removeCollisionShape(temp_box_id);

    return !hasCollision;  // Return true if no collision, false otherwise
}

bool ompl::base::RobustStateValidityChecker::checkWorkspace(const std::vector<double> &state) const
{
    // Check x workspace bound
    if(state.at(0) < qmin_.at(0) || state.at(0) > qmax_.at(0))
        return false;
    // Check y workspace bound
    if(state.at(1) < qmin_.at(1) || state.at(1) > qmax_.at(1))
        return false;
    // Check z workspace bound
    if(state.at(2) < qmin_.at(2) || state.at(2) > qmax_.at(2))
        return false;

    return true;
}