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

// SYSTEM
#include "ros/ros.h"

// Context
#include "planning/context/PlanningContext.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "plan");
    ros::NodeHandle nh;

    // Wait for a sensitivity services
    ros::service::waitForService("compute_dynamic");
    ros::service::waitForService("compute_sensitivity");
    ros::service::waitForService("localopt");
    
    auto planning = std::make_shared<ompl_interface::PlanningContext>("plan");

    if(!planning->isInitialized())
    {   
        std::cout << "Not able to initialize the planning problem !" << std::endl;
        ros::shutdown();
        return 0;
    }

    ROS_WARN("Press ENTER to plan !");
    std::cin.get();
        
    if(planning->solve())
        std::cout << "Trajectory successfully planned, check the generated files !" << std::endl;
    else
        std::cout << "Not able to solve the planning problem !" << std::endl;

    ROS_WARN("Press ENTER to kill !");
    std::cin.get();
    ros::shutdown();
    return 0;
}
