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

#ifndef OMPL_ROBOT_QUADROTOR_
#define OMPL_ROBOT_QUADROTOR_

// Robot
#include "robots/Robot.h"

namespace ompl
{
    OMPL_CLASS_FORWARD(Quadrotor);

    class Quadrotor : public Robot
    {
        /* Quadrotor nominal state: x, y, z, vx, vy, vz, qw, qx, qy, qz, wx, wy, wz */
        public:
            Quadrotor(unsigned int desired_dim, unsigned int nominal_dim, const ros::NodeHandle& node_handler): Robot(desired_dim, nominal_dim, node_handler)
            {
                const std::string key_params = "robot_parameters/";

                if(!nh_.getParam(key_params + "perch_length", perch_length_)){
                    ROS_WARN("Cannot load 'perch_length' param from ROS server: default perch_length_ = 0.5");
                    perch_length_ = 0.5;
                }
                if(!nh_.getParam(key_params + "angle_perche", angle_perche_)){
                    ROS_WARN("Cannot load 'angle_perche' param from ROS server: default eef_planning_ = 0.0");
                    angle_perche_ = 0.0;
                }
                if(!nh_.getParam(key_params + "perche_altitude", perche_altitude_)){
                    ROS_WARN("Cannot load 'perche_altitude' param from ROS server: default perche_altitude_ = 0.0");
                    perche_altitude_ = 0.0;
                }
                if(!nh_.getParam(key_params + "perch_planning_relative_vel_val", perch_planning_relative_vel_val_)){
                    ROS_WARN("Cannot load 'perch_planning_relative_vel_val' param from ROS server: default perch_planning_relative_vel_val_ = 0.0");
                    perch_planning_relative_vel_val_ = 0.0;
                }
            }
            ~Quadrotor();

            virtual std::vector<float> getRobotLearningInput(const std::vector<std::vector<double>>& des_state) const;

            double getPerchLenght() const
            {
                return perch_length_;
            }

            double getPerchAngle() const
            {
                return angle_perche_;
            }

            double getPerchAlt() const
            {
                return perche_altitude_;
            }

            double getPerchVel() const
            {
                return perch_planning_relative_vel_val_;
            }

        private:
            
            // Gravity
            double g_ = 9.81;

            /// \brief Parameters for the perch end effector planning in the quadrotor case
            double perch_length_, angle_perche_, perche_altitude_, perch_planning_relative_vel_val_;
    };
}
#endif
