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

#ifndef OMPL_ROBOT_UNICYCLE_
#define OMPL_ROBOT_UNICYCLE_

// Robot
#include "robots/Robot.h"

namespace ompl
{
    OMPL_CLASS_FORWARD(Unicycle);

    class Unicycle : public Robot
    {
        public:
            Unicycle(unsigned int desired_dim, unsigned int nominal_dim, const ros::NodeHandle& node_handler): Robot(desired_dim, nominal_dim, node_handler), nh_(node_handler)
            {
                // Ros params topic name
                const std::string key_params = "robot_parameters/";

                // Load the robot name
                if(!nh_.getParam(key_params + "v_norm", v_norm)){
                    ROS_WARN("Cannot load 'v_norm' param from ROS server: default v_norm = 1.0");
                    v_norm = 1.0;
                }
            }
            ~Unicycle();

            virtual std::vector<float> getRobotLearningInput(const std::vector<std::vector<double>>& des_state) const;

            /** \brief Set the velocity norm */
            void setV(double v)
            {
                v_norm = v;
            }

            /** \brief Get the velocity norm */
            double getV()
            {
                return v_norm;
            }

            // Node handler
            ros::NodeHandle nh_;

        private:

            /** \brief THe constant velocity norm for Dubins path velocities parametrization */
            double v_norm;
    };
}
#endif