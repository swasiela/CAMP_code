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

#ifndef OMPL_ROBOT_
#define OMPL_ROBOT_

// SYSTEM
#include "ros/ros.h"
#include <unistd.h>
#include <ios>
#include <iostream>
#include <fstream>
#include <string>
#include <filesystem>

// OMPL
#include "ompl/util/ClassForward.h"

// Learning
#include "learning_sensitivity/SensitivityNN.h"

// ODE services
#include "services_msgs/DynamicSrv.h"
#include "services_msgs/LocaloptSrv.h"
#include "services_msgs/SensitivitySrv.h"

// Utils
#include "utils/transform.h"

namespace ompl
{
    OMPL_CLASS_FORWARD(Robot);

    class Robot
    {
        public:
            Robot(unsigned int desired_dim, unsigned int nominal_dim, const ros::NodeHandle& node_handler) : desired_dim_(desired_dim), nominal_dim_(nominal_dim), nh_(node_handler), sensiNN_(node_handler)
            {
                // Get the current CAMP path 
                std::filesystem::path campDir;
                std::filesystem::path sourceFilePath = __FILE__; 
                std::filesystem::path currentDir = sourceFilePath.parent_path();
                // Traverse upwards until the root of the filesystem
                while (currentDir.has_parent_path()) {
                    if (currentDir.filename() == "CAMP") {  // Adjust this condition if "CAMP" is not the directory name
                        campDir = currentDir;
                        break;
                    }
                    currentDir = currentDir.parent_path();
                }
                
                if (campDir.empty()) {
                    ROS_ERROR("CAMP directory not found.");
                }

                // Ros params topic name
                const std::string key_params = "robot_parameters/";

                // Load the robot name
                if(!nh_.getParam(key_params + "robot_id", robot_id_)){
                    ROS_WARN("Cannot load 'robot_id' param from ROS server: default robot_id_ = NONE");
                    robot_id_ = "NONE";
                }

                // Get the robot model files paths
                if(!nh_.getParam(key_params + "urdf", urdf_)){
                    ROS_WARN("Cannot load 'urdf' param from ROS server: default urdf_ = NONE");
                    urdf_ = "NONE";
                }
                else
                {
                    // Combine the CAMP directory with the relative URDF path
                    std::filesystem::path fullPath = std::filesystem::absolute(campDir / urdf_);

                    // Normalize the path to remove redundant ".." and "."
                    fullPath = fullPath.lexically_normal();

                    // Convert to string and use the full path
                    std::string resolvedPath = fullPath.string();

                    ROS_INFO_STREAM("Resolved URDF path: " << resolvedPath);

                    urdf_ = resolvedPath;
                }

                // Get the robot scene file path
                if(!nh_.getParam(key_params + "scene", scene_)){
                    ROS_WARN("Cannot load 'scene' param from ROS server: default scene_ = NONE");
                    scene_ = "NONE";
                }
                else
                {
                    // Combine the CAMP directory with the relative URDF path
                    std::filesystem::path fullPath = std::filesystem::absolute(campDir / scene_);

                    // Normalize the path to remove redundant ".." and "."
                    fullPath = fullPath.lexically_normal();

                    // Convert to string and use the full path
                    std::string resolvedPath = fullPath.string();

                    ROS_INFO_STREAM("Resolved SCENE path: " << resolvedPath);

                    scene_ = resolvedPath;
                }

                // Get the robot steering method
                if(!nh_.getParam(key_params + "traj", traj_type_)){
                    ROS_WARN("Cannot load 'traj' param from ROS server: traj_type_ = Dubins");
                    traj_type_ = "Dubins";
                }

                // Get the robot dynamics integrator
                if(!nh_.getParam(key_params + "integrator", integrator_)){
                    ROS_WARN("Cannot load 'integrator' param from ROS server: integrator_ = RK2");
                    integrator_ = "RK2";
                }

                // Load the controller gains
                int gains_idx = 0;
                while(1)
                {
                    std::vector<double> g;
                    if(!nh_.getParam(key_params + "gains_" + std::to_string(gains_idx), g))
                    {
                        break;
                    }
                    gains_.push_back(g);
                    gains_idx++;
                }

                // Load the maximum control inputs values allowed
                if(!nh_.getParam(key_params + "inputs_max", inputs_max_)){
                    ROS_WARN("Cannot load 'inputs_max' param from ROS server: default inputs_max_ = [0.0, 0.0, 0.0, 0.0]");
                    inputs_max_ = {0.0, 0.0, 0.0, 0.0};
                }

                // Load the nominal values of the uncertain parameters
                if(!nh_.getParam(key_params + "uncertain_params", uncertain_params_)){
                    ROS_WARN("Cannot load 'uncertain_params' param from ROS server: default uncertain_params_ = [0.0]");
                    uncertain_params_ = {0.0};
                }

                // Load the uncertainty range
                if(!nh_.getParam(key_params + "delta_p", delta_p_)){
                    ROS_WARN("Cannot load 'delta_p' param from ROS server: default delta_p_ = [0.0]");
                    delta_p_ = {0.0};
                }

                // Load the workspace bounds
                double ws_value;

                // Controlled states bounds
                if(!nh_.getParam(key_params + "q_max", q_max_)){
                    ROS_WARN("Cannot load 'q_max' param from ROS server: default q_max_ = []");
                    q_max_ = {};
                }
                if(!nh_.getParam(key_params + "q_min", q_min_)){
                    ROS_WARN("Cannot load 'q_min' param from ROS server: default q_min_ = []");
                    q_min_ = {};
                }

                // Load the kinodynamics bounds
                int ks_max_idx = 0;
                std::vector<double> limits;
                while(1)
                {   
                    if(!nh_.getParam(key_params + "KS_max_" + std::to_string(ks_max_idx), limits))
                        break;

                    for(int i=0; i<limits.size();i++)
                        ks_max_values_.push_back(limits[i]);
                        
                    limits.clear();
                    ks_max_idx++;
                }

                // Robot mass
                if(!nh_.getParam(key_params + "mass", m_)){
                    ROS_WARN("Cannot load 'mass' param from ROS server: default m_ = 0.0");
                    m_ = 0.0;
                }

                // Goal threshold 
                if(!nh_.getParam(key_params + "goal_threshold", goal_threshold_)){
                    ROS_WARN("Cannot load 'goal_threshold' param from ROS server: default goal_threshold_ = 0.0");
                    goal_threshold_ = 0.0;
                }

                // Load initial robot state and get initial position according to the robot type
                if(!nh_.getParam(key_params + "initial_state", initial_state_)){
                    ROS_WARN("Cannot load 'initial_state' param from ROS server: default initial_state_ = [NONE]");
                    initial_state_ = {};
                }

                if(initial_state_.empty())
                    init_q_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0}; //x y z qx qy qz qw
                else
                {
                    if(robot_id_ == "quadrotor")
                    {
                        Eigen::Quaterniond q_init = euler2Quaternion(0.0, 0.0, initial_state_.at(3));
                        init_q_ = {initial_state_.at(0), initial_state_.at(1), initial_state_.at(2), q_init.x(), q_init.y(), q_init.z(), q_init.w()}; //x y z qx qy qz qw
                    }
                    else if(robot_id_ == "unicycle")
                    {
                        Eigen::Quaterniond q_init;
                        if(traj_type_ == "Dubins")
                            q_init = euler2Quaternion(0.0, 0.0, initial_state_.at(2));
                        else if(traj_type_ == "Kinospline")
                            q_init = euler2Quaternion(0.0, 0.0, 0.0);
                        init_q_ = {initial_state_.at(0), initial_state_.at(1), 0.0, q_init.x(), q_init.y(), q_init.z(), q_init.w()}; //x y z qx qy qz qw
                    }
                }

                ros::NodeHandle nh_srv;
                
                dynamic_ = nh_srv.serviceClient<services_msgs::DynamicSrv>("compute_dynamic");

                sensiODE_ = nh_srv.serviceClient<services_msgs::SensitivitySrv>("compute_sensitivity");

                localoptSrv_ = nh_srv.serviceClient<services_msgs::LocaloptSrv>("localopt");
            }
            ~Robot(){};

            /** \brief Cast this instance to a desired type. */
            template <class T>
            T *as()
            {
                /** \brief Make sure the type we are casting to is indeed a Robot */
                BOOST_CONCEPT_ASSERT((boost::Convertible<T *, Robot *>));

                return static_cast<T *>(this);
            }

            /** \brief Cast this instance to a desired type. */
            template <class T>
            const T *as() const
            {
                /** \brief Make sure the type we are casting to is indeed a Robot */
                BOOST_CONCEPT_ASSERT((boost::Convertible<T *, Robot *>));

                return static_cast<const T *>(this);
            }

            /// \brief Fills the NN input vector with the desired values available in the desired state.
            virtual std::vector<float> getRobotLearningInput(const std::vector<std::vector<double>>& des_state) const = 0;

            /// \brief Get the robot desired state dimension
            unsigned int getDesiredDim() const
            {
                return desired_dim_;
            }

            /// \brief Get the robot nominal state dimension
            unsigned int getNominalDim() const
            {
                return nominal_dim_;
            }

            /// \brief Get the robot name
            std::string getName() const
            {
                return robot_id_;
            }

            /// \brief Get the robot model path (urdf)
            std::string getUrdf() const
            {
                return urdf_;
            }

            /// \brief Get the scene path
            std::string getScene() const
            {
                return scene_;
            }

            /// \brief Get the steering method used
            std::string getTrajType() const
            {
                return traj_type_;
            }

            /// \brief Get the integrator used
            std::string getIntegratorType() const
            {
                return integrator_;
            }

            /** \brief Get the robot mass */
            double getMass() const
            {
                return m_;
            }

            /** \brief Get the threshold for the goal reaching distance */
            double getGoalThreshold() const
            {
                return goal_threshold_;
            }

            /// \brief Get the robot controler gains
            std::vector<std::vector<double>> getGains() const
            {
                return gains_;
            }

            /// \brief Get the robot initial configuration
            std::vector<double> getInitConfiguration() const
            {
                return init_q_;
            }

            /// \brief Get the robot maximum control input values
            std::vector<double> getMaxInputs() const
            {
                return inputs_max_;
            }

            /// \brief Get the robot uncertain parameters nominal values
            std::vector<double> getUncertainParams() const
            {
                return uncertain_params_;
            }

            /// \brief Get the robot parameters uncertainty range (i.e. delta_p)
            std::vector<double> getUncertaintyRange() const
            {
                return delta_p_;
            }

            /// \brief Get the robot position bounds
            std::vector<double> getQUpperBounds() const
            {
                return q_max_;
            }

            /// \brief Get the robot orientation bounds
            std::vector<double> getQLowerBounds() const
            {
                return q_min_;
            }

            /// \brief Get the robot kinodynamic limits
            std::vector<double> getKinodynamicLimits() const
            {
                return ks_max_values_;
            }

            /// \brief Set the robot uncertain parameters nominal values
            void setUncertainParams(const std::vector<double> &uncertain_params)
            {
                uncertain_params_.clear();
                uncertain_params_ = uncertain_params;
            }

            /// \brief Set the robot parameters uncertainty range (i.e. delta_p)
            void setUncertaintyRange(const std::vector<double> &delta_p)
            {
                delta_p_.clear();
                delta_p_ = delta_p;
            }

            /// \brief Set the robot gains
            void setGains(const std::vector<std::vector<double>> &new_gains)
            {
                gains_.clear();
                gains_ = new_gains;
            }

            /// @brief  Neural network object to predict uncertainty tubes
            SensitivityNN sensiNN_;

            /// @brief Ros client to communicate with the service in charge of the Sensitivity computation with JiTCODE
            ros::ServiceClient sensiODE_;

            /// @brief Object to compute the robot dynamic in the 'traditional' way by solving ODEs
            ros::ServiceClient dynamic_;

            /// @brief Ros client to communicate with the service in charge of optimizing the trajectory
            ros::ServiceClient localoptSrv_;

            // Node handler
            ros::NodeHandle nh_;

        private:

            // Dimension of the desired state according to the controller
            unsigned int desired_dim_;

            // Dimension of the nominal robot state
            unsigned int nominal_dim_;

            /** \brief Robot mass */
            double m_;

            /** \brief The threshold for the goal distance function, default is 0.0 meaning that we want te reach exactly the specified goal */
            double goal_threshold_;

            /// \brief Name of the robot
            std::string robot_id_;

            /// \brief Path to the urdf representation of the robot
            std::string urdf_;

            /// \brief Path to the scene file
            std::string scene_;

            /// \brief The steering method used
            std::string traj_type_;

            /// \brief The integrator for the dynamics
            std::string integrator_;

            /// \brief Controller gains
            std::vector<std::vector<double>> gains_;

            /// \brief Initial robot configuration
            std::vector<double> initial_state_, init_q_;

            /// \brief Max control inputs values
            std::vector<double> inputs_max_;

            /// \brief Nominal values of the uncertain parameters defined in the model
            std::vector<double> uncertain_params_;

            /// \brief Range of the uncertain parameters defined in the model
            std::vector<double> delta_p_;

            /// \brief The controllable robot states configuration bounds 
            std::vector<double> q_max_, q_min_;

            /// \brief The controllable robot states kinodynamic bounds
            std::vector<double> ks_max_values_;
    };
    typedef std::shared_ptr<Robot> RobotPtr;
}
#endif