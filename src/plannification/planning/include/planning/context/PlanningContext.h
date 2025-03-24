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

#ifndef OMPL_INTERFACE_PLANNING_CONTEXT_
#define OMPL_INTERFACE_PLANNING_CONTEXT_

// System
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <std_msgs/ColorRGBA.h>
#include <limits>
#include <boost/lexical_cast.hpp>
#include <chrono>
#include <boost/tokenizer.hpp>
#include <string>
#include <iostream>
#include <fstream>
#include <filesystem>  // C++17 and later

// OMPL
#include "ompl/geometric/SimpleSetup.h"
#include "ompl/base/terminationconditions/CostConvergenceTerminationCondition.h"
#include "ompl/base/StateSampler.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/geometric/PathGeometric.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "ompl/base/OptimizationObjective.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/ScopedState.h"
#include "ompl/util/ClassForward.h"
#include "ompl/util/RandomNumbers.h"
#include "ompl/util/Console.h"

// Robots
#include "robots/Quadrotor.h"
#include "robots/Unicycle.h"

// Optimization objectives
#include "planning/opt_objectives/KS_time_obj.h"
#include "planning/opt_objectives/Sensi_obj.h"

// State spaces
#include "planning/state_spaces/kinospline/KinoQuadrotorSpace.h"
#include "planning/state_spaces/kinospline/KinoUnicycleSpace.h"
#include "planning/state_spaces/dubins/DubinsUnicycleSpace.h"

// Samplers
#include "planning/samplers/DubinsStateSampler.h"
#include "planning/samplers/KinosplineStateSampler.h"

// State validity checker
#include "planning/state_checker/RobustStateValidityChecker.h"

// Motion validators
#include "planning/motion_validator/DiscreteRobustMotionValidator.h"

// Planners
#include "planning/planners/FMT.h"
#include "planning/planners/Generator.h"
#include "planning/planners/RandUPRRT.h"
#include "planning/planners/RRT.h"
#include "planning/planners/RRTstar.h"
#include "planning/planners/R_SAFMT_NN.h"
#include "planning/planners/R_SARRT_NN.h"
#include "planning/planners/R_SARRT.h"
#include "planning/planners/R_SARRTstar_NN.h"
#include "planning/planners/R_SARRTstar.h"
#include "planning/planners/SARRT.h"
#include "planning/planners/SARRTstar.h"
#include "planning/planners/SST.h"
#include "planning/planners/SSTstar.h"
#include "planning/planners/R_SASST.h"
#include "planning/planners/R_SASSTstar.h"
#include "planning/planners/TRRT.h"

// Post processor
#include "planning/post_processing/PostProcessor.h"
#include "planning/post_processing/Shortcut.h"
#include "planning/post_processing/Smooth.h"
#include "planning/post_processing/ExtendedShortcut.h"
#include "planning/post_processing/NloptLocalopt.h"
#include "planning/post_processing/STOMP.h"

// Utils
#include "utils/transform.h"

// Bullet
#include "planning/bullet/PhysicsServers.h"

// Visual tools
#include "planning/visual_tools/BulletVisualTools.h"

namespace ompl_interface
{

  /// \brief Definition of a dual arm planning context.  This context plans in the workspace
  ///  for a given group.  This context is NOT thread safe.
  class PlanningContext
  {
    public:
      PlanningContext(const std::string& ros_namespace);

      virtual ~PlanningContext();

      /// \brief Clear all data structures used by the planner
      virtual void clear();

      /// \brief Solve the motion planning problem.
      /// This function should not clear data structures before computing. The constructor
      /// and clear() do that.
      virtual bool solve();

      /// \brief Stop planning
      virtual bool terminate();

      void readTrajFile(const std::string &filename, std::vector<ompl::base::State*> &refTraj, std::vector<int> &wpt_idx);

      void readGainFile(const std::string &filename, std::vector<std::vector<double>>& gains);

      /// \brief Returns whether the planning context is correctly initialized or not.
      bool isInitialized() const
      {
        return initialized_;
      }

    protected:

      /**
         \brief Specify the settings for a particular planning algorithm.
        \note Settings with unknown keys are ignored. Settings for unknown groups are ignored.
      */
      struct PlannerConfigurationSettings
      {
        /** \brief Key-value pairs of settings that get passed to the planning algorithm */
        std::map<std::string, std::string> config;
      };

      /// \brief Definition of a PlannerAllocator function.  This function takes the OMPL SpaceInformation
      /// object, the new name of the planner (if any), and a map containing planner configuration items.
      typedef boost::function<ompl::base::PlannerPtr(const ompl::base::SpaceInformationPtr&, const std::string&,
                                                    const std::map<std::string, std::string>&, const BulletVisualToolsPtr&, const std::string &ros_namespace)> PlannerAllocator;
      
      /// \brief Read planning group context parameters from the ROS param server
      virtual void getPlannersParameters();

      /// @brief  Initialized the robot model according to the parameters defined in config/robot_params.yaml 
      virtual void initializeModelParameters();
      
      /// @brief  Initialized the planning parameters (i.e. postprocessin, goals, etc.) according to the parameters defined in config/planning_params.yaml
      virtual void initializePlanningParameters();

      /// @brief  Initialized the planning context (i.e. state space, motion validator, etc.)
      virtual void initializePlanningContext();

      /// @brief  Initialized the post processor algorithm (i.e shortcut, MPC, NEB, STOMP, etc.)
      virtual void initializePostProcessor();

      /// \brief Initialize all of the planner allocators for planners this context is aware of
      void initializePlannerAllocators();

      /// \brief Associate the given planner_id string with the given planner allocator
      void registerPlannerAllocator(const std::string &planner_id, const PlannerAllocator &pa);

      /// \brief Simplify the solution path (in simple setup).  Use no more than max_time seconds.
      virtual double simplifySolution(double max_time);

      /** \brief Interpolate solution adated to the aeroarm case 
       * @param colorPath Color of the trajectory
       * @param index_wpt_in_traj Position of the desired waypoint we want to optimize the accuracy in the global trajectory
      */
      void interpolateSolution(const double dt, std::vector<int>& index_wpt_in_traj);

      /// \brief Allocate the StateSpace for the given specification.  This will initialize the
      /// \e state_space_ member.
      virtual void allocateStateSpace();

      /// \brief Set the optimization objective according to the one specified in the config file
      virtual void setOptimizationObjectives();

      /// \brief Allocate a (possibly constrained) state sampler.  If there are no path constraints, the
      /// sampler is the default from OMPL. Otherwise, a custom sampler is created to sample states from
      /// the constraints specified in the motion plan request.
      virtual ompl::base::StateSamplerPtr allocSampler(const ompl::base::StateSpace *space) const;

      /// \brief A method that is invoked immediately before every call to solve()
      virtual void preSolve();

      /// \brief A method that is invoked immediately after every call to solve()
      virtual void postSolve();

      /// \brief Set the currently running termination condition.  Used for terminate()
      void registerTerminationCondition(const ompl::base::PlannerTerminationCondition &ptc);

      /// \brief Clear the currently running termination condition.  Used for terminate()
      void unregisterTerminationCondition();

      /// \brief Return an instance of the given planner_name configured with the given parameters
      virtual ompl::base::PlannerPtr configurePlanner(const std::string& planner_name, const std::map<std::string, std::string>& params);

      /** \brief Allocate the ompl starting state according to the kinospline state space */
      ompl::base::State* getKinoStartState();

      /** \brief Allocate the ompl starting state according to the dubins state space */
      ompl::base::State* getDubinsStartState();

      /** \brief Create an ompl goal state according to the kinospline state space filled with the values in goal */
      ompl::base::State* getKinoGoalState(const std::vector<double>& goal);

      /** \brief Create an ompl goal state according to the dubins state space filled with the values in goal */
      ompl::base::State* getDubinsGoalState(const std::vector<double>& goal);
      
      /** \brief Performs N simulations (depending on configuration) with random uncertain parameters and exports the result */
      void simulate_and_export(ompl::geometric::PathGeometric &pg, std::vector<int>& index_wpt_in_traj_);

      void exportSolutionPath(ompl::geometric::PathGeometric &pg, std::vector<int>& index_wpt_in_traj_);

      void testNN_and_export(ompl::geometric::PathGeometric &pg);

      //###############################################################################################################
      // PLANNING CONTEXT

      /// \brief Node handler
      ros::NodeHandle nh_;

      /// \brief True when the context is properly initialized
      bool initialized_;

      /// \brief Pointer to the OMPL SimpleSetup object
      ompl::geometric::SimpleSetupPtr simple_setup_;

      /// \brief Motion validator used to check a local motion
      ompl::base::MotionValidatorPtr mv_;

      /// \brief Pointer to the (derived) OMPL StateSpace object
      ompl::base::StateSpacePtr state_space_;

      /// \brief The set of planner allocators that have been registered
      std::map<std::string, PlannerAllocator> planner_allocators_;

      /// \brief The currently registered planner termination condition
      const ompl::base::PlannerTerminationCondition *ptc_;

      /// \brief Bullet server
      owds::BulletClient* blt_client_;

      /// \brief Visual tools
      BulletVisualToolsPtr visual_tools_;
      
      /// \brief Mutex around ptc_ for thread safety.
      boost::mutex ptc_lock_;

      //###############################################################################################################
      // ROBOT PARAMETERS

      /// \brief Pointer to the Robot object
      ompl::RobotPtr robot_;
      
      //###############################################################################################################
      // ROBOT'S PLANNING PARAMETERS
      
      /// \brief Vector to store the initial robot state from the config file
      std::vector<double> initial_state_;

      /// \brief Vector to store all the robot goals from the config file
      std::vector<std::vector<double>> goals_;

      /// \brief True when planning for the robot end effector
      bool eef_planning_;

      //###############################################################################################################
      // PLANNER PARAMETERS

      /// \brief Name of the planner used
      std::string planner_id_;

      /// \brief Maximum plan time allowed
      double planner_timeout_;

      /// \brief Frequency used for CC in planning
      double dt_planning_; 

      /// \brief Output frequency of the final trajectory
      double dt_export_;

      /// \brief Files names
      std::string cost_file_, traj_file_, gains_file_;

      /** \brief The planner parameters map */
      std::map<std::string, PlannerConfigurationSettings> pconfig_;

      /** \brief The post processor pointer */
      ompl::PostProcessorPtr postProcessor_;

      //###############################################################################################################
      // GLOBAL PLANNING PARAMETERS

      /// \brief Number of maximum planning attempt
      int max_planning_attempt_;

      /// \brief Number of simulation to be perform on the final solution
      int nb_simu_;

      /// \brief Number of iteration for the post processing algorithm (e.g. number of SAShortcut attempt)
      int nb_iter_post_;

      /// \brief Cost threshold to stop the optimal planners (avoid to be stuck with the cost convergence condition)
      double cost_threshold_;

      /// \brief If true, the solution path will be shortened after planning and before post processing.
      bool simplify_;

      /// \brief True when only simulating a given trajectory (the trajectory file must be provided in this case)
      bool simulate_only_;

      /// \brief True when only want to compare the NN results vs the ODE results 
      bool test_NN_;

      /// \brief True if a post processor is specified, False if NONE
      bool post_process_;

      /// \brief True if a post processor comparison is specified
      bool post_process_comparison_;

      /// \brief Path of the CAMP repository
      std::filesystem::path campDir_;

      /// \brief Name of the post processing method to use
      std::string post_process_id_;

      /// \brief Optimization objective
      std::string opti_obj_;

      /// \brief The ros namespace
      std::string ros_namespace_;

      /// \brief The coloer to display the trajectory
      std_msgs::ColorRGBA colorPath;

      /** \brief The random number generator */
      ompl::RNG rng_;
  };
}
#endif
