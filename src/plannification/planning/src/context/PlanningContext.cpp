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

#include "planning/context/PlanningContext.h"

namespace og = ompl::geometric;

using namespace ompl_interface;

PlanningContext::PlanningContext(const std::string& ros_namespace) : ros_namespace_(ros_namespace), nh_(ros_namespace)
{
    // Get the current CAMP path 
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

    // Initialized all the planners according to their definition in XXXX
    initializePlannerAllocators();

    // Initialize the planners parameters
    getPlannersParameters();

    // Initialized the robot model according to the parameters defined in config/robot_params.yaml
    initializeModelParameters();

    // Initialized the planning parameters (i.e. dt, postprocessing, goals, etc.) according to the parameters defined in config/planning_params.yaml
    initializePlanningParameters();

    // Initialized planning context (i.e. state space, motion validator, etc.)
    initializePlanningContext();

    // Initialized post processor
    initializePostProcessor();

    // To clear the file 
    std::ofstream ofs;
    ofs.open(cost_file_, std::ofstream::out | std::ofstream::trunc);
    ofs.close();
    ofs.open(traj_file_, std::ofstream::out | std::ofstream::trunc);
    ofs.close();
    ofs.open(gains_file_, std::ofstream::out | std::ofstream::trunc);
    ofs.close();

    // Set the color path
    colorPath.r = 1.0f;
    colorPath.g = 0.0f;
    colorPath.b = 0.0f;
    colorPath.a = 1.0f;

    // This context is now initialized
    initialized_ = true;
    ROS_INFO("Initialized PlanningContext");
}

PlanningContext::~PlanningContext()
{
}

template<typename T>
static ompl::base::PlannerPtr allocatePlanner(const ompl::base::SpaceInformationPtr &si,
                                              const std::string &new_name, const std::map<std::string, std::string>& params, const BulletVisualToolsPtr &visual_tools,
                                              const std::string &ros_namespace)
{
    ompl::base::PlannerPtr planner(new T(si, visual_tools, ros_namespace));
    if (!new_name.empty())
        planner->setName(new_name);
    planner->params().setParams(params, true);
    return planner;
}

void PlanningContext::getPlannersParameters()
{
    // Retrieve the parameters for each planner configured for this group
    std::vector<std::string> config_names;
    if (nh_.getParam("planner_configs/planners_name", config_names))
    {
        for (size_t j = 0; j < config_names.size(); ++j)
        {
            std::string planner_config = static_cast<std::string>(config_names[j]);
            XmlRpc::XmlRpcValue xml_config;
            if (nh_.getParam("planner_configs/" + planner_config, xml_config))
            {
                if (xml_config.getType() == XmlRpc::XmlRpcValue::TypeStruct)
                {
                    PlannerConfigurationSettings pc;

                    // read parameters specific for this configuration
                    for (XmlRpc::XmlRpcValue::iterator it = xml_config.begin() ; it != xml_config.end() ; ++it)
                    {
                        switch(it->second.getType())
                        {
                            case XmlRpc::XmlRpcValue::TypeString:
                                pc.config[it->first] = static_cast<std::string>(it->second);
                                break;
                            case XmlRpc::XmlRpcValue::TypeDouble:
                                pc.config[it->first] = boost::lexical_cast<std::string>(static_cast<double>(it->second));
                                break;
                            case XmlRpc::XmlRpcValue::TypeInt:
                                pc.config[it->first] = boost::lexical_cast<std::string>(static_cast<int>(it->second));
                                break;
                            case XmlRpc::XmlRpcValue::TypeBoolean:
                                pc.config[it->first] = boost::lexical_cast<std::string>(static_cast<bool>(it->second));
                                break;
                        }
                    }
                    pconfig_[planner_config] = pc;
                }
            }
        }
    }
}

void PlanningContext::initializePlannerAllocators()
{
    registerPlannerAllocator("geometric::FMT", boost::bind(&allocatePlanner<og::FMT>, _1, _2, _3, _4, _5));
    registerPlannerAllocator("geometric::Generator", boost::bind(&allocatePlanner<og::Generator>, _1, _2, _3, _4, _5));
    registerPlannerAllocator("geometric::RandUPRRT", boost::bind(&allocatePlanner<og::RandUPRRT>, _1, _2, _3, _4, _5));
    registerPlannerAllocator("geometric::RRTstar", boost::bind(&allocatePlanner<og::RRTstar>, _1, _2, _3, _4, _5));
    registerPlannerAllocator("geometric::RRT", boost::bind(&allocatePlanner<og::RRT>, _1, _2, _3, _4, _5));
    registerPlannerAllocator("geometric::R_SAFMT_NN", boost::bind(&allocatePlanner<og::R_SAFMT_NN>, _1, _2, _3, _4, _5));
    registerPlannerAllocator("geometric::R_SARRT", boost::bind(&allocatePlanner<og::R_SARRT>, _1, _2, _3, _4, _5));
    registerPlannerAllocator("geometric::R_SARRT_NN", boost::bind(&allocatePlanner<og::R_SARRT_NN>, _1, _2, _3, _4, _5));
    registerPlannerAllocator("geometric::R_SARRTstar_NN", boost::bind(&allocatePlanner<og::R_SARRTstar_NN>, _1, _2, _3, _4, _5));
    registerPlannerAllocator("geometric::R_SARRTstar", boost::bind(&allocatePlanner<og::R_SARRTstar>, _1, _2, _3, _4, _5));
    registerPlannerAllocator("geometric::SARRT", boost::bind(&allocatePlanner<og::SARRT>, _1, _2, _3, _4, _5));
    registerPlannerAllocator("geometric::SARRTstar", boost::bind(&allocatePlanner<og::SARRTstar>, _1, _2, _3, _4, _5));
    registerPlannerAllocator("geometric::SST", boost::bind(&allocatePlanner<og::SST>, _1, _2, _3, _4, _5));
    registerPlannerAllocator("geometric::SSTstar", boost::bind(&allocatePlanner<og::SSTstar>, _1, _2, _3, _4, _5));
    registerPlannerAllocator("geometric::R_SASST", boost::bind(&allocatePlanner<og::R_SASST>, _1, _2, _3, _4, _5));
    registerPlannerAllocator("geometric::R_SASSTstar", boost::bind(&allocatePlanner<og::R_SASSTstar>, _1, _2, _3, _4, _5));
    registerPlannerAllocator("geometric::TRRT", boost::bind(&allocatePlanner<og::TRRT>, _1, _2, _3, _4, _5));
}

void PlanningContext::initializeModelParameters()
{
    const std::string key_params = "robot_parameters/";

    // Load initial robot state
    if(!nh_.getParam(key_params + "initial_state", initial_state_)){
        ROS_WARN("Cannot load 'initial_state' param from ROS server: default initial_state_ = [NONE]");
        initial_state_ = {};
    }

    // Load the robot goals
    int goal_idx = 0;
    while(1)
    {
        std::vector<double> goal;
        if(!nh_.getParam(key_params + "goal_" + std::to_string(goal_idx), goal))
            break;
        goals_.push_back(goal);
        goal_idx++;
    }

    // Load parameters for the perch end effector planning
    if(!nh_.getParam(key_params + "eef_planning", eef_planning_)){
        ROS_WARN("Cannot load 'eef_planning' param from ROS server: default eef_planning_ = false");
        eef_planning_ = false;
    }
    
    // Create the robot model
    std::string robot_name;
    if(!nh_.getParam(key_params + "robot_id", robot_name)){
        ROS_WARN("Cannot load 'robot_id' param from ROS server: default robot_name = NONE");
        robot_name = "NONE";
    }
    if(robot_name == "NONE")
    {
        ROS_ERROR("Unable to create the robot model, robot is NONE !!");
    }
    else if(robot_name == "quadrotor")
    {
        assert(initial_state_.size()%3 == 0.0); // We check if q, qdot and qddot are of same size
        robot_.reset(new ompl::Quadrotor(int(initial_state_.size()/3), 13, nh_));
    }
    else if (robot_name == "unicycle")
    {
        assert(initial_state_.size()%3 == 0.0); // We check if q, qdot and qddot are of same size
        robot_.reset(new ompl::Unicycle(int(initial_state_.size()/3), 3, nh_));
    }
    ROS_INFO("Robot initialized");
}

void PlanningContext::initializePlanningParameters()
{
    const std::string key_params = "planning_parameters/";

    // Load the time steps for planning, post processing and export
    if(!nh_.getParam(key_params + "dt_planning", dt_planning_)){
        ROS_WARN("Cannot load 'dt_cc' param from ROS server: default dt_cc = 0.05");
        dt_planning_ = 0.05;
    }

    // Load max planning attempt
    if(!nh_.getParam(key_params + "max_planning_attempt", max_planning_attempt_)){
        ROS_WARN("Cannot load 'max_planning_attempt' param from ROS server: default max_planning_attempt_ = 1");
        max_planning_attempt_ = 1;
    }

    // Load the planner id
    if(!nh_.getParam(key_params + "planner_id", planner_id_)){
        ROS_WARN("Cannot load 'planner_id' param from ROS server: default planner_id_ = RRT");
        planner_id_ = "RRT";
    }

    ROS_INFO("Planning with : %s", planner_id_.c_str());

    // Load the planning timeout
    if(!nh_.getParam(key_params + "plan_time", planner_timeout_)){
        ROS_WARN("Cannot load 'plan_time' param from ROS server: default planner_timeout_ = 30s");
        planner_timeout_ = 30;
    }

    // Load the files
    if(!nh_.getParam(key_params + "cost_file", cost_file_)){
        ROS_WARN("Cannot load 'cost_file' param from ROS server: default cost_file_ = /cost.txt");
        cost_file_ = "/cost.txt";
    }
    else
    {
        // Combine the CAMP directory with the relative file path
        std::filesystem::path fullPath = std::filesystem::absolute(campDir_ / cost_file_);

        // Normalize the path to remove redundant ".." and "."
        fullPath = fullPath.lexically_normal();

        // Convert to string and use the full path
        cost_file_ = fullPath.string();
    }

    if(!nh_.getParam(key_params + "traj_file", traj_file_)){
        ROS_WARN("Cannot load 'traj_file' param from ROS server: default traj_file_ = /traj.txt");
        traj_file_ = "/traj.txt";
    }
    else
    {
        // Combine the CAMP directory with the relative file path
        std::filesystem::path fullPath = std::filesystem::absolute(campDir_ / traj_file_);

        // Normalize the path to remove redundant ".." and "."
        fullPath = fullPath.lexically_normal();

        // Convert to string and use the full path
        traj_file_ = fullPath.string();
    }

    if(!nh_.getParam(key_params + "gains_file", gains_file_)){
        ROS_WARN("Cannot load 'gains_file' param from ROS server: default gains_file_ = /gains.txt");
        gains_file_ = "/gains.txt";
    }
    else
    {
        // Combine the CAMP directory with the relative file path
        std::filesystem::path fullPath = std::filesystem::absolute(campDir_ / gains_file_);

        // Normalize the path to remove redundant ".." and "."
        fullPath = fullPath.lexically_normal();

        // Convert to string and use the full path
        gains_file_ = fullPath.string();
    }

    // Load simulation parameters
    if(!nh_.getParam(key_params + "simulate_only", simulate_only_)){
        ROS_WARN("Cannot load 'simulate_only' param from ROS server: default simulate_only_ = false");
        simulate_only_ = false;
    }
    if(!nh_.getParam(key_params + "N_simu", nb_simu_)){
        ROS_WARN("Cannot load 'N_simu' param from ROS server: default nb_simu_ = 1");
        nb_simu_ = 1;
    }

    // Load neural networks parameters
    if(!nh_.getParam(key_params + "test_NN", test_NN_)){
        ROS_WARN("Cannot load 'test_NN' param from ROS server: default test_NN_ = false");
        test_NN_ = false;
    }

    // Load post processing parameters
    if(!nh_.getParam(key_params + "simplify", simplify_)){
        ROS_WARN("Cannot load 'simplify' param from ROS server: default simplify_ = false");
        simplify_ = false;
    }
    if(!nh_.getParam(key_params + "post_processing", post_process_id_)){
        ROS_WARN("Cannot load 'post_processing' param from ROS server: default post_process_id_ = NONE");
        post_process_id_ = "NONE";
    }
    if(!nh_.getParam(key_params + "comparison", post_process_comparison_)){
        ROS_WARN("Cannot load 'comparison' param from ROS server: default post_process_comparison_ = NONE");
        post_process_comparison_ = false;
    }

    // Load optimization objective
    if(!nh_.getParam(key_params + "optimization_objective", opti_obj_)){
        ROS_WARN("Cannot load 'optimization_objective' param from ROS server: default opti_obj_ = KS_time");
        opti_obj_ = "KS_time";
    }

    // Load optimization objective threshold
    if(!nh_.getParam(key_params + "cost_threshold", cost_threshold_)){
        ROS_WARN("Cannot load 'cost_threshold' param from ROS server: default cost_threshold_ = 0.0");
        cost_threshold_ = 0.0;
    }
}

void PlanningContext::initializePlanningContext()
{
    // Bullet
    bool visualize;
    if(!nh_.getParam("planning_parameters/visualize_tree", visualize))
    {
        ROS_WARN("Value of visualize_tree not found. Using default visualize = false.");
        visualize = false;
    }

    std::vector<double> camera_params;
    if(!nh_.getParam("planning_parameters/camera_params", camera_params))
    {
        ROS_WARN("Value of camera_params not found. Using default camera_params = {}.");
        camera_params = {};
    }

    if(visualize)
    {
        blt_client_ = owds::PhysicsServers::connectPhysicsServer(owds::CONNECT_GUI);
        blt_client_->configureDebugVisualizer(COV_ENABLE_GUI, false);
        blt_client_->configureDebugVisualizer(COV_ENABLE_DEPTH_BUFFER_PREVIEW, false);
        blt_client_->configureDebugVisualizer(COV_ENABLE_SHADOWS, false);
        blt_client_->configureDebugVisualizer(COV_ENABLE_PLANAR_REFLECTION, false);
        blt_client_->configureDebugVisualizer(COV_ENABLE_RGB_BUFFER_PREVIEW, false);
        blt_client_->configureDebugVisualizer(COV_ENABLE_SEGMENTATION_MARK_PREVIEW, false);
        blt_client_->configureDebugVisualizer(COV_ENABLE_WIREFRAME, false);
        blt_client_->configureDebugVisualizer(COV_ENABLE_RENDERING, true);
        blt_client_->resetDebugVisualizerCamera(camera_params[0], camera_params[1], camera_params[2], {camera_params[3], camera_params[4], camera_params[5]}); 
    }
    else
    {
        blt_client_ = owds::PhysicsServers::connectPhysicsServer(owds::CONNECT_DIRECT);
    }

    // Visual tools
    visual_tools_.reset(new BulletVisualTools(blt_client_, robot_));
    
    // OMPL StateSpace
    allocateStateSpace();

    // OMPL SimpleSetup
    simple_setup_.reset(new ompl::geometric::SimpleSetup(state_space_));

    // Optimization objectives
    setOptimizationObjectives();
    ROS_INFO("Objective function initialized");
    
    // OMPL Planner
    if (planner_id_ != "")
    {   
        ompl::base::PlannerPtr planner = configurePlanner(planner_id_, pconfig_.find(planner_id_)->second.config);
        simple_setup_->setPlanner(planner);
        ROS_INFO("Planner initialized");
    }

    // OMPL StateSampler
    state_space_->setStateSamplerAllocator(boost::bind(&PlanningContext::allocSampler, this, _1));
    ROS_INFO("Sampler initialized");

    // State validity checker
    simple_setup_->setStateValidityChecker(ompl::base::StateValidityCheckerPtr(new ompl::base::RobustStateValidityChecker(simple_setup_->getSpaceInformation(), robot_, blt_client_, nh_)));

    // Motion validator
    mv_.reset(new ompl::base::DiscreteRobustMotionValidator(simple_setup_->getSpaceInformation(), dt_planning_, robot_, nh_));
    simple_setup_->getSpaceInformation()->setMotionValidator(mv_);
    ROS_INFO("Motion validator initialized");

    // Setup the post processing object
}

void PlanningContext::initializePostProcessor()
{
    // If no post-processor has been found, or if the value is NONE, no post-processing is performed.
    if(post_process_id_ == "NONE" && !post_process_comparison_)
    {
        post_process_ = false;
    }
    else if(post_process_id_ == "Shortcut")
    {
        post_process_ = true;
        postProcessor_.reset(new ompl::Shortcut(simple_setup_->getSpaceInformation(), robot_, nh_));
        ROS_INFO("Shortcut initialized");
    }
    else if(post_process_id_ == "ExtendedShortcut")
    {
        post_process_ = true;
        postProcessor_.reset(new ompl::ExtendedShortcut(simple_setup_->getSpaceInformation(), robot_, nh_));
        ROS_INFO("ExtendedShortcut initialized");
    }
    else if(post_process_id_ == "NloptLocalopt")
    {
        post_process_ = true;
        postProcessor_.reset(new ompl::NloptLocalopt(simple_setup_->getSpaceInformation(), robot_, nh_));
        ROS_INFO("NloptLocalopt initialized");
    }
    else if(post_process_id_ == "STOMP")
    {
        post_process_ = true;
        postProcessor_.reset(new ompl::STOMP(simple_setup_->getSpaceInformation(), robot_, nh_));
        ROS_INFO("STOMP initialized");
    }
    else if(post_process_id_ == "NONE")
    {
        post_process_ = true;
        ROS_INFO("Post processor comparator initialized");
    }
}

void PlanningContext::registerPlannerAllocator(const std::string &planner_id, const PlannerAllocator &pa)
{
    planner_allocators_[planner_id] = pa;
}

void PlanningContext::allocateStateSpace()
{
    if(robot_->getName() == "")
    {
        ROS_ERROR("Unable to instantiate a state space because robot is NONE !!");
    }
    else if(robot_->getName() == "quadrotor")
    {
        state_space_.reset(new ompl::base::KinoQuadrotorSpace(robot_->getDesiredDim(), opti_obj_, robot_));
        ROS_INFO_STREAM("State space allocated");
    }
    else if (robot_->getName() == "unicycle")
    {
        if(robot_->getTrajType() == "Kinospline")
        {
            state_space_.reset(new ompl::base::KinoUnicycleSpace(robot_->getDesiredDim(), opti_obj_, robot_));
        }
        else if(robot_->getTrajType() == "Dubins")
        {
            double turn_radius = 1.0;
            state_space_.reset(new ompl::base::DubinsUnicycleSpace(turn_radius, false, opti_obj_, robot_));
        }
        ROS_INFO_STREAM("State space allocated");
    }
}

void PlanningContext::setOptimizationObjectives()
{
    ompl::base::ProblemDefinitionPtr pdef = simple_setup_->getProblemDefinition();
    
    if (opti_obj_ == "KS_time")
    {
        if(robot_->getTrajType() != "Kinospline")
        {
            ROS_ERROR("Can't use KS_time as objective function wihtout using Kinospline as steering method !");
            ros::shutdown();
        }
            
        auto opt_ = std::make_shared<ompl::base::MinKSTime>(simple_setup_->getSpaceInformation(),  dt_planning_);

        if(cost_threshold_ > 0.0)
            opt_->setCostThreshold(ompl::base::Cost(cost_threshold_));

        // Store the new objective in the problem def
        pdef->setOptimizationObjective(opt_);
    }
    else if (opti_obj_ == "Sensi")
    {
        auto opt_ = std::make_shared<ompl::base::SensiObj>(simple_setup_->getSpaceInformation(),  dt_planning_);

        if(cost_threshold_ > 0.0)
            opt_->setCostThreshold(ompl::base::Cost(cost_threshold_));

        // Store the new objective in the problem def
        pdef->setOptimizationObjective(opt_);
    }
    
    else
    {
        ROS_WARN("Optimization objective not implemented ! Using default path length !");
        auto opt_ = std::make_shared<ompl::base::PathLengthOptimizationObjective>(simple_setup_->getSpaceInformation());

        if(cost_threshold_ > 0.0)
            opt_->setCostThreshold(ompl::base::Cost(cost_threshold_));

        // Store the new objective in the problem def
        pdef->setOptimizationObjective(opt_);

    }
}

ompl::base::StateSamplerPtr PlanningContext::allocSampler(const ompl::base::StateSpace *space) const
{
    if (state_space_.get() != space)
    {
        ROS_ERROR("Attempted to allocate a state sampler for an unknown state space");
        return ompl::base::StateSamplerPtr();
    }     
    else
    {
        if(robot_->getTrajType() == "Kinospline")
        {
            return std::make_shared<ompl::base::KinosplineStateSampler>(space, robot_->getDesiredDim());
        }
        else if(robot_->getTrajType() == "Dubins")
        {
             return std::make_shared<ompl::base::DubinsStateSampler>(space, robot_->getDesiredDim());
        }
    }
        
}

void PlanningContext::clear()
{
    simple_setup_->clear();
}

void PlanningContext::preSolve()
{
    simple_setup_->getProblemDefinition()->clearSolutionPaths();
    const ompl::base::PlannerPtr planner = simple_setup_->getPlanner();
    if(planner)
        planner->clear();
    simple_setup_->getSpaceInformation()->getMotionValidator()->resetMotionCounter();  
}

void PlanningContext::postSolve()
{
    if (simple_setup_->getProblemDefinition()->hasApproximateSolution())
        ROS_WARN("Solution is approximate");
}

bool PlanningContext::solve()
{
    if(simulate_only_)
    {
        // Combine the CAMP directory with the relative traj file path
        std::filesystem::path fullPath_traj = std::filesystem::absolute(campDir_ / "src/results/LocalOpt/RefTraj/Traj0/robust_traj.txt");
        fullPath_traj = fullPath_traj.lexically_normal();
        std::string traj_ref_file = fullPath_traj.string();

        // Combine the CAMP directory with the relative gains file path
        std::filesystem::path fullPath_gains = std::filesystem::absolute(campDir_ / "src/results/LocalOpt/default_gains.txt");
        fullPath_gains = fullPath_gains.lexically_normal();
        std::string opti_gains_file = fullPath_gains.string();

        std::vector<int> wpt_idx;
        std::vector<std::vector<double>> opti_gains;
        std::vector<ompl::base::State*> refTraj;

        og::PathGeometric ref_traj(simple_setup_->getSpaceInformation(), simple_setup_->getSpaceInformation()->allocState());

        readTrajFile(traj_ref_file, refTraj, wpt_idx);
        readGainFile(opti_gains_file, opti_gains);

        // Set the read trajectory and gains
        std::swap(ref_traj.getStates(), refTraj);
        robot_->setGains(opti_gains);
        
        exportSolutionPath(ref_traj, wpt_idx);
        simulate_and_export(ref_traj, wpt_idx);   
        ROS_WARN("Simulations done !");
        ros::shutdown();
        return true;
    }
    else
    {
        // The vector of waypoints
        std::vector<ompl::base::State*> result_wpt_planning_;
        std::vector<int> index_wpt_in_traj_;

        //We push the first state of the motion according to the current state space
        if(robot_->getTrajType() == "Kinospline")
        {
            ompl::base::State* start_st = getKinoStartState(); 
            result_wpt_planning_.push_back(start_st);
        }
        else if(robot_->getTrajType() == "Dubins")
        {
            ompl::base::State* start_st = getDubinsStartState(); 
            result_wpt_planning_.push_back(start_st);
        }
        
        // The planning time counter
        double plan_time = 0.0;

        // The number of planning attempt
        int attempt = 0;

        for(int i = 0; i<goals_.size(); i++)
        {
            if(attempt > max_planning_attempt_)
            {
                ROS_WARN("Unable to solve the planning problem");
                return false;
            }

            preSolve();

            // Create the probleme definition (i.e. start, goal, stopping criterion)
            ompl::base::ProblemDefinitionPtr pdef = simple_setup_->getProblemDefinition();

            // Set the start and goal states
            // Set Start
            ompl::base::ScopedState<> start_st(simple_setup_->getStateSpace(), result_wpt_planning_.back());

            // Set Goal
            ompl::base::State* goal_state;
            if(robot_->getTrajType() == "Kinospline")
            {
                goal_state = getKinoGoalState(goals_.at(i));
                ompl::base::ScopedState<> goal_st(simple_setup_->getStateSpace(), goal_state);

                // Set the start and goal state
                pdef->setStartAndGoalStates(start_st, goal_st);
            }
            else if(robot_->getTrajType() == "Dubins")
            {
                goal_state = getDubinsGoalState(goals_.at(i));
                ompl::base::ScopedState<> goal_st(simple_setup_->getStateSpace(), goal_state);
                
                // We use a goal region instead of default state goal for the Dubins case as this steering method do not allow complete implementation of RRT
                // Set the start and goal state
                pdef->setStartAndGoalStates(start_st, goal_st, robot_->getGoalThreshold());
            }
            
            // Planner termination condition
            ompl::base::PlannerTerminationCondition ptc = ompl::base::timedPlannerTerminationCondition(planner_timeout_);
            // ompl::base::PlannerTerminationCondition ptc = ompl::base::CostConvergenceTerminationCondition(pdef, 3, 0.1);
            registerTerminationCondition(ptc);

            // Solve
            if(simple_setup_->solve(ptc) == ompl::base::PlannerStatus::EXACT_SOLUTION)
            {
                if(planner_id_ == "Generator")
                {  
                    ROS_WARN("GENERATION DONE SAVE FILE !");
                    std::cin.get();
                    ros::shutdown();
                    return true;
                }
                og::PathGeometric &between_wpt = simple_setup_->getSolutionPath(); 
                // The first state of the path is the last one of the previous and has already been pushed
                for(size_t t=1; t<between_wpt.getStateCount(); ++t)
                {
                    ompl::base::State* wpt = simple_setup_->getSpaceInformation()->allocState();
                    simple_setup_->getSpaceInformation()->copyState(wpt, between_wpt.getState(t));
                    result_wpt_planning_.push_back(wpt);
                }
                index_wpt_in_traj_.push_back(result_wpt_planning_.size()-1);
                postSolve();

                attempt = 0;
                ROS_WARN("Trajectory find for the current goal, GOOD JOB ! Planning for the next one !");
                
            }
            else
            {
                // We replan for the same pair of states and increment the attempt counter
                postSolve();
                ROS_WARN("Replanning.");
                
                i--;
                attempt++;
            }
            // Clear visualization
            visual_tools_->removeAll();
            unregisterTerminationCondition(); 
        }

        // Get the path object and swap it with the full trajectory
        og::PathGeometric &traj_wpt_planning = simple_setup_->getSolutionPath();
        std::swap(traj_wpt_planning.getStates(), result_wpt_planning_);

        ROS_WARN("Planning done. Press ENTER to visualize and post process or export.");
        std::cin.get();

        // If we only want to test the neural network, we don't use the export time step in this case
        if(test_NN_)
        {
            // Interpolate and display the solution with the same time step (dt) used by the GRU
            interpolateSolution(dt_planning_, index_wpt_in_traj_);
            ROS_DEBUG("Returning successful solution with %lu states", simple_setup_->getSolutionPath().getStateCount());
            // Compute the tubes with ODEs and GRU and export them
            testNN_and_export(simple_setup_->getSolutionPath());
        }
        else
        {
            std::vector<ompl::base::State*> post_process_states;

            // Interpolate and display the solution 
            interpolateSolution(dt_planning_, index_wpt_in_traj_);

            if(simplify_)
            {
                postProcessor_.reset(new ompl::Smooth(simple_setup_->getSpaceInformation(), robot_, nh_));
                post_process_states = postProcessor_->postProcess(simple_setup_->getSolutionPath(), index_wpt_in_traj_);
                // Get the path object and swap it with the post processed trajectory
                std::swap(simple_setup_->getSolutionPath().getStates(), post_process_states); 
                // Visualization
                visual_tools_->removeAll();
                visual_tools_->addTraj(simple_setup_->getSolutionPath().getStates());
            }
            
            if(!post_process_)
            {
                // No post processing/optimization asked, we export the results
                exportSolutionPath(simple_setup_->getSolutionPath(), index_wpt_in_traj_);
                simulate_and_export(simple_setup_->getSolutionPath(), index_wpt_in_traj_);
            }
            else
            {   
                if(post_process_comparison_)
                {
                    // Store initial trajectory
                    og::PathGeometric init_traj = simple_setup_->getSolutionPath();
                    std::vector<int> init_wpt(index_wpt_in_traj_);
                    ROS_WARN("Performing post processing comparison ...");
                    
                    // Compare local optimization
                    // STOMP
                    postProcessor_.reset(new ompl::STOMP(simple_setup_->getSpaceInformation(), robot_, nh_));
                    // Post process
                    post_process_states = postProcessor_->postProcess(simple_setup_->getSolutionPath(), index_wpt_in_traj_);
                    // Get the path object and swap it with the post processed trajectory
                    std::swap(simple_setup_->getSolutionPath().getStates(), post_process_states);  
                    exportSolutionPath(simple_setup_->getSolutionPath(), index_wpt_in_traj_);
                    //simulate_and_export(simple_setup_->getSolutionPath(), index_wpt_in_traj_);
                    ROS_WARN("Post processing done and exported. Press ENTER to continue to next post processor.");
                    std::cin.get();

                    // ExtendedShortcut
                    // Reset
                    std::swap(simple_setup_->getSolutionPath().getStates(), init_traj.getStates());  
                    init_traj = simple_setup_->getSolutionPath();
                    index_wpt_in_traj_ = init_wpt;
                    postProcessor_.reset(new ompl::ExtendedShortcut(simple_setup_->getSpaceInformation(), robot_, nh_));
                    // Post process
                    post_process_states = postProcessor_->postProcess(simple_setup_->getSolutionPath(), index_wpt_in_traj_);
                    // Get the path object and swap it with the post processed trajectory
                    std::swap(simple_setup_->getSolutionPath().getStates(), post_process_states);  
                    exportSolutionPath(simple_setup_->getSolutionPath(), index_wpt_in_traj_);
                    //simulate_and_export(simple_setup_->getSolutionPath(), index_wpt_in_traj_);
                    ROS_WARN("Post processing done and exported. Press ENTER to continue to next post processor.");
                    std::cin.get();

                    // Change ball radius
                    nh_.setParam("post_processing_configs/ExtendedShortcut/ball_radius", 0.01);

                    // ExtendedShortcut
                    // Reset
                    std::swap(simple_setup_->getSolutionPath().getStates(), init_traj.getStates());  
                    init_traj = simple_setup_->getSolutionPath();
                    index_wpt_in_traj_ = init_wpt;
                    postProcessor_.reset(new ompl::ExtendedShortcut(simple_setup_->getSpaceInformation(), robot_, nh_));
                    // Post process
                    post_process_states = postProcessor_->postProcess(simple_setup_->getSolutionPath(), index_wpt_in_traj_);
                    // Get the path object and swap it with the post processed trajectory
                    std::swap(simple_setup_->getSolutionPath().getStates(), post_process_states);  
                    exportSolutionPath(simple_setup_->getSolutionPath(), index_wpt_in_traj_);
                    //simulate_and_export(simple_setup_->getSolutionPath(), index_wpt_in_traj_);
                    ROS_WARN("Post processing done and exported. Press ENTER to continue to next post processor.");
                    std::cin.get();

                    // Shortcut
                    // Reset
                    std::swap(simple_setup_->getSolutionPath().getStates(), init_traj.getStates());  
                    init_traj = simple_setup_->getSolutionPath();
                    index_wpt_in_traj_ = init_wpt;
                    postProcessor_.reset(new ompl::Shortcut(simple_setup_->getSpaceInformation(), robot_, nh_));
                    // Post process
                    post_process_states = postProcessor_->postProcess(simple_setup_->getSolutionPath(), index_wpt_in_traj_);
                    // Get the path object and swap it with the post processed trajectory
                    std::swap(simple_setup_->getSolutionPath().getStates(), post_process_states);  
                    exportSolutionPath(simple_setup_->getSolutionPath(), index_wpt_in_traj_);
                    //simulate_and_export(simple_setup_->getSolutionPath(), index_wpt_in_traj_);
                    ROS_WARN("Post processing done and exported. Press ENTER to continue to next post processor.");
                    std::cin.get();
                    
                    // Nlopt
                    // Reset
                    std::swap(simple_setup_->getSolutionPath().getStates(), init_traj.getStates());  
                    index_wpt_in_traj_ = init_wpt;
                    postProcessor_.reset(new ompl::NloptLocalopt(simple_setup_->getSpaceInformation(), robot_, nh_));
                    // Post process
                    post_process_states = postProcessor_->postProcess(simple_setup_->getSolutionPath(), index_wpt_in_traj_);
                    // Get the path object and swap it with the post processed trajectory
                    std::swap(simple_setup_->getSolutionPath().getStates(), post_process_states);  
                    exportSolutionPath(simple_setup_->getSolutionPath(), index_wpt_in_traj_);
                    //simulate_and_export(simple_setup_->getSolutionPath(), index_wpt_in_traj_);
                    ROS_WARN("Post processing done and exported. Press ENTER to continue to next post processor.");
                    std::cin.get();
                }
                else
                {
                    // Initialized post processor
                    initializePostProcessor();
                    ROS_WARN("Performing post processing...");
                    // Post process
                    post_process_states = postProcessor_->postProcess(simple_setup_->getSolutionPath(), index_wpt_in_traj_);
                    // Get the path object and swap it with the post processed trajectory
                    std::swap(simple_setup_->getSolutionPath().getStates(), post_process_states);  
                }
   
                ROS_WARN("Post processing done. Press ENTER to visualize and export.");
                std::cin.get();

                // Visualization
                visual_tools_->removeAll();
                visual_tools_->addTraj(simple_setup_->getSolutionPath().getStates());

                exportSolutionPath(simple_setup_->getSolutionPath(), index_wpt_in_traj_);
                simulate_and_export(simple_setup_->getSolutionPath(), index_wpt_in_traj_);
            }
        }
    }

    og::PathGeometric &pg = simple_setup_->getSolutionPath();
    ROS_DEBUG("Returning successful solution with %lu states", pg.getStateCount());
    
    return true;
}

double PlanningContext::simplifySolution(double max_time)
{
    simple_setup_->simplifySolution(max_time);

    return simple_setup_->getLastSimplificationTime();
}

void PlanningContext::registerTerminationCondition(const ompl::base::PlannerTerminationCondition &ptc)
{
    boost::mutex::scoped_lock slock(ptc_lock_);
    ptc_ = &ptc;
}

void PlanningContext::unregisterTerminationCondition()
{
    boost::mutex::scoped_lock slock(ptc_lock_);
    ptc_ = NULL;
}

bool PlanningContext::terminate()
{
    boost::mutex::scoped_lock slock(ptc_lock_);
    if (ptc_)
        ptc_->terminate();
    return true;
}

ompl::base::PlannerPtr PlanningContext::configurePlanner(const std::string& planner_name, const std::map<std::string, std::string>& params)
{
    std::map<std::string, PlannerAllocator>::const_iterator it = planner_allocators_.find("geometric::"+planner_name);
    // Allocating planner using planner allocator
    if (it != planner_allocators_.end())
        return it->second(simple_setup_->getSpaceInformation(), planner_name, params, visual_tools_, ros_namespace_);

    // No planner configured by this name
    ROS_WARN("No planner allocator found with name '%s'", planner_name.c_str());
    for(std::map<std::string, PlannerAllocator>::const_iterator it = planner_allocators_.begin(); it != planner_allocators_.end(); ++it)
        ROS_WARN("  %s", it->first.c_str());
    return ompl::base::PlannerPtr();
}

void PlanningContext::interpolateSolution(const double dt, std::vector<int>& index_wpt_in_traj)
{
    if (simple_setup_->haveSolutionPath())
    {
        // Clear the visualization if one exists
        visual_tools_->removeAll();

        og::PathGeometric &pg = simple_setup_->getSolutionPath();
        
        std::cout<<"\t Initial state count = "<<pg.getStateCount()<<std::endl;

        //// Build the geometric path at the desired time resolution
        // Create the new trajectory object
        std::vector<ompl::base::State*> new_trajectory;

        // To know at which desired accurate waypoint we are
        int compt_wpt = 0;
        
        for(size_t i=0; i<pg.getStateCount()-1; ++i)
        {          
            // Interpolate the trajectory between two desired state at a new resolution
            std::vector<ompl::base::State*> new_res_states;
            simple_setup_->getStateSpace()->interpolate(pg.getState(i), pg.getState(i+1), dt, new_res_states);

            std::cout<<"Interpolating..."<<std::endl;

            // We don't want to copy twice the last state which is equal to the previous first state
            for(size_t j=0; j<new_res_states.size()-1; j++)
            {
                new_trajectory.push_back(new_res_states[j]);
            }

            /** We check if the current final state (pg.getState(i+1)) is a waypoint for which we want to optimize the accuracy. 
             * If yes we update its index position in the new trajectory.
            */
            if(i+1 == index_wpt_in_traj.at(compt_wpt))
            {
                index_wpt_in_traj.at(compt_wpt) = new_trajectory.size();
                compt_wpt++;
            }

            if( i == pg.getStateCount()-2)
            {
                // Copy the last state
                new_trajectory.push_back(new_res_states.back());
            }
        }

        // We replace the current trajectory by the new one computed at the desired resolution
        std::swap(pg.getStates(), new_trajectory);

        // Visualization
        visual_tools_->addTraj(pg.getStates());

        std::cout<<"\t Final state count = "<<pg.getStateCount()<<std::endl;
    }
}

ompl::base::State* PlanningContext::getKinoStartState()
{
    ompl::base::State* st = simple_setup_->getSpaceInformation()->allocState();
    std::vector<double> q, qdot, qddot;
    for(int i = 0; i<robot_->getDesiredDim(); i++)
    {
        q.push_back(initial_state_.at(i));
        qdot.push_back(initial_state_.at(robot_->getDesiredDim()+i));
        qddot.push_back(initial_state_.at(2*robot_->getDesiredDim()+i));
    }
    st->as<ompl::base::KinosplineStateSpace::StateType>()->setQValues(q);
    st->as<ompl::base::KinosplineStateSpace::StateType>()->setQdotValues(qdot);
    st->as<ompl::base::KinosplineStateSpace::StateType>()->setQddotValues(qddot);

    // Set initial GRU hidden state
    st->as<ompl::base::KinosplineStateSpace::StateType>()->h0_ = torch::zeros({1,1,robot_->sensiNN_.getHiddenSize()});

    // Set robot initial nominal state
    /* Quadrotor nominal state: x, y, z, vx, vy, vz, qw, qx, qy, qz, wx, wy, wz */

    if(robot_->getName() == "quadrotor")
    {
        st->as<ompl::base::KinosplineStateSpace::StateType>()->nominal_state_.push_back(q.at(0)); // x
        st->as<ompl::base::KinosplineStateSpace::StateType>()->nominal_state_.push_back(q.at(1)); // y
        st->as<ompl::base::KinosplineStateSpace::StateType>()->nominal_state_.push_back(q.at(2)); // z
        st->as<ompl::base::KinosplineStateSpace::StateType>()->nominal_state_.push_back(qdot.at(0)); // vx
        st->as<ompl::base::KinosplineStateSpace::StateType>()->nominal_state_.push_back(qdot.at(1)); // vy
        st->as<ompl::base::KinosplineStateSpace::StateType>()->nominal_state_.push_back(qdot.at(2)); // vz

        Eigen::Quaterniond q_init = euler2Quaternion(0.0, 0.0, q.at(3));
        st->as<ompl::base::KinosplineStateSpace::StateType>()->nominal_state_.push_back(q_init.w()); // qw
        st->as<ompl::base::KinosplineStateSpace::StateType>()->nominal_state_.push_back(q_init.x()); // qx
        st->as<ompl::base::KinosplineStateSpace::StateType>()->nominal_state_.push_back(q_init.y()); // qy
        st->as<ompl::base::KinosplineStateSpace::StateType>()->nominal_state_.push_back(q_init.z()); // qz
        st->as<ompl::base::KinosplineStateSpace::StateType>()->nominal_state_.push_back(0.0); // wx
        st->as<ompl::base::KinosplineStateSpace::StateType>()->nominal_state_.push_back(0.0); // wy
        st->as<ompl::base::KinosplineStateSpace::StateType>()->nominal_state_.push_back(qdot.at(3)); // wz
    }
    else if (robot_->getName() == "unicycle")
    {
        st->as<ompl::base::KinosplineStateSpace::StateType>()->nominal_state_.push_back(q.at(0)); // x
        st->as<ompl::base::KinosplineStateSpace::StateType>()->nominal_state_.push_back(q.at(1)); // y
        st->as<ompl::base::KinosplineStateSpace::StateType>()->nominal_state_.push_back(qdot.at(0)); // vx
        st->as<ompl::base::KinosplineStateSpace::StateType>()->nominal_state_.push_back(qdot.at(1)); // vy

        Eigen::Quaterniond q_init = euler2Quaternion(0.0, 0.0, 0.0);
        st->as<ompl::base::KinosplineStateSpace::StateType>()->nominal_state_.push_back(q_init.w()); // qw
        st->as<ompl::base::KinosplineStateSpace::StateType>()->nominal_state_.push_back(q_init.x()); // qx
        st->as<ompl::base::KinosplineStateSpace::StateType>()->nominal_state_.push_back(q_init.y()); // qy
        st->as<ompl::base::KinosplineStateSpace::StateType>()->nominal_state_.push_back(q_init.z()); // qz
        st->as<ompl::base::KinosplineStateSpace::StateType>()->nominal_state_.push_back(0.0); // wz
    }
    
    // Set robot initial PI and PI_xi
    // Nb robot states used in sensitivity computation (q nominal) * Nb parameters 
    for(int i = 0; i<robot_->getNominalDim()*robot_->getUncertainParams().size(); i++)
        st->as<ompl::base::KinosplineStateSpace::StateType>()->PI_.push_back(0.0);
    // Nb control states used in sensitivity computation (integral terms) * Nb parameters 
    for(int i = 0; i<3*robot_->getUncertainParams().size(); i++)
        st->as<ompl::base::KinosplineStateSpace::StateType>()->PI_xi_.push_back(0.0);

    st->as<ompl::base::KinosplineStateSpace::StateType>()->st_time_ = 0.0;

    return st;
}

ompl::base::State* PlanningContext::getDubinsStartState()
{
    ompl::base::State* st = simple_setup_->getSpaceInformation()->allocState();
    if(robot_->getName() == "quadrotor")
    {
        ROS_ERROR("Quadrotor robot can't use Dubins trajectory representation !!");
    }
    else if (robot_->getName() == "unicycle")
    {
        std::vector<double> q;
        q.push_back(initial_state_.at(0)); // x
        q.push_back(initial_state_.at(1)); // y
        q.push_back(initial_state_.at(2)); // yaw

        st->as<ompl::base::DubinsUnicycleSpace::StateType>()->setXY(q.at(0), q.at(1));
        st->as<ompl::base::DubinsUnicycleSpace::StateType>()->setYaw(q.at(2));

        // Set initial GRU hidden state
        st->as<ompl::base::DubinsUnicycleSpace::StateType>()->h0_ = torch::zeros({1,1,robot_->sensiNN_.getHiddenSize()});

        // Set robot initial nominal state
        /* Quadrotor nominal state: x, y, z, vx, vy, vz, qw, qx, qy, qz, wx, wy, wz */
        st->as<ompl::base::DubinsUnicycleSpace::StateType>()->nominal_state_.push_back(q.at(0)); // x
        st->as<ompl::base::DubinsUnicycleSpace::StateType>()->nominal_state_.push_back(q.at(1)); // y
        st->as<ompl::base::DubinsUnicycleSpace::StateType>()->nominal_state_.push_back(0.0); // vx
        st->as<ompl::base::DubinsUnicycleSpace::StateType>()->nominal_state_.push_back(0.0); // vy
   
        Eigen::Quaterniond q_init = euler2Quaternion(0.0, 0.0, q.at(2));
        st->as<ompl::base::DubinsUnicycleSpace::StateType>()->nominal_state_.push_back(q_init.w()); // qw
        st->as<ompl::base::DubinsUnicycleSpace::StateType>()->nominal_state_.push_back(q_init.x()); // qx
        st->as<ompl::base::DubinsUnicycleSpace::StateType>()->nominal_state_.push_back(q_init.y()); // qy
        st->as<ompl::base::DubinsUnicycleSpace::StateType>()->nominal_state_.push_back(q_init.z()); // qz
        st->as<ompl::base::DubinsUnicycleSpace::StateType>()->nominal_state_.push_back(0.0); // yawdot
    }
    // Set robot initial PI and PI_xi
    // Nb robot states used in sensitivity computation (q nominal) * Nb parameters 
    for(int i = 0; i<robot_->getNominalDim()*robot_->getUncertainParams().size(); i++)
        st->as<ompl::base::DubinsUnicycleSpace::StateType>()->PI_.push_back(0.0);
    // Nb control states used in sensitivity computation (integral terms) * Nb parameters 
    for(int i = 0; i<3*robot_->getUncertainParams().size(); i++)
        st->as<ompl::base::DubinsUnicycleSpace::StateType>()->PI_xi_.push_back(0.0);

    st->as<ompl::base::DubinsUnicycleSpace::StateType>()->xi_.push_back(0.1);
    st->as<ompl::base::DubinsUnicycleSpace::StateType>()->xi_.push_back(0.0);
    st->as<ompl::base::DubinsUnicycleSpace::StateType>()->xi_.push_back(0.0);

    st->as<ompl::base::KinosplineStateSpace::StateType>()->st_time_ = 0.0;
    return st;
}

ompl::base::State* PlanningContext::getKinoGoalState(const std::vector<double>& goal)
{
    ompl::base::State* st = simple_setup_->getSpaceInformation()->allocState();
    std::vector<double> q, qdot, qddot;
    for(int i = 0; i<robot_->getDesiredDim(); i++)
    {
        q.push_back(goal.at(i));
        qdot.push_back(goal.at(robot_->getDesiredDim()+i));
        qddot.push_back(goal.at(2*robot_->getDesiredDim()+i));
    }

    // If we want to plan for a desired eef state, convert this one into the platform space, in a rigid body angular-based states are not modified
    if(eef_planning_)
    {
        Eigen::Rotation2Df rot2(q.at(3));
        Eigen::Vector2f relative_speed_d(robot_->as<ompl::Quadrotor>()->getPerchVel(),0.0);
        Eigen::Vector2f global_speed_d = rot2*relative_speed_d;
        if(abs(global_speed_d[0])<1e-2)
            global_speed_d[0] = 0.0;
        if(abs(global_speed_d[1])<1e-2)
            global_speed_d[1] = 0.0;
        qdot[0] = global_speed_d[0];
        qdot[1] = global_speed_d[1];

        q.at(3) = q.at(3)-robot_->as<ompl::Quadrotor>()->getPerchAngle(); // desired yaw of the platform to have the perche align with the ring
        simple_setup_->getStateSpace()->as<ompl::base::KinosplineStateSpace>()->getPlatformFromEEF(q, qddot);
    }

    st->as<ompl::base::KinosplineStateSpace::StateType>()->setQValues(q);
    st->as<ompl::base::KinosplineStateSpace::StateType>()->setQdotValues(qdot);
    st->as<ompl::base::KinosplineStateSpace::StateType>()->setQddotValues(qddot);

    return st;
}

ompl::base::State* PlanningContext::getDubinsGoalState(const std::vector<double>& goal)
{
    ompl::base::State* st = simple_setup_->getSpaceInformation()->allocState();
    if(robot_->getName() == "quadrotor")
    {
        ROS_ERROR("Quadrotor robot can't use Dubins trajectory representation !!");
    }
    else if (robot_->getName() == "unicycle")
    {
        std::vector<double> q;
        q.push_back(goal.at(0)); // x
        q.push_back(goal.at(1)); // y
        q.push_back(goal.at(2)); // yaw

        st->as<ompl::base::DubinsUnicycleSpace::StateType>()->setXY(q.at(0), q.at(1));
        st->as<ompl::base::DubinsUnicycleSpace::StateType>()->setYaw(q.at(2));
    }   

    return st;
}

void PlanningContext::readTrajFile(const std::string &filename, std::vector<ompl::base::State*> &refTraj, std::vector<int> &wpt_idx)
{
    std::ifstream file(filename);
    std::string line;

    std::vector<double> kX, kV, kI, kR, kOmega;

    bool reading_ref = false;
    bool reading_uncer = false;
    bool reading_simu = false;
    bool reading_idx = false;
    
    while (std::getline(file, line))
    {
        if (line == "Reference ")
        {
            reading_ref = true;
            reading_uncer = false;
            reading_simu = false;
            reading_idx = false;
            continue;
        }
        else if (line == "True tubes ")
        {
            reading_ref = false;
            reading_uncer = true;
            reading_simu = false;
            reading_idx = false;
            continue;
        }
        else if (line == "Index Waypoints ")
        {
            reading_ref = false;
            reading_uncer = false;
            reading_simu = false;
            reading_idx = true;
            continue;
        }
        else if (line == "True inputs ")
        {
            reading_ref = false;
            reading_uncer = false;
            reading_simu = false;
            reading_idx = false;
            continue;
        }
        else if (line == "Nominal trajectory ")
        {
            reading_ref = false;
            reading_uncer = false;
            reading_simu = true;
            reading_idx = false;
            continue;
        }

        if(reading_ref)
        {
            std::vector<double> state_val, pos, vel, acc;
            //Split the line with ;  separator
            std::istringstream linestream(line);
            std::string token;
            int compt_elem = 0;
            while (std::getline(linestream, token, ';'))
            {
                if(compt_elem == 0)
                {
                    compt_elem++;
                    continue;
                }
                state_val.push_back(std::stod(token));
            }

            ompl::base::State* st = simple_setup_->getSpaceInformation()->allocState();

            for(int i = 0; i<4; i++)
                pos.push_back(state_val.at(i));
            for(int i = 4; i<8; i++)
                vel.push_back(state_val.at(i));
            for(int i = 8; i<12; i++)
                acc.push_back(state_val.at(i));

            st->as<ompl::base::KinosplineStateSpace::StateType>()->setQValues(pos);
            st->as<ompl::base::KinosplineStateSpace::StateType>()->setQdotValues(vel);
            st->as<ompl::base::KinosplineStateSpace::StateType>()->setQddotValues(acc);
            
            refTraj.push_back(st);
        }
        if(reading_uncer)
            continue;
        if(reading_simu)
            continue;
        if(reading_idx)
        {
            //Split the line with ;  separator
            std::istringstream linestream(line);
            std::string token;
            int compt_elem = 0;
            while (std::getline(linestream, token, ';'))
            {
                if(compt_elem == 0)
                {
                    compt_elem++;
                    continue;
                }
                wpt_idx.push_back(std::stoi(token));
            }
        }
    }
}

void PlanningContext::readGainFile(const std::string &filename, std::vector<std::vector<double>>& gains)
{
    std::ifstream file(filename);
    std::string line;

    std::vector<double> kX, kV, kI, kR, kOmega;

    bool reading_kX = false;
    bool reading_kV = false;
    bool reading_kI = false;
    bool reading_kR = false;
    bool reading_kOmega = false;
    bool reading_cost = false;
    
    while (std::getline(file, line))
    {
    if (line.find("Gains")!= std::string::npos)
    {
        kX.clear();
        kV.clear();
        kI.clear();
        kR.clear();
        kOmega.clear();
        continue;
    }
    else if (line == "Vector number : 0")
    {
        reading_kX = true;
        reading_kV = false;
        reading_kI = false;
        reading_kR = false;
        reading_kOmega = false;
        reading_cost = false;
        continue;
    }
    else if (line == "Vector number : 1")
    {
        reading_kX = false;
        reading_kV = true;
        reading_kI = false;
        reading_kR = false;
        reading_kOmega = false;
        reading_cost = false;
        continue;
    }
    else if (line == "Vector number : 2")
    {
        reading_kX = false;
        reading_kV = false;
        reading_kI = true;
        reading_kR = false;
        reading_kOmega = false;
        reading_cost = false;
        continue;
    }
    else if (line == "Vector number : 3")
    {
        reading_kX = false;
        reading_kV = false;
        reading_kI = false;
        reading_kR = true;
        reading_kOmega = false;
        reading_cost = false;
        continue;
    }
    else if (line == "Vector number : 4")
    {
        reading_kX = false;
        reading_kV = false;
        reading_kI = false;
        reading_kR = false;
        reading_kOmega = true;
        reading_cost = false;
        continue;
    }
    else if (line == "Cost ")
    {
        reading_kX = false;
        reading_kV = false;
        reading_kI = false;
        reading_kR = false;
        reading_kOmega = false;
        reading_cost = true;
        continue;
    }

    if(reading_cost)
        continue;
    if(reading_kX)
        kX.push_back(std::stod(line));
    if(reading_kV)
        kV.push_back(std::stod(line));
    if(reading_kI)
        kI.push_back(std::stod(line));
    if(reading_kR)
        kR.push_back(std::stod(line));
    if(reading_kOmega)
        kOmega.push_back(std::stod(line));
    }
    gains.push_back(kX);
    gains.push_back(kV);
    gains.push_back(kI);
    gains.push_back(kR);
    gains.push_back(kOmega);
}

void PlanningContext::exportSolutionPath(og::PathGeometric &pg, std::vector<int>& index_wpt_in_traj_)
{
    // Get the number of trajectory states
    std::size_t traj_count = pg.getStateCount();

    // Open the file
    std::ofstream trajectory_generated(traj_file_, std::ios::app);
    std::cout.precision(6);

    // Get the reference trajectory (i.e. the desired one) and export it
    trajectory_generated<<"Reference \n";
    if(robot_->getTrajType() == "Dubins")
    {
        /* Compute velocities and acceleration by differentation */
        std::vector<double> des_x, des_y, times;
        for(int i = 0; i<traj_count; i++)
        {
            des_x.push_back(pg.getState(i)->as<ompl::base::DubinsUnicycleSpace::StateType>()->getX());
            des_y.push_back(pg.getState(i)->as<ompl::base::DubinsUnicycleSpace::StateType>()->getY());
            times.push_back(pg.getState(i)->as<ompl::base::DubinsUnicycleSpace::StateType>()->st_time_);
        }

        std::vector<double> des_vx(des_x.size()), des_vy(des_x.size()), des_ax(des_x.size()), des_ay(des_x.size());

        // Finite difference
        des_vx[0] = (des_x[0] - pg.getState(0)->as<ompl::base::DubinsUnicycleSpace::StateType>()->getPrevX()) / (pg.getState(0)->as<ompl::base::DubinsUnicycleSpace::StateType>()->st_time_ - pg.getState(0)->as<ompl::base::DubinsUnicycleSpace::StateType>()->getPrevTime());
        des_vy[0] = (des_y[0] - pg.getState(0)->as<ompl::base::DubinsUnicycleSpace::StateType>()->getPrevY()) / (pg.getState(0)->as<ompl::base::DubinsUnicycleSpace::StateType>()->st_time_ - pg.getState(0)->as<ompl::base::DubinsUnicycleSpace::StateType>()->getPrevTime());

        // Main loop
        for (size_t i = 1; i < des_x.size(); ++i) {
            des_vx[i] = (des_x[i] - des_x[i - 1]) / (times[i] - times[i - 1]);
            des_vy[i] = (des_y[i] - des_y[i - 1]) / (times[i] - times[i - 1]);
        }

        // std::cout << "PrevX : " << des_states.at(0)->as<StateType>()->getPrevX() << std::endl;

        for(std::size_t i=0; i< traj_count; ++i)
        {
            trajectory_generated<<";"<< pg.getState(i)->as<ompl::base::DubinsUnicycleSpace::StateType>()->getX();
            trajectory_generated<<";"<< pg.getState(i)->as<ompl::base::DubinsUnicycleSpace::StateType>()->getY();
            trajectory_generated<<";"<< pg.getState(i)->as<ompl::base::DubinsUnicycleSpace::StateType>()->getYaw();
            trajectory_generated<<";"<< des_vx.at(i);
            trajectory_generated<<";"<< des_vy.at(i);
            trajectory_generated<<"\n"; 
        }
    }
    else if(robot_->getTrajType() == "Kinospline")
    {
        for(std::size_t i=0; i< traj_count; ++i)
        {
            for(std::size_t j=0; j<robot_->getDesiredDim(); ++j)
            {
                trajectory_generated<<";"<< pg.getState(i)->as<ompl::base::KinosplineStateSpace::StateType>()->getQValues()[j];
            }
            for(std::size_t j=0; j<robot_->getDesiredDim(); ++j)
            {
                trajectory_generated<<";"<< pg.getState(i)->as<ompl::base::KinosplineStateSpace::StateType>()->getQdotValues()[j];
            }
            for(std::size_t j=0; j<robot_->getDesiredDim(); ++j)
            {
                trajectory_generated<<";"<< pg.getState(i)->as<ompl::base::KinosplineStateSpace::StateType>()->getQddotValues()[j];
            }
            trajectory_generated<<"\n"; 
        }
    }

    trajectory_generated<<"TimeVec \n";
    for(std::size_t i=0; i< traj_count; ++i)
    {
        trajectory_generated<<";"<< pg.getState(i)->as<ompl::base::RobustStateSpace::StateType>()->st_time_;
        trajectory_generated<<"\n"; 
    }

    // Get the nominal trajectory (i.e. the one after simulation under nominal parameters) and export it
    trajectory_generated<<"Nominal trajectory \n";
    std::vector<std::vector<double>> simu_states, control_inputs, radii;
    if(simple_setup_->getStateSpace()->simulateStatesAndTubes(pg.getStates(), dt_planning_, simu_states, control_inputs, radii))
    {
        for(size_t i=0; i< simu_states.size(); i++)
        {
            for(size_t j = 0; j<simu_states.at(0).size(); j++)
            {
                trajectory_generated<<";"<< simu_states.at(i).at(j); // q_ij
            }
            trajectory_generated<<"\n";
        }

        trajectory_generated<<"True tubes \n";
        for(size_t i=0; i< radii.at(0).size(); i++)
        {
            for(size_t j = 0; j<radii.size(); j++)
            {
                trajectory_generated<<";"<< radii.at(j).at(i); // Rq/u_ij
            }
            trajectory_generated<<"\n";
        }

        trajectory_generated<<"True inputs \n";
        for(size_t i=0; i< control_inputs.at(0).size(); i++)
        {
            for(size_t j = 0; j<control_inputs.size(); j++)
            {
                trajectory_generated<<";"<< control_inputs.at(j).at(i); //U
            }
            trajectory_generated<<"\n";
        }
    }

    trajectory_generated<<"Index Waypoints \n";
    for(size_t i=0; i< index_wpt_in_traj_.size(); i++)
    {
        trajectory_generated<<";"<< index_wpt_in_traj_.at(i) << "\n";
    }

    trajectory_generated.close();
}

void PlanningContext::simulate_and_export(og::PathGeometric &pg, std::vector<int>& index_wpt_in_traj_)
{
    std::filesystem::path fullPath = std::filesystem::absolute(campDir_ / "src/results/simulated_traj.txt");
    fullPath = fullPath.lexically_normal();
    std::string file_name = fullPath.string();

    // To clear the file 
    std::ofstream ofs;
    ofs.open(file_name, std::ofstream::out | std::ofstream::trunc);
    ofs.close();

    std::size_t traj_count = pg.getStateCount();
    std::ofstream trajectory_generated(file_name, std::ios::app);
    std::cout.precision(6);

    // Get the reference trajectory (i.e. the desired one) and export it
    trajectory_generated<<"Reference \n";
    for(std::size_t i=0; i< traj_count; ++i)
    {
        if(robot_->getTrajType() == "Kinospline")
        {
            for(std::size_t j=0; j<robot_->getDesiredDim(); ++j)
            {
                trajectory_generated<<";"<< pg.getState(i)->as<ompl::base::KinosplineStateSpace::StateType>()->getQValues()[j];
            }
            for(std::size_t j=0; j<robot_->getDesiredDim(); ++j)
            {
                trajectory_generated<<";"<< pg.getState(i)->as<ompl::base::KinosplineStateSpace::StateType>()->getQdotValues()[j];
            }
            for(std::size_t j=0; j<robot_->getDesiredDim(); ++j)
            {
                trajectory_generated<<";"<< pg.getState(i)->as<ompl::base::KinosplineStateSpace::StateType>()->getQddotValues()[j];
            }
        }
        else if(robot_->getTrajType() == "Dubins")
        {
            trajectory_generated<<";"<< pg.getState(i)->as<ompl::base::DubinsUnicycleSpace::StateType>()->getX();
            trajectory_generated<<";"<< pg.getState(i)->as<ompl::base::DubinsUnicycleSpace::StateType>()->getY();
            trajectory_generated<<";"<< pg.getState(i)->as<ompl::base::DubinsUnicycleSpace::StateType>()->getYaw();
        }
        trajectory_generated<<"\n";
    }

    trajectory_generated<<"TimeVec \n";
    for(std::size_t i=0; i< traj_count; ++i)
    {
        trajectory_generated<<";"<< pg.getState(i)->as<ompl::base::RobustStateSpace::StateType>()->st_time_;
        trajectory_generated<<"\n"; 
    }

    // Get the nominal trajectory (i.e. the one after simulation under nominal parameters) and export it
    trajectory_generated<<"Nominal trajectory \n";
    std::vector<std::vector<double>> nom_states;
    if(simple_setup_->getStateSpace()->simulateStates(pg.getStates(), dt_planning_, nom_states))
    {
        for(size_t i=0; i< nom_states.size(); i++)
        {
            for(size_t j = 0; j<nom_states.at(0).size(); j++)
            {
                trajectory_generated<<";"<< nom_states.at(i).at(j); // q_ij
            }
            trajectory_generated<<"\n";
        }
    }

    // Recover the nominal parameters
    std::vector<double> nominal_case = robot_->getUncertainParams(); // Nominal parameters 

    // Get the range delta_p for the uncertain parameters
    std::vector<double> delta_p = robot_->getUncertaintyRange();

    for(int l = 0; l<nb_simu_; l++)
    {
        bool valid = true;
        trajectory_generated<<"Trajectory " << l << "\n";

        // Sample random uncertainty aroung the nominal case and project in the ellipsoid if the sampled point is outside
        double dev_lim = 0.9; // sample in the ellipsoid and not in the hypercube 90% seams to be a good value, otherwise see Andreas's work with ellipsoids superquadriq
        std::vector<double> rand_params, semi_axes;
        for(int j = 0; j<delta_p.size(); j++)
        {
            if (nominal_case.at(j) == 0.0)
            {
                semi_axes.push_back(delta_p.at(j));
                rand_params.push_back(rng_.uniformReal(-delta_p.at(j)*dev_lim, delta_p.at(j)*dev_lim));
            }  
            else
            {
                semi_axes.push_back(nominal_case.at(j)*delta_p.at(j));
                rand_params.push_back(nominal_case.at(j)*(1+rng_.uniformReal(-delta_p.at(j)*dev_lim, delta_p.at(j)*dev_lim)));
            }      
        }

        // Check if the params are within the ellipsoid and get the projection otherwise
        rand_params = checkAndProject(rand_params, nominal_case, semi_axes); 

        // Set the uncertain parameters
        robot_->setUncertainParams(rand_params);

        // Simulate the robot dynamic with the uncertain parameters
        std::vector<std::vector<double>> simu_states, control_inputs, radii;
        if(simple_setup_->getStateSpace()->simulateStatesAndTubes(pg.getStates(), dt_planning_, simu_states, control_inputs, radii))
        {
            for(size_t i=0; i< simu_states.size(); i++)
            {
                for(size_t j = 0; j<simu_states.at(0).size(); j++)
                {
                    trajectory_generated<<";"<< simu_states.at(i).at(j); // q_ij
                }
                trajectory_generated<<"\n";
            }
        }

        // Check if the simulated trajectory is valid
        std::vector<double> fake_tube{}; // Fake the uncertainty tubes
        for(int i=0; i< simu_states.size(); i++)
        {
            if(!simple_setup_->getSpaceInformation()->isStateValid(simu_states[i], fake_tube))
            {
                valid = false;
                break;
            }
        }
        trajectory_generated<<"Validity;" << valid << "\n";
    }

    trajectory_generated<<"Index Waypoints \n";
    for(size_t i=0; i< index_wpt_in_traj_.size(); i++)
    {
        trajectory_generated<<";"<< index_wpt_in_traj_.at(i) << "\n";
    }

    trajectory_generated.close();
}

void PlanningContext::testNN_and_export(og::PathGeometric &pg)
{
    // Get the number of trajectory states
    std::size_t traj_count = pg.getStateCount();

    // Open the file
    std::ofstream trajectory_generated(traj_file_, std::ios::app);
    std::cout.precision(6);

    // Get the reference trajectory (i.e. the desired one) and export it
    trajectory_generated<<"Reference \n";
    for(std::size_t i=0; i< traj_count; ++i)
    {
        for(std::size_t j=0; j<robot_->getDesiredDim(); ++j)
        {
            trajectory_generated<<";"<< pg.getState(i)->as<ompl::base::KinosplineStateSpace::StateType>()->getQValues()[j];
        }
        for(std::size_t j=0; j<robot_->getDesiredDim(); ++j)
        {
            trajectory_generated<<";"<< pg.getState(i)->as<ompl::base::KinosplineStateSpace::StateType>()->getQdotValues()[j];
        }
        for(std::size_t j=0; j<robot_->getDesiredDim(); ++j)
        {
            trajectory_generated<<";"<< pg.getState(i)->as<ompl::base::KinosplineStateSpace::StateType>()->getQddotValues()[j];
        }
        trajectory_generated<<"\n"; 
    }

    trajectory_generated<<"TimeVec \n";
    for(std::size_t i=0; i< traj_count; ++i)
    {
        trajectory_generated<<";"<< pg.getState(i)->as<ompl::base::RobustStateSpace::StateType>()->st_time_;
        trajectory_generated<<"\n"; 
    }

    // Get the nominal trajectory (i.e. the one after simulation under nominal parameters) and export it
    trajectory_generated<<"Nominal trajectory \n";
    std::vector<std::vector<double>> simu_states, control_inputs, radii;
    if(simple_setup_->getStateSpace()->simulateStatesAndTubes(pg.getStates(), dt_planning_, simu_states, control_inputs, radii))
    {
        for(size_t i=0; i< simu_states.size(); i++)
        {
            for(size_t j = 0; j<simu_states.at(0).size(); j++)
            {
                trajectory_generated<<";"<< simu_states.at(i).at(j); // q_ij
            }
            trajectory_generated<<"\n";
        }

        trajectory_generated<<"True tubes \n";
        for(size_t i=0; i< radii.at(0).size(); i++)
        {
            for(size_t j = 0; j<radii.size(); j++)
            {
                trajectory_generated<<";"<< radii.at(j).at(i); // Rq/u_ij
            }
            trajectory_generated<<"\n";
        }

        trajectory_generated<<"True inputs \n";
        for(size_t i=0; i< control_inputs.at(0).size(); i++)
        {
            for(size_t j = 0; j<control_inputs.size(); j++)
            {
                trajectory_generated<<";"<< control_inputs.at(j).at(i); //U
            }
            trajectory_generated<<"\n";
        }
    }
    
    // Get the predicted tubes and export them
    trajectory_generated<<"Predicted tubes \n";

    // Create a tensor in the format bacth sequence features
    std::vector<std::vector<float>> vector_to_tensor_tubes;
    for(int i = 0; i<pg.getStates().size(); i++)
    {
        std::vector<double> qdot = pg.getStates().at(i)->as<ompl::base::KinosplineStateSpace::StateType>()->getQdotValues(); 
        std::vector<double> qddot = pg.getStates().at(i)->as<ompl::base::KinosplineStateSpace::StateType>()->getQdotValues(); 

        std::vector<std::vector<double>> des_learning;
        des_learning.push_back(qdot);
        des_learning.push_back(qddot);

        vector_to_tensor_tubes.push_back(robot_->getRobotLearningInput(des_learning));
    }

    std::vector<torch::jit::IValue> inputs;

    auto options = torch::TensorOptions().dtype(at::kFloat);
    torch::Tensor x_tubes = torch::zeros({1,vector_to_tensor_tubes.size(),vector_to_tensor_tubes[0].size()}, options);
    for (int i = 0; i < vector_to_tensor_tubes.size(); i++)
        x_tubes[0].slice(0, i,i+1) = torch::from_blob(vector_to_tensor_tubes[i].data(), {vector_to_tensor_tubes[0].size()}, options);

    // Initialize the inputs
    torch::Tensor h0 = pg.getStates().at(0)->as<ompl::base::RobustStateSpace::StateType>()->h0_;
    inputs.push_back(x_tubes);
    inputs.push_back(h0);

    auto start_pred = std::chrono::high_resolution_clock::now();

    c10::intrusive_ptr<c10::ivalue::Tuple> tubes_prediction = robot_->sensiNN_.predict(inputs);

    auto end_pred = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_pred - start_pred);
    std::cout << duration.count() << std::endl;

    torch::Tensor tubes_pred = tubes_prediction->elements()[0].toTensor();

    for(size_t i=0; i< simu_states.size(); i++)
    {
        for(size_t j = 0; j < robot_->sensiNN_.getOutSize(); j++)
        {
            trajectory_generated<<";"<< (tubes_pred[0][i][j].item().toFloat())*robot_->sensiNN_.getStdOutTubes(j)+robot_->sensiNN_.getMeanOutTubes(j)+robot_->sensiNN_.getQuartileTubes(j); 
        }
        trajectory_generated<<"\n";
    }
}
