#ifndef ACCURACY_OPTIMIZATION_TASK_H_
#define ACCURACY_OPTIMIZATION_TASK_H_

#include "planning/post_processing/stomp/task.h"
#include "planning/post_processing/stomp/multivariate_gaussian.hpp"

// Robots
#include "robots/Robot.h"

double EPSILON_KINO = 1e-4; // Epsilon for acceleration and velocity check

using namespace stomp;

class AccuracyOptimizationTask : public stomp::Task
{
    public:
        /**
         * @brief A simple task for demonstrating how to use Stomp
         * @param parameters_bias default parameter bias used for computing cost.
         * @param std_dev standard deviation used for generating noisy parameters
         */
        AccuracyOptimizationTask(const Eigen::MatrixXd& initial_traj,
                                const Eigen::MatrixXd& times,
                                const std::vector<double>& std_dev,
                                std::string cost_id,
                                std::string post_process_file,
                                std::string stopping_condition,
                                int max_iter, 
                                int window_size,
                                double tolerance,
                                double max_time,
                                double dt,
                                ompl::base::SpaceInformation *si,
                                const ompl::RobotPtr robot)
            : Task(max_iter, window_size, tolerance, max_time, stopping_condition), 
            initial_traj_(initial_traj), times_(times), std_dev_(std_dev), cost_id_(cost_id), post_process_file_(post_process_file), dt_(dt), si_(si), robot_(robot)
        {
            // generate smoothing matrix
            int num_timesteps = initial_traj.cols();
            stomp::generateSmoothingMatrix(num_timesteps, dt_, smoothing_M_);

            // Set fixed start and end states
            int last_row = smoothing_M_.rows() - 1;
            int last_col = smoothing_M_.cols() - 1;
            smoothing_M_.row(0).setZero();
            smoothing_M_.row(last_row).setZero();
            smoothing_M_(0,0) = 1.0;
            smoothing_M_(last_row,last_col) = 1.0;

            srand(time(0));
        }

        /**
         * @brief Convert an Eigen::MatrixXd to a OMPL vector format
         * @param traj The optimizd trajectory
         * @param lambda The vector of lambda max
         * @return Formated OMPL representing data
         */
        double acc_cost(const std::vector<double> &lambda) const
        {
            // Uncertainty based cost
            double w1 = 0.5;
            double w2 = 0.5;
            std::vector<double> L; // The vector to store all the pnorm at the desired states in the trajectory

            L.push_back(lambda.back());

            // First element of the cost function link to the eigenvalues of the projector (mean of the lambda max for the geometric output (x y z r p y))
            double mean = 0.;
            for(int t = 0; t<L.size(); t++)
            {
                mean += L.at(t);
            }
            mean = mean/L.size();

            double var = 0.;
            for(int t = 0; t<L.size(); t++)
            {
                var += std::pow((L.at(t)-mean),2);
            }
            var = var/L.size();

            return w1*mean + w2*var;
        }

        /**
         * @brief Convert an Eigen::MatrixXd to a OMPL vector format
         * @param data The item to be converted
         * @return Formated OMPL representing data
         */
        std::vector<ompl::base::State*> toOMPL(const Eigen::MatrixXd& parameters)
        {
            std::vector<ompl::base::State*> possible_stomp;

            // Compute velocity and acceleration for each dimension
            std::vector<Eigen::VectorXd> parameters_vec, parameters_vel, parameters_acc;
            toVector(parameters, parameters_vec); // Convert the trajectory from matrix format to vector format
            parameters_vel = parameters_vec;
            parameters_acc = parameters_vec;

            for (size_t i = 0; i < parameters.rows(); i++)
            {
                stomp::custom_differentiate(parameters_vec.at(i), times_, stomp::DerivativeOrders::STOMP_VELOCITY, dt_, parameters_vel.at(i));
                stomp::custom_differentiate(parameters_vec.at(i), times_, stomp::DerivativeOrders::STOMP_ACCELERATION, dt_, parameters_acc.at(i));
            }

            for(int i = 0; i<parameters.row(0).cols(); i++)
            {
                ompl::base::State* st = si_->getStateSpace()->allocState();
                std::vector<double> q, qdot, qddot;

                for (size_t j = 0; j < parameters.rows(); j++)
                {
                    q.push_back(parameters(j, i));
                    qdot.push_back(parameters_vel.at(j)[i]);
                    qddot.push_back(parameters_acc.at(j)[i]);
                }

                st->as<ompl::base::KinosplineStateSpace::StateType>()->setQValues(q); 
                st->as<ompl::base::KinosplineStateSpace::StateType>()->setQdotValues(qdot); 
                st->as<ompl::base::KinosplineStateSpace::StateType>()->setQddotValues(qddot); 

                if(i == 0)
                {
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
                }
                possible_stomp.push_back(st);
            }

            // Set initial GRU hidden state
            possible_stomp.at(0)->as<ompl::base::KinosplineStateSpace::StateType>()->h0_ = torch::zeros({1,1,robot_->sensiNN_.getHiddenSize()});

            // Set robot initial PI and PI_xi
            // Nb robot states used in sensitivity computation (q nominal) * Nb parameters 
            for(int i = 0; i<robot_->getNominalDim()*robot_->getUncertainParams().size(); i++)
                possible_stomp.at(0)->as<ompl::base::KinosplineStateSpace::StateType>()->PI_.push_back(0.0);
            // Nb control states used in sensitivity computation (integral terms) * Nb parameters 
            for(int i = 0; i<3*robot_->getUncertainParams().size(); i++)
                possible_stomp.at(0)->as<ompl::base::KinosplineStateSpace::StateType>()->PI_xi_.push_back(0.0);
            
            return possible_stomp;
        }

        /**
         * @brief Free a state vector
         * @param data The state vector
         */
        void freeStateVector(std::vector<ompl::base::State*> &states) const
        {
            for (auto &state :  states)
            {
                if (state != nullptr)
                si_->freeState(state);
            }
        }

        /**
         * @brief Generates a noisy trajectory from the parameters.
         * @param parameters        A matrix [num_dimensions][num_parameters] of the current optimized parameters
         * @param start_timestep    The start index into the 'parameters' array, usually 0.
         * @param num_timesteps     The number of elements to use from 'parameters' starting from 'start_timestep'
         * @param iteration_number  The current iteration count in the optimization loop
         * @param rollout_number    The index of the noisy trajectory.
         * @param parameters_noise  The parameters + noise
         * @param noise             The noise applied to the parameters
         * @return True if cost were properly computed, otherwise false
         */
        bool generateNoisyParameters(const Eigen::MatrixXd& parameters,
                                    std::size_t start_timestep,
                                    std::size_t num_timesteps,
                                    int iteration_number,
                                    int rollout_number,
                                    Eigen::MatrixXd& parameters_noise,
                                    Eigen::MatrixXd& noise) override
        {
            parameters_noise = Eigen::MatrixXd::Zero(num_timesteps, num_timesteps);

            // Five-point stencil constants
            static const std::vector<double> ACC_MATRIX_DIAGONAL_VALUES = { -1.0 / 12.0, 16.0 / 12.0, -30.0 / 12.0, 16.0 / 12.0,
                                                                            -1.0 / 12.0 };
            static const std::vector<int> ACC_MATRIX_DIAGONAL_INDICES = { -2, -1, 0, 1, 2 };

            auto fill_diagonal = [](Eigen::MatrixXd& m, double coeff, int diag_index) {
                std::size_t size = m.rows() - std::abs(diag_index);
                m.diagonal(diag_index) = Eigen::VectorXd::Constant(size, coeff);
            };

            // creating finite difference acceleration matrix
            Eigen::MatrixXd acceleration = Eigen::MatrixXd::Zero(num_timesteps, num_timesteps);
            for (auto i = 0u; i < ACC_MATRIX_DIAGONAL_INDICES.size(); i++)
            {
                fill_diagonal(acceleration, ACC_MATRIX_DIAGONAL_VALUES[i], ACC_MATRIX_DIAGONAL_INDICES[i]);
            }

            // create and scale covariance matrix
            Eigen::MatrixXd covariance = Eigen::MatrixXd::Identity(num_timesteps, num_timesteps);
            covariance = acceleration.transpose() * acceleration;
            covariance = covariance.fullPivLu().inverse();
            covariance /= covariance.array().abs().matrix().maxCoeff();

            // create random generators
            std::vector<math::MultivariateGaussianPtr> rand_generators(std_dev_.size());
            for (auto& r : rand_generators)
            {
                r = std::make_shared<math::MultivariateGaussian>(Eigen::VectorXd::Zero(num_timesteps), covariance);
            }

            auto raw_noise = std::make_shared<Eigen::VectorXd>(num_timesteps);

            for (int i = 0; i < parameters.rows(); ++i)
            {
                rand_generators[i]->sample(*raw_noise);
                raw_noise->head(1).setZero();
                raw_noise->tail(1).setZero();  // zeroing out the start and end noise values
                noise.row(i).transpose() = std_dev_.at(i) * (*raw_noise);
            }
            parameters_noise = parameters + noise;

            return true;
        }

        /**
         * @brief computes the state costs as a function of the distance from the bias parameters
         * @param parameters        A matrix [num_dimensions][num_parameters] of the policy parameters to execute
         * @param start_timestep    The start index into the 'parameters' array, usually 0.
         * @param num_timesteps     The number of elements to use from 'parameters' starting from 'start_timestep'
         * @param iteration_number  The current iteration count in the optimization loop
         * @param costs             A vector containing the state costs per timestep.
         * @param validity          Whether or not the trajectory is valid
         * @return True if cost were properly computed, otherwise false
         */
        bool computeCosts(const Eigen::MatrixXd& parameters,
                            std::size_t start_timestep,
                            std::size_t num_timesteps,
                            int iteration_number,
                            Eigen::VectorXd& costs,
                            bool& validity) override
        {
            return computeNoisyCosts(parameters, start_timestep, num_timesteps, iteration_number, -1, costs, validity);
        }

        /**
         * @brief computes the state costs as a function of the distance from the bias parameters
         * @param parameters        A matrix [num_dimensions][num_parameters] of the policy parameters to execute
         * @param start_timestep    The start index into the 'parameters' array, usually 0.
         * @param num_timesteps     The number of elements to use from 'parameters' starting from 'start_timestep'
         * @param iteration_number  The current iteration count in the optimization loop
         * @param rollout_number    The index of the noisy trajectory.
         * @param costs             A vector containing the state costs per timestep.
         * @param validity          Whether or not the trajectory is valid
         * @return True if cost were properly computed, otherwise false
         */
        bool computeNoisyCosts(const Eigen::MatrixXd& parameters,
                                std::size_t start_timestep,
                                std::size_t num_timesteps,
                                int iteration_number,
                                int rollout_number,
                                Eigen::VectorXd& costs,
                                bool& validity) override
        {
            costs.setZero(num_timesteps);
            double cost = 0.0;
            validity = true;

            // Convert the trajectory to the OMPL format
            std::vector<ompl::base::State*> possible_stomp = toOMPL(parameters);

            // Check the feasability of the whole noisy trajectory with tubes and compute its cost
            std::vector<std::vector<double>> collision_st, control_inputs_st, radii_st;
            if(!si_->getStateSpace()->simulateStatesAndTubes(possible_stomp, dt_, collision_st, control_inputs_st, radii_st))
            {
                std::cout << "Tubes computation failed in STOMP process ! Trajectory marked as invalid" << std::endl;
                validity = false;
                costs.setConstant(std::numeric_limits<double>::infinity());
                freeStateVector(possible_stomp);
                return true;
            }
            
            int rqsize = robot_->sensiNN_.getRqSize();
            int usize = robot_->sensiNN_.getUSize();
            std::vector<double> kino_limits = robot_->getKinodynamicLimits();
            for (std::size_t t = 0u; t < collision_st.size(); t++)
            {
                double st_cost = 0.0;
                
                // First add state cost according to the cost function
                if(cost_id_ == "Length")
                {
                    if(t > 0)
                        st_cost += si_->getStateSpace()->distance(possible_stomp.at(t-1), possible_stomp.at(t));
                }
                else if(cost_id_ == "Sensi")
                {
                    st_cost += radii_st.back().at(t);
                }
                else if (cost_id_ == "Accuracy")
                {
                    if(t == collision_st.size()-1)
                    {
                        st_cost += acc_cost(radii_st.back());
                    }
                }
                else
                {
                    std::cout << "STOMP cost function not implemented for this type of cost !" << std::endl;
                    validity = false;
                    return true;
                }

                std::vector<double> u, ru;
                for(int j = 0; j<usize; j++)
                {
                    u.push_back(control_inputs_st[j][t]);
                    ru.push_back(radii_st[rqsize+j][t]);
                }
                if(!si_->isInputsValid(u, ru))
                {
                    validity = false;
                    st_cost += 1.0; // Input infeasability penalty
                }
                
                //Check state collision with the scene
                std::vector<double> r_pos;
                for(int j = 0; j<rqsize; j++)
                {
                    r_pos.push_back(radii_st[j][t]);
                }
                if(!si_->isStateValid(collision_st[t], r_pos))
                {
                    validity =  false;
                    st_cost += 1.0; // Collision penalty
                }

                costs(t) = st_cost;
            }

            freeStateVector(possible_stomp);
            return true;
        }

        /**
         * @brief Filters the given parameters which is applied after the update. It could be used for clipping of joint
         * limits or projecting into the null space of the Jacobian.
         *
         * @param start_timestep    The start index into the 'parameters' array, usually 0.
         * @param num_timesteps     The number of elements to use from 'parameters' starting from 'start_timestep'
         * @param iteration_number  The current iteration count in the optimization loop
         * @param parameters        The optimized parameters
         * @param updates           The updates to the parameters
         * @return                  True if successful, otherwise false
         */
        bool filterParameterUpdates(std::size_t start_timestep,
                                    std::size_t num_timesteps,
                                    int iteration_number,
                                    const Eigen::MatrixXd& parameters,
                                    Eigen::MatrixXd& updates) override
        {
            return smoothParameterUpdates(start_timestep, num_timesteps, iteration_number, updates);
        }

        /** \brief Export the current best trajectory found with its cost */
        void exportShct(std::vector<ompl::base::State *>& traj, double cost, double plan_time) const
        {
            std::ofstream post_process_file(post_process_file_, std::ios::app);
            std::cout.precision(6);
            post_process_file << "Trajectory" << "\n";

            for(std::size_t i=0; i< traj.size(); ++i)
            {
                for(std::size_t j=0; j<robot_->getDesiredDim(); ++j)
                {
                    post_process_file<<";"<< traj.at(i)->as<ompl::base::KinosplineStateSpace::StateType>()->getQValues()[j];
                }
                for(std::size_t j=0; j<robot_->getDesiredDim(); ++j)
                {
                    post_process_file<<";"<< traj.at(i)->as<ompl::base::KinosplineStateSpace::StateType>()->getQdotValues()[j];
                }
                for(std::size_t j=0; j<robot_->getDesiredDim(); ++j)
                {
                    post_process_file<<";"<< traj.at(i)->as<ompl::base::KinosplineStateSpace::StateType>()->getQddotValues()[j];
                }
                post_process_file<<"\n"; 
            }
            post_process_file << "Cost" << "\n";
            post_process_file << cost << "\n";
            post_process_file << "Time" << "\n";
            post_process_file << plan_time << "\n";
            post_process_file.close();
        }

        /** \brief Cost convergence termination condition */
        bool costConvergence()
        {
            bool converged = false;

            // If the window exceeds the desired size, remove the oldest cost
            if (costsWindow_.size() > windowSize_)
            {
                costsWindow_.erase(costsWindow_.begin());
            }

            // Check for convergence only if we have enough data in the window
            if (costsWindow_.size() == windowSize_)
            {
                // Compute the maximum percentage variation between consecutive costs
                double max_percentage_variation = 0.0;
                for (std::size_t i = 1; i < costsWindow_.size(); ++i)
                {
                    double current_cost = costsWindow_[i];
                    double previous_cost = costsWindow_[i - 1];

                    // Avoid division by zero (check if previous_cost is close to zero)
                    if (std::abs(previous_cost) > 1e-10)
                    {
                        // Calculate the percentage variation
                        double percentage_variation = (std::abs(current_cost - previous_cost) / std::abs(previous_cost)) * 100.0;

                        // Keep track of the maximum percentage variation
                        if (percentage_variation > max_percentage_variation)
                        {
                            max_percentage_variation = percentage_variation;
                        }
                    }
                }

                // If the maximum percentage variation is less than the tolerance, consider it converged
                if (max_percentage_variation < tolerance_)
                {
                    converged = true;
                }
            }

            return converged;
        }

        /** \brief Termination condition */
        bool stoppingCriterion()
        {
            if(stopping_condition_ == "Iter")
                return nb_iter_ > max_iter_;
            else if(stopping_condition_ == "Convergence")
                return costConvergence();
            else if(stopping_condition_ == "Time")
            {
                auto current_time = std::chrono::steady_clock::now();
                double elapsed_time = std::chrono::duration<double>(current_time - start_time_).count();
                return elapsed_time > max_time_;
            }
            else
            {
                ROS_ERROR("Stopping criterion not implemented for post processing !");
                return false;
            } 
        }

    protected:
        /**
         * @brief Perform a smooth update given a noisy update
         * @param start_timestep starting timestep
         * @param num_timesteps number of timesteps
         * @param iteration_number number of interations allowed
         * @param updates returned smooth update
         * @return True if successful, otherwise false
         */
        bool smoothParameterUpdates(std::size_t start_timestep,
                                    std::size_t num_timesteps,
                                    int iteration_number,
                                    Eigen::MatrixXd& updates)
        {
            for (auto d = 0u; d < updates.rows(); d++)
            {
                updates.row(d).transpose() = smoothing_M_ * (updates.row(d).transpose());
            }

            return true;
        }

    protected:
        Eigen::MatrixXd initial_traj_;        /**< Initial trajectory */
        Eigen::MatrixXd times_;               /**< Time vector */
        std::vector<double> std_dev_;         /**< Standard deviation used for generating noisy parameters */
        Eigen::MatrixXd smoothing_M_;         /**< Matrix used for smoothing the trajectory */
        std::string cost_id_; // Cost id to use
        std::string post_process_file_; // File path for the result export
        double dt_; // Current time step used

        /** \brief The instance of space information this state validity checker operates on */
        ompl::base::SpaceInformation *si_;

        /** \brief The current robot */
        ompl::RobotPtr robot_;
};
#endif 